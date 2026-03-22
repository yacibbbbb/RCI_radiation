// radiation_virtual_sensor.cpp
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/RayShape.hh>
#include <gazebo/common/common.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

#include <ignition/math/Vector3.hh>

#include <regex>
#include <map>
#include <mutex>
#include <optional>
#include <atomic>
#include <thread>
#include <memory>
#include <cmath>

namespace radiation_sim
{

class RadiationVirtualSensor : public gazebo::WorldPlugin
{
public:
  RadiationVirtualSensor() : debug_(false) {}

  ~RadiationVirtualSensor() override
  {
    stopRosSpin();
  }

  void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf) override
  {
    world_ = _world;

    // ---- ROS2 node / executor ----
    if (!rclcpp::ok()) {
      int argc = 0; char **argv = nullptr;
      rclcpp::init(argc, argv);
    }
    node_ = std::make_shared<rclcpp::Node>("radiation_virtual_sensor");

    // params from SDF
    if (_sdf && _sdf->HasElement("debug"))
      debug_ = _sdf->Get<bool>("debug");
    if (_sdf && _sdf->HasElement("air_attenuation_coeff"))
      air_mu_ = _sdf->Get<double>("air_attenuation_coeff");

    // QoS (기본 reliable, depth 10)
    rclcpp::QoS qos(10);

    // I/O topics
    pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/virtual_sensor/pose", qos,
      std::bind(&RadiationVirtualSensor::OnPose, this, std::placeholders::_1));

    intensity_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
      "/virtual_sensor/detected_intensity", qos);

    RCLCPP_INFO(node_->get_logger(), "[RVS] Loaded. debug=%d", debug_ ? 1 : 0);

    // Gazebo update hook
    update_conn_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&RadiationVirtualSensor::OnUpdate, this));

    // 최초 스캔
    ScanEntities();

    // ROS spin thread 시작
    startRosSpin();
  }

  void Reset() override
  {
    // 월드 리셋 시 엔티티 재스캔
    ScanEntities();
  }

private:
  // ---------- ROS spin thread ----------
  void startRosSpin()
  {
    if (spin_running_) return;
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_running_ = true;
    spin_thread_ = std::thread([this]() {
      while (spin_running_ && rclcpp::ok()) {
        executor_->spin_some(std::chrono::milliseconds(50));
      }
    });
  }

  void stopRosSpin()
  {
    spin_running_ = false;
    if (executor_) executor_->cancel();
    if (spin_thread_.joinable()) spin_thread_.join();
    executor_.reset();
  }

  // ---------- Gazebo update ----------
  void OnUpdate()
  {
    // 주기적으로 월드 엔티티 재스캔
    if (++frame_cnt_ % 300 == 0) ScanEntities();

    // 새 포즈가 없으면 아무 것도 하지 않음
    if (!have_pose_.load(std::memory_order_acquire)) return;

    geometry_msgs::msg::Pose pose;
    {
      std::lock_guard<std::mutex> lk(pose_mtx_);
      if (!pending_pose_) return;
      pose = *pending_pose_;
      pending_pose_.reset();
      have_pose_.store(false, std::memory_order_release);
    }

    // ---- 강도 계산 (Gazebo 스레드에서만 수행) ----
    ignition::math::Vector3d sensor_pos(
      pose.position.x, pose.position.y, pose.position.z);

    double total = 0.0;

    for (auto &kv : sources_)
    {
      auto src_model = kv.second;
      if (!src_model || !src_model->GetWorld()) continue;

      double I = src_intensity_[kv.first];
      if (I <= 0.0) continue;

      ignition::math::Vector3d src_pos = src_model->WorldPose().Pos();
      double dist = sensor_pos.Distance(src_pos);
      if (dist < 1e-6) continue;

      double total_thick = 0.0;
      double muL = 0.0;

      for (auto &ov : obstacles_)
      {
        double L = RayMeshIntersection(src_pos, sensor_pos, ov.second);
        if (L <= 0) continue;
        total_thick += L;
        muL += obs_coeff_[ov.first] * L;
      }

      double airL   = std::max(0.0, dist - total_thick);
      double airAtt = std::exp(-air_mu_ * airL);
      double obsAtt = std::exp(-muL);
      double distAtt = std::min(1.0, 1.0 / (dist * dist));
      // double distAtt = std::min(1.0, 1.0 / dist);

      total += I * distAtt * airAtt * obsAtt;
    }

    std_msgs::msg::Float64 out;
    out.data = total;
    intensity_pub_->publish(out);
  }

  // ---------- ROS pose callback (값만 저장) ----------
  void OnPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (debug_) {
      RCLCPP_INFO(node_->get_logger(), "pose recieved (%.2f, %.2f, %.2f)",
                  msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    }
    std::lock_guard<std::mutex> lk(pose_mtx_);
    pending_pose_ = msg->pose;
    have_pose_.store(true, std::memory_order_release);
  }

  // ---------- 월드 엔티티 스캔 ----------
  void ScanEntities()
  {
    if (!world_) return;

    sources_.clear();
    obstacles_.clear();

    std::regex src_rgx(".*radiation_source.*");
    std::regex obs_rgx(".*radiation_obstacle.*");

    auto nModels = world_->ModelCount();
    for (unsigned int i = 0; i < nModels; ++i)
    {
      auto m = world_->ModelByIndex(i);
      if (!m) continue;
      const std::string name = m->GetName();

      if (std::regex_search(name, src_rgx))
      {
        // 소스 강도 구독
        if (!src_intensity_.count(name)) {
          src_intensity_[name] = 0.0;
          const std::string topic = "/" + name + "/intensity";
          src_subs_[name] =
            node_->create_subscription<std_msgs::msg::Float64>(
              topic, rclcpp::QoS(10),
              [this, name](const std_msgs::msg::Float64::SharedPtr msg)
              { src_intensity_[name] = msg->data; });
        }
        sources_[name] = m;
      }
      else if (std::regex_search(name, obs_rgx))
      {
        // 장애물 감쇠계수 구독
        if (!obs_coeff_.count(name)) obs_coeff_[name] = 2.0; // default
        const std::string topic = "/" + name + "/attenuation_coeff";
        obs_subs_[name] =
          node_->create_subscription<std_msgs::msg::Float64>(
            topic, rclcpp::QoS(10),
            [this, name](const std_msgs::msg::Float64::SharedPtr msg)
            { obs_coeff_[name] = msg->data; });
        obstacles_[name] = m;
      }
    }
  }

  // ---------- 레이-메시 관통 길이 ----------
  double RayMeshIntersection(const ignition::math::Vector3d &start,
                             const ignition::math::Vector3d &end,
                             gazebo::physics::ModelPtr model)
  {
    if (!world_) return 0.0;

    auto engine = world_->Physics();
    auto ray = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
      engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

    ignition::math::Vector3d cur = start;
    ignition::math::Vector3d dir = (end - start).Normalize();
    const double max_len = (end - start).Length();
    double acc = 0.0;

    while (true)
    {
      ray->SetPoints(cur, end);
      double dist = 0.0;
      std::string ent;
      ray->GetIntersection(dist, ent);

      const double traveled = (cur - start).Length();
      if (ent.empty() || traveled + dist >= max_len) break;

      if (ent.find(model->GetName()) != std::string::npos)
        acc += dist;

      // 살짝 앞으로 전진하여 다음 교차를 찾음
      cur += dir * (dist + 1e-4);
    }
    return acc;
  }

private:
  // Gazebo
  gazebo::physics::WorldPtr world_;
  gazebo::event::ConnectionPtr update_conn_;

  // ROS
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::atomic<bool> spin_running_{false};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr intensity_pub_;

  // scene objects
  std::map<std::string, gazebo::physics::ModelPtr> sources_;
  std::map<std::string, gazebo::physics::ModelPtr> obstacles_;

  // dynamic params/topics
  std::map<std::string, double> src_intensity_;
  std::map<std::string, double> obs_coeff_;
  std::map<std::string, rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> src_subs_;
  std::map<std::string, rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> obs_subs_;

  // pose handoff
  std::mutex pose_mtx_;
  std::optional<geometry_msgs::msg::Pose> pending_pose_;
  std::atomic<bool> have_pose_{false};

  // params
  double air_mu_ = 0.001;
  bool debug_ = false;

  // misc
  int frame_cnt_ = 0;
};

// Gazebo classic plugin export
GZ_REGISTER_WORLD_PLUGIN(RadiationVirtualSensor)

} // namespace radiation_sim
