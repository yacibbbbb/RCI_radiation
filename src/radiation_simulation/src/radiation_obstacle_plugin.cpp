#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <string>
#include <map>
#include <regex>

namespace gazebo
{

class RadiationObstaclePlugin : public ModelPlugin
{
public: RadiationObstaclePlugin() : attenuation_coeff_(0.05) {}

public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  model_ = _model;

  if (_sdf->HasElement("material_type"))
    material_type_ = _sdf->Get<std::string>("material_type");
  else
    material_type_ = "unknown";
    
  // Initialize ROS node
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  
  std::string node_name = "radiation_obstacle_node_" + model_->GetName();
  // 특수문자 제거 (ROS 노드 이름에 허용되지 않는 문자)
  node_name = std::regex_replace(node_name, std::regex("[^a-zA-Z0-9_]"), "_");
  
  ros_node_ = std::make_shared<rclcpp::Node>(node_name);
  
  // 장애물 정보를 발행하는 퍼블리셔 - 모델 이름으로 토픽 생성
  attenuation_pub_ = ros_node_->create_publisher<std_msgs::msg::Float64>(
    model_->GetName() + "/attenuation_coeff", 10);
    
  // 매개변수 서버에 재질과 두께 정보 등록
  ros_node_->declare_parameter("material_type", material_type_);
  ros_node_->declare_parameter("attenuation_coeff", attenuation_coeff_);
  
  // 장애물 감쇠 계수 계산 (재질별 기본값)
  CalculateAttenuationCoefficient();

  // 타이머 생성 (100ms마다 갱신 - 10Hz)
  timer_ = ros_node_->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&RadiationObstaclePlugin::PublishAttenuationCoeff, this));
    
  // Gazebo 업데이트 이벤트 연결
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
    std::bind(&RadiationObstaclePlugin::OnUpdate, this));
}

private: void CalculateAttenuationCoefficient()
{
  // 재질별 기본 감쇠계수 맵
  std::map<std::string, double> material_coeffs = {
    {"aluminium", 0.7}, // 알루미늄
    {"stainlessSteel", 1.5}, // 스테인리스 강철
    {"concrete", 0.5}, // 콘크리트
    {"boratedConcrete", 20.0}, // 붕산화 콘크리트
    
    // 테스트 월드에 있는 모델들을 위해 남겨둠.
    {"steel", 0.15}, // 강철
    {"wood", 0.5}, // 목재
    {"lead", 2.0}, // 납 
    {"none", 0.0}, // 감쇠 미적용
  };
  
  
  attenuation_coeff_ = 2.0;

  if (material_coeffs.find(material_type_) != material_coeffs.end()) {
    attenuation_coeff_ = material_coeffs[material_type_];
  } else {
    RCLCPP_WARN(ros_node_->get_logger(), 
      "Unknown material_type '%s', using default attenuation coeff: %f",
      material_type_.c_str(), attenuation_coeff_);
  }
  
  // 파라미터 업데이트
  ros_node_->set_parameter(rclcpp::Parameter("attenuation_coeff", attenuation_coeff_));
}

private: void PublishAttenuationCoeff()
{
  // 현재 감쇠계수 발행
  auto msg = std_msgs::msg::Float64();
  msg.data = attenuation_coeff_;
  attenuation_pub_->publish(msg);
}

private: void OnUpdate()
{
  // ROS 콜백 처리
  rclcpp::spin_some(ros_node_);
}

private: physics::ModelPtr model_;
private: std::shared_ptr<rclcpp::Node> ros_node_;
private: rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr attenuation_pub_;
private: rclcpp::TimerBase::SharedPtr timer_;
private: event::ConnectionPtr update_connection_;
private: double attenuation_coeff_; // 감쇠계수 (μ)
private: std::string material_type_; // 재질 종류
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(RadiationObstaclePlugin)

}  // namespace gazebo
