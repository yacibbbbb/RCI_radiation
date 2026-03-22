#include "radiation_simulation/radiation_sensor_plugin.hpp"

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <regex>
#include <cmath>
#include <thread>

// test
#include <fstream>
#include <ctime>

namespace gazebo
{

////////////////////////////////////////////////////
/// \brief 기본 생성자
RadiationSensorPlugin::RadiationSensorPlugin() 
: total_detected_intensity(0.0), 
  debug_mode(false)
  {}

////////////////////////////////////////////////////
/// \brief 플러그인 로드
void RadiationSensorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  try{
    // Gazebo 모델 포인터 저장
    this->model = _model;

    // ROS2 노드 초기화
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }

    // SDF에서 파라미터 읽기 (없으면 기본값 사용)
    if (_sdf->HasElement("detector_radius"))
      this->detector_radius = _sdf->Get<double>("detector_radius");
    else
      this->detector_radius = 0.1;

    if (_sdf->HasElement("attenuation_coeff"))
      this->attenuation_coeff = _sdf->Get<double>("attenuation_coeff");
    else
      this->attenuation_coeff = 0.005;

    if (_sdf->HasElement("debug_mode"))
      this->debug_mode = _sdf->Get<bool>("debug_mode");
    else
      this->debug_mode = true;  // 디버깅을 쉽게 하기 위해 기본값을 true로 설정

    // 센서 링크 이름 파라미터 읽기 -> 방사능 센서의 정확한 위치를 가져오기 위함
    // -> 방사능 강도 계산 및 탐지 기능에 큰 영향을 끼치기 때문에 필요함
    std::string sensor_link_name = "radiation_sensor_link";
    if (_sdf->HasElement("sensor_link_name"))
      sensor_link_name = _sdf->Get<std::string>("sensor_link_name");

    this->sensor_link = this->model->GetLink(sensor_link_name);

    if (!this->sensor_link) {
      gzerr << "[RadiationSensorPlugin] 센서 링크를 찾을 수 없습니다: " << sensor_link_name << std::endl;
      return;
    } else {
      std::cout << "[RadiationSensorPlugin] 센서 링크 연결됨: " << sensor_link_name << std::endl;
    }

    // 노드 이름 중복 방지
    std::string node_name = "radiation_sensor_node_" + this->model->GetName();
    // 특수문자 제거
    node_name = std::regex_replace(node_name, std::regex("[^a-zA-Z0-9_]"), "_");
    
    // 로그 출력
    std::cout << "\n\n<센서 설정>"
    << "\n모델명= " << this->model->GetName()
    << "\n노드명= " << node_name
    << "\n반경= " << this->detector_radius 
    << "\n감쇠 계수= " << this->attenuation_coeff 
    << "\n디버그= " << (this->debug_mode ? "True" : "False") << std::endl << std::endl;

    // this->ros_node = std::make_shared<rclcpp::Node>("radiation_sensor_node");
    this->ros_node = std::make_shared<rclcpp::Node>(node_name);

    // 센서가 감지한 값을 퍼블리시하는 publisher
    this->detected_pub =
      this->ros_node->create_publisher<std_msgs::msg::Float64>(
        "/radiation_sensor/detected_intensity", 10);
        
    // 감지된 개별 소스 정보를 퍼블리시하는 publisher
    this->sources_info_pub =
      this->ros_node->create_publisher<std_msgs::msg::String>(
        "/radiation_sensor/sources_info", 10);
        
    // 디버그 정보를 퍼블리시하는 publisher (디버그 모드일 때만 사용)
    this->debug_info_pub =
      this->ros_node->create_publisher<std_msgs::msg::String>(
        "/radiation_sensor/debug_info", 10);

    // 디버깅 정보 외에 파싱 전용 장애물 정보 퍼블리셔
    this->obstacles_info_pub =
      this->ros_node->create_publisher<std_msgs::msg::String>(
        "/radiation_sensor/obstacles_info", 10);

    // 센서 위치 퍼블리셔
    this->sensor_pose_pub = 
      this->ros_node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/radiation_sensor/sensor_pose", rclcpp::QoS(10));
      
    // Source 위치 퍼블리셔
    this->source_pose_pub = 
      this->ros_node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/radiation_sensor/source_poses", 10);

    std::cout << "\n\n소스 및 장애물 검색 시작\n\n";

    // ROS2 파라미터 서버에 감지된 방사선 소스 목록 등록
    this->ros_node->declare_parameter("detected_sources", std::vector<std::string>{});

    // 초기 소스 및 장애물 검색
    this->FindAndSubscribeToSources();

    // 장애물 감쇠 정보를 위한 맵 초기화
    obstacle_attenuations.clear();

    // Gazebo 업데이트 이벤트에 연결
    this->update_connection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&RadiationSensorPlugin::OnUpdate, this));
  }

  
  catch (const std::exception &e)
  {
    std::cerr << "Load 함수 예외 발생: " << e.what() << std::endl;
  }
  catch (...)
  {
    std::cerr << "Load 함수 알 수 없는 예외 발생!" << std::endl;
  }
} 

////////////////////////////////////////////////////
/// \brief 방사능 소스 세기를 수신하는 콜백
void RadiationSensorPlugin::SourceIntensityCallback(const std_msgs::msg::Float64::SharedPtr msg,
                                                   const std::string& source_name)
{
  // 특정 방사능 소스의 강도를 맵에 저장
  this->source_intensities[source_name] = msg->data;
}

////////////////////////////////////////////////////
/// \brief 장애물 감쇠계수를 수신하는 콜백
void RadiationSensorPlugin::ObstacleAttenuationCallback(const std_msgs::msg::Float64::SharedPtr msg,
                                                       const std::string& obstacle_name)
{
  // 특정 장애물의 감쇠계수를 맵에 저장
  this->obstacle_attenuations[obstacle_name] = msg->data;
  
  if (this->debug_mode) {
    auto debug_msg = std_msgs::msg::String();
    debug_msg.data = "장애물 감쇠계수 업데이트: " + obstacle_name + " = " + std::to_string(msg->data);
    this->debug_info_pub->publish(debug_msg);
  }
}

////////////////////////////////////////////////////
/// \brief 방사능 소스 및 장애물 검색 및 구독 설정
void RadiationSensorPlugin::FindAndSubscribeToSources()
{
  // 월드 탐색
  physics::World* world = this->model->GetWorld().get();
  if (!world) {
    std::cout << "월드 객체가 null입니다!" << std::endl;
    return;
  }
  auto model_count = world->ModelCount();
  
  std::string debug_msg = "모델 검색 결과:\n";

  static int cnt = 0;

  if(this->debug_mode){
    // 1000회 업데이트 당 1회 출력
    if(cnt % 1000 == 0){
      // 더 자세한 디버깅
      printf("========== 방사능원 검색 시작 ==========\n");
      printf("\nWorld 내 총 식별 가능한 모델 수: %d\n", model_count);
      for (unsigned int i = 0; i < model_count; ++i) {
        physics::ModelPtr model = world->ModelByIndex(i);
        if (model) {
          printf("\n모델 #%d: %s \n(위치: %f, %f, %f)\n", 
                i, model->GetName().c_str(),
                model->WorldPose().Pos().X(),
                model->WorldPose().Pos().Y(),
                model->WorldPose().Pos().Z());
        } else {
          printf("모델 #%d: null\n", i);
        }
      }
      std::cout << std::endl; // 정렬용
    }
  }

  std::vector<std::string> detected_sources;

  // "radiation_source"가 포함된 것을 방사능원으로 간주
  std::regex source_regex(".*radiation_source.*");

  // 현재 장애물들은 벽만 존재하므로, wall이 포함된 객체를 장애물로 간주
  // std::regex obstacle_regex(".*wall.*");  // 벽 이름에 wall이 포함된 모든 모델
  // "radiation_obstalce"이 포함된 것을 장애물로 간주
  std::regex obstacle_regex(".*radiation_obstacle.*");
  
  // std::string debug_msg = "모델 검색 결과:\n";
  
  // 월드의 모든 모델 검색
  for (unsigned int i = 0; i < model_count; ++i)
  {
    physics::ModelPtr model = world->ModelByIndex(i);

    if (!model) {
      std::cout << "모델 #" << i << "가 null입니다!" << std::endl;
      continue;
    }

    std::string model_name = model->GetName();

    // 패턴과 일치하는 이름을 가진 모델을 방사선 소스로 간주
    if (std::regex_search(model_name, source_regex))
    {
      // 이미 구독 중인 소스는 건너뜀
      if (this->source_subscribers.find(model_name) != this->source_subscribers.end())
      {
        detected_sources.push_back(model_name);
        continue;
      }

      // // 1000회 업데이트 당 1회 출력 - 기존 방사능원 반복 출력 방지
      // if(cnt % 1000 == 0) printf("방사능원 발견: %s\n", model_name.c_str());
      
      printf("방사능원 발견: %s\n", model_name.c_str());

      // 새 방사선 소스 발견
      std::string topic_name;
      
      // 기본 소스인 경우 (radiation_source 형식)
      if (model_name == "radiation_source") {
        topic_name = "/radiation_source/intensity";
      } 
      // 추가 생성된 소스인 경우 (radiation_source_X 형식)
      else if (model_name.find("radiation_source_") != std::string::npos) {
        topic_name = "/" + model_name + "/intensity";
      }
      // 그 외 다른 형태의 소스 이름인 경우
      else {
        topic_name = "/" + model_name + "/intensity";
      }
      
      // 수정: 로그 추가 - 토픽 구독 시작
      RCLCPP_INFO(this->ros_node->get_logger(), "토픽 구독: %s, 모델=%s\n", topic_name.c_str(), model_name.c_str());

      // 소스별 콜백 함수를 생성하여 구독 설정
      auto callback = [this, model_name](const std_msgs::msg::Float64::SharedPtr msg) {
        this->SourceIntensityCallback(msg, model_name);
      };
      
      auto sub = this->ros_node->create_subscription<std_msgs::msg::Float64>(
        topic_name, 10, callback);
        
      this->source_subscribers[model_name] = sub;
      this->source_models[model_name] = model;
      this->source_intensities[model_name] = 0.0; // 초기값
      detected_sources.push_back(model_name);
      
      if (this->debug_mode) {
        debug_msg += "새 방사선 소스 구독: " + model_name + " (토픽: " + topic_name + ")\n";
      }
    }
    // 방사선 장애물 찾기
    else if (std::regex_search(model_name, obstacle_regex))
    {
      // 장애물 모델 맵에 추가
      this->obstacle_models[model_name] = model;
      
      // 이미 구독 중인 장애물은 건너뜀
      if (this->obstacle_subscribers.find(model_name) != this->obstacle_subscribers.end())
      {
        continue;
      }

      // 새 장애물 발견 - 감쇠계수 토픽 구독
      std::string topic_name = model_name + "/attenuation_coeff";

      // 장애물별 콜백 함수를 생성하여 구독 설정
      auto callback = [this, model_name](const std_msgs::msg::Float64::SharedPtr msg) {
        this->ObstacleAttenuationCallback(msg, model_name);
      };
      
      auto sub = this->ros_node->create_subscription<std_msgs::msg::Float64>(
        topic_name, 10, callback);
        
      this->obstacle_subscribers[model_name] = sub;
      
      // 재질에 따른 초기 감쇠계수 설정 (기본값)
      double initial_attenuation = 1.0; // 더 큰 기본값 사용
      
      this->obstacle_attenuations[model_name] = initial_attenuation;
      
      if (this->debug_mode) {
        debug_msg += "새 장애물 구독: " + model_name + " (토픽: " + topic_name + ")\n";
      }
    }
  }

  // 발견된 방사선 소스 목록 업데이트
  this->ros_node->set_parameter(rclcpp::Parameter("detected_sources", detected_sources));
  
  // 1000회 업데이트 당 1회 출력
  if(cnt % 1000 == 0 && this->debug_mode) printf("\n방사능원 검색 완료: 총 %zu개\n\n", detected_sources.size());

  // 디버그 모드일 때만 디버그 정보 발행
  if (this->debug_mode) {
    auto debug_msg_ros = std_msgs::msg::String();
    debug_msg_ros.data = debug_msg;
    this->debug_info_pub->publish(debug_msg_ros);
  }
  
  // 제거된 장애물 정리
  std::vector<std::string> obsolete_obstacles;
  for (const auto& pair : this->obstacle_models) {
    std::string obstacle_name = pair.first;
    physics::ModelPtr obstacle_model = pair.second;
    
    if (!obstacle_model || !obstacle_model->GetWorld()) {
      obsolete_obstacles.push_back(obstacle_name);
    }
  }
  
  // 제거된 장애물 맵에서 삭제
  for (const auto& name : obsolete_obstacles) {
    this->obstacle_models.erase(name);
    this->obstacle_subscribers.erase(name);
    this->obstacle_attenuations.erase(name);
    
    if (this->debug_mode) {
      auto debug_msg_ros = std_msgs::msg::String();
      debug_msg_ros.data = "장애물 제거됨: " + name;
      this->debug_info_pub->publish(debug_msg_ros);
    }
  }

  cnt++;
}

////////////////////////////////////////////////////
/// \brief 두 점 사이의 직선과 박스의 교차 길이 계산
double RadiationSensorPlugin::CalculateIntersectionLength(
    const ignition::math::Vector3d& start,
    const ignition::math::Vector3d& end,
    const physics::ModelPtr& obstacle_model)
{
  // 장애물이 유효하지 않은 경우
  if (!obstacle_model || !obstacle_model->GetWorld()) return 0.0;
  
  // 장애물의 모든 링크와 충돌 상자에 대해 교차 길이를 계산
  double total_intersection_length = 0.0;
  
  physics::LinkPtr link = obstacle_model->GetLink();
  if (!link) return 0.0;
  
  auto collisions = link->GetCollisions();
  if (collisions.empty()) return 0.0;
  
  // 충돌하는 모든 객체들을 대상으로 함
  // 하나의 모델에 여러 번의 충돌이 있을 수 있기 때문에 반복문 수행
  for (auto collision : collisions) {
    physics::ShapePtr shape = collision->GetShape();
    if (!shape) continue;
    
    // 충돌체 형태에 따라 다른 계산 방법 적용

    // box 형태일 때
    if (shape->HasType(physics::Base::BOX_SHAPE)) {
      auto box = boost::dynamic_pointer_cast<physics::BoxShape>(shape);
      if (!box) continue;
      
      // 장애물의 월드 포즈 가져오기
      ignition::math::Pose3d obstacle_pose = obstacle_model->WorldPose();
      ignition::math::Vector3d obstacle_pos = obstacle_pose.Pos();
      ignition::math::Quaterniond obstacle_rot = obstacle_pose.Rot();
      
      // 박스의 크기 가져오기
      ignition::math::Vector3d box_size = box->Size();
      
      // 광선 변환: 장애물의 로컬 좌표계로 변환
      ignition::math::Vector3d local_start = obstacle_rot.RotateVectorReverse(start - obstacle_pos);
      ignition::math::Vector3d local_end = obstacle_rot.RotateVectorReverse(end - obstacle_pos);
      
      // 로컬 좌표계에서의 방향 벡터
      ignition::math::Vector3d local_dir = local_end - local_start;
      double ray_length = local_dir.Length();
      
      if (ray_length < 1e-6) return 0.0; // 길이가 0인 광선은 계산하지 않음
      
      local_dir = local_dir / ray_length; // 정규화
      
      // 박스의 반너비
      double box_half_x = box_size.X() * 0.5;
      double box_half_y = box_size.Y() * 0.5;
      double box_half_z = box_size.Z() * 0.5;
      
      // 슬랩 메서드를 사용하여 박스-광선 교차 계산
      double t_min = -std::numeric_limits<double>::infinity();
      double t_max = std::numeric_limits<double>::infinity();
      
      // X축에 대한 슬랩 테스트
      if (std::abs(local_dir.X()) > 1e-6) {
        double t1 = (-box_half_x - local_start.X()) / local_dir.X();
        double t2 = (box_half_x - local_start.X()) / local_dir.X();
        t_min = std::max(t_min, std::min(t1, t2));
        t_max = std::min(t_max, std::max(t1, t2));
      } else if (local_start.X() < -box_half_x || local_start.X() > box_half_x) {
        continue; // X축 교차 없음
      }
      
      // Y축에 대한 슬랩 테스트
      if (std::abs(local_dir.Y()) > 1e-6) {
        double t1 = (-box_half_y - local_start.Y()) / local_dir.Y();
        double t2 = (box_half_y - local_start.Y()) / local_dir.Y();
        t_min = std::max(t_min, std::min(t1, t2));
        t_max = std::min(t_max, std::max(t1, t2));
      } else if (local_start.Y() < -box_half_y || local_start.Y() > box_half_y) {
        continue; // Y축 교차 없음
      }
      
      // Z축에 대한 슬랩 테스트
      if (std::abs(local_dir.Z()) > 1e-6) {
        double t1 = (-box_half_z - local_start.Z()) / local_dir.Z();
        double t2 = (box_half_z - local_start.Z()) / local_dir.Z();
        t_min = std::max(t_min, std::min(t1, t2));
        t_max = std::min(t_max, std::max(t1, t2));
      } else if (local_start.Z() < -box_half_z || local_start.Z() > box_half_z) {
        continue; // Z축 교차 없음
      }
      
      // 교차가 있는지 확인
      if (t_max < t_min || t_max < 0 || t_min > ray_length) {
        continue; // 교차 없음
      }
      
      // 교차점이 라인 세그먼트 내에 있도록 제한
      t_min = std::max(0.0, t_min);
      t_max = std::min(t_max, ray_length);
      
      // 교차 길이 계산
      double intersection_length = t_max - t_min;
      total_intersection_length += intersection_length;
    }

    else if (shape->HasType(physics::Base::SPHERE_SHAPE)) {
      // 구 형태 지원 (필요 시 구현)
      // TODO: 구 형태의 장애물에 대한 교차 계산 추가
    }

    else if (shape->HasType(physics::Base::CYLINDER_SHAPE)) {
      // 실린더 형태 지원 (필요 시 구현)
      // TODO: 실린더 형태의 장애물에 대한 교차 계산 추가
    }

    // mesh 형태(.dae 기반)일 경우 - iterative ray-casting method
    else if (shape->HasType(physics::Base::MESH_SHAPE)) {
      ignition::math::Vector3d sensor_pos = start;
      ignition::math::Vector3d direction = (end - start).Normalize();
      double max_dist = (end - start).Length();

      physics::PhysicsEnginePtr physics_engine = obstacle_model->GetWorld()->Physics();
      physics::RayShapePtr mesh_ray = boost::dynamic_pointer_cast<physics::RayShape>(
          physics_engine->CreateShape("ray", nullptr));

      while (true) {
        std::string entity_name;
        double dist;
        mesh_ray->SetPoints(sensor_pos, end);
        mesh_ray->GetIntersection(dist, entity_name);

        if (entity_name.empty() || dist + (sensor_pos - start).Length() >= max_dist) break;

        if (entity_name.find(obstacle_model->GetName()) != std::string::npos) {
          total_intersection_length += dist;
          sensor_pos += direction * (dist + 1e-4);
        } else {
          sensor_pos += direction * (dist + 1e-4);
        }
      }
    }
  }
  
  return total_intersection_length;
}

////////////////////////////////////////////////////
/// \brief Gazebo가 매 시뮬레이션 업데이트마다 호출하는 함수
void RadiationSensorPlugin::OnUpdate()
{
  // ROS 콜백을 주기적으로 처리
  rclcpp::spin_some(this->ros_node);
  this->FindAndSubscribeToSources();

  // 총 감지된 방사선 세기 초기화
  this->total_detected_intensity = 0.0;
  
  // 개별 소스 정보를 저장할 문자열
  std::string sources_info = "";
  std::string debug_info = "";
  
  if (this->debug_mode) {
    debug_info = "센서 위치: " + 
                 std::to_string(this->sensor_link->WorldPose().Pos().X()) + ", " +
                 std::to_string(this->sensor_link->WorldPose().Pos().Y()) + ", " +
                 std::to_string(this->sensor_link->WorldPose().Pos().Z()) + "\n";
                  
    // 장애물 정보 추가
    debug_info += "등록된 장애물 목록 (총 " + std::to_string(this->obstacle_models.size()) + "개):\n";
    for (const auto& obstacle_pair : this->obstacle_attenuations) {
      const std::string& obstacle_name = obstacle_pair.first;
      double attenuation = obstacle_pair.second;
      
      auto model_it = this->obstacle_models.find(obstacle_name);
      std::string position_str = "알 수 없음";
      
      if (model_it != this->obstacle_models.end() && model_it->second) {
        const auto& pos = model_it->second->WorldPose().Pos();
        position_str = std::to_string(pos.X()) + ", " + std::to_string(pos.Y()) + ", " + std::to_string(pos.Z());
      }
      
      debug_info += "  - " + obstacle_name + ": 감쇠계수=" + 
                    std::to_string(attenuation) + ", 위치=" + position_str + "\n";
    }
    
    debug_info += "소스별 분석:\n";
  }
  
  // 센서(현재 모델)의 위치
  // position은 거리 계산 등 다른 로직에서 사용되므로, 회전 방향에 대한 정보는 따로 수집
  ignition::math::Vector3d sensor_pos = this->sensor_link->WorldPose().Pos();
  ignition::math::Quaterniond sensor_rot = this->sensor_link->WorldPose().Rot();
  geometry_msgs::msg::PoseStamped sensor_pose_msg;
  sensor_pose_msg.header.stamp = ros_node->now();
  sensor_pose_msg.header.frame_id = "map";
  sensor_pose_msg.pose.position.x = sensor_pos.X();
  sensor_pose_msg.pose.position.y = sensor_pos.Y();
  sensor_pose_msg.pose.position.z = sensor_pos.Z();

  sensor_pose_msg.pose.orientation.x = sensor_rot.X();
  sensor_pose_msg.pose.orientation.y = sensor_rot.Y();
  sensor_pose_msg.pose.orientation.z = sensor_rot.Z();
  sensor_pose_msg.pose.orientation.w = sensor_rot.W();

  sensor_pose_pub->publish(sensor_pose_msg);

  // 모든 방사선 소스에 대해 계산
  for (const auto& source_pair : this->source_models) {
    const std::string& source_name = source_pair.first;
    physics::ModelPtr source_model = source_pair.second;
    
    if (!source_model || !source_model->GetWorld()) continue;
    
    // 현재 소스의 강도 가져오기
    double current_intensity = this->source_intensities[source_name];
    if (current_intensity <= 0.0) continue;
    
    // 방사능 소스 모델의 위치
    ignition::math::Vector3d source_pos = source_model->WorldPose().Pos();

    // (테스트) GP_Filter.py 에서 사용 >> 소스 위치 전달
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = "map";
    pose_msg.header.stamp = ros_node->now();
    pose_msg.pose.position.x = source_pos.X();
    pose_msg.pose.position.y = source_pos.Y();
    pose_msg.pose.position.z = source_pos.Z();
    pose_msg.pose.orientation.w = 1.0;
    pose_msg.header.frame_id = source_name;

    source_pose_pub->publish(pose_msg);
    
    // 거리 계산
    double distance = sensor_pos.Distance(source_pos);
    
    // 기본 감쇠계수로 초기화 - 테스트용이므로 필요없어서 주석 처리
    // double total_attenuation = this->attenuation_coeff;
    
    if (this->debug_mode) {
      debug_info += "  - " + source_name + " (강도=" + std::to_string(current_intensity) + 
                   ", 거리=" + std::to_string(distance) + "):\n";
    }
    
    // ***** 모든 장애물과의 교차 길이를 계산하여 감쇠 적용 *****
    std::vector<std::string> intersected_obstacles;
    double total_obstacle_effect = 0.0;
    double total_obstacle_thickness = 0.0;
    
    // 활성 장애물만 처리
    for (const auto& obstacle_pair : this->obstacle_models) {
      std::string obstacle_name = obstacle_pair.first;
      physics::ModelPtr obstacle_model = obstacle_pair.second;
      
      if (!obstacle_model || !obstacle_model->GetWorld()) continue;
      
      // 소스-센서 직선과 장애물의 교차 길이 계산
      double intersection_length = CalculateIntersectionLength(source_pos, sensor_pos, obstacle_model);
      
      // 교차가 있는 경우 감쇠 계산
      if (intersection_length > 0.0) {

        // 교차 길이 누적 (공기 중 거리 계산용)
        // 공기 중 감쇠 작용 거리 = 총 거리 - 장애물들과의 총 교차 거리
        // 위 계산은 CaculateDetectedIntensity 함수에 가서 계산됨.
        total_obstacle_thickness += intersection_length;

        double obstacle_attenuation = 2.0; // 기본값
        
        // 등록된 감쇠계수가 있으면 사용
        auto it = obstacle_attenuations.find(obstacle_name);
        if (it != obstacle_attenuations.end()) {
          obstacle_attenuation = it->second;
        }
        
        // 교차 길이와 장애물 감쇠계수를 곱하여 총 감쇠 증가
        // 교차 길이와 감쇠계수 곱만 누적 (exp 계산은 CalculateDetectedIntensity 내부에서 수행)
        double effect = obstacle_attenuation * intersection_length;
        total_obstacle_effect += effect;
        
        // 교차한 장애물 목록에 추가
        intersected_obstacles.push_back(obstacle_name);
        
        if (this->debug_mode) {
          debug_info += "    장애물 " + obstacle_name + 
                       ": 교차 길이=" + std::to_string(intersection_length) + 
                       ", 감쇠계수=" + std::to_string(obstacle_attenuation) + 
                       ", 효과=" + std::to_string(effect) + "\n";
        }
      } else if (this->debug_mode) {
        debug_info += "    장애물 " + obstacle_name + ": 교차 없음\n";
      }
    }
    
    if (this->debug_mode && intersected_obstacles.empty()) {
      debug_info += "    장애물 없음: 기본 감쇠계수만 적용\n";
    }
    
    // 현재 소스에 대한 감지 세기 계산
    double detected_intensity = this->CalculateDetectedIntensity(
      distance, total_obstacle_effect, total_obstacle_thickness, current_intensity);
    
    // if (this->debug_mode) {
    //   debug_info += "    감쇠 계산: 거리=" + std::to_string(distance) + 
    //                ", 총감쇠계수=" + std::to_string(total_obstacle_effect) + 
    //                ", 감지강도=" + std::to_string(detected_intensity) + "\n";
    // }
      
    // 총 감지 세기에 더함
    this->total_detected_intensity += detected_intensity;
    
    // 소스 정보 추가
    sources_info += source_name + ": 강도=" + std::to_string(current_intensity) + 
                    ", 거리=" + std::to_string(distance) + 
                    ", 감지=" + std::to_string(detected_intensity);
    
    // 감쇠계수 정보 추가
    sources_info += ", 감쇠계수=" + std::to_string(total_obstacle_effect);
    
    // 장애물 정보 추가
    if (!intersected_obstacles.empty()) {
      sources_info += ", 장애물=";
      for (size_t i = 0; i < intersected_obstacles.size(); ++i) {
        if (i > 0) sources_info += ",";
        sources_info += intersected_obstacles[i];
      }
    }
    
    sources_info += "\n";
  }
  
  // 디버그용 출력
  // if (this->debug_mode) {
  //   debug_info += "총 감지 강도: " + std::to_string(this->total_detected_intensity) + "\n";
  // }

  // 수정: 로그 추가 - 방사선 감지 결과
  // if (counter % 20 == 0) {  // 화면 출력 빈도 줄임
  //   printf("\n방사선 감지: 총 강도 = %f\n", this->total_detected_intensity);
  // }

  static int cnt = 0;
  // 1000회 업데이트 당 1회 출력
  if(cnt++ % 1000 == 0 && this->debug_mode) printf("\n방사선 감지: 총 강도 = %f\n", this->total_detected_intensity);
    
  // 계산된 총 감지 강도를 퍼블리시
  auto msg = std_msgs::msg::Float64();
  msg.data = this->total_detected_intensity;
  this->detected_pub->publish(msg);
  // 디버그용 출력
  // printf("토픽 발행 시도: /radiation_sensor/detected_intensity = %f\n", msg.data);
  
  
  // 개별 소스 정보 퍼블리시
  if (!sources_info.empty()) {
    auto info_msg = std_msgs::msg::String();
    info_msg.data = sources_info;
    this->sources_info_pub->publish(info_msg);
  }
  
  // 디버깅 정보 퍼블리시 (디버그 모드일 때만)
  if (this->debug_mode && !debug_info.empty()) {
    auto debug_msg = std_msgs::msg::String();
    debug_msg.data = debug_info;
    this->debug_info_pub->publish(debug_msg);
  }

  // 장애물 파싱 전용 토픽 발행 (정규식 파싱에 최적화된 포맷)
  std_msgs::msg::String obstacles_info_msg;
  std::stringstream ss;

  for (const auto& pair : obstacle_models) {
    const auto& name = pair.first;
    const auto& model = pair.second;
    auto pos = model->WorldPose().Pos();
    auto links = model->GetLinks();

    // 기본 크기 초기화
    double size_x = 1.0, size_y = 1.0, size_z = 1.0;

    if (!links.empty()) {
      auto bbox = links[0]->BoundingBox();
      auto size = bbox.Size();
      size_x = size.X();
      size_y = size.Y();
      size_z = size.Z();
    }

    // 감쇠 계수 추출
    double attenuation = obstacle_attenuations[name];

    // Mesh 경로 추출
    std::string mesh_path = "";
    auto collisions = links[0]->GetCollisions();
    if (!collisions.empty()) {
      for (auto& collision : collisions) {
        auto shape = collision->GetShape();
        if (!shape) continue;

        // mesh shape인 경우에만 추출
        if (shape->HasType(physics::Base::MESH_SHAPE)) {
          auto mesh_shape = boost::dynamic_pointer_cast<gazebo::physics::MeshShape>(shape);
          if (mesh_shape) {
            std::string full_uri = mesh_shape->GetMeshURI();  // 예: model://reactor_room/models/wall.dae

            // model:// 부분 제거하여 상대 경로 생성
            std::regex model_uri_regex(R"(model://)");
            mesh_path = std::regex_replace(full_uri, model_uri_regex, "");  // 예: reactor_room/models/wall.dae
          }
        }
      }
    }

    ss << "장애물 " << name
      << " 위치=" << pos.X() << ", " << pos.Y() << ", " << pos.Z()
      << " 크기=" << size_x << ", " << size_y << ", " << size_z
      << " 감쇠계수=" << attenuation;

    if (!mesh_path.empty()) {
      ss << " mesh_path=" << mesh_path;
    }
  
    ss << "\n";
  }

  obstacles_info_msg.data = ss.str();
  obstacles_info_pub->publish(obstacles_info_msg);
}

////////////////////////////////////////////////////
/// \brief 감지 세기 계산 로직 (거리 역제곱, 지수 감쇠 등)
double RadiationSensorPlugin::CalculateDetectedIntensity(
  double distance, double total_obstacle_attenuation, double total_obstacle_thickness, double source_intensity)
{
  // 고정된 센서 반지름 (논문에서 r로 표현) >> 터틀봇의 SDF에서 설정되어 있는 값과 연동
  // double sensor_radius = this->detector_radius;

  // 1. Solid angle 기반 거리 감쇠 (1m에서의 값을 보정 계수로 사용 - 기존 패키지)
  // double correction_factor = 0.5 * (1 - (1 / std::sqrt(1 + sensor_radius * sensor_radius)));
  // double solid_angle_term = (1 / correction_factor) * 0.5 * (1.0 - distance / std::sqrt(sensor_radius * sensor_radius + distance * distance));
  // double solid_angle_term = sensor_radius/sensor_radius; // 감쇠 작용이 너무 심하여 일단 없는 것으로 간주. -> sensor_radius 변수 사용해야 해서 1로 만들기.

  // 1. 거리 감쇠
  // 거리 제곱의 역수
  double distance_attenuation = 1 / (distance * distance) <= 1 ? 1 / (distance * distance) : 1;

  // 거리의 역수
  // double distance_attenuation = 1 / distance <= 1 ? 1 / distance : 1;

  // 2. 공기 감쇠 (공기 중 거리 x 공기 감쇠계수)
  double air_path_length = std::max(0.0, distance - total_obstacle_thickness);
  double air_attenuation_coeff = 0.001; // 공기 감쇠계수 [m^-1]
  double air_attenuation = std::exp(-air_attenuation_coeff * air_path_length);

  // 3. 장애물 감쇠 + 공기 감쇠 포함하여 exponential 적용
  double obstacle_attenuation = std::exp(-total_obstacle_attenuation);

  // 4. 방향 감도 (아직은 적용 안함)
  double directionality_eta = 1.0;

  // 5. 최종 감쇠 인자 (공기 중 감쇠 x 장애물 감쇠)
  double attenuation_factor = air_attenuation * obstacle_attenuation;

  // 최종 감지 강도
  double detected_intensity = directionality_eta * source_intensity * distance_attenuation * attenuation_factor;

  return std::max(0.0, detected_intensity);  // 음수 방지
}

// Gazebo 플러그인 등록 매크로
GZ_REGISTER_MODEL_PLUGIN(RadiationSensorPlugin)

} // namespace gazebo
