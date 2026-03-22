#include "radiation_simulation/radiation_layer.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <tf2/utils.h>  // tf2::getYaw
#include <angles/angles.h>

using nav2_costmap_2d::LETHAL_OBSTACLE; // == 254

// IDW 계산 시 필요한 상수 선언
/*
    IDW로 추정하는 범위(d)를 역계산해보면,
    d = root(-2 * SIGMA^2 * ln w) 이고,
    w = 10e-6 일 때, d = 5.257 * SIGMA 이다.

    따라서, d(radius_)와 SIGMA는 정비례한다.

    radius_는 config/nav2_params.yaml에서 local costmap의 width, height를 통해 계산
    SIGMA_ 와 threshold는 config/IDW.yaml에서 불러옴
*/

namespace radiation_layer
{
// =========================================================================================
// [ Local & Global 공통 구현부 ]
// =========================================================================================

// [0] 생성자 & 소멸자
RadiationBaseLayer::RadiationBaseLayer() {}
RadiationBaseLayer::~RadiationBaseLayer() {}

// [1] Layer 리셋 함수
void RadiationBaseLayer::reset() {
  RCLCPP_INFO(rclcpp::get_logger("RadiationLayer"), "Resetting radiation layer");
}

// [2] 초기화 함수
void RadiationBaseLayer::onInitialize(){
  // rclcpp_lifecycle::LifecycleNode의 weak_ptr이기 때문에 lock() 후 get_parameter()를 호출해야 함.
  auto node = node_.lock();
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("RadiationLayer"), "node expired!");
    return;
  }

  // [2-1] 파라미터 선언 및 불러오기
  node->declare_parameter(name_ + ".map_data_yaml", "");
  node->declare_parameter(name_ + ".idw_yaml", "");

  // [2-2] 경로 설정
  std::string map_yaml_path = node->get_parameter(name_ + ".map_data_yaml").as_string();
  std::string idw_yaml_path = node->get_parameter(name_ + ".idw_yaml").as_string();
  if (map_yaml_path.empty() || idw_yaml_path.empty()) {
    RCLCPP_ERROR(logger_, "YAML path is empty!!!");
    return;
  }

  // [2-3] Map Yaml 로딩 및 불러오기
  auto map_config = YAML::LoadFile(map_yaml_path)["map_data"];
  map_width_ = map_config["map_width"].as<double>();
  map_height_ = map_config["map_height"].as<double>();
  origin_x_ = map_config["map_origin_x"].as<double>();
  origin_y_ = map_config["map_origin_y"].as<double>();
  resolution_ = map_config["map_resolution"].as<double>();
  max_intensity_ = map_config["max_intensity"].as<double>();

  // [2-4] IDW 추정 반경(radius_) 계산 (from nav2_params.yaml)
  /*
      local costmap의 width와 height는 시스템 설정상 int로 고정되어 있음.
      따라서 강제로 double로 형변환하여 사용
  */
  int width = 0, height = 0;
  node->get_parameter("width", width);
  node->get_parameter("height", height);
  radius_ = std::min(static_cast<double>(width), static_cast<double>(height)) / 2.0;

  // [2-5] IDW Yaml 로딩 및 불러오기
  auto idw_config = YAML::LoadFile(idw_yaml_path)["idw"];
  SIGMA_ = idw_config["sigma"].as<double>();
  threshold_ = idw_config["threshold"].as<double>();

  // [2-6] Grid 크기 초기화
  int grid_width = static_cast<int>(map_width_ / resolution_);
  int grid_height = static_cast<int>(map_height_ / resolution_);

  value_sum_grid_ = std::vector<std::vector<float>>(grid_height, std::vector<float>(grid_width, 0.0));
  weight_sum_grid_ = std::vector<std::vector<float>>(grid_height, std::vector<float>(grid_width, 0.0));
  marked_254_grid_ = std::vector<std::vector<bool>>(grid_height, std::vector<bool>(grid_width, false));

  // [2-7] isEnabled() 설정
  node->declare_parameter(name_ + ".enabled", rclcpp::ParameterValue(true));
  enabled_ = node->get_parameter(name_ + ".enabled").as_bool();

  // [2-8] current_ 플래그
  current_ = true;
}

// [3] IDW - 누적합 함수
void RadiationBaseLayer::accumulateIntensityAt(double sensor_x, double sensor_y, double intensity) {

  // [3-1] 센서(로봇) 중심 cell index (Global 기준)
  int center_i = static_cast<int>((sensor_x - origin_x_) / resolution_);
  int center_j = static_cast<int>((sensor_y - origin_y_) / resolution_);

  // [3-2] radius_ 를 cell 개수로 환산
  int max_offset = static_cast<int>(radius_ / resolution_);

  // [3-3] Global costmap 크기
  int grid_width = value_sum_grid_[0].size();
  int grid_height = value_sum_grid_.size();

  // [3-4] 누적합
  for (int dx = -max_offset; dx <= max_offset; ++dx) {
    for (int dy = -max_offset; dy <= max_offset; ++dy) {
      /*
          center를 중심으로 
          x,y 방향에 최대 offset(local costmap 크기)만큼 +- 한 범위에 대해
          value(intensity)와 weight를 누적

          (i,j) cell에서 추정된 intensity 값은 value_sum_grid_[j][i] / weight_sum_grid_[j][i] 이다.
      */

      int i = center_i + dx;
      int j = center_j + dy;

      // Global costmap 이외의 범위는 제외
      if (i < 0 || i >= grid_width || j < 0 || j >= grid_height) continue;

      // 임의의 cell의 index -> Global 기준의 index
      double world_x = origin_x_ + i * resolution_;
      double world_y = origin_y_ + j * resolution_;

      // 임의의 cell (i,j)와 센서 간 사이 거리
      double dist = std::hypot(world_x - sensor_x, world_y - sensor_y);

      // 범위 외 거리에선 pass
      if (dist > radius_) continue;

      // [IDW] 가중치 계산 -> weight = exp(-d^2/2s^2)
      double weight = std::exp(-(dist * dist) / (2.0 * SIGMA_ * SIGMA_));

      // [IDW] value 및 가중치 계산 누적
      value_sum_grid_[j][i] += intensity * weight;
      weight_sum_grid_[j][i] += weight;
    }
  }
}


// =========================================================================================
// [ Local Layer 구현부 ] 
// =========================================================================================


// [0] 생성자 & 소멸자
RadiationLocalLayer::RadiationLocalLayer()
: RadiationBaseLayer() {}
RadiationLocalLayer::~RadiationLocalLayer() {}

// [1] 초기화 함수
void RadiationLocalLayer::onInitialize()
{
  // BaseLayer 초기화 먼저 호출
  RadiationBaseLayer::onInitialize();

  auto node = node_.lock();
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("RadiationLocalLayer"), "node expired!");
    return;
  }

  // [1-1] subscriber 등록
  intensity_sub_ = node->create_subscription<std_msgs::msg::Float64>(
    "/radiation_sensor/detected_intensity", rclcpp::QoS(1),
    std::bind(&RadiationLocalLayer::intensityCallback, this, std::placeholders::_1));

  sensor_pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/radiation_sensor/sensor_pose", rclcpp::QoS(1),
    std::bind(&RadiationLocalLayer::sensorposeCallback, this, std::placeholders::_1));

  // [1-2] 퍼블리셔 초기화
  value_pub_ = node->create_publisher<std_msgs::msg::Float32MultiArray>("/radiation_local_layer/value_grid", rclcpp::QoS(1));
  weight_pub_ = node->create_publisher<std_msgs::msg::Float32MultiArray>("/radiation_local_layer/weight_grid", rclcpp::QoS(1));
  marked_pub_ = node->create_publisher<std_msgs::msg::Int8MultiArray>("/radiation_local_layer/marked_grid", rclcpp::QoS(1));
}

// [2] 센서 pose 콜백 함수
void RadiationLocalLayer::sensorposeCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  latest_pose_ = *msg;
  has_new_pose_ = true;
  tryFilterUpdate();
}

// [3] Intensity 콜백 함수
void RadiationLocalLayer::intensityCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  latest_intensity_ = msg->data;
  has_new_intensity_ = true;
  tryFilterUpdate();
}

// [4] 콜백 필터링 함수
void RadiationLocalLayer::tryFilterUpdate()
{
  if (!has_new_pose_ || !has_new_intensity_) return;

  double sensor_x = latest_pose_.pose.position.x;
  double sensor_y = latest_pose_.pose.position.y;

  accumulateIntensityAt(sensor_x, sensor_y, latest_intensity_);

  current_ = true;
  has_new_pose_ = false;
  has_new_intensity_ = false;

  publishGrids(); // Radiation Global Layer에 전송
}

// [5] Global layer에 publish 하는 함수
void RadiationLocalLayer::publishGrids()
{
  std_msgs::msg::Float32MultiArray value_msg;
  std_msgs::msg::Float32MultiArray weight_msg;
  std_msgs::msg::Int8MultiArray marked_msg;  // bool 값만 넘기면 되기 때문에 용량 작은 것으로 선언

  int height = value_sum_grid_.size();
  int width = value_sum_grid_[0].size();

  // 기본적인 틀 선언
  value_msg.layout.dim.resize(2);
  value_msg.layout.dim[0].label = "height";
  value_msg.layout.dim[0].size = height;
  value_msg.layout.dim[0].stride = height * width;
  value_msg.layout.dim[1].label = "width";
  value_msg.layout.dim[1].size = width;
  value_msg.layout.dim[1].stride = width;

  // 틀 복제
  weight_msg.layout = value_msg.layout;
  marked_msg.layout = value_msg.layout;

  value_msg.data.reserve(height * width);
  weight_msg.data.reserve(height * width);
  marked_msg.data.reserve(height * width);

  for (int j = 0; j < height; ++j) {
    for (int i = 0; i < width; ++i) {
      value_msg.data.push_back(value_sum_grid_[j][i]);
      weight_msg.data.push_back(weight_sum_grid_[j][i]);
      marked_msg.data.push_back(marked_254_grid_[j][i] ? 1 : 0);
    }
  }

  // Publish
  value_pub_->publish(value_msg);
  weight_pub_->publish(weight_msg);
  marked_pub_->publish(marked_msg);
}

// [6] cost update 범위 지정 함수
void RadiationLocalLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!isEnabled() || !current_) return;

  *min_x = std::min(*min_x, robot_x - radius_);
  *min_y = std::min(*min_y, robot_y - radius_);
  *max_x = std::max(*max_x, robot_x + radius_);
  *max_y = std::max(*max_y, robot_y + radius_);
}

// [7] cost update 함수
void RadiationLocalLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!isEnabled() || !current_) return;

  // [7-1] Grid 크기 초기화
  int grid_width = static_cast<int>(value_sum_grid_[0].size());
  int grid_height = static_cast<int>(value_sum_grid_.size());

  // [7-2] 센서(로봇)의 pose 및 추정 범위 index로 변환 및 초기화
  double sensor_x = latest_pose_.pose.position.x;
  double sensor_y = latest_pose_.pose.position.y;
  double sensor_yaw = tf2::getYaw(latest_pose_.pose.orientation);

  int sensor_vi = static_cast<int>((sensor_x - origin_x_) / resolution_);
  int sensor_vj = static_cast<int>((sensor_y - origin_y_) / resolution_);
  int max_offset = static_cast<int>(radius_ / resolution_);

  // [7-3] 회피해야 할 고방사선 지역(cell)을 장애물(LETHAL_OBSTACLE)로 간주할 범위
  /*
      obs_R: 두께가 있는 호의 중심선
      inflation: 호의 두께
      obstacle_range: 장애물로 간주할 호의 중심각 (로봇의 바라보는 방향의 직선에 대해 ccw 방향의 각이므로 실제로는 2배가 됨.)
  */
  double obs_R = max_offset; // * 0.9;
  double inflation = max_offset * 0.15;
  double obstacle_range = M_PI / 6;

  // [7-4] 업데이트 해야 할 범위만큼 cost 업데이트
  /*
      Local: min_i ~ max_i는 3m 범위
      Global: min_i ~ max_i는 전체 맵 크기
  */
  for (int i = min_i; i < max_i; ++i) {
    for (int j = min_j; j < max_j; ++j) {
      // 1. local grid (i,j) → world 좌표
      double wx, wy;
      master_grid.mapToWorld(i, j, wx, wy); // Costmap2D API

      // 2. world 좌표 → global V/W grid index
      int vi = static_cast<int>((wx - origin_x_) / resolution_);
      int vj = static_cast<int>((wy - origin_y_) / resolution_);
      // 전체 맵 범위 밖에선 연산 X
      if (vi < 0 || vi >= grid_width || vj < 0 || vj >= grid_height) continue;

      // 3. 가중치가 너무 작은 경우(거리가 먼 경우) 무시
      double weight_sum = weight_sum_grid_[vj][vi];
      double value_sum = value_sum_grid_[vj][vi];
      if (weight_sum <= 1e-6) continue;

      // 4. 해당 cell에서 추정된 평균 intensity 및 normalized
      double avg_intensity = value_sum / weight_sum;
      double norm = std::clamp(avg_intensity / max_intensity_, 0.0, 1.0);

      // 5. 기본 cost 설정(논문 방식) 
      uint8_t cost = static_cast<uint8_t>(norm * 252.0);

      // 6. 기존 obstacle & inflation cost
      uint8_t existing_cost = master_grid.getCost(i, j);
      
      // 7. LETHAL_OBSTACLE로 지정된 곳은 호의 형태로 고정
      if (marked_254_grid_[vj][vi]) {
        if (avg_intensity >= threshold_) {
          cost = 254; // LETHAL_OBSTACLE
        }
        else{
          // 만약 IDW를 통해서 장애물로 고정되었던 cell이 낮은 방사선 강도를 가진다면, 다시 일반 cost로 설정
          marked_254_grid_[vj][vi] = false;
        }
      } else {
        /*
          C: costmap 내부의 임의의 cell index
          S: 센서 위치 index

          dist: C와 S 간의 거리
          angle_to_cell: X축을 기준으로 선분 CS까지의 각
          angle_diff: 센서(로봇)이 바라보는 방향(odom의 x축)과 C의 각도
        */
        double dist = hypot(vi - sensor_vi, vj - sensor_vj);
        double angle_to_cell = atan2((vj - sensor_vj) * 1.0, (vi - sensor_vi) * 1.0);
        double angle_diff = angles::shortest_angular_distance(sensor_yaw, angle_to_cell);

        /*
          distance_check: C와 S 간의 거리가 호 범위 내에 있는가?
          angle_check: C와 S가 이루는 각도가 센서(로봇)이 바라보는 방향에 대해 일정 범위(obstacle_range) 내부에 있는가?
          intensity_check: IDW로 추정된 Cell의 intensity가 threshold_보다 큰가?
        */
        bool distance_check = abs(dist - obs_R) <= inflation;
        bool angle_check = abs(angle_diff) < obstacle_range;
        bool intensity_check = avg_intensity >= threshold_;

        if (distance_check && angle_check && intensity_check)
        {
          cost = 254;
          marked_254_grid_[vj][vi] = true;
        }
      }
      
      // 8. 이미 존재(obstacle & inflation)하는 cost와 비교해서 가장 큰 값으로 set
      master_grid.setCost(i, j, std::max(existing_cost, cost));
    }
  }

  current_ = false;
}


// =========================================================================================
// [ Global Layer 구현부 ]
// =========================================================================================

// [0] 생성자 & 소멸자
RadiationGlobalLayer::RadiationGlobalLayer()
: RadiationBaseLayer() {}
RadiationGlobalLayer::~RadiationGlobalLayer() {}

// [1] 초기화 함수
void RadiationGlobalLayer::onInitialize()
{
  // BaseLayer 초기화만 호출
  RadiationBaseLayer::onInitialize();
 
  auto node = node_.lock();
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("RadiationGlobalLayer"), "node expired!");
    return;
  }

  // Subscription
  value_sub_ = node->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/radiation_local_layer/value_grid", rclcpp::QoS(1),
    std::bind(&RadiationGlobalLayer::valueCallback, this, std::placeholders::_1));

  weight_sub_ = node->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/radiation_local_layer/weight_grid", rclcpp::QoS(1),
    std::bind(&RadiationGlobalLayer::weightCallback, this, std::placeholders::_1));

  marked_sub_ = node->create_subscription<std_msgs::msg::Int8MultiArray>(
    "/radiation_local_layer/marked_grid", rclcpp::QoS(1),
    std::bind(&RadiationGlobalLayer::markedCallback, this, std::placeholders::_1));
}

// [2] Value 콜백
void RadiationGlobalLayer::valueCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  const auto & data = msg->data;
  int height = value_sum_grid_.size();
  int width = value_sum_grid_[0].size();

  if (static_cast<int>(data.size()) != height * width) {
    RCLCPP_WARN(logger_, "valueCallback: Grid size mismatch.");
    return;
  }

  for (int j = 0; j < height; ++j) {
    for (int i = 0; i < width; ++i) {
      int idx = j * width + i;
      value_sum_grid_[j][i] += data[idx];  // 누적
    }
  }

  current_ = true;
}

// [3] Weight 콜백
void RadiationGlobalLayer::weightCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  const auto & data = msg->data;
  int height = weight_sum_grid_.size();
  int width = weight_sum_grid_[0].size();

  if (static_cast<int>(data.size()) != height * width) {
    RCLCPP_WARN(logger_, "weightCallback: Grid size mismatch.");
    return;
  }

  for (int j = 0; j < height; ++j) {
    for (int i = 0; i < width; ++i) {
      int idx = j * width + i;
      weight_sum_grid_[j][i] += data[idx];  // 누적
    }
  }

  current_ = true;
}

// [4] 고정 장애물 콜백
void RadiationGlobalLayer::markedCallback(const std_msgs::msg::Int8MultiArray::SharedPtr msg)
{
  const auto & data = msg->data;
  int height = marked_254_grid_.size();
  int width = marked_254_grid_[0].size();

  if (static_cast<int>(data.size()) != height * width) {
    RCLCPP_WARN(logger_, "markedCallback: Grid size mismatch.");
    return;
  }

  for (int j = 0; j < height; ++j) {
    for (int i = 0; i < width; ++i) {
      int idx = j * width + i;
      if (data[idx] > 0) {
        marked_254_grid_[j][i] = true;
      }
    }
  }

  current_ = true;
}

// [2] cost update 범위 지정 함수
void RadiationGlobalLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, 
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!isEnabled() || !current_) return;

  // global은 맵 전체에 대해 update -> LETHAL_OBSTACLE 유지 위함
  *min_x = origin_x_;
  *min_y = origin_y_;
  *max_x = origin_x_ + map_width_;
  *max_y = origin_y_ + map_height_;
}

// [3] cost update 함수
void RadiationGlobalLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!isEnabled() || !current_) return;

  // [3-1] Global costmap 크기
  int grid_width = static_cast<int>(value_sum_grid_[0].size());
  int grid_height = static_cast<int>(value_sum_grid_.size());

  // [3-2] 맵 전역에 대해서 update
  for (int i = min_i; i < max_i; ++i) {
    for (int j = min_j; j < max_j; ++j) {
      // 1. local grid (i,j) → world 좌표 얻기
      double wx, wy;
      master_grid.mapToWorld(i, j, wx, wy);

      // 2. world 좌표 → global V/W grid index
      int vi = static_cast<int>((wx - origin_x_) / resolution_);
      int vj = static_cast<int>((wy - origin_y_) / resolution_);
      // 전체 맵 범위 밖에선 연산 X
      if (vi < 0 || vi >= grid_width || vj < 0 || vj >= grid_height) continue;

      // 3. 가중치가 너무 작은 경우(거리가 먼 경우) 무시
      double weight_sum = weight_sum_grid_[vj][vi];
      double value_sum = value_sum_grid_[vj][vi];

      // 거리가 멀지만, 고정 장애물인 경우 그대로 장애물로 간주
      if (weight_sum <= 1e-6){
        if (marked_254_grid_[vj][vi]) {
          master_grid.setCost(i, j, 254);
        }
        continue;
      }

      // 4. 해당 cell에서 추정된 평균 intensity 및 normalized
      double avg_intensity = value_sum / weight_sum;
      double norm = std::clamp(avg_intensity / max_intensity_, 0.0, 1.0);

      // 5. 기본 cost 설정(논문 방식) 
      uint8_t cost = static_cast<uint8_t>(norm * 252.0);

      // 6. 기존 obstacle & inflation cost
      uint8_t existing_cost = master_grid.getCost(i, j);

      // 고정 obstacle 처리
      if (marked_254_grid_[vj][vi]) {
        if (avg_intensity >= threshold_) {
          cost = 254; // LETHAL_OBSTACLE
        }
        else{
          // 만약 IDW를 통해서 장애물로 고정되었던 cell이 낮은 방사선 강도를 가진다면, 다시 일반 cost로 설정
          marked_254_grid_[vj][vi] = false;
        }
      }

      // 8. 이미 존재(obstacle & inflation)하는 cost와 비교해서 가장 큰 값으로 set
      master_grid.setCost(i, j, std::max(existing_cost, cost));
    }
  }

  current_ = false;
}

}  // namespace radiation_layer

PLUGINLIB_EXPORT_CLASS(radiation_layer::RadiationLocalLayer, nav2_costmap_2d::Layer)
PLUGINLIB_EXPORT_CLASS(radiation_layer::RadiationGlobalLayer, nav2_costmap_2d::Layer)
