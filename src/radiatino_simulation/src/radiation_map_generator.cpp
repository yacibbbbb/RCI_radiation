#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <opencv2/opencv.hpp> 

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <regex>
#include <mutex>
#include <fstream>
#include <cmath>
#include <filesystem>
#include <cstdlib>
#include <algorithm> 

class RadiationMapGenerator : public rclcpp::Node
{
public:
  RadiationMapGenerator() : Node("radiation_map_generator")
  {
    // 파라미터 초기화
    this->declare_parameter("map_resolution", 0.1);                         // 맵 해상도 (m/cell)
    this->declare_parameter("map_width", 10.0);                             // 맵 너비 (m)
    this->declare_parameter("map_height", 10.0);                            // 맵 높이 (m)
    this->declare_parameter("map_origin_x", -5.0);                          // 맵 원점 X (m)
    this->declare_parameter("map_origin_y", -5.0);                          // 맵 원점 Y (m)
    this->declare_parameter("map_update_rate", 30.0);                       // 맵 업데이트 주기 (Hz)
    this->declare_parameter("max_intensity", 400.0);                        // 최대 방사선 강도 (정규화 목적)
    this->declare_parameter("map_frame", "map");                            // 맵 프레임
    this->declare_parameter("radiation_sensor_frame", "radiation_sensor");  // 센서 프레임
    this->declare_parameter("clear_map_interval", 5.0);                     // 맵 초기화 주기 (초)
    this->declare_parameter("obstacle_visualization", true);                // 장애물 시각화 활성화
    
    // 파라미터 읽기
    map_resolution_ = this->get_parameter("map_resolution").as_double();
    map_width_ = this->get_parameter("map_width").as_double();
    map_height_ = this->get_parameter("map_height").as_double();
    map_origin_x_ = this->get_parameter("map_origin_x").as_double();
    map_origin_y_ = this->get_parameter("map_origin_y").as_double();
    map_update_rate_ = this->get_parameter("map_update_rate").as_double();
    max_intensity_ = this->get_parameter("max_intensity").as_double();
    map_frame_ = this->get_parameter("map_frame").as_string();
    radiation_sensor_frame_ = this->get_parameter("radiation_sensor_frame").as_string();
    clear_map_interval_ = this->get_parameter("clear_map_interval").as_double();
    obstacle_visualization_ = this->get_parameter("obstacle_visualization").as_bool();
    
    // 맵 크기 계산
    map_width_cells_ = static_cast<int>(map_width_ / map_resolution_);
    map_height_cells_ = static_cast<int>(map_height_ / map_resolution_);
    map_size_ = map_width_cells_ * map_height_cells_;
    
    // 맵 데이터 초기화 (모든 셀을 -1로 설정: 미탐색 영역)
    grid_data_.resize(map_size_, -1);               // normalized intensity data
    intensity_data_.resize(map_size_, 0.0);         // raw intensity data
    cell_update_time_.resize(map_size_, 0.0);       // 각 셀의 마지막 업데이트 시간 추적
    obstacle_data_.resize(map_size_, false);        // 장애물 위치 정보 추가
    obstacle_attenuation_.resize(map_size_, 0.0);   // 0.0 = 감쇠 없음
    
    // 구독자 설정
    intensity_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "radiation_sensor/detected_intensity", 10,
      std::bind(&RadiationMapGenerator::intensityCallback, this, std::placeholders::_1));
      
    sources_info_sub_ = this->create_subscription<std_msgs::msg::String>(
      "radiation_sensor/sources_info", 10,
      std::bind(&RadiationMapGenerator::sourcesInfoCallback, this, std::placeholders::_1));
    
    // 장애물 관련 구독 추가
    // obstacles_sub_ = this->create_subscription<std_msgs::msg::String>(
    //   "radiation_sensor/debug_info", 10,
    //   std::bind(&RadiationMapGenerator::obstaclesInfoCallback, this, std::placeholders::_1));
    
    obstacles_info_sub_ = this->create_subscription<std_msgs::msg::String>(
      "radiation_sensor/obstacles_info", 10,
      std::bind(&RadiationMapGenerator::obstaclesInfoCallback, this, std::placeholders::_1));

    source_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/radiation_sensor/source_poses", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        std::string source_name = msg->header.frame_id;
        source_pose_map_[source_name] = msg->pose;
      });
    
    // 발행자 설정
    grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "radiation_map", 10);
      
    markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "radiation_markers", 10);
      
    obstacle_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "obstacle_markers", 10);
    
    // tf2 설정
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // 맵 업데이트 타이머
    map_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / map_update_rate_)),
      std::bind(&RadiationMapGenerator::updateMap, this));
    
    RCLCPP_INFO(this->get_logger(), "방사선 맵 생성기가 초기화되었습니다.");
  }


  
private:

  void intensityCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_intensity_ = msg->data;
    received_intensity_ = true;
  }

/* --------------------------------------------------------------------------------------------- */
  
  void sourcesInfoCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    sources_info_ = msg->data;
    received_sources_info_ = true;
    
    // 소스 정보 파싱
    parseSourcesInfo();
  }
  
/* --------------------------------------------------------------------------------------------- */

  void obstaclesInfoCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    obstacles_info_ = msg->data;
    
    // 장애물 정보 파싱
    parseObstaclesInfo();
  }
  
/* --------------------------------------------------------------------------------------------- */

  void parseObstaclesInfo()
  {
    // 디버그 정보에서 장애물 정보 추출
    std::istringstream iss(obstacles_info_);
    std::string line;
    
    // 기존 장애물 정보를 일시적으로 저장 (업데이트를 위해)
    std::map<std::string, ObstacleData> temp_obstacles;

    std::regex obstacle_pattern(R"(장애물\s+([a-zA-Z0-9_]+)\s+위치=([+-]?[0-9.]+),\s*([+-]?[0-9.]+),\s*([+-]?[0-9.]+)\s+크기=([+-]?[0-9.]+),\s*([+-]?[0-9.]+),\s*([+-]?[0-9.]+)\s+감쇠계수=([0-9.]+)(?:\s+mesh_path=(\S+))?)");
    
    while (std::getline(iss, line)) {
      std::smatch match;
      if (std::regex_search(line, match, obstacle_pattern) && match.size() == 10) {
        std::string obstacle_name = match[1];
        double x = std::stod(match[2]);
        double y = std::stod(match[3]);
        double z = std::stod(match[4]);
        double size_x = std::stod(match[5]);
        double size_y = std::stod(match[6]);
        double size_z = std::stod(match[7]);
        double attenuation = std::stod(match[8]);
        std::string mesh_path = match[9];
      
        ObstacleData obstacle;
        obstacle.name = obstacle_name;
        obstacle.x = x;
        obstacle.y = y;
        obstacle.z = z;
        obstacle.size_x = size_x;
        obstacle.size_y = size_y;
        obstacle.size_z = size_z;
        obstacle.attenuation = attenuation;
        obstacle.mesh_path = mesh_path;
      
        temp_obstacles[obstacle_name] = obstacle;
      }
    }
    
    // 유효한 장애물 정보가 있으면 업데이트
    if (!temp_obstacles.empty()) {
      obstacles_ = temp_obstacles;
      updateObstacleMap();
    }
  }

/* --------------------------------------------------------------------------------------------- */

  void updateObstacleMap()
  {
      std::fill(obstacle_data_.begin(), obstacle_data_.end(), false); 
      std::fill(obstacle_attenuation_.begin(), obstacle_attenuation_.end(), 0.0);

      for (const auto& obstacle_pair : obstacles_) {
          const auto& obstacle = obstacle_pair.second;

          // 장애물의 4변 좌표
          double min_x = obstacle.x - obstacle.size_x / 2.0;
          double max_x = obstacle.x + obstacle.size_x / 2.0;
          double min_y = obstacle.y - obstacle.size_y / 2.0;
          double max_y = obstacle.y + obstacle.size_y / 2.0;

          for (int y = 0; y < map_height_cells_; y++) {
              for (int x = 0; x < map_width_cells_; x++) {
                  double world_x, world_y;
                  gridToWorld(x, y, world_x, world_y);

                  if (world_x >= min_x && world_x <= max_x &&
                      world_y >= min_y && world_y <= max_y) {
                      
                      int index = gridToIndex(x, y);
                      obstacle_data_[index] = true;
                      obstacle_attenuation_[index] = obstacle.attenuation;
                  }
              }
          }
      }
  }
  
/* --------------------------------------------------------------------------------------------- */

  void parseSourcesInfo()
  {
    source_data_.clear();
    
    std::istringstream iss(sources_info_);
    std::string line;
    
    while (std::getline(iss, line)) {
      if (line.empty()) continue;
      
      size_t name_end = line.find(": ");
      if (name_end == std::string::npos) continue;
      
      std::string source_name = line.substr(0, name_end);
      std::string info_part = line.substr(name_end + 2);
      
      RadiationSourceData source_data;
      source_data.name = source_name;
      
      // 정규 표현식으로 값 추출
      std::regex intensity_pattern("강도=([0-9.]+)");
      std::regex distance_pattern("거리=([0-9.]+)");
      std::regex detected_pattern("감지=([0-9.]+)");
      std::regex obstacle_pattern("장애물=([^,]+)");
      
      std::smatch match;
      
      // 강도 추출
      if (std::regex_search(info_part, match, intensity_pattern) && match.size() > 1) {
        try {
          source_data.intensity = std::stod(match.str(1));
        } catch (...) {
          source_data.intensity = 0.0;
        }
      }
      
      // 거리 추출
      if (std::regex_search(info_part, match, distance_pattern) && match.size() > 1) {
        try {
          source_data.distance = std::stod(match.str(1));
        } catch (...) {
          source_data.distance = 0.0;
        }
      }
      
      // 감지된 강도 추출
      if (std::regex_search(info_part, match, detected_pattern) && match.size() > 1) {
        try {
          source_data.detected = std::stod(match.str(1));
        } catch (...) {
          source_data.detected = 0.0;
        }
      }
      
      // 장애물 이름 추출
      if (std::regex_search(info_part, match, obstacle_pattern) && match.size() > 1) {
        source_data.obstacle = match.str(1);
        source_data.has_obstacle = true;
      }
      
      source_data_[source_name] = source_data;
    }
  }
  
/* --------------------------------------------------------------------------------------------- */

  bool getSensorPose(geometry_msgs::msg::Pose& pose)
  {
    static bool first_success = true;  // 첫 성공 여부를 추적하는 정적 변수

    try {
      // 센서 프레임에서 맵 프레임으로의 TF 변환 가져오기
      geometry_msgs::msg::TransformStamped transform_stamped = 
        tf_buffer_->lookupTransform(map_frame_, radiation_sensor_frame_, tf2::TimePointZero);

      // 변환을 Pose로 변환
      pose.position.x = transform_stamped.transform.translation.x;
      pose.position.y = transform_stamped.transform.translation.y;
      pose.position.z = transform_stamped.transform.translation.z;
      pose.orientation = transform_stamped.transform.rotation;

      // 첫 성공 시에만 로그 출력
      if (first_success) {
        RCLCPP_INFO(this->get_logger(), "센서 위치 가져오기 성공! 센서 위치: (%.4lf, %.4lf, %.4lf)", pose.position.x, pose.position.y, pose.position.z);
        first_success = false;  // 플래그 해제
      }
      
      return true;
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "센서 위치를 가져올 수 없습니다: %s", ex.what());
      return false;
    }
  }

/* --------------------------------------------------------------------------------------------- */
  
  // 월드 좌표를 그리드 셀 인덱스로 변환
  bool worldToGrid(double world_x, double world_y, int& grid_x, int& grid_y)
  {
    grid_x = static_cast<int>((world_x - map_origin_x_) / map_resolution_);
    grid_y = static_cast<int>((world_y - map_origin_y_) / map_resolution_);
    
    return (grid_x >= 0 && grid_x < map_width_cells_ && 
            grid_y >= 0 && grid_y < map_height_cells_);
  }

/* --------------------------------------------------------------------------------------------- */
  
  // 그리드 셀 인덱스를 월드 좌표로 변환
  void gridToWorld(int grid_x, int grid_y, double& world_x, double& world_y)
  {
    world_x = map_origin_x_ + (grid_x + 0.5) * map_resolution_;
    world_y = map_origin_y_ + (grid_y + 0.5) * map_resolution_;
  }

/* --------------------------------------------------------------------------------------------- */
  
  // 그리드 셀 인덱스를 1차원 배열 인덱스로 변환
  int gridToIndex(int grid_x, int grid_y)
  {
    return grid_y * map_width_cells_ + grid_x;
  }

/* --------------------------------------------------------------------------------------------- */

  void updateMap()
  {
    geometry_msgs::msg::Pose sensor_pose;
    if (!getSensorPose(sensor_pose)) {
      return;
    }
    
    double current_time = this->now().seconds();
    
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      
      if (!received_intensity_) {
        return;
      }
      
      // 센서 위치를 그리드 셀로 변환
      int grid_x, grid_y;
      if (!worldToGrid(sensor_pose.position.x, sensor_pose.position.y, grid_x, grid_y)) {
        RCLCPP_WARN(this->get_logger(), "센서가 맵 영역을 벗어났습니다");
        return;
      }
      
      // 센서 위치에 측정된 강도 기록
      int index = gridToIndex(grid_x, grid_y);
      if (index >= 0 && index < static_cast<int>(intensity_data_.size())) {
    
        intensity_data_[index] = current_intensity_;
        
        cell_update_time_[index] = current_time;  // 업데이트 시간 기록
        
        // 확인용 log
        // RCLCPP_INFO(this->get_logger(), "센서 위치 intensity = %.2f, normalized = %.2f", intensity_value, intensity_value / max_intensity_);
      }
      
      // 강도 데이터를 OccupancyGrid로 변환 (0-100 사이의 값)
      for (size_t i = 0; i < intensity_data_.size(); i++) {
        // 값이 0이면 미탐색 영역(-1)으로 설정
        if (intensity_data_[i] <= 0.0) {
          grid_data_[i] = -1;
        } else {
          int tmp = 0;

          // 정규화 후 0-100 범위로 변환
          if (intensity_data_[i] >= max_intensity_) tmp = 100;
          else {
            tmp = static_cast<int>((intensity_data_[i] / max_intensity_) * 100.0);
          }

          // 범위 제한
          if (tmp > 100) tmp = 100;
          if (tmp < 0) tmp = 0;

          grid_data_[i] = static_cast<int8_t>(tmp);
        }
      }
    }
    
    // OccupancyGrid 메시지 생성 및 발행
    publishOccupancyGrid();
    
    // 마커 발행 (3D 시각화)
    publishRadiationMarkers(sensor_pose);
    
    // 장애물 마커 발행
    if (obstacle_visualization_) {
      publishObstacleMarkers();
    }

    update_counter_++;
    if (update_counter_ % 100 == 0) {  // 100번마다 저장
      exportIntensityDataToCSV();
      
      if(update_counter_ <= 100){ // 장애물과 방사능원은 한두번만 저장해도 ok
        exportObstacleAndSourceDataToCSV();
      }
    }
  }

/* --------------------------------------------------------------------------------------------- */

  // grid 별로 저장된 방사능 강도를 csv 형태로 저장하는 함수
  void exportIntensityDataToCSV()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    std::filesystem::path out_dir =
      std::filesystem::path(__FILE__).parent_path().parent_path() / "data" / "current_map";
    std::filesystem::create_directories(out_dir);

    std::string filename = (out_dir / "intensity.csv").string();

    std::ofstream file(filename);

    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "CSV 파일을 열 수 없습니다: %s", filename.c_str());
        return;
    }

    // 헤더 작성
    file << "x,y,intensity\n";

    for (int y = 0; y < map_height_cells_; y++) {
        for (int x = 0; x < map_width_cells_; x++) {
            int index = gridToIndex(x, y);

            double world_x, world_y;
            gridToWorld(x, y, world_x, world_y);

            double intensity = intensity_data_[index];
            if (intensity <= 0.0) {
                intensity = -1.0;  // 측정되지 않은 셀
            }

            file << world_x << "," << world_y << "," << intensity << "\n";
        }
    }

    file.close();
    RCLCPP_INFO(this->get_logger(), "방사능 intensity 데이터를 CSV로 저장했습니다: %s", filename.c_str());
  }

/* --------------------------------------------------------------------------------------------- */

  void exportObstacleAndSourceDataToCSV()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    std::filesystem::path out_dir =
      std::filesystem::path(__FILE__).parent_path().parent_path() / "data" / "current_map";
    std::filesystem::create_directories(out_dir);

    // 장애물 CSV
    std::ofstream obs_file((out_dir / "obstacles.csv").string());
    obs_file << "name,x,y,size_x,size_y\n";
    for (const auto& obstacle_pair : obstacles_) {
        const auto& obs = obstacle_pair.second;
        obs_file << obs.name << "," << obs.x << "," << obs.y << "," << obs.size_x << "," << obs.size_y << "\n";
    }
    obs_file.close();

    // 방사선 소스 CSV
    std::ofstream src_file((out_dir / "sources.csv").string());
    src_file << "name,x,y,radius\n";

    // source_data_ 이름에서 위치 추출
    for (const auto& source_pair : source_data_) {
        const auto& src = source_pair.first;

        auto it = source_pose_map_.find(src);
        if (it != source_pose_map_.end()) {
          const auto& pose = it->second;
          double sx = pose.position.x;
          double sy = pose.position.y;
          double radius = 0.2;
    
          src_file << src << "," << sx << "," << sy << "," << radius << "\n";
        }
        // std::regex pos_pattern("radiation_source_.*_([\\-0-9\\.]+)_([\\-0-9\\.]+)_([\\-0-9\\.]+)");
        // std::smatch match;

        // if (std::regex_search(source_pair.first, match, pos_pattern) && match.size() > 3) {
        //     double sx = std::stod(match.str(1));
        //     double sy = std::stod(match.str(2));
        //     double radius = 0.2;  // world 파일에서 radius 고정값
        //     src_file << src.name << "," << sx << "," << sy << "," << radius << "\n";
        // }
    }
    src_file.close();

    RCLCPP_INFO(this->get_logger(), "obstacles.csv 와 sources.csv 저장 완료.");
  }

/* --------------------------------------------------------------------------------------------- */

  void publishOccupancyGrid()
  {
    auto grid_msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    
    // 헤더 설정
    grid_msg->header.stamp = this->now();
    grid_msg->header.frame_id = map_frame_;
    
    // 맵 메타데이터 설정
    grid_msg->info.resolution = map_resolution_;
    grid_msg->info.width = map_width_cells_;
    grid_msg->info.height = map_height_cells_;
    grid_msg->info.origin.position.x = map_origin_x_;
    grid_msg->info.origin.position.y = map_origin_y_;
    grid_msg->info.origin.position.z = 0.0;
    grid_msg->info.origin.orientation.w = 1.0;
    
    // 데이터 복사
    // 센서가 강도 측정 -> 강도를 0~100 사이로 정규화 -> cell 별로 class 내부 변수인 grid_data_에 저장함.
    grid_msg->data = grid_data_;
    
    // 발행
    grid_pub_->publish(std::move(grid_msg));
  }
  
/* --------------------------------------------------------------------------------------------- */

  void publishRadiationMarkers(const geometry_msgs::msg::Pose& sensor_pose)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    
    // 현재 센서 위치 마커
    visualization_msgs::msg::Marker sensor_marker;
    sensor_marker.header.frame_id = map_frame_;
    sensor_marker.header.stamp = this->now();
    sensor_marker.ns = "radiation_sensor";
    sensor_marker.id = 0;
    sensor_marker.type = visualization_msgs::msg::Marker::SPHERE;
    sensor_marker.action = visualization_msgs::msg::Marker::ADD;
    sensor_marker.pose = sensor_pose;
    sensor_marker.scale.x = 0.04;
    sensor_marker.scale.y = 0.04;
    sensor_marker.scale.z = 0.04;
    sensor_marker.color.r = 1.0;
    sensor_marker.color.g = 1.0;
    sensor_marker.color.b = 0.0;
    sensor_marker.color.a = 1.0;
    sensor_marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    
    // marker_array.markers.push_back(sensor_marker);
    
    // 방사선 강도 히트맵 점 마커 >> 구 형태로
    visualization_msgs::msg::Marker points_marker;
    points_marker.header.frame_id = map_frame_;
    points_marker.header.stamp = this->now();
    points_marker.ns = "radiation_heatmap";
    points_marker.id = 1;
    points_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    points_marker.action = visualization_msgs::msg::Marker::ADD;
    points_marker.pose.orientation.w = 1.0;
    double points_size = map_resolution_;
    points_marker.scale.x = points_size;
    points_marker.scale.y = points_size;
    points_marker.scale.z = points_size;
    points_marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    
    // 점 마커용 포인트 및 색상 설정
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (int y = 0; y < map_height_cells_; y += 2) {  // 모든 셀이 아닌 간격을 두고 샘플링
      for (int x = 0; x < map_width_cells_; x += 2) {
        int index = gridToIndex(x, y);
        if (index >= 0 && index < static_cast<int>(intensity_data_.size())) {
          double intensity = intensity_data_[index];
          if (intensity > 0.0) {
            // 월드 좌표 계산
            double world_x, world_y;
            gridToWorld(x, y, world_x, world_y);
            
            // 점 추가
            geometry_msgs::msg::Point point;
            point.x = world_x;
            point.y = world_y;
            point.z = 0.1;  // 바닥 위에 약간 띄워서 표시
            points_marker.points.push_back(point);
            
            // 색상 추가 (강도에 따라)
            std_msgs::msg::ColorRGBA color;
            
            // 강도에 따른 색상 그라데이션 (파랑 -> 녹색 -> 노랑 -> 빨강)
            double normalized = (intensity / max_intensity_) > 1.0 ? 1.0 : (intensity / max_intensity_);
            if (normalized < 0.25) {
              // 파랑 -> 청록
              color.r = 0.0;
              color.g = normalized * 4.0;
              color.b = 1.0;
            } else if (normalized < 0.5) {
              // 청록 -> 녹색
              color.r = 0.0;
              color.g = 1.0;
              color.b = 1.0 - (normalized - 0.25) * 4.0;
            } else if (normalized < 0.75) {
              // 녹색 -> 노랑
              color.r = (normalized - 0.5) * 4.0;
              color.g = 1.0;
              color.b = 0.0;
            } else if (normalized < 1.0){
              // 노랑 -> 빨강
              color.r = 1.0;
              color.g = 1.0 - (normalized - 0.75) * 4.0;
              color.b = 0.0;
            } else {
              color.r = 1.0;
              color.g = 0.0;
              color.b = 0.0;
            }
            
            color.a = 1.0;
            points_marker.colors.push_back(color);
          }
        }
      }
    }
    
    if (!points_marker.points.empty()) {
      marker_array.markers.push_back(points_marker);
    }
      
    // 방사선 소스 마커 (알려진 소스 위치 표시)
    int source_id = 100;
    for (const auto& source_pair : source_data_) {
      const auto& source = source_pair.second;
      
      // 소스 위치는 직접 얻을 수 없으므로 TF에서 가져오거나 시뮬레이션 한다
      // 여기서는 간단히 이름에서 추출 시도
      std::regex pos_pattern("radiation_source_.*_([\\-0-9\\.]+)_([\\-0-9\\.]+)_([\\-0-9\\.]+)");
      std::smatch match;
      
      if (std::regex_search(source.name, match, pos_pattern) && match.size() > 3) {
        try {
          double source_x = std::stod(match.str(1));
          double source_y = std::stod(match.str(2));
          double source_z = std::stod(match.str(3));
          
          visualization_msgs::msg::Marker source_marker;
          source_marker.header.frame_id = map_frame_;
          source_marker.header.stamp = this->now();
          source_marker.ns = "radiation_sources";
          source_marker.id = source_id++;
          source_marker.type = visualization_msgs::msg::Marker::SPHERE;
          source_marker.action = visualization_msgs::msg::Marker::ADD;
          source_marker.pose.position.x = source_x;
          source_marker.pose.position.y = source_y;
          source_marker.pose.position.z = source_z;
          source_marker.pose.orientation.w = 1.0;
          source_marker.scale.x = 0.04;
          source_marker.scale.y = 0.04;
          source_marker.scale.z = 0.04;
          source_marker.color.r = 1.0;
          source_marker.color.g = 0.0;
          source_marker.color.b = 0.0;
          source_marker.color.a = 1.0;
          source_marker.lifetime = rclcpp::Duration::from_seconds(1.0);
          
          marker_array.markers.push_back(source_marker);
        } catch (...) {
          // 파싱 오류 무시
        }
      }
    }

    // 마커 발행
    markers_pub_->publish(marker_array);
  }

/* --------------------------------------------------------------------------------------------- */

  void publishObstacleMarkers()
  {
    visualization_msgs::msg::MarkerArray marker_array;

    // 장애물 마커
    int obstacle_id = 0;

    for (const auto& obstacle_pair : obstacles_) {
      const auto& obstacle = obstacle_pair.second;
      
      // 장애물 마커 생성
      visualization_msgs::msg::Marker obstacle_marker;
      obstacle_marker.header.frame_id = map_frame_;
      obstacle_marker.header.stamp = this->now();
      obstacle_marker.ns = "obstacles";
      obstacle_marker.id = obstacle_id++;
      obstacle_marker.action = visualization_msgs::msg::Marker::ADD;
      obstacle_marker.pose.position.x = obstacle.x;
      obstacle_marker.pose.position.y = obstacle.y;
      obstacle_marker.pose.position.z = obstacle.z;
      obstacle_marker.pose.orientation.w = 1.0;
      obstacle_marker.lifetime = 	builtin_interfaces::msg::Duration(); // 무제한 유지

      if (!obstacle.mesh_path.empty()) {
        // mesh 형태로 시각화
        obstacle_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  
        // 패키지 기준의 mesh 경로
        obstacle_marker.mesh_resource = "package://radiation_simulation/worlds/" + obstacle.mesh_path;
  
        // mesh 스케일 설정 (dae 내부에 스케일 적용되었다면 1.0, 아니면 실제 size 기반으로)
        obstacle_marker.scale.x = 1.0;
        obstacle_marker.scale.y = 1.0;
        obstacle_marker.scale.z = 1.0;
  
        // 재질 정보 사용
        obstacle_marker.mesh_use_embedded_materials = true;
      } 
      else { // mesh 파일 경로가 없을 경우 >> 기본 BOX 모델
        // 기본 box 모델 시각화
        obstacle_marker.type = visualization_msgs::msg::Marker::CUBE;
        obstacle_marker.scale.x = obstacle.size_x;
        obstacle_marker.scale.y = obstacle.size_y;
        obstacle_marker.scale.z = obstacle.size_z;
      } 

      // 장애물 색상
      obstacle_marker.color.r = 0.5;
      obstacle_marker.color.g = 0.5;
      obstacle_marker.color.b = 0.5;
      obstacle_marker.color.a = 1.0;
      
      marker_array.markers.push_back(obstacle_marker);
    }

    // 마커 발행
    obstacle_markers_pub_->publish(marker_array);
  }

/* --------------------------------------------------------------------------------------------- */
// 변수 선언

// 구독자 및 발행자
rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr intensity_sub_;
rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sources_info_sub_;
rclcpp::Subscription<std_msgs::msg::String>::SharedPtr obstacles_info_sub_;
rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr source_pose_sub_; // (테스트)

rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_markers_pub_;


// TF 관련
std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

// 타이머
rclcpp::TimerBase::SharedPtr map_timer_;

// 맵 파라미터
double map_resolution_;
double map_width_;
double map_height_;
double map_origin_x_;
double map_origin_y_;
double map_update_rate_;
double max_intensity_;
double clear_map_interval_;
bool obstacle_visualization_;
std::string map_frame_;
std::string radiation_sensor_frame_;

// 맵 업데이트 카운터 변수 >> 100번마다 방사능 강도 데이터 저장하기 위함
int update_counter_ = 0;  // 맵 갱신 카운터

// 맵 크기 (셀 단위)
int map_width_cells_;
int map_height_cells_;
int map_size_;

// 맵 데이터
std::vector<int8_t> grid_data_;         // OccupancyGrid용 데이터 (-1 ~ 100)
std::vector<double> intensity_data_;    // 원시 강도 데이터
std::vector<double> cell_update_time_;  // 셀별 마지막 업데이트 시간
std::vector<bool> obstacle_data_;       // 장애물 위치 데이터
std::vector<double> obstacle_attenuation_;  // 셀별 감쇠 계수 저장

// 센서 데이터
double current_intensity_ = 0.0;
std::string sources_info_;
std::string obstacles_info_;
bool received_intensity_ = false;
bool received_sources_info_ = false;

// 소스 정보
struct RadiationSourceData {
  std::string name;
  double intensity = 0.0;
  double distance = 0.0;
  double detected = 0.0;
  std::string obstacle = "";
  bool has_obstacle = false;
};
std::map<std::string, RadiationSourceData> source_data_;

// 장애물 정보
struct ObstacleData {
std::string name;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double size_x = 1.0;
  double size_y = 1.0;
  double size_z = 1.0;
  double attenuation = 0.1;       // 기본 감쇠계수
  std::string mesh_path = "";     // mesh 파일을 통한 rviz에 obstacle 시각화
};
std::map<std::string, ObstacleData> obstacles_;

std::map<std::string, geometry_msgs::msg::Pose> source_pose_map_; // (테스트)

// 스레드 안전성
std::mutex data_mutex_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RadiationMapGenerator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}