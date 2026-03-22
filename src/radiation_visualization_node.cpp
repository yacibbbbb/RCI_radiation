#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <string>
#include <map>
#include <regex>
#include <sstream>

class RadiationVisualizationNode : public rclcpp::Node
{
public:
  RadiationVisualizationNode() : Node("radiation_visualization")
  {
    // 방사선 강도 데이터 구독
    detected_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "radiation_sensor/detected_intensity", 10,
      std::bind(&RadiationVisualizationNode::detectedCallback, this, std::placeholders::_1));

    source_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "radiation_source/intensity", 10,
      std::bind(&RadiationVisualizationNode::sourceCallback, this, std::placeholders::_1));
      
    // 개별 소스 정보 구독
    sources_info_sub_ = this->create_subscription<std_msgs::msg::String>(
      "radiation_sensor/sources_info", 10,
      std::bind(&RadiationVisualizationNode::sourcesInfoCallback, this, std::placeholders::_1));
      
    // 센서 디버그 정보 구독
    debug_info_sub_ = this->create_subscription<std_msgs::msg::String>(
      "radiation_sensor/debug_info", 10,
      std::bind(&RadiationVisualizationNode::debugInfoCallback, this, std::placeholders::_1));

    // 시각화 타이머 (1초마다 업데이트)
    visualization_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&RadiationVisualizationNode::visualizationTimerCallback, this));
      
    // 터미널 초기화 (깜빡임 방지를 위해 한 번만 지우기)
    std::cout << "\033[2J\033[H";

    RCLCPP_INFO(this->get_logger(), "방사선 시각화 노드가 시작되었습니다.");
  }

private:
  void detectedCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    detected_intensity_ = msg->data;
    received_detected_ = true;
  }

  void sourceCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    source_intensity_ = msg->data;
    received_source_ = true;
  }
  
  void sourcesInfoCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    // 소스 정보 저장 및 파싱
    sources_info_ = msg->data;
    parseSourceInfo();
  }
  
  void debugInfoCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    // 디버그 정보 저장
    debug_info_ = msg->data;
  }
  
  void parseSourceInfo()
  {
    // 이전 데이터 유지 (유효한 새 데이터가 없을 경우)
    std::map<std::string, SourceData> new_source_data;
    double new_total_detected_sum = 0.0;
    
    std::istringstream iss(sources_info_);
    std::string line;
    
    // 각 줄 처리
    while (std::getline(iss, line)) {
      if (line.empty()) continue;
      
      size_t name_end = line.find(": ");
      if (name_end == std::string::npos) continue;
      
      std::string source_name = line.substr(0, name_end);
      std::string info_part = line.substr(name_end + 2);
      
      SourceData data;
      data.name = source_name;
      
      // 정규 표현식으로 값 추출
      std::regex intensity_pattern("강도=([0-9.]+)");
      std::regex distance_pattern("거리=([0-9.]+)");
      std::regex detected_pattern("감지=([0-9.]+)");
      std::regex obstacle_pattern("장애물=([^,]+)");
      std::regex attenuation_pattern("감쇠계수=([0-9.]+)");
      
      std::smatch match;
      
      // 강도 추출
      if (std::regex_search(info_part, match, intensity_pattern) && match.size() > 1) {
        try {
          data.intensity = std::stod(match.str(1));
        } catch (...) {
          data.intensity = 0.0;
        }
      }
      
      // 거리 추출
      if (std::regex_search(info_part, match, distance_pattern) && match.size() > 1) {
        try {
          data.distance = std::stod(match.str(1));
        } catch (...) {
          data.distance = 0.0;
        }
      }
      
      // 감지된 강도 추출
      if (std::regex_search(info_part, match, detected_pattern) && match.size() > 1) {
        try {
          data.detected = std::stod(match.str(1));
          new_total_detected_sum += data.detected; // 합계 계산
        } catch (...) {
          data.detected = 0.0;
        }
      }
      
      // 장애물 이름 추출
      if (std::regex_search(info_part, match, obstacle_pattern) && match.size() > 1) {
        data.obstacle = match.str(1);
      }
      
      // 감쇠계수 추출
      if (std::regex_search(info_part, match, attenuation_pattern) && match.size() > 1) {
        try {
          data.attenuation_coeff = std::stod(match.str(1));
        } catch (...) {
          data.attenuation_coeff = 0.0;
        }
      }
      
      // 감쇠율 계산 (원본 대비 감소 비율)
      if (data.intensity > 0) {
        data.attenuation_rate = (1.0 - (data.detected / data.intensity)) * 100.0;
      } else {
        data.attenuation_rate = 0.0;
      }
      
      new_source_data[source_name] = data;
    }
    
    // 유효한 데이터가 있으면 업데이트
    if (!new_source_data.empty()) {
      source_data_ = new_source_data;
      total_detected_sum_ = new_total_detected_sum;
    }
  }
  
  void visualizationTimerCallback()
  {
    // 타이머로 정기적으로 업데이트
    if (received_detected_ || received_source_ || !source_data_.empty()) {
      printRadiationData();
    }
  }

  void printRadiationData()
  {
    // 터미널에 방사선 강도 정보 출력 (화면을 지우지 않고 커서 위치만 변경)
    std::cout << "\033[H"; // 커서를 홈 위치로 이동
    std::cout << "\033[J"; // 커서 위치부터 화면 끝까지 지우기
    std::cout << std::endl;
    std::cout << "===========================================================" << std::endl;
    std::cout << "                  방사선 감지 데이터                       " << std::endl;
    std::cout << "===========================================================" << std::endl;

    // 개별 소스 정보 표시
    if (!source_data_.empty()) {
      std::cout << "개별 방사선 소스 정보:" << std::endl;
      std::cout << "-----------------------------------------------------------" << std::endl;
      std::cout << "소스명              | 거리(m)  | 강도      | 감쇠율(%) | 감지 강도" << std::endl;
      std::cout << "-----------------------------------------------------------" << std::endl;
      
      for (const auto& pair : source_data_) {
        const auto& data = pair.second;
        
        // 소스 이름이 너무 길면 잘라서 표시
        std::string display_name = data.name;
        if (display_name.length() > 19) {
          display_name = display_name.substr(0, 16) + "...";
        }
        
        // ANSI 컬러 코드 (강도에 따라 색상 변경)
        std::string color_code;
        if (data.detected > 250) {
          color_code = "\033[1;31m"; // 빨강 (위험)
        } else if (data.detected > 100) {
          color_code = "\033[1;33m"; // 노랑 (주의)
        } else if (data.detected > 50) {
          color_code = "\033[1;32m"; // 녹색 (안전)
        } else {
          color_code = "\033[1;36m"; // 청록색 (매우 안전)
        }
        
        std::string reset_code = "\033[0m";
        
        std::cout << std::left << std::setw(20) << display_name << "| "
                  << std::right << std::fixed << std::setprecision(2) << std::setw(8) << data.distance << " | "
                  << std::setw(9) << data.intensity << " | "
                  << std::setw(9) << data.attenuation_rate << " | "
                  << color_code << std::setw(9) << data.detected << reset_code;
                  
        // 장애물 정보 표시 (있을 경우)
        // if (!data.obstacle.empty()) {
        //   std::cout << " | 장애물: " << data.obstacle;
        // }
        
        std::cout << std::endl;
      }
      std::cout << "-----------------------------------------------------------" << std::endl;
    }
    
    // 합계 표시
    std::cout << "\n모든 소스의 감지된 강도 합계: " << std::fixed << std::setprecision(2) << total_detected_sum_ << std::endl;
    
    // 디버그 정보 표시 (옵션)
    if (!debug_info_.empty()) {
      std::cout << "\n디버그 정보:" << std::endl;
      std::cout << "-----------------------------------------------------------" << std::endl;
      std::cout << debug_info_ << std::endl;
      std::cout << "-----------------------------------------------------------" << std::endl;
    }
    
    std::cout << "===========================================================" << std::endl;
  }

  // 구독
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr detected_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr source_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sources_info_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr debug_info_sub_;
  rclcpp::TimerBase::SharedPtr visualization_timer_;
  
  // 기본 데이터
  double detected_intensity_ = 0.0;
  double source_intensity_ = 1.0;
  std::string sources_info_ = "";
  std::string debug_info_ = "";
  bool received_detected_ = false;
  bool received_source_ = false;
  
  // 소스 정보 저장 구조체
  struct SourceData {
    std::string name;
    double distance = 0.0;
    double intensity = 0.0;
    double detected = 0.0;
    double attenuation_rate = 0.0; // 감쇠율 (%)
    std::string obstacle = ""; // 경로상의 장애물 이름
    double attenuation_coeff = 0.0; // 총 감쇠계수
  };
  
  // 소스 데이터 맵
  std::map<std::string, SourceData> source_data_;
  
  // 합계 값
  double total_detected_sum_ = 0.0;
};

int main(int argc, char** argv)
{
  // 터미널 초기화 (깜빡임 방지를 위해 한 번만 지우기)
  std::cout << "\033[2J\033[H";
  
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RadiationVisualizationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
