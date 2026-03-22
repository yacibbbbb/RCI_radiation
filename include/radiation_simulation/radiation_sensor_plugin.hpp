#ifndef RADIATION_SENSOR_PLUGIN_HPP
#define RADIATION_SENSOR_PLUGIN_HPP

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <regex>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <thread>

namespace gazebo
{

class RadiationSensorPlugin : public ModelPlugin
{
public: RadiationSensorPlugin();

public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

private: void SourceIntensityCallback(const std_msgs::msg::Float64::SharedPtr msg,
                                     const std::string& source_name);

private: void ObstacleAttenuationCallback(const std_msgs::msg::Float64::SharedPtr msg,
                                         const std::string& obstacle_name);

private: void FindAndSubscribeToSources();

public: virtual void OnUpdate();

private: double CalculateIntersectionLength(
  const ignition::math::Vector3d& start,
  const ignition::math::Vector3d& end,
  const physics::ModelPtr& obstacle_model);

private: double CalculateDetectedIntensity(
  double distance, 
  double total_obstacle_attenuation, 
  double total_obstacle_thickness, 
  double source_intensity);

private: physics::ModelPtr model;
private: physics::LinkPtr sensor_link;
private: event::ConnectionPtr update_connection;
private: std::shared_ptr<rclcpp::Node> ros_node;

private: rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr detected_pub;

private: rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sources_info_pub;
private: rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_info_pub;
private: rclcpp::Publisher<std_msgs::msg::String>::SharedPtr obstacles_info_pub;

private: rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr sensor_pose_pub;
private: rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr source_pose_pub;

// 여러 방사선 소스 관리를 위한 변수들
private: std::map<std::string, rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> source_subscribers;
private: std::map<std::string, physics::ModelPtr> source_models;
private: std::map<std::string, double> source_intensities;

// 장애물 관리를 위한 변수들
private: std::map<std::string, rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> obstacle_subscribers;
private: std::map<std::string, physics::ModelPtr> obstacle_models;
private: std::map<std::string, double> obstacle_attenuations;
private: double total_detected_intensity; // 합산된 감지 강도
private: double detector_radius; // r: 탐지기의 반지름

// α: 기본 감쇠 계수 (예전 버전에서 사용되었으며 현재는 비활성 상태)
// 공기 감쇠 등 다른 매커니즘이 적용되므로 현재는 사용하지 않음
private: double attenuation_coeff;
private: std::string source_pattern; // 방사선원 모델 이름 패턴
private: bool debug_mode; // 디버그 모드 플래그
};

}
#endif
