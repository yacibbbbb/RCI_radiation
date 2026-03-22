#ifndef RADIATION_SOURCE_PLUGIN_HPP
#define RADIATION_SOURCE_PLUGIN_HPP

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64.hpp"
#include <rclcpp/logging.hpp>

namespace gazebo
{
  class RadiationSourcePlugin : public ModelPlugin
  {
    public: RadiationSourcePlugin();
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    
    private: void PublishIntensity();
    private: void IntensityCallback(const std_msgs::msg::Float64::SharedPtr msg);
    private: void OnUpdate();  
    
    private: physics::ModelPtr model;
    private: event::ConnectionPtr update_connection;
    private: std::shared_ptr<rclcpp::Node> ros_node;
    private: rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr intensity_pub;
    private: rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr intensity_sub;
    private: rclcpp::TimerBase::SharedPtr timer;
    
    private: double initial_intensity;  // I_0: 초기 방사선 세기
    private: double radius;             // r: 탐지기의 반지름
  };
}

#endif