#ifndef RADIATION_LAYER_HPP
#define RADIATION_LAYER_HPP

#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace radiation_layer
{

// [ BASE ]
class RadiationBaseLayer : public nav2_costmap_2d::CostmapLayer
{
    public:
        RadiationBaseLayer();
        virtual ~RadiationBaseLayer();

        virtual void onInitialize() override;
        virtual void reset() override;

        virtual void updateBounds(
            double robot_x, double robot_y, double robot_yaw,
            double *min_x, double *min_y, double *max_x, double *max_y) override = 0;
        virtual void updateCosts(
            nav2_costmap_2d::Costmap2D &master_grid,
            int min_i, int min_j, int max_i, int max_j) override = 0;

    protected:
        // map info
        double map_width_, map_height_;
        double origin_x_, origin_y_;
        double resolution_;
        double max_intensity_;

        /* IDW 관련 --------------------------------------------------- */
        // IDW parameter
        double radius_;
        double SIGMA_;
        double threshold_;

        // IDW grids
        std::vector<std::vector<float>> weight_sum_grid_;
        std::vector<std::vector<float>> value_sum_grid_;
        std::vector<std::vector<bool>> marked_254_grid_;

        // V/W 가중합 누적 함수
        void accumulateIntensityAt(double sensor_x, double sensor_y, double intensity);
        /* ----------------------------------------------------------- */
};

// [ LOCAL ]
class RadiationLocalLayer : public RadiationBaseLayer
{
    public:
        RadiationLocalLayer();
        ~RadiationLocalLayer();

        void onInitialize() override;

        void updateBounds(double robot_x, double robot_y, double robot_yaw,
            double * min_x, double * min_y, double * max_x, double * max_y) override;

        void updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                int min_i, int min_j, int max_i, int max_j) override;

        bool isClearable() override { return true; }
   
    private:    
        // Sensor 데이터 및 pose 구독용
        geometry_msgs::msg::PoseStamped latest_pose_;
        double latest_intensity_;
        bool has_new_pose_ = false;
        bool has_new_intensity_ = false;

        // Subscription (단일 intensity & sensor pose)
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr intensity_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sensor_pose_sub_;

        // Callback 함수
        void sensorposeCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void intensityCallback(const std_msgs::msg::Float64::SharedPtr msg);

        // Publisher
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr value_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr weight_pub_;
        rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr marked_pub_;

        // Publishing 함수
        void publishGrids();

        void tryFilterUpdate();
};

// [ GLOBAL ]
class RadiationGlobalLayer : public RadiationBaseLayer
{
    public:
        RadiationGlobalLayer();
        ~RadiationGlobalLayer();

        void onInitialize() override;

        void updateBounds(double robot_x, double robot_y, double robot_yaw,
            double * min_x, double * min_y, double * max_x, double * max_y) override;

        void updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                int min_i, int min_j, int max_i, int max_j) override;

        bool isClearable() override { return false; }

    private:
        // Subscription
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr value_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr weight_sub_;
        rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr marked_sub_;

        // 콜백 함수들
        void valueCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
        void weightCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
        void markedCallback(const std_msgs::msg::Int8MultiArray::SharedPtr msg);
};

}  // namespace radiation_layer

#endif  // RADIATION_LAYER_HPP