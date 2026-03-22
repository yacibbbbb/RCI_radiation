#pragma once

#include "nav2_costmap_2d/static_layer.hpp"

namespace radiation_static_layer
{

class RadiationStaticLayer : public nav2_costmap_2d::StaticLayer
{
    public:
        RadiationStaticLayer() = default;
        virtual ~RadiationStaticLayer() = default;

    protected:
        void onInitialize() override
        {
            nav2_costmap_2d::StaticLayer::onInitialize();

            auto node = node_.lock();
            if (!node) {
            RCLCPP_ERROR(rclcpp::get_logger("RadiationStaticLayer"), "node expired!");
            return;
            }

            // radiation_static_layer의 map_topic 설정
            // 이 파라미터는 nav2_params.yaml에서 설정할 수 있으며, 기본값은 radiation_static_map
            std::string map_topic = "/radiation_static_map";
            // node->declare_parameter(name_ + ".map_topic", map_topic);
            node->get_parameter(name_ + ".map_topic", map_topic_);
        }
};

}  // namespace radiation_static_layer

/*
    Obstacle과 Radiation을 서로 합쳐서 하나의 새로운 static_layer(merged_map.pgm & .yaml)을 사용함.
    따라서 이 플러그인은 단독으로 Radiation Static Layer만 확인할 때만 사용함.
    이 외에는 사용할 일 없음.
*/