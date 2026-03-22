#include "radiation_simulation/radiation_static_layer.hpp"
#include "pluginlib/class_list_macros.hpp"

/*
    Obstacle과 Radiation을 서로 합쳐서 하나의 새로운 static_layer(merged_map.pgm & .yaml)을 사용함.
    따라서 이 플러그인은 단독으로 Radiation Static Layer만 확인할 때만 사용함.
    이 외에는 사용할 일 없음.
*/

PLUGINLIB_EXPORT_CLASS(radiation_static_layer::RadiationStaticLayer, nav2_costmap_2d::Layer)
