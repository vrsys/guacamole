#ifndef GUACAMOLE_NRP_CONFIG_H
#define GUACAMOLE_NRP_CONFIG_H

#include <fstream>
#include <gua/config.hpp>
#include <gua/nrp/platform.hpp>
#include <iostream>
#include <simpleini/SimpleIni.h>
#include <string>

namespace gua
{
namespace nrp
{
class GUA_NRP_DLL NRPConfig
{
  public:
    static NRPConfig &get_instance()
    {
        std::string path("nrp_config.ini");
        static NRPConfig instance(path);
        return instance;
    }

    const char *SECTION_NETWORK = "SECTION_NETWORK";
    const char *SECTION_STRING_BINDINGS = "SECTION_STRING_BINDINGS";
    const char *SECTION_RENDERING = "SECTION_RENDERING";

    int get_network_max_timeout();
    std::string get_master_host();
    unsigned int get_master_port();
    unsigned int get_request_queue_limit();
    float get_full_scene_update_frequency();
    unsigned int get_interactive_node_queue_limit();
    float get_interactive_node_update_frequency();
    int64_t get_worker_wait_milliseconds();
    int get_max_scene_frames_till_update();
    std::string get_interactive_transform_name();
    unsigned int get_shadow_map_size();
    float get_shadow_max_distance();
    float get_shadow_offset();
    std::vector<float> get_shadow_cascaded_splits();
    float get_shadow_near_clipping();
    float get_shadow_far_clipping();
    float get_point_light_falloff();
    float get_point_light_softness();
    float get_light_brightness_multiplier();
    std::string get_nrp_grid_material();
    std::string get_nrp_gazebo_material();
    std::string get_nrp_sky_map();

  private:
    const char *NETWORK_MAX_TIMEOUT = "NETWORK_MAX_TIMEOUT";
    const char *MASTER_HOST = "MASTER_HOST";
    const char *MASTER_PORT = "MASTER_PORT";
    const char *REQUEST_QUEUE_LIMIT = "REQUEST_QUEUE_LIMIT";
    const char *FULL_SCENE_UPDATE_FREQUENCY = "FULL_SCENE_UPDATE_FREQUENCY";
    const char *INTERACTIVE_NODE_QUEUE_LIMIT = "INTERACTIVE_NODE_QUEUE_LIMIT";
    const char *INTERACTIVE_NODE_UPDATE_FREQUENCY = "INTERACTIVE_NODE_UPDATE_FREQUENCY";
    const char *WORKER_WAIT_MILLISECONDS = "WORKER_WAIT_MILLISECONDS";
    const char *MAX_SCENE_FRAMES_TILL_UPDATE = "MAX_SCENE_FRAMES_TILL_UPDATE";
    const char *INTERACTIVE_TRANSFORM_NAME = "INTERACTIVE_TRANSFORM_NAME";
    const char *SHADOW_MAP_SIZE = "SHADOW_MAP_SIZE";
    const char *SHADOW_MAX_DISTANCE = "SHADOW_MAX_DISTANCE";
    const char *SHADOW_OFFSET = "SHADOW_OFFSET";
    const char *SHADOW_CASCADED_SPLIT_0 = "SHADOW_CASCADED_SPLIT_0";
    const char *SHADOW_CASCADED_SPLIT_1 = "SHADOW_CASCADED_SPLIT_1";
    const char *SHADOW_CASCADED_SPLIT_2 = "SHADOW_CASCADED_SPLIT_2";
    const char *SHADOW_CASCADED_SPLIT_3 = "SHADOW_CASCADED_SPLIT_3";
    const char *SHADOW_NEAR_CLIPPING = "SHADOW_NEAR_CLIPPING";
    const char *SHADOW_FAR_CLIPPING = "SHADOW_FAR_CLIPPING";
    const char *POINT_LIGHT_FALLOFF = "POINT_LIGHT_FALLOFF";
    const char *POINT_LIGHT_SOFTNESS = "POINT_LIGHT_SOFTNESS";
    const char *LIGHT_BRIGHTNESS_MULTIPLIER = "LIGHT_BRIGHTNESS_MULTIPLIER";
    const char *NRP_GRID_MATERIAL = "NRP_GRID_MATERIAL";
    const char *NRP_GAZEBO_MATERIAL = "NRP_GAZEBO_MATERIAL";
    const char *NRP_SKY_MAP = "NRP_SKY_MAP";

    CSimpleIniA _config;

    /// No instantiation please
    NRPConfig() = default;
    ~NRPConfig() = default;

    explicit NRPConfig(std::string &path);
};
} // namespace nrp
} // namespace gua

#endif // GUACAMOLE_NRP_CONFIG_H
