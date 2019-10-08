#include <fcntl.h>
#include <gua/nrp/nrp_config.hpp>
#include <zconf.h>

namespace gua
{
namespace nrp
{
NRPConfig::NRPConfig(std::string &path) : _config(true, false, false)
{
    if(access((path).c_str(), F_OK) == -1)
    {
        std::ifstream src(std::string(GUACAMOLE_INSTALL_DIR) + "/resources/default.ini", std::ios::binary);
        std::ofstream dst(path, std::ios::binary);

        dst << src.rdbuf();
    }

    if(_config.LoadFile(path.c_str()) < 0)
    {
        throw std::runtime_error("Configuration file parsing error");
    }
}
int NRPConfig::get_network_max_timeout() { return atoi(_config.GetValue(SECTION_NETWORK, NETWORK_MAX_TIMEOUT, "0")); }
std::string NRPConfig::get_master_host() { return _config.GetValue(SECTION_NETWORK, MASTER_HOST, "0"); }
unsigned int NRPConfig::get_master_port() { return atoi(_config.GetValue(SECTION_NETWORK, MASTER_PORT, "0")); }
unsigned int NRPConfig::get_request_queue_limit() { return atoi(_config.GetValue(SECTION_NETWORK, REQUEST_QUEUE_LIMIT, "0")); }
float NRPConfig::get_full_scene_update_frequency() { return static_cast<float>(atof(_config.GetValue(SECTION_NETWORK, FULL_SCENE_UPDATE_FREQUENCY, "0"))); }
unsigned int NRPConfig::get_interactive_node_queue_limit() { return atoi(_config.GetValue(SECTION_NETWORK, INTERACTIVE_NODE_QUEUE_LIMIT, "0")); }
float NRPConfig::get_interactive_node_update_frequency() { return static_cast<float>(atof(_config.GetValue(SECTION_NETWORK, INTERACTIVE_NODE_UPDATE_FREQUENCY, "0"))); }
int64_t NRPConfig::get_worker_wait_milliseconds() { return atoi(_config.GetValue(SECTION_NETWORK, WORKER_WAIT_MILLISECONDS, "0")); }
int NRPConfig::get_max_scene_frames_till_update() { return atoi(_config.GetValue(SECTION_NETWORK, MAX_SCENE_FRAMES_TILL_UPDATE, "0")); }
std::string NRPConfig::get_interactive_transform_name() { return _config.GetValue(SECTION_STRING_BINDINGS, INTERACTIVE_TRANSFORM_NAME, "0"); }
unsigned int NRPConfig::get_shadow_map_size() { return atoi(_config.GetValue(SECTION_RENDERING, SHADOW_MAP_SIZE, "0")); }
float NRPConfig::get_shadow_max_distance() { return static_cast<float>(atof(_config.GetValue(SECTION_RENDERING, SHADOW_MAX_DISTANCE, "0"))); }
float NRPConfig::get_shadow_offset() { return static_cast<float>(atof(_config.GetValue(SECTION_RENDERING, SHADOW_OFFSET, "0"))); }
std::vector<float> NRPConfig::get_shadow_cascaded_splits()
{
    std::vector<float> splits(4);
    splits[0] = static_cast<float>(atof(_config.GetValue(SECTION_RENDERING, SHADOW_CASCADED_SPLIT_0, "0")));
    splits[1] = static_cast<float>(atof(_config.GetValue(SECTION_RENDERING, SHADOW_CASCADED_SPLIT_1, "0")));
    splits[2] = static_cast<float>(atof(_config.GetValue(SECTION_RENDERING, SHADOW_CASCADED_SPLIT_2, "0")));
    splits[3] = static_cast<float>(atof(_config.GetValue(SECTION_RENDERING, SHADOW_CASCADED_SPLIT_3, "0")));
    return splits;
}
float NRPConfig::get_shadow_near_clipping() { return static_cast<float>(atof(_config.GetValue(SECTION_RENDERING, SHADOW_NEAR_CLIPPING, "0"))); }
float NRPConfig::get_shadow_far_clipping() { return static_cast<float>(atof(_config.GetValue(SECTION_RENDERING, SHADOW_FAR_CLIPPING, "0"))); }
float NRPConfig::get_point_light_falloff() { return static_cast<float>(atof(_config.GetValue(SECTION_RENDERING, POINT_LIGHT_FALLOFF, "0"))); }
float NRPConfig::get_point_light_softness() { return static_cast<float>(atof(_config.GetValue(SECTION_RENDERING, POINT_LIGHT_SOFTNESS, "0"))); }
float NRPConfig::get_light_brightness_multiplier() { return static_cast<float>(atof(_config.GetValue(SECTION_RENDERING, LIGHT_BRIGHTNESS_MULTIPLIER, "0"))); }
std::string NRPConfig::get_nrp_grid_material() { return _config.GetValue(SECTION_STRING_BINDINGS, NRP_GRID_MATERIAL, "0"); }
std::string NRPConfig::get_nrp_gazebo_material() { return _config.GetValue(SECTION_STRING_BINDINGS, NRP_GAZEBO_MATERIAL, "0"); }
std::string NRPConfig::get_nrp_sky_map() { return _config.GetValue(SECTION_STRING_BINDINGS, NRP_SKY_MAP, "0"); }
} // namespace nrp
} // namespace gua
