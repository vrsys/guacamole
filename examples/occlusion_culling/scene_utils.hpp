#ifndef GUA_OCCLUSION_CULLING_APP_SCENE_UTILS_HPP
#define GUA_OCCLUSION_CULLING_APP_SCENE_UTILS_HPP

#include <gua/guacamole.hpp>

void place_objects_randomly(std::string const& model_path, int32_t num_models_to_place, float random_pos_cube_dimensions, std::shared_ptr<gua::node::TransformNode>& scene_root_node);

void rebuild_object_hierarchy_SAH_based(std::shared_ptr<gua::node::TransformNode>& scene_root_node);

#endif