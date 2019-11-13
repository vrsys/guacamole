#ifndef GUA_OCCLUSION_CULLING_APP_SCENE_UTILS_HPP
#define GUA_OCCLUSION_CULLING_APP_SCENE_UTILS_HPP

#include <gua/guacamole.hpp>
void print_draw_times(gua::Renderer const& renderer, std::shared_ptr<gua::GlfwWindow> const& window);
void print_graph(std::shared_ptr<gua::node::Node> const& scene_root_node, int depth = 0);

void place_objects_randomly(std::string const& model_path, int32_t num_models_to_place, float random_pos_cube_dimensions, std::shared_ptr<gua::node::Node> scene_root_node);

void show_scene_bounding_boxes(std::shared_ptr<gua::node::Node> const& scene_root_node, bool enable);
#endif