#ifndef GUA_OCCLUSION_CULLING_APP_SCENE_UTILS_HPP
#define GUA_OCCLUSION_CULLING_APP_SCENE_UTILS_HPP

#include <gua/guacamole.hpp>

#define MAKE_OCCLUSION_CULLING_APP_VERBOSE

void print_draw_times(gua::Renderer const& renderer, std::shared_ptr<gua::GlfwWindow> const& window);
void print_graph(std::shared_ptr<gua::node::Node> const& scene_root_node, int depth = 0);

void place_objects_randomly(std::string const& model_path, int32_t num_models_to_place, float random_pos_cube_dimensions, std::shared_ptr<gua::node::Node> scene_root_node);
void create_occlusion_scene(std::string const& model_path_plane, std::string const& model_path_objects, std::shared_ptr<gua::node::Node> scene_root_node);

void create_city_scene(std::shared_ptr<gua::node::Node> scene_root_node);


void create_city_quarter(std::shared_ptr<gua::node::Node> scene_root_node, 
					   	 int const start_position_x, 
						 int const end_position_x,
						 int const start_position_z,
						 int const end_position_z);

//void create_trees(std::shared_ptr<gua::node::Node> scene_root_node);

void create_simple_debug_scene( std::shared_ptr<gua::node::Node> scene_root_node);


void create_simple_demo_scene( std::shared_ptr<gua::node::Node> scene_root_node);

void show_scene_bounding_boxes(std::shared_ptr<gua::node::Node> const& scene_root_node, bool enable, int bb_vis_level = -1, int current_node_level = 0);

#endif