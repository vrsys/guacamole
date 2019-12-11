#ifndef GUA_OCCLUSION_CULLING_APP_SCENE_UTILS_HPP
#define GUA_OCCLUSION_CULLING_APP_SCENE_UTILS_HPP

#include <gua/guacamole.hpp>

#define MAKE_OCCLUSION_CULLING_APP_VERBOSE

void print_draw_times(gua::Renderer const& renderer, std::shared_ptr<gua::GlfwWindow> const& window);
void print_graph(std::shared_ptr<gua::node::Node> const& scene_root_node, int depth = 0);

void place_objects_randomly(std::string const& model_path, int32_t num_models_to_place, float random_pos_cube_dimensions, std::shared_ptr<gua::node::Node> scene_root_node);
void create_occlusion_scene(std::string const& model_path_plane, std::string const& model_path_objects, std::shared_ptr<gua::node::Node> scene_root_node);

void create_simple_debug_scene( std::shared_ptr<gua::node::Node> scene_root_node);
void create_simple_debug_scene_01( std::shared_ptr<gua::node::Node> scene_root_node);
void transform_trimesh_model(std::shared_ptr<gua::node::Node>& scene_root_node, 
							 std::string const& name, 
							 std::string const& path, 
							 gua::TriMeshLoader& loader, 
							 bool const is_behind,
                             bool const randomized, 
							 float x_trans = 0,
    						 float y_trans = 0,
						     float z_trans = 0,
						     float angle_x = 0,
						     float angle_y = 0,
						     float angle_z = 0,
						     float scale = 1);

void show_scene_bounding_boxes(std::shared_ptr<gua::node::Node> const& scene_root_node, bool enable, int bb_vis_level = -1, int current_node_level = 0);

#endif