
#include "scene_utils.hpp"



void print_draw_times(gua::Renderer const& renderer, std::shared_ptr<gua::GlfwWindow> const& window) {

    float application_fps = renderer.get_application_fps();
    float rendering_fps   = window->get_rendering_fps();

    float elapsed_application_time_milliseconds = 0.0;

    if(application_fps > 0.0f) {
        elapsed_application_time_milliseconds = 1000.0 / application_fps;
    }

    std::cout << "elapsed application time ms: " << elapsed_application_time_milliseconds << " ms" << std::endl;

    float elapsed_rendering_time_milliseconds = 0.0;

    if(rendering_fps > 0.0f) {
        elapsed_rendering_time_milliseconds = 1000.0 / rendering_fps;
    }

    std::cout << "elapsed rendering time ms: " << elapsed_rendering_time_milliseconds << " ms" << std::endl;

    std::cout << std::endl;
}


void place_objects_randomly(std::string const& model_path,  int32_t num_models_to_place, float random_pos_cube_dimensions, std::shared_ptr<gua::node::Node> scene_root_node) {

    std::size_t found = model_path.find_last_of("/\\");
    
    std::string obj_name = model_path.substr(found+1);

    for(int model_index = 0; model_index < num_models_to_place; ++model_index) {
        std::string const random_object_name = obj_name + "_" + std::to_string(model_index);
        gua::TriMeshLoader loader;

        // we provide our model with the gua-default material
        auto model_material(gua::MaterialShaderDatabase::instance()->lookup("gua_default_material")->make_new_material());
        model_material->set_show_back_faces(false);
        model_material->set_render_wireframe(false);


        auto new_model(loader.create_geometry_from_file(random_object_name, model_path, model_material, gua::TriMeshLoader::NORMALIZE_SCALE));
        

        float rand_x_trans = random_pos_cube_dimensions * std::rand() / (float)RAND_MAX - random_pos_cube_dimensions / 2.0f;
        float rand_y_trans = random_pos_cube_dimensions * std::rand() / (float)RAND_MAX - random_pos_cube_dimensions / 2.0f;
        float rand_z_trans = random_pos_cube_dimensions * std::rand() / (float)RAND_MAX - random_pos_cube_dimensions / 2.0f;

        float rand_angle_x = 360.0f * std::rand() / (float)RAND_MAX;
        float rand_angle_y = 360.0f * std::rand() / (float)RAND_MAX;
        float rand_angle_z = 360.0f * std::rand() / (float)RAND_MAX;

        float rand_scale = ( 50.0f * std::rand() / (float)RAND_MAX ) / 10.0f;


        // we want to have controle over the scaling for now, so we get the matrix that was used to create the normalizatin in scaling 
        auto norm_scale_mat = new_model->get_transform();

        gua::math::mat4 model_trans =   
                                        gua::math::mat4(scm::math::make_translation(rand_x_trans, rand_y_trans, rand_z_trans)) * // 5. we apply the random translation
                                        //gua::math::mat4(scm::math::make_rotation(rand_angle_z, 0.0f, 0.0f, 1.0f)) * // 4. we rotate the model around x
                                        //gua::math::mat4(scm::math::make_rotation(rand_angle_y, 0.0f, 1.0f, 0.0f)) * // 3. we rotate the model around y
                                        //gua::math::mat4(scm::math::make_rotation(rand_angle_x, 1.0f, 0.0f, 0.0f)) *  // 2. we rotate the model around x
                                        //gua::math::mat4(scm::math::make_scale(rand_scale, rand_scale, rand_scale)) *   // final
                                        norm_scale_mat;                                                           // 1. we scale the model such that the longest size is unit size


        // override the model's transform with our calculated transformation
        new_model->set_transform(model_trans);


        new_model->set_draw_bounding_box(false);

        //add this below our reference group node
        scene_root_node->add_child(new_model);
    }

    scene_root_node->set_draw_bounding_box(false);
}

void split_scene_graph(std::shared_ptr<gua::node::Node> scene_occlusion_group_node){
    if (scene_occlusion_group_node->get_children().size() > 2) {
        scene_occlusion_group_node->update_cache();



        auto children_sorted_x = scene_occlusion_group_node->get_children();
        auto children_sorted_y = scene_occlusion_group_node->get_children();
        auto children_sorted_z = scene_occlusion_group_node->get_children();

        sorting_based_on_axis(children_sorted_x, 0);
        sorting_based_on_axis(children_sorted_y, 1);
        sorting_based_on_axis(children_sorted_z, 2);

        std::cout << std::endl;
        auto AP_origin = scene_occlusion_group_node->get_bounding_box().surface_area();

        std::cout<<AP_origin <<std::endl;

        int best_splitting_axis = -1;
        double best_splitting_cost = std::numeric_limits<double>::max(); // get the maximal value for our type

        split_children(scene_occlusion_group_node, children_sorted_x);
        double cost_x = calculate_cost(scene_occlusion_group_node);

        if(cost_x < best_splitting_cost) {
            best_splitting_cost = cost_x;
            best_splitting_axis = 0;
        }

        std::cout<< cost_x << std::endl;

        split_children(scene_occlusion_group_node, children_sorted_y);
        double cost_y = calculate_cost(scene_occlusion_group_node);

        if(cost_y < best_splitting_cost) {
            best_splitting_cost = cost_y;
            best_splitting_axis = 1;
        }


        std::cout<< cost_y <<std::endl;

        split_children(scene_occlusion_group_node, children_sorted_z);
        double cost_z = calculate_cost(scene_occlusion_group_node);
        if(cost_z < best_splitting_cost) {
            best_splitting_cost = cost_z;
            best_splitting_axis = 2;
        }

        std::cout<< cost_z <<std::endl;


 
        if(best_splitting_axis == 0 || best_splitting_axis == 1) {
            split_children(scene_occlusion_group_node, best_splitting_axis == 0 ? children_sorted_x : children_sorted_y);
        }

        auto children =  scene_occlusion_group_node->get_children();
        auto child_L = children[0];
        auto child_R = children[1];

        split_scene_graph(child_L);
        split_scene_graph(child_R);
    }
}

// input i = 1 for x axis, i = 2 for y axis, i = 3 for z axis
void sorting_based_on_axis(std::vector<std::shared_ptr<gua::node::Node>>& v, int axis){
        // sort according to positions
        std::sort(v.begin(), v.end(), 
            [axis] (const std::shared_ptr<gua::node::Node> & a, const std::shared_ptr<gua::node::Node> & b) -> bool {
                return (a->get_world_position()[axis] < b->get_world_position()[axis]);
            });

}


void split_children(std::shared_ptr<gua::node::Node> scene_occlusion_group_node, std::vector<std::shared_ptr<gua::node::Node>> & sorted_vector){
    scene_occlusion_group_node->clear_children();

    auto transform_node_L = scene_occlusion_group_node->add_child<gua::node::TransformNode>("transform_node_L");
    auto transform_node_R = scene_occlusion_group_node->add_child<gua::node::TransformNode>("transform_node_R");

    int vector_size_half = sorted_vector.size()/2;
    int index = 0;

    for(auto i = sorted_vector.begin(); i != sorted_vector.end(); ++i) {
        if(index<vector_size_half) {
            transform_node_L->add_child(*i);
        } else {
            transform_node_R->add_child(*i);
        }
        ++index;
    }



}



double calculate_cost(std::shared_ptr<gua::node::Node> node){
    node->update_cache();
    auto parent_surface_area = node->get_bounding_box().surface_area();
    std::cout << "Surface area Parent: " << parent_surface_area << std::endl;
    auto children = node->get_children();

    float summed_weighted_surface_area = 0.0f;

    for(auto& child : children) {
        child->update_cache();
        summed_weighted_surface_area += child->get_bounding_box().surface_area() * (child->get_children()).size();;
    }

    return (summed_weighted_surface_area) / parent_surface_area;

}


void show_scene_bounding_boxes(std::shared_ptr<gua::node::Node> const& current_node, bool enable, int bb_vis_level, int current_node_level) {
     
    if(enable) {
        if(bb_vis_level == -1 || bb_vis_level == current_node_level || current_node->get_children().empty()) {
            current_node->set_draw_bounding_box(true);
        } else {
            current_node->set_draw_bounding_box(false);
        }
    } else {
        current_node->set_draw_bounding_box(false);
    }

    // recursively call show_scene_bounding_boxes for children
    for(auto const& child : current_node->get_children()) {
        show_scene_bounding_boxes(child, enable, bb_vis_level, current_node_level + 1);
    }
}

void print_graph(std::shared_ptr<gua::node::Node> const& scene_root_node, int depth) {
    
    //see https://en.wikipedia.org/wiki/Box-drawing_character#Unicode for ascii table characters
    for(int dash_index = 0; dash_index < depth; ++dash_index) {
        std::cout <<" ";
    }

    //2 unicode characters for the table elements
    std::cout << "\u2517";
    std::cout << "\u2501";
    // name, tabs, typestring
    std::cout << " " << scene_root_node->get_name() << "\t\t" << scene_root_node->get_type_string() << std::endl;

    //print

    //std::cout << std::endl;

    for(auto const& child : scene_root_node->get_children()) {
        print_graph(child, depth+1);
    }

/*
    scene_root_node->set_draw_bounding_box(enable);

    // recursively call show_scene_bounding_boxes for children
    for(auto const& child : scene_root_node->get_children()) {
        show_scene_bounding_boxes(child, enable);
    }
*/
}