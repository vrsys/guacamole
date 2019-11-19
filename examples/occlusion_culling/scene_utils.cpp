    
#include "scene_utils.hpp"

#include <queue>


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


        auto new_model(loader.create_geometry_from_file(random_object_name, model_path, model_material, gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::NORMALIZE_SCALE));
        

        float rand_x_trans = random_pos_cube_dimensions * std::rand() / (float)RAND_MAX - random_pos_cube_dimensions / 2.0f;
        float rand_y_trans = random_pos_cube_dimensions * std::rand() / (float)RAND_MAX - random_pos_cube_dimensions / 2.0f;
        float rand_z_trans = random_pos_cube_dimensions * std::rand() / (float)RAND_MAX - random_pos_cube_dimensions / 2.0f;

        float rand_angle_x = 360.0f * std::rand() / (float)RAND_MAX;
        float rand_angle_y = 360.0f * std::rand() / (float)RAND_MAX;
        float rand_angle_z = 360.0f * std::rand() / (float)RAND_MAX;

        float rand_scale = ( 20.0f * std::rand() / (float)RAND_MAX ) / 10.0f;


        // we want to have controle over the scaling for now, so we get the matrix that was used to create the normalizatin in scaling 
        auto norm_scale_mat = new_model->get_transform();

        gua::math::mat4 model_trans =   
                                        gua::math::mat4(scm::math::make_translation(rand_x_trans, rand_y_trans, rand_z_trans)) * // 5. we apply the random translation
                                        gua::math::mat4(scm::math::make_rotation(rand_angle_z, 0.0f, 0.0f, 1.0f)) * // 4. we rotate the model around x
                                        gua::math::mat4(scm::math::make_rotation(rand_angle_y, 0.0f, 1.0f, 0.0f)) * // 3. we rotate the model around y
                                        gua::math::mat4(scm::math::make_rotation(rand_angle_x, 1.0f, 0.0f, 0.0f)) *  // 2. we rotate the model around x
                                        gua::math::mat4(scm::math::make_scale(rand_scale, rand_scale, rand_scale)) *   // final
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


    std::queue<std::shared_ptr<gua::node::Node>> splitting_queue;


    //if we have less than 3 children, then the grouping is as good as it gets
    if(scene_occlusion_group_node->get_children().size() > 2) {
        splitting_queue.push(scene_occlusion_group_node);
    }

    std::array<std::vector<std::shared_ptr<gua::node::Node> >, 3> children_sorted_by_xyz;


    while(!splitting_queue.empty() ) {
        //get next node to split
        auto current_node_to_split = splitting_queue.front();
        splitting_queue.pop();

        current_node_to_split->update_cache();

#ifdef MAKE_OCCLUSION_CULLING_APP_VERBOSE
        std::cout << std::endl;
#endif //MAKE_OCCLUSION_CULLING_APP_VERBOSE
        auto AP_origin = current_node_to_split->get_bounding_box().surface_area();

        int best_splitting_axis = -1;
        double best_splitting_cost = std::numeric_limits<double>::max(); // get the maximal value for our type
        int best_candidate_index = 0;

#ifdef MAKE_OCCLUSION_CULLING_APP_VERBOSE
        std::cout<<AP_origin <<std::endl;
#endif //MAKE_OCCLUSION_CULLING_APP_VERBOSE

        for(unsigned int dim_idx = 0; dim_idx < 3; ++dim_idx) {
            children_sorted_by_xyz[dim_idx] = current_node_to_split->get_children();
        }

        // handle case where there are less than 32 children in the vector
        unsigned int candidate_element_offset = 1;
        unsigned int num_elements_to_distribute = children_sorted_by_xyz[0].size();
        unsigned int num_candidates = num_elements_to_distribute;

        if(num_elements_to_distribute > 32) { 
            num_candidates = 32;
            candidate_element_offset = num_elements_to_distribute / 32;
        }

        for(unsigned int dim_idx = 0; dim_idx < 3; ++dim_idx) {
#ifdef MAKE_OCCLUSION_CULLING_APP_VERBOSE
            std::cout << "Evaluating axis: " << dim_idx << std::endl;
#endif //MAKE_OCCLUSION_CULLING_APP_VERBOSE
            
            for(unsigned int candidate_index = 1; candidate_index < num_candidates; ++candidate_index) { 
                sorting_based_on_axis(children_sorted_by_xyz[dim_idx], dim_idx);

                split_children(current_node_to_split, children_sorted_by_xyz[dim_idx], candidate_index, candidate_element_offset);
                
                double cost_for_current_axis = calculate_cost(current_node_to_split);

    #ifdef MAKE_OCCLUSION_CULLING_APP_VERBOSE
                std::cout << "Cost for current axis: " << cost_for_current_axis << std::endl;
    #endif //MAKE_OCCLUSION_CULLING_APP_VERBOSE

                if(cost_for_current_axis < best_splitting_cost) {
                    best_splitting_cost = cost_for_current_axis;
                    best_splitting_axis = dim_idx;
                    best_candidate_index = candidate_index;
                }

                cleanup_intermediate_nodes(current_node_to_split);
            }
        }

    #ifdef MAKE_OCCLUSION_CULLING_APP_VERBOSE
        std::cout << "Best split: " << best_splitting_axis << " at index: " << best_candidate_index << std::endl;
    #endif //MAKE_OCCLUSION_CULLING_APP_VERBOSE
        // restore best candidate configuration by splitting once more
        split_children(current_node_to_split, children_sorted_by_xyz[best_splitting_axis], best_candidate_index, candidate_element_offset);

        auto children = current_node_to_split->get_children();

        for(unsigned int child_idx = 0; child_idx < 2; ++child_idx) {
            auto& current_child = children[child_idx];

            if(current_child->get_children().size() > 2) {
                splitting_queue.push(children[child_idx]);
            }

        }
    }

}

void sorting_based_on_axis(std::vector<std::shared_ptr<gua::node::Node>>& v, int axis){
        std::sort(v.begin(), v.end(), 
            [axis] (const std::shared_ptr<gua::node::Node> & a, const std::shared_ptr<gua::node::Node> & b) -> bool {
                return (a->get_world_position()[axis] < b->get_world_position()[axis]);
            });

}

void cleanup_intermediate_nodes(std::shared_ptr<gua::node::Node> scene_occlusion_group_node) {

    for(auto& intermediate_node : scene_occlusion_group_node->get_children()) {
        intermediate_node->clear_children();
    }
}


void split_children(std::shared_ptr<gua::node::Node> scene_occlusion_group_node, std::vector<std::shared_ptr<gua::node::Node>> & sorted_vector, 
                    unsigned int candidate_index, unsigned int candidate_element_offset) {
    scene_occlusion_group_node->clear_children();

    auto transform_node_L = scene_occlusion_group_node->add_child<gua::node::TransformNode>("transform_node_L");
    auto transform_node_R = scene_occlusion_group_node->add_child<gua::node::TransformNode>("transform_node_R");

    unsigned int index = 0;

    unsigned int pivot = candidate_index * candidate_element_offset;

    if (pivot == 1) {
        scene_occlusion_group_node->add_child(sorted_vector[0]);
        for(auto it = sorted_vector.begin(); it != sorted_vector.end(); ++it) {
            if(index > pivot) {
                transform_node_R->add_child(*it);
            }
            ++index;
        }
    } else if (pivot == sorted_vector.size()-1){
                scene_occlusion_group_node->add_child(sorted_vector[pivot-1]);
        for(auto it = sorted_vector.begin(); it != sorted_vector.end(); ++it) {
            if(index < pivot) {
                transform_node_L->add_child(*it);
            }
            ++index;
        }
    } else {
        for(auto it = sorted_vector.begin(); it != sorted_vector.end(); ++it) {
            if(index < pivot) {
                transform_node_L->add_child(*it);
            } else {
                transform_node_R->add_child(*it);
            }
            ++index;
        }
    }
}



double calculate_cost(std::shared_ptr<gua::node::Node> node){
    node->update_cache();
    auto parent_surface_area = node->get_bounding_box().surface_area();

#ifdef MAKE_OCCLUSION_CULLING_APP_VERBOSE
    std::cout << "Surface area Parent: " << parent_surface_area << std::endl;
#endif //MAKE_OCCLUSION_CULLING_APP_VERBOSE

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
        if(bb_vis_level == -1 || bb_vis_level == current_node_level || (current_node->get_children().empty() && current_node_level < bb_vis_level ) ) {
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

}