
#include "scene_utils.hpp"
struct cost_vector{
    double cost;
    std::vector<std::shared_ptr<gua::node::Node>> split_vector;
};


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
    auto children_sorted_x = scene_occlusion_group_node->get_children();
    auto children_sorted_y = scene_occlusion_group_node->get_children();
    auto children_sorted_z = scene_occlusion_group_node->get_children();

    sorting_based_on_axis(children_sorted_x, 1);
    sorting_based_on_axis(children_sorted_y, 2);
    sorting_based_on_axis(children_sorted_z, 3);

    auto AP_origin = surface_area_bounding_box(scene_occlusion_group_node);

    std::cout<<AP_origin <<std::endl;

    split_children(scene_occlusion_group_node, children_sorted_x);
    double cost_x = calculate_cost(scene_occlusion_group_node);
    cost_vector cost_vector_x = cost_vector{cost_x, children_sorted_x};
    std::cout<< cost_x <<std::endl;

    split_children(scene_occlusion_group_node, children_sorted_y);
    double cost_y = calculate_cost(scene_occlusion_group_node);
    cost_vector cost_vector_y = cost_vector{cost_y, children_sorted_y};
    std::cout<< cost_y <<std::endl;

    split_children(scene_occlusion_group_node, children_sorted_z);
    double cost_z = calculate_cost(scene_occlusion_group_node);
    cost_vector cost_vector_z = cost_vector{cost_z, children_sorted_z};
    std::cout<< cost_z <<std::endl;


    std::vector<cost_vector> joined_cost_vector= {cost_vector_x, cost_vector_y, cost_vector_z};
    auto min = std::min_element(joined_cost_vector.begin(), joined_cost_vector.end(), [] (const cost_vector & a, const cost_vector & b) -> bool {return a.cost < b.cost;});

    split_children(scene_occlusion_group_node, (*min).split_vector);

}

// input i = 1 for x axis, i = 2 for y axis, i = 3 for z axis
void sorting_based_on_axis(std::vector<std::shared_ptr<gua::node::Node>>& v, int axis){

        // scene_occlusion_group_node->clear_children();
        switch(axis)
        {
            case 1:
            // sort according to x position
            std::sort(v.begin(), v.end(), 
                [] (const std::shared_ptr<gua::node::Node> & a, const std::shared_ptr<gua::node::Node> & b) -> bool {
                    return (a->get_world_position().x > b->get_world_position().x);
                });

            case 2:
            // sort according to y position
            std::sort(v.begin(), v.end(),
                [] (const std::shared_ptr<gua::node::Node> & a, const std::shared_ptr<gua::node::Node> & b) -> bool {
                    return (a->get_world_position().y > b->get_world_position().y);
                });

            case 3:
            // sort according to z position
            std::sort(v.begin(), v.end(),
                [] (const std::shared_ptr<gua::node::Node> & a, const std::shared_ptr<gua::node::Node> & b) -> bool {
                    return (a->get_world_position().z > b->get_world_position().z);
                });  

        } 
}


void split_children(std::shared_ptr<gua::node::Node> scene_occlusion_group_node, std::vector<std::shared_ptr<gua::node::Node>> & sorted_vector){
    scene_occlusion_group_node->clear_children();

    auto transform_node_0 = scene_occlusion_group_node->add_child<gua::node::TransformNode>("transform_node_0");
    auto transform_node_1 = scene_occlusion_group_node->add_child<gua::node::TransformNode>("transform_node_1");

    int vector_size = sorted_vector.size();
    int index = 0;

    for(auto i = sorted_vector.begin(); i != sorted_vector.end(); ++i) {
        if(index<vector_size/2) {
            transform_node_0->add_child(*i);
        } else {
            transform_node_1->add_child(*i);
        }
        index ++;
    }



}

double surface_area_bounding_box(std::shared_ptr<gua::node::Node> node){
    node->update_bounding_box();

    auto min = node->get_bounding_box().min;
    auto max = node->get_bounding_box().max;

    return 2*(((max.x - min.x)* (max.y - min.y)) + 
        ((max.x - min.x)* (max.z - min.z)) + 
        ((max.y - min.y)* (max.z - min.z)));


}

double calculate_cost(std::shared_ptr<gua::node::Node> node){
    auto ap = surface_area_bounding_box(node);
    auto children = node->get_children();

    auto child0 = children[0];
    auto surface_area0 = surface_area_bounding_box(child0) * (child0->get_children()).size();


    auto child1 = children[1];
    auto surface_area1 = surface_area_bounding_box(child1) * (child1->get_children()).size();

    return (surface_area0 + surface_area1)/ap;

}


void show_scene_bounding_boxes(std::shared_ptr<gua::node::Node> const& scene_root_node, bool enable) {
     scene_root_node->set_draw_bounding_box(enable);

    // recursively call show_scene_bounding_boxes for children
    for(auto const& child : scene_root_node->get_children()) {
        show_scene_bounding_boxes(child, enable);
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