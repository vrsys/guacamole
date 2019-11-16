
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
    auto root_node_bounding_box = scene_occlusion_group_node->get_bounding_box();
    auto children_sorted_x = scene_occlusion_group_node->get_children();
    auto children_sorted_y = scene_occlusion_group_node->get_children();
    auto children_sorted_z = scene_occlusion_group_node->get_children();

    //Vector nach x sortieren - Evtl eine generische Funktion schreiben?
    std::sort(children_sorted_x.begin(), children_sorted_x.end(),
        [] (const std::shared_ptr<gua::node::Node> & a, const std::shared_ptr<gua::node::Node> & b) -> bool {
        return (a->get_world_position().x > b->get_world_position().x);
    });

    //Vector nach y sortieren - Evtl eine generische Funktion schreiben?
    std::sort(children_sorted_y.begin(), children_sorted_y.end(),
        [] (const std::shared_ptr<gua::node::Node> & a, const std::shared_ptr<gua::node::Node> & b) -> bool {
        return (a->get_world_position().x > b->get_world_position().x);
    });

    //Vector nach z sortieren - Evtl eine generische Funktion schreiben?
    std::sort(children_sorted_z.begin(), children_sorted_z.end(),
        [] (const std::shared_ptr<gua::node::Node> & a, const std::shared_ptr<gua::node::Node> & b) -> bool {
        return (a->get_world_position().x > b->get_world_position().x);
    });

    scene_occlusion_group_node->clear_children();
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