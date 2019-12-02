    
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


void create_occlusion_scene(std::string const& model_path_plane, std::string const& model_path_central, std::shared_ptr<gua::node::Node> scene_root_node) {
    

    // create a central bus object 
    std::size_t found_central = model_path_central.find_last_of("/\\"); //could be entered directly by path
    std::string obj_name_central = model_path_central.substr(found_central+1);

    std::string const random_object_name_central = obj_name_central;
    gua::TriMeshLoader loader_central;

    auto model_material_central(gua::MaterialShaderDatabase::instance()->lookup("gua_default_material")->make_new_material());
    model_material_central->set_show_back_faces(true);
    model_material_central->set_render_wireframe(false);

    auto new_model_central(loader_central.create_geometry_from_file(random_object_name_central, model_path_central, model_material_central , gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::NORMALIZE_SCALE));
    auto new_model_matrix = new_model_central->get_transform();
    
    gua::math::mat4 model_trans_central;
    
    model_trans_central =   
            gua::math::mat4(scm::math::make_translation(0.0f, 0.0f, 0.0f)) * 
            gua::math::mat4(scm::math::make_rotation(0.0f, 0.0f, 0.0f, 1.0f)) *
            gua::math::mat4(scm::math::make_scale(100.0f, 100.0f, 100.0f)) *
            new_model_matrix;  
    // override the model's transform with our calculated transformation
    new_model_central->set_transform(model_trans_central);
    new_model_central->set_draw_bounding_box(false);
    scene_root_node->add_child(new_model_central);

    //create 5 walls surrounding a central object
    std::size_t found = model_path_plane.find_last_of("/\\"); //could be entered directly by path
    std::string obj_name = model_path_plane.substr(found+1);


    auto model_material_backface_culling_off(gua::MaterialShaderDatabase::instance()->lookup("gua_default_material")->make_new_material());
    model_material_backface_culling_off->set_show_back_faces(true);
    model_material_backface_culling_off->set_render_wireframe(false);
    
    for (int i = 0; i < 5; ++i)
    {
        std::string const random_object_name = obj_name;
        gua::TriMeshLoader loader;

        auto new_model(loader.create_geometry_from_file(random_object_name, model_path_plane, model_material_backface_culling_off, gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::NORMALIZE_SCALE));

        auto norm_scale_mat = new_model->get_transform();
        
        gua::math::mat4 model_trans;
        float translation_offset = 10.0f;
                    
        gua::math::mat4 turn_plane_around_mat =  gua::math::mat4(scm::math::make_scale(4.0f, 4.0f, -4.0f));

        if (i != 4)
        {
            if (i%2 == 0)
            {
                model_trans =   
                            gua::math::mat4(scm::math::make_translation(translation_offset*(1-i), 0.0f, 0.0f)) * // 5. we apply the random translation
                            gua::math::mat4(scm::math::make_rotation(90.0f*(1-i), 0.0f, 0.0f, 1.0f)) *
                            gua::math::mat4(scm::math::make_scale(10.0f, 10.0f, 10.0f)) *
                            turn_plane_around_mat * norm_scale_mat;  
            } else {
                
                model_trans =   
                            gua::math::mat4(scm::math::make_translation(0.0f, translation_offset*(2-i), 0.0f)) * // 5. we apply the random translation
                            gua::math::mat4(scm::math::make_rotation(180.0f*(i%3), 0.0f, 0.0f, 1.0f)) *
                            gua::math::mat4(scm::math::make_scale(10.0f, 10.0f, 10.0f)) *
                            turn_plane_around_mat * norm_scale_mat;             
            } 

        } else {
                
            model_trans = 
                gua::math::mat4(scm::math::make_translation(0.0f, 0.0f, -translation_offset)) * // 5. we apply the random translation
                gua::math::mat4(scm::math::make_rotation(90.0f, 1.0f, 0.0f, 0.0f)) *
                gua::math::mat4(scm::math::make_scale(10.0f, 10.0f, 10.0f)) *
                turn_plane_around_mat * norm_scale_mat;
            
        }

        // override the model's transform with our calculated transformation
        new_model->set_transform(model_trans);
        new_model->set_draw_bounding_box(false);
        scene_root_node->add_child(new_model);
    }

    scene_root_node->set_draw_bounding_box(false);

    //creating teacentrals
    /**
    std::size_t found_central = model_path_objects.find_last_of("/\\"); //could be entered directly by path
    std::string obj_name_central = model_path_objects.substr(found+1);

    for (int i = 0; i < 10; ++i) {
        std::string const random_object_name_central = obj_name_central;
        gua::TriMeshLoader loader_central;

        auto model_material_central(gua::MaterialShaderDatabase::instance()->lookup("gua_default_material")->make_new_material());
        model_material_central->set_show_back_faces(true);
        model_material_central->set_render_wireframe(false);

        auto new_model_central(loader.create_geometry_from_file(random_object_name_central, model_path_objects, model_material, gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::NORMALIZE_SCALE));

        auto norm_scale_mat_central = new_model_central->get_transform();

        float y_pos = std::sin(i / 30.0);

        gua::math::mat4 model_trans_central =   
                                    gua::math::mat4(scm::math::make_translation(0.0f, 0.5f * y_pos , float(-5.0f+i))) * // 5. we apply the random translation
                                    gua::math::mat4(scm::math::make_scale(0.5f, 0.5f, 0.5f)) *
                                    norm_scale_mat_central;  

        // override the model's transform with our calculated transformation
        new_model_central->set_transform(model_trans_central);
        new_model_central->set_draw_bounding_box(false);
        scene_root_node->add_child(new_model_central);
    }

    scene_root_node->set_draw_bounding_box(false);
        **/
}

void place_objects_randomly(std::string const& model_path,  int32_t num_models_to_place, float random_pos_cube_dimensions,
    std::shared_ptr<gua::node::Node> scene_root_node) {

    std::size_t found = model_path.find_last_of("/\\");
    
    std::string obj_name = model_path.substr(found+1);

    for(int model_index = 0; model_index < num_models_to_place; ++model_index) {
        std::string const random_object_name = obj_name + "_" + std::to_string(model_index);
        gua::TriMeshLoader loader;

        // we provide our model with the gua-default material
        auto model_material(gua::MaterialShaderDatabase::instance()->lookup("gua_default_material")->make_new_material());
        model_material->set_show_back_faces(true);
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