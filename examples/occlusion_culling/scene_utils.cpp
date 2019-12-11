    
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

void create_simple_debug_scene( std::shared_ptr<gua::node::Node> scene_root_node) {
    gua::TriMeshLoader loader_central;

    auto model_material_central(gua::MaterialShaderDatabase::instance()->lookup("gua_default_material")->make_new_material());
    model_material_central->set_show_back_faces(false);
    model_material_central->set_render_wireframe(false);

    auto new_model_central(loader_central.create_geometry_from_file("teapot_1", "/opt/3d_models/hairball/hairball.dae", model_material_central , gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::NORMALIZE_SCALE));
    auto new_model_matrix = new_model_central->get_transform();
    
    auto new_model_central2(loader_central.create_geometry_from_file("teapot_2", "/opt/3d_models/hairball/hairball.dae", model_material_central , gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::NORMALIZE_SCALE));


    auto new_model_central3(loader_central.create_geometry_from_file("teapot_3", "/opt/3d_models/hairball/hairball.dae", model_material_central , gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::NORMALIZE_SCALE));

    
    auto new_model_central4(loader_central.create_geometry_from_file("teapot_4", "/opt/3d_models/hairball/hairball.dae", model_material_central , gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::NORMALIZE_SCALE));


    auto plane(loader_central.create_geometry_from_file("plane", "./data/objects/plane.obj", model_material_central , gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::NORMALIZE_SCALE));
   // auto new_plane_model = new_model_central->get_transform();

    float plane_scaling = 30.0f;

    auto plane_transform =  gua::math::mat4(scm::math::make_translation(0.0f, 0.0f, 0.0f)) * 
                            gua::math::mat4(scm::math::make_rotation(90.0f, 1.0f, 0.0f, 0.0f)) *
                            gua::math::mat4(scm::math::make_scale(plane_scaling, plane_scaling, plane_scaling));

    plane->set_transform(plane_transform);

    gua::math::mat4 model_trans_central;
    
    model_trans_central =   
            gua::math::mat4(scm::math::make_translation(-5.0f, 2.0f, -15.0f)) * 
            gua::math::mat4(scm::math::make_rotation(0.0f, 0.0f, 0.0f, 1.0f)) *
            gua::math::mat4(scm::math::make_scale(10.0f, 10.0f, 10.0f)) *
            new_model_matrix;


    // override the model's transform with our calculated transformation
    new_model_central->set_transform(model_trans_central);
    new_model_central->set_draw_bounding_box(false);

    new_model_central->translate(0.0, 0.0, 0.0);

    new_model_central2->set_transform(model_trans_central);
    new_model_central2->set_draw_bounding_box(false);
    new_model_central2->translate(2.0, 0.0, 0.0);

    new_model_central3->set_transform(model_trans_central);
    new_model_central3->set_draw_bounding_box(false);
    new_model_central3->translate(2.0, 2.0, 0.0);

    new_model_central4->set_transform(model_trans_central);
    new_model_central4->set_draw_bounding_box(false);
    new_model_central4->translate(0.0, 2.0, 0.0);


    scene_root_node->add_child(new_model_central);
    scene_root_node->add_child(new_model_central2);
    scene_root_node->add_child(new_model_central3);
    scene_root_node->add_child(new_model_central4);
    
    scene_root_node->add_child(plane);
}



void transform_trimesh_model(std::shared_ptr<gua::node::Node>& scene_root_node,
                             std::string const& name, 
                             std::string const& path, 
                             gua::TriMeshLoader& loader,
                             bool const is_behind,
                             bool const randomized,
                             float x_trans,
                             float y_trans,
                             float z_trans,
                             float angle_x,
                             float angle_y,
                             float angle_z,
                             float scale)
{
    auto material(gua::MaterialShaderDatabase::instance()->lookup("gua_default_material")->make_new_material());
    material->set_show_back_faces(false);
    material->set_render_wireframe(false);

    auto trimesh_model(
        loader.create_geometry_from_file(name, path, material , 
                                         gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::NORMALIZE_SCALE));
    auto trimesh_model_matrix = trimesh_model->get_transform();

    if (is_behind && randomized)
    {
        
        x_trans = 10.0f * std::rand() / (float)RAND_MAX - 10.0f  / 2.0f;
        y_trans = 10.0f * std::rand() / (float)RAND_MAX - 10.0f  / 2.0f;
        z_trans = 100.0f * std::rand() / (float)RAND_MAX - 120.0f;

/*        angle_x = 360.0f * std::rand() / (float)RAND_MAX;
        angle_y = 360.0f * std::rand() / (float)RAND_MAX;
        angle_z = 360.0f * std::rand() / (float)RAND_MAX;
        scale = ( 20.0f * std::rand() / (float)RAND_MAX ) / 10.0f;
*/
    }


    gua::math::mat4 trimesh_model_trans =   
                                        gua::math::mat4(scm::math::make_translation(x_trans, y_trans, z_trans)) *
                                        gua::math::mat4(scm::math::make_rotation(angle_z, 0.0f, 0.0f, 1.0f)) * 
                                        gua::math::mat4(scm::math::make_rotation(angle_y, 0.0f, 1.0f, 0.0f)) * 
                                        gua::math::mat4(scm::math::make_rotation(angle_x, 1.0f, 0.0f, 0.0f)) *  
                                        gua::math::mat4(scm::math::make_scale(scale, scale, scale)) * 
                                        trimesh_model_matrix;  
    trimesh_model->set_transform(trimesh_model_trans);
    trimesh_model->set_draw_bounding_box(false);
    scene_root_node->add_child(trimesh_model);


}

void create_simple_debug_scene_01(std::shared_ptr<gua::node::Node> scene_root_node) {

    gua::TriMeshLoader trimesh_loader;

    for (unsigned int i = 0; i < 250; ++i)
    {

        transform_trimesh_model(scene_root_node,  
                            "hb" + i, 
                            "/opt/3d_models/hairball/hairball.dae", 
                            trimesh_loader, true, true);
    }

    //transform_trimesh_model(scene_root_node, "plane1", "./data/objects/plane.obj", trimesh_loader, false, false, 0.0f,0,-5.0f,90.0f,0,0,10.0f);
    //transform_trimesh_model(scene_root_node, "plane2", "./data/objects/plane.obj", trimesh_loader, false, false, 0.0f,1.0f,-20.0f,90.0f,0,0,3.0f);
    //ransform_trimesh_model(scene_root_node, "plane3", "./data/objects/plane.obj", trimesh_loader, false, false, 0.0f,-2.0f,-50.0f,90.0f,0,0,4.0f);

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