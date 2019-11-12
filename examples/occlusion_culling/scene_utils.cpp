
#include "scene_utils.hpp"



void print_draw_times(gua::Renderer const& renderer, std::shared_ptr<gua::GlfwWindow> const& window) {

    float application_fps = renderer.get_application_fps();
    float rendering_fps   = window->get_rendering_fps();

    float elapsed_application_time_milliseconds = 0.0;

    if(application_fps > 0.0f) {
        elapsed_application_time_milliseconds = 1.0 / application_fps;
    }

    std::cout << "elapsed application time ms: " << elapsed_application_time_milliseconds << " ms" << std::endl;

    float elapsed_rendering_time_milliseconds = 0.0;

    if(rendering_fps > 0.0f) {
        elapsed_rendering_time_milliseconds = 1.0 / rendering_fps;
    }

    std::cout << "elapsed rendering time ms: " << elapsed_rendering_time_milliseconds << " ms" << std::endl;

    std::cout << std::endl;
}


void place_objects_randomly(std::string const& model_path,  int32_t num_models_to_place, float random_pos_cube_dimensions, std::shared_ptr<gua::node::TransformNode>& scene_root_node) {

    for(int model_index = 0; model_index < num_models_to_place; ++model_index) {
        std::string const random_object_name = "obj_" + std::to_string(model_index);
        gua::TriMeshLoader loader;
        auto model_matterial(gua::MaterialShaderDatabase::instance()->lookup("gua_default_material")->make_new_material());


        auto new_model(loader.create_geometry_from_file(random_object_name, model_path, model_matterial, gua::TriMeshLoader::NORMALIZE_SCALE | gua::TriMeshLoader::MAKE_PICKABLE));
        

        float rand_x_trans = random_pos_cube_dimensions * std::rand() / (float)RAND_MAX - random_pos_cube_dimensions / 2.0f;
        float rand_y_trans = random_pos_cube_dimensions * std::rand() / (float)RAND_MAX - random_pos_cube_dimensions / 2.0f;
        float rand_z_trans = random_pos_cube_dimensions * std::rand() / (float)RAND_MAX - random_pos_cube_dimensions / 2.0f;

        float rand_angle_x = 360.0f * std::rand() / (float)RAND_MAX;
        float rand_angle_y = 360.0f * std::rand() / (float)RAND_MAX;
        float rand_angle_z = 360.0f * std::rand() / (float)RAND_MAX;

        // we want to have controle over the scaleing for now, so we get the matrix that was used to create the normalizatin in scaling 
        auto norm_scale_mat = new_model->get_transform();

        gua::math::mat4 model_trans =   
                                        gua::math::mat4(scm::math::make_translation(rand_x_trans, rand_y_trans, rand_z_trans)) * // 5. we apply the random translation
                                        gua::math::mat4(scm::math::make_rotation(rand_angle_z, 0.0f, 0.0f, 1.0f)) * // 4. we rotate the model around x
                                        gua::math::mat4(scm::math::make_rotation(rand_angle_y, 0.0f, 1.0f, 0.0f)) * // 3. we rotate the model around y
                                        gua::math::mat4(scm::math::make_rotation(rand_angle_x, 1.0f, 0.0f, 0.0f))   // 2. we rotate the model around x
                                        * norm_scale_mat;                                                           // 1. we scale the model such that the longest size is unit size


        // override the model's transform with our calculated transformation
        new_model->set_transform(model_trans);

        //new_model->set_draw_bounding_box(true);

        //add this below our reference group node
        scene_root_node->add_child(new_model);
    }
}



void rebuild_object_hierarchy_SAH_based(std::shared_ptr<gua::node::TransformNode>& scene_root_node, 
                                        std::vector<std::shared_ptr<gua::node::TransformNode>>& geometry_nodes) {

}