
#include "scene_utils.hpp"

void place_objects_randomly(std::string const& model_path, std::shared_ptr<gua::node::TransformNode>& scene_root_node) {
    for(int i = 0; i < 1000; ++i) {
        std::string const random_object_name = "obj_" + std::to_string(i);
        gua::TriMeshLoader loader;
        auto model_mat(gua::MaterialShaderDatabase::instance()->lookup("gua_default_material")->make_new_material());


        auto new_model(loader.create_geometry_from_file(random_object_name, model_path, model_mat, gua::TriMeshLoader::NORMALIZE_SCALE | gua::TriMeshLoader::MAKE_PICKABLE));
        
        float rand_x_trans = 7.0 * std::rand() / (float)RAND_MAX - 3.5;
        float rand_y_trans = 7.0 * std::rand() / (float)RAND_MAX - 3.5;
        float rand_z_trans = 7.0 * std::rand() / (float)RAND_MAX - 3.5;

        float rand_angle  = 360.0 * std::rand() / (float)RAND_MAX;
        float rand_angle_2 = 360.0 * std::rand() / (float)RAND_MAX;
        float rand_angle_3 = 360.0 * std::rand() / (float)RAND_MAX;
        float rand_x_rot = 360.0 * std::rand() / (float)RAND_MAX;
        float rand_y_rot = 360.0 * std::rand() / (float)RAND_MAX;
        float rand_z_rot = 360.0 * std::rand() / (float)RAND_MAX;

        //new_model->rotate(i, 0.0, 1.0, 0.0);
        //new_model->translate(rand_x_trans, rand_y_trans, rand_z_trans);

        auto norm_scale_mat = new_model->get_transform();

        gua::math::mat4 model_trans =   
                                        gua::math::mat4(scm::math::make_translation(rand_x_trans, rand_y_trans, rand_z_trans)) *
                                        gua::math::mat4(scm::math::make_rotation(rand_angle_3, 0.0f, 0.0f, 1.0f)) *
                                        gua::math::mat4(scm::math::make_rotation(rand_angle_2, 0.0f, 1.0f, 0.0f)) *
                                        gua::math::mat4(scm::math::make_rotation(rand_angle, 1.0f, 0.0f, 0.0f)) * norm_scale_mat;

                                        //std::cout << "Rand angle: " << rand_angle << std::endl;

        new_model->set_transform(model_trans);

        //new_model->set_draw_bounding_box(true);
        scene_root_node->add_child(new_model);
    }
}