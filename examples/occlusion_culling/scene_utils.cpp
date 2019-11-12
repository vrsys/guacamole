
#include "scene_utils.hpp"



void place_objects_randomly(std::string const& model_path,  int32_t num_models_to_place, std::shared_ptr<gua::node::TransformNode>& scene_root_node) {
    
    for(int model_index = 0; model_index < num_models_to_place; ++model_index) {
        std::string const random_object_name = "obj_" + std::to_string(model_index);
        gua::TriMeshLoader loader;
        auto model_mat(gua::MaterialShaderDatabase::instance()->lookup("gua_default_material")->make_new_material());


        auto new_model(loader.create_geometry_from_file(random_object_name, model_path, model_mat, gua::TriMeshLoader::NORMALIZE_SCALE | gua::TriMeshLoader::MAKE_PICKABLE));
        
        float rand_x_trans = 8.0 * std::rand() / (float)RAND_MAX - 4.0;
        float rand_y_trans = 8.0 * std::rand() / (float)RAND_MAX - 4.0;
        float rand_z_trans = 8.0 * std::rand() / (float)RAND_MAX - 4.0;

        float rand_angle_1 = 360.0 * std::rand() / (float)RAND_MAX;
        float rand_angle_2 = 360.0 * std::rand() / (float)RAND_MAX;
        float rand_angle_3 = 360.0 * std::rand() / (float)RAND_MAX;

        auto norm_scale_mat = new_model->get_transform();

        gua::math::mat4 model_trans =   
                                        gua::math::mat4(scm::math::make_translation(rand_x_trans, rand_y_trans, rand_z_trans)) *
                                        gua::math::mat4(scm::math::make_rotation(rand_angle_3, 0.0f, 0.0f, 1.0f)) *
                                        gua::math::mat4(scm::math::make_rotation(rand_angle_2, 0.0f, 1.0f, 0.0f)) *
                                        gua::math::mat4(scm::math::make_rotation(rand_angle_1, 1.0f, 0.0f, 0.0f)) * norm_scale_mat;

                                        //std::cout << "Rand angle: " << rand_angle << std::endl;

        new_model->set_transform(model_trans);

        //new_model->set_draw_bounding_box(true);
        scene_root_node->add_child(new_model);
    }
}



void rebuild_object_hierarchy_SAH_based(std::shared_ptr<gua::node::TransformNode>& scene_root_node, 
                                        std::vector<std::shared_ptr<gua::node::TransformNode>>& geometry_nodes) {

}