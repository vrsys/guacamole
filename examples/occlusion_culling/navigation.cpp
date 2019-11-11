#include "navigation.hpp"

// actual WASD_state which is also referred to in main.cpp
WASD_state cam_navigation_state;

void update_cam_matrix(std::shared_ptr<gua::node::CameraNode> const& cam_node, std::shared_ptr<gua::node::TransformNode>& nav_node, float elapsed_milliseconds) {
    auto cam_world_matrix = cam_node->get_world_transform();

    if(cam_navigation_state.moving_forward ) {
        auto forward_cam_vector = -cam_world_matrix.column(2);

        auto delta = forward_cam_vector * cam_navigation_state.cam_translation_speed * elapsed_milliseconds;

        cam_navigation_state.accumulated_translation_world_space = scm::math::mat4f(scm::math::make_translation( delta[0], delta[1], delta[2])) * cam_navigation_state.accumulated_translation_world_space;
    }

    if(cam_navigation_state.moving_backward ) {
        auto backward_cam_vector = cam_world_matrix.column(2);

        auto delta = backward_cam_vector * cam_navigation_state.cam_translation_speed * elapsed_milliseconds;;

        cam_navigation_state.accumulated_translation_world_space = scm::math::mat4f(scm::math::make_translation( delta[0], delta[1], delta[2])) * cam_navigation_state.accumulated_translation_world_space;
    }

    if(cam_navigation_state.moving_left) {
        auto left_cam_vector = -cam_world_matrix.column(0);

        auto delta = left_cam_vector * cam_navigation_state.cam_translation_speed * elapsed_milliseconds;;

        cam_navigation_state.accumulated_translation_world_space = scm::math::mat4f(scm::math::make_translation( delta[0], delta[1], delta[2])) * cam_navigation_state.accumulated_translation_world_space;
    }

    if(cam_navigation_state.moving_right) {
        auto right_cam_vector = cam_world_matrix.column(0);

        auto delta = right_cam_vector * cam_navigation_state.cam_translation_speed * elapsed_milliseconds;;

        cam_navigation_state.accumulated_translation_world_space = scm::math::mat4f(scm::math::make_translation( delta[0], delta[1], delta[2])) * cam_navigation_state.accumulated_translation_world_space;
    }


    if(cam_navigation_state.moving_upward) {
        auto up_cam_vector = cam_world_matrix.column(1);

        auto delta = up_cam_vector * cam_navigation_state.cam_translation_speed * elapsed_milliseconds;;

        cam_navigation_state.accumulated_translation_world_space = scm::math::mat4f(scm::math::make_translation( delta[0], delta[1], delta[2])) * cam_navigation_state.accumulated_translation_world_space;
    }

    if(cam_navigation_state.moving_downward) {
        auto down_cam_vector = -cam_world_matrix.column(1);

        auto delta = down_cam_vector * cam_navigation_state.cam_translation_speed * elapsed_milliseconds;;

        cam_navigation_state.accumulated_translation_world_space = scm::math::mat4f(scm::math::make_translation( delta[0], delta[1], delta[2])) * cam_navigation_state.accumulated_translation_world_space;
    }


    if(cam_navigation_state.rotate_around_y_pos) {

        auto up_axis = cam_world_matrix.column(1);

        //nav
        //nav_node->rotate(50.0f * elapsed_milliseconds, up_axis);

        auto rot_angle = cam_navigation_state.cam_rotation_speed * elapsed_milliseconds;

        cam_navigation_state.accumulated_rotation_around_y_world_space = scm::math::mat4f(scm::math::make_rotation(rot_angle, up_axis[0], up_axis[1], up_axis[2])) * cam_navigation_state.accumulated_rotation_around_y_world_space;
        //std::cout << "Cam forward vector: " << forward_cam_vector << std::endl;   
    }
    
    if(cam_navigation_state.rotate_around_y_neg) {

        auto up_axis = cam_world_matrix.column(1);

        //nav
        //nav_node->rotate(50.0f * elapsed_milliseconds, up_axis);

        auto rot_angle = -cam_navigation_state.cam_rotation_speed * elapsed_milliseconds;

        cam_navigation_state.accumulated_rotation_around_y_world_space = scm::math::mat4f(scm::math::make_rotation(rot_angle, up_axis[0], up_axis[1], up_axis[2])) * cam_navigation_state.accumulated_rotation_around_y_world_space;
        //std::cout << "Cam forward vector: " << forward_cam_vector << std::endl;   
    }

    if(cam_navigation_state.rotate_around_x_pos) {

        auto right_axis = cam_world_matrix.column(0);

        //nav
        //nav_node->rotate(50.0f * elapsed_milliseconds, up_axis);

        auto rot_angle = cam_navigation_state.cam_rotation_speed * elapsed_milliseconds;

        cam_navigation_state.accumulated_rotation_around_y_world_space = scm::math::mat4f(scm::math::make_rotation(rot_angle, right_axis[0], right_axis[1], right_axis[2])) * cam_navigation_state.accumulated_rotation_around_y_world_space;
        //std::cout << "Cam forward vector: " << forward_cam_vector << std::endl;   
    }
    
    if(cam_navigation_state.rotate_around_x_neg) {

        auto right_axis = cam_world_matrix.column(0);

        //nav
        //nav_node->rotate(50.0f * elapsed_milliseconds, up_axis);

        auto rot_angle = -cam_navigation_state.cam_rotation_speed * elapsed_milliseconds;

        cam_navigation_state.accumulated_rotation_around_y_world_space = scm::math::mat4f(scm::math::make_rotation(rot_angle, right_axis[0], right_axis[1], right_axis[2])) * cam_navigation_state.accumulated_rotation_around_y_world_space;
        //std::cout << "Cam forward vector: " << forward_cam_vector << std::endl;   
    }

    nav_node->set_transform( gua::math::mat4( cam_navigation_state.accumulated_translation_world_space * cam_navigation_state.accumulated_rotation_around_y_world_space) );

}