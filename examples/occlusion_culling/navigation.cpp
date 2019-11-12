#include "navigation.hpp"

// actual WASD_state which is also referred to in main.cpp
WASD_state cam_navigation_state;


// the crucial information for navigation only with respect to the camera is to know the camera coordinate frame. In this example, we extract the three
// axes from the columns of the camera node in world space (because that tells us how the axes are oriented in world)
void update_cam_matrix(std::shared_ptr<gua::node::CameraNode> const& cam_node, std::shared_ptr<gua::node::TransformNode>& nav_node, float elapsed_milliseconds) {
    //get the camera matrix after it is transformed in world, a.k.a. all matrices of the nodes above have been applied to it
    auto cam_world_matrix = cam_node->get_world_transform();

    if(cam_navigation_state.moving_forward ) {
    	// the third column corresponds to the forward vector
        auto forward_cam_vector = -cam_world_matrix.column(2);
        //frame rate independent movement is achieved by multiplying our offset with the elapsed time. If the application is twice as slow, we move twice as much
        auto delta = forward_cam_vector * cam_navigation_state.cam_translation_speed * elapsed_milliseconds;

        // we separate the translational and the rotational components in our example. W,A,S,D,Q and E will add their translation to a relative translation matrix
        cam_navigation_state.accumulated_translation_world_space = scm::math::mat4f(scm::math::make_translation( delta[0], delta[1], delta[2])) * cam_navigation_state.accumulated_translation_world_space;
    }

    // see comments above, just backwards
    if(cam_navigation_state.moving_backward ) {
        auto backward_cam_vector = cam_world_matrix.column(2);
        auto delta = backward_cam_vector * cam_navigation_state.cam_translation_speed * elapsed_milliseconds;;

        cam_navigation_state.accumulated_translation_world_space = scm::math::mat4f(scm::math::make_translation( delta[0], delta[1], delta[2])) * cam_navigation_state.accumulated_translation_world_space;
    }

    // see comments above, just left
    if(cam_navigation_state.moving_left) {
        auto left_cam_vector = -cam_world_matrix.column(0);
        auto delta = left_cam_vector * cam_navigation_state.cam_translation_speed * elapsed_milliseconds;;

        cam_navigation_state.accumulated_translation_world_space = scm::math::mat4f(scm::math::make_translation( delta[0], delta[1], delta[2])) * cam_navigation_state.accumulated_translation_world_space;
    }

    // see comments above, just right
    if(cam_navigation_state.moving_right) {
        auto right_cam_vector = cam_world_matrix.column(0);
        auto delta = right_cam_vector * cam_navigation_state.cam_translation_speed * elapsed_milliseconds;;

        cam_navigation_state.accumulated_translation_world_space = scm::math::mat4f(scm::math::make_translation( delta[0], delta[1], delta[2])) * cam_navigation_state.accumulated_translation_world_space;
    }

    // see comments above, just upward
    if(cam_navigation_state.moving_upward) {
        auto up_cam_vector = cam_world_matrix.column(1);
        auto delta = up_cam_vector * cam_navigation_state.cam_translation_speed * elapsed_milliseconds;;

        cam_navigation_state.accumulated_translation_world_space = scm::math::mat4f(scm::math::make_translation( delta[0], delta[1], delta[2])) * cam_navigation_state.accumulated_translation_world_space;
    }

    // see comments above, just downward
    if(cam_navigation_state.moving_downward) {
        auto down_cam_vector = -cam_world_matrix.column(1);
        auto delta = down_cam_vector * cam_navigation_state.cam_translation_speed * elapsed_milliseconds;;

        cam_navigation_state.accumulated_translation_world_space = scm::math::mat4f(scm::math::make_translation( delta[0], delta[1], delta[2])) * cam_navigation_state.accumulated_translation_world_space;
    }


    // here we rotate around the y-Axis
    if(cam_navigation_state.rotate_around_y_pos) {

        auto up_axis = cam_world_matrix.column(1);
        auto rot_angle = cam_navigation_state.cam_rotation_speed * elapsed_milliseconds;

        //rotation takes 4 arguments: the amount of rotation in degree, and 3 scalar components corresponding to an axis of rotation
        cam_navigation_state.accumulated_rotation_around_y_world_space = scm::math::mat4f(scm::math::make_rotation(rot_angle, up_axis[0], up_axis[1], up_axis[2])) * cam_navigation_state.accumulated_rotation_around_y_world_space;
    }
    
    // same as before but negative
    if(cam_navigation_state.rotate_around_y_neg) {

        auto up_axis = cam_world_matrix.column(1);
        auto rot_angle = -cam_navigation_state.cam_rotation_speed * elapsed_milliseconds;

        cam_navigation_state.accumulated_rotation_around_y_world_space = scm::math::mat4f(scm::math::make_rotation(rot_angle, up_axis[0], up_axis[1], up_axis[2])) * cam_navigation_state.accumulated_rotation_around_y_world_space;
    }

    // here we rotate around the x-Axis
    if(cam_navigation_state.rotate_around_x_pos) {

        auto right_axis = cam_world_matrix.column(0);
        auto rot_angle = cam_navigation_state.cam_rotation_speed * elapsed_milliseconds;

        cam_navigation_state.accumulated_rotation_around_y_world_space = scm::math::mat4f(scm::math::make_rotation(rot_angle, right_axis[0], right_axis[1], right_axis[2])) * cam_navigation_state.accumulated_rotation_around_y_world_space;
    }
    
     // same as before but negative
    if(cam_navigation_state.rotate_around_x_neg) {

        auto right_axis = cam_world_matrix.column(0);

        //nav
        //nav_node->rotate(50.0f * elapsed_milliseconds, up_axis);

        auto rot_angle = -cam_navigation_state.cam_rotation_speed * elapsed_milliseconds;

        cam_navigation_state.accumulated_rotation_around_y_world_space = scm::math::mat4f(scm::math::make_rotation(rot_angle, right_axis[0], right_axis[1], right_axis[2])) * cam_navigation_state.accumulated_rotation_around_y_world_space;
        //std::cout << "Cam forward vector: " << forward_cam_vector << std::endl;   
    }


    ///////
    // At this point we actually update our navigation node (!) based on the parsed events. Since we put the navigation node directly under root, it is
    // correct to call set_transform (which updates the local transform) with the information accumulated in world space. If navigation would have been
    // located deeper in the hierarchy, we would have needed to transform our deltas into the reference frame of the navigation node explicitly.
    ///////
    nav_node->set_transform( gua::math::mat4( cam_navigation_state.accumulated_translation_world_space * cam_navigation_state.accumulated_rotation_around_y_world_space) );

}