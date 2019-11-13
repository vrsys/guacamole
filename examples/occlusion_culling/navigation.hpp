#ifndef GUA_OCCLUSION_CULLING_APP_NAVIGATION_HPP
#define GUA_OCCLUSION_CULLING_APP_NAVIGATION_HPP

#include <gua/guacamole.hpp>

// struct to model camera movement state
struct WASD_state {
    bool moving_forward = false;
    bool moving_left = false;
    bool moving_backward = false;
    bool moving_right = false;
    bool moving_upward = false;
    bool moving_downward = false;

    bool rotate_around_y_pos = false;
    bool rotate_around_y_neg = false;

    bool rotate_around_x_pos = false;
    bool rotate_around_x_neg = false;

    scm::math::mat4 accumulated_translation_world_space = scm::math::mat4::identity();

    scm::math::mat4 accumulated_rotation_world_space = scm::math::mat4::identity();
    //scm::math::vec3f accumulated_translation = scm::math::vec3(0.0, 0.0, 0.0);
    double accumulated_rotation_around_up = 0.0;
    double accumulated_rotation_around_right = 0.0;

    double cam_translation_speed = 5.0;
    double cam_rotation_speed = 50.0;
};



// updates camera matrix according to current movement state
void update_cam_matrix(std::shared_ptr<gua::node::CameraNode> const& cam_node, 
					   std::shared_ptr<gua::node::TransformNode>& nav_node, float elapsed_milliseconds);


#endif //GUA_OCCLUSION_CULLING_APP_NAVIGATION_HPP