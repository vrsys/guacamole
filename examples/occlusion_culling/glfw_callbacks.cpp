#include "glfw_callbacks.hpp" // function declarations
#include "navigation.hpp" //include WASD_state type

extern WASD_state cam_navigation_state;
extern bool print_times;
extern bool visualize_depth_complexity;

// forward mouse interaction to trackball
void mouse_button(gua::utils::Trackball& trackball, int mousebutton, int action, int mods)
{
    gua::utils::Trackball::button_type button;
    gua::utils::Trackball::state_type state;

    switch(mousebutton)
    {
    case 0:
        button = gua::utils::Trackball::left;
        break;
    case 2:
        button = gua::utils::Trackball::middle;
        break;
    case 1:
        button = gua::utils::Trackball::right;
        break;
    };

    switch(action)
    {
    case 0:
        state = gua::utils::Trackball::released;
        break;
    case 1:
        state = gua::utils::Trackball::pressed;
        break;
    };

    trackball.mouse(button, state, trackball.posx(), trackball.posy());
}




void key_press(gua::PipelineDescription& pipe, gua::SceneGraph& graph, int key, int scancode, int action, int mods)
{

    //action == 1: key was pressed
    //action == 0: key was released

    switch(std::tolower(key)) {
        case 'w': { //moves the camera forward with respect to the camera vector described in global coordinates
            if(1 == action) {
                cam_navigation_state.moving_forward  = true;
            } else if(0 == action) {
                cam_navigation_state.moving_forward  = false;               
            }

            break;
        }

        case 'a': { //moves the camera to the left with respect to the camera vector described in global coordinates
            if(1 == action) {
                cam_navigation_state.moving_left = true;
            } else if(0 == action) {
                cam_navigation_state.moving_left = false;          
            }

            break;
        }

        case 's': { //moves the camera backward with respect to the camera vector described in global coordinates
            if(1 == action) {
                cam_navigation_state.moving_backward = true;
            } else if(0 == action) {
                cam_navigation_state.moving_backward = false;            
            }

            break;
        }

        case 'd': { //moves the camera to the right with respect to the camera vector described in global coordinates
            if(1 == action) {
                cam_navigation_state.moving_right = true;
            } else if(0 == action) {
                cam_navigation_state.moving_right = false;            
            }

            break;
        }

        case 'q': { //moves the camera upward with respect to the camera vector described in global coordinates
            if(1 == action) {
                cam_navigation_state.moving_upward = true;
            } else if(0 == action) {
                cam_navigation_state.moving_upward = false;            
            }

            break;
        }

        case 'e': { //moves the camera downward with respect to the camera vector described in global coordinates
            if(1 == action) {
                cam_navigation_state.moving_downward = true;
            } else if(0 == action) {
                cam_navigation_state.moving_downward = false;            
            }

            break;
        }

        case 'l': { //moves the camera to the right with respect to the camera vector described in global coordinates
            if(1 == action) {
                cam_navigation_state.rotate_around_y_neg = true;
            } else if(0 == action) {
                cam_navigation_state.rotate_around_y_neg = false;            
            }

            break;
        }

        case 'j': { //moves the camera to the right with respect to the camera vector described in global coordinates
            if(1 == action) {
                cam_navigation_state.rotate_around_y_pos = true;
            } else if(0 == action) {
                cam_navigation_state.rotate_around_y_pos = false;            
            }

            break;
        }

        case 'i': { //moves the camera to the right with respect to the camera vector described in global coordinates
            if(1 == action) {
                cam_navigation_state.rotate_around_x_pos = true;
            } else if(0 == action) {
                cam_navigation_state.rotate_around_x_pos = false;            
            }

            break;
        }

        case 'k': { //moves the camera to the right with respect to the camera vector described in global coordinates
            if(1 == action) {
                cam_navigation_state.rotate_around_x_neg = true;
            } else if(0 == action) {
                cam_navigation_state.rotate_around_x_neg = false;            
            }

            break;
        }

        case 'p': {
            //toggle print state on keypress
            if(1 == action) {
                print_times = !print_times;
            }
            break;
        }

        case 'v': {
            //toggle print state on keypress
            if(1 == action) {
                visualize_depth_complexity = !visualize_depth_complexity;
            }
            break;
        }

        

        default: { //no assigned key
            break;
        }
    }

    if(action == 0)
        return;

}