#include "glfw_callbacks.hpp" // function declarations
#include "navigation.hpp" //include WASD_state type

extern WASD_state cam_navigation_state;

extern int current_bb_level_to_visualize;

extern bool print_frame_times;

void key_press(gua::PipelineDescription& pipe, gua::SceneGraph& graph, int key, int scancode, int action, int mods)
{

    //std::cout << "scancode: " << scancode << std::endl;
    switch(scancode) {
        
        
        //scancode for up arrow key
        case 111: {
            if(action == 1) {
                cam_navigation_state.cam_translation_speed *= 2.0;
                std::cout << "Set cam translation speed to:" << cam_navigation_state.cam_translation_speed << std::endl;
            }
            break;
        }

        //scancode for down arrow key
        case 116: {
            if(action == 1) {
                cam_navigation_state.cam_translation_speed /= 2.0;
                std::cout << "Set cam translation speed to:" << cam_navigation_state.cam_translation_speed << std::endl;
            }

            break;
        }

        

        default: {
            break;
        }
    }
    //action == 1: key was pressed
    //action == 0: key was released

    switch(std::tolower(key)) {
        case 'w': { //moves the camera forward with respect to the camera vector described in global coordinates
            if(1 == action) {
                cam_navigation_state.moving_forward  = true;
            } else if(0 == action) {
                cam_navigation_state.moving_forward  = false;               
            }
            std::cout << "w" << std::endl;
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
                cam_navigation_state.yaw_neg = true;
            } else if(0 == action) {
                cam_navigation_state.yaw_neg = false;            
            }

            break;
        }

        case 'j': { //moves the camera to the right with respect to the camera vector described in global coordinates
            if(1 == action) {
                cam_navigation_state.yaw_pos = true;
            } else if(0 == action) {
                cam_navigation_state.yaw_pos = false;            
            }

            break;
        }

        case 'i': { //moves the camera to the right with respect to the camera vector described in global coordinates
            if(1 == action) {
                cam_navigation_state.pitch_pos = true;
            } else if(0 == action) {
                cam_navigation_state.pitch_pos = false;            
            }

            break;
        }

        case 'k': { //moves the camera to the right with respect to the camera vector described in global coordinates
            if(1 == action) {
                cam_navigation_state.pitch_neg = true;
            } else if(0 == action) {
                cam_navigation_state.pitch_neg = false;            
            }

            break;
        }
  
        case 'u': { //moves the camera to the right with respect to the camera vector described in global coordinates
            if(1 == action) {
                cam_navigation_state.roll_neg = true;
            } else if(0 == action) {
                cam_navigation_state.roll_neg = false;            
            }

            break;
        }

        case 'o': { //moves the camera to the right with respect to the camera vector described in global coordinates
            if(1 == action) {
                cam_navigation_state.roll_pos = true;
            } else if(0 == action) {
                cam_navigation_state.roll_pos = false;            
            }

            break;
        }

        case 't': { //toggle printing of frame times
            if(1 == action) {
                print_frame_times = !print_frame_times;
            }
            break;
        }

        case 'p': { //moves the camera to the right with respect to the camera vector described in global coordinates
            if(1 == action) {
                std::cout << std::endl;
                std::cout << "'t': {//toggle printing of frame times" << std::endl;
                std::cout << std::endl;
                std::cout << "Keyboard camera controls:" << std::endl;
                std::cout << "=========================" << std::endl;
                std::cout <<"'w': { //moves the camera forward" << std::endl;
                std::cout <<"'a': { //moves the camera to the left" << std::endl;
                std::cout <<"'s': { //moves the camera backward" << std::endl;
                std::cout << "'d': { //moves the camera to the right" << std::endl;
                std::cout << "'q': { //moves the camera upward" << std::endl;
                std::cout << "'e': { //moves the camera downward" << std::endl;
                std::cout << std::endl;
                std::cout << "'i/j/k/l': { //pitch/yaw camera" << std::endl;
                std::cout << "up/down arrow keys: { increase/decrease camera translation speed" << std::endl;  
                std::cout << std::endl;
                std::cout << std::endl;

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