#include "glfw_callbacks.hpp" // function declarations
#include "navigation.hpp" //include WASD_state type

#include <gua/renderer/OcclusionCullingTriMeshPass.hpp>
#include <gua/renderer/FullscreenColorBufferViewPass.hpp>

extern WASD_state cam_navigation_state;
extern bool print_times;
extern bool show_bounding_boxes;
extern bool was_set_to_show_bounding_boxes;

extern int current_bb_level_to_visualize;

extern bool print_scenegraph_once;

extern bool update_camera;

extern std::shared_ptr<gua::PipelineDescription> occlusion_culling_pipeline_description;

uint64_t const max_occlusion_culling_fragment_treshold = 1000000;

uint64_t num_occlusion_culling_fragment_threshold = 100;


#define OC_TRIMESH   

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

    //std::cout << "scancode: " << scancode << std::endl;
    switch(scancode) {
        
        // NUMPAD 8
        case 80: {
            if(action == 1) {
                auto oc_tri_mesh_pass = occlusion_culling_pipeline_description->get_occlusion_culling_tri_mesh_pass();
                uint64_t current_occlusion_threshold = oc_tri_mesh_pass->get_occlusion_culling_fragment_threshold();

                uint64_t new_num_occlusion_threshold = std::min(max_occlusion_culling_fragment_treshold, current_occlusion_threshold * 2);

                oc_tri_mesh_pass->set_occlusion_culling_fragment_threshold(new_num_occlusion_threshold);

                std::cout << "Set Approximate Culling Fragment Threshold to: " << new_num_occlusion_threshold << std::endl; 

                oc_tri_mesh_pass->touch();
            }
            break;
        }

        // NUMPAD 5
        case 84: {
            if(action == 1) {
                auto oc_tri_mesh_pass = occlusion_culling_pipeline_description->get_occlusion_culling_tri_mesh_pass();
                uint64_t current_occlusion_threshold = oc_tri_mesh_pass->get_occlusion_culling_fragment_threshold();

                uint64_t new_num_occlusion_threshold = std::max(uint64_t(1), current_occlusion_threshold / 2);

                oc_tri_mesh_pass->set_occlusion_culling_fragment_threshold(new_num_occlusion_threshold);

                std::cout << "Set Approximate Culling Fragment Threshold to: " << new_num_occlusion_threshold << std::endl; 

                oc_tri_mesh_pass->touch();
            }
            break;
        }

        // NUMPAD 1
        case 87: {
            if(action == 1) {
                auto oc_tri_mesh_pass = occlusion_culling_pipeline_description->get_occlusion_culling_tri_mesh_pass();
                bool was_enabled = oc_tri_mesh_pass->get_enable_coarse_depth_sorting();

                bool will_be_enabled = !was_enabled;
                oc_tri_mesh_pass->set_enable_coarse_depth_sorting(will_be_enabled);

                std::cout << "Enable depth sorting: " << will_be_enabled << std::endl; 

                oc_tri_mesh_pass->touch();
            }
            break;
        }

        //scancode for 1 key
        case 10: {
            if(action == 1) {

#ifdef OC_TRIMESH
                occlusion_culling_pipeline_description->get_occlusion_culling_tri_mesh_pass()->set_occlusion_culling_strategy(gua::OcclusionCullingStrategy::No_Culling);

                std::cout << "Set Occlusion_Culling_Strategy to 'No Culling'" << std::endl;
                //calling touch is necessary for guacamole to notice that the pass has changed
                occlusion_culling_pipeline_description->get_occlusion_culling_tri_mesh_pass()->touch();
#else

                occlusion_culling_pipeline_description->get_tri_mesh_pass()->set_occlusion_culling_strategy(gua::OcclusionCullingStrategy::No_Culling);

                std::cout << "Set Occlusion_Culling_Strategy to 'No Culling'" << std::endl;
                //calling touch is necessary for guacamole to notice that the pass has changed
                occlusion_culling_pipeline_description->get_tri_mesh_pass()->touch();
#endif
            }
            break;
        }
        //scancode for 2 key
        case 11: {
            if(action == 1) {
#ifdef OC_TRIMESH
                occlusion_culling_pipeline_description->get_occlusion_culling_tri_mesh_pass()->set_occlusion_culling_strategy(gua::OcclusionCullingStrategy::Naive_Stop_And_Wait);


                std::cout << "Set Occlusion_Culling_Strategy to 'Naive Stop and Wait'" << std::endl;
                //calling touch is necessary for guacamole to notice that the pass has changed
                occlusion_culling_pipeline_description->get_occlusion_culling_tri_mesh_pass()->touch();
#else

                occlusion_culling_pipeline_description->get_tri_mesh_pass()->set_occlusion_culling_strategy(gua::OcclusionCullingStrategy::Coherent_Hierarchical_Culling_PlusPlus);

                std::cout << "Set Occlusion_Culling_Strategy to 'CHC++'" << std::endl;
                //calling touch is necessary for guacamole to notice that the pass has changed
                occlusion_culling_pipeline_description->get_tri_mesh_pass()->touch();
#endif
            }
            break;
        }

        //scancode for 3 key
        case 12: {
            if(action == 1) {
                occlusion_culling_pipeline_description->get_occlusion_culling_tri_mesh_pass()->set_occlusion_culling_strategy(gua::OcclusionCullingStrategy::Hierarchical_Stop_And_Wait);


                std::cout << "Set Occlusion_Culling_Strategy to 'Hierarchical Stop and Wait'" << std::endl;
                //calling touch is necessary for guacamole to notice that the pass has changed
                occlusion_culling_pipeline_description->get_occlusion_culling_tri_mesh_pass()->touch();
            }
            break;
        }

        //scancode for 4 key
        case 13: {
            if(action == 1) {
                occlusion_culling_pipeline_description->get_occlusion_culling_tri_mesh_pass()->set_occlusion_culling_strategy(gua::OcclusionCullingStrategy::Coherent_Hierarchical_Culling);

                std::cout << "Set Occlusion_Culling_Strategy to 'Coherent Hierarchical Culling'" << std::endl;
                //calling touch is necessary for guacamole to notice that the pass has changed
                occlusion_culling_pipeline_description->get_occlusion_culling_tri_mesh_pass()->touch();
            }
            break;
        }

        //scancode for 5 key
        case 14: {
            if(action == 1) {
                occlusion_culling_pipeline_description->get_occlusion_culling_tri_mesh_pass()->set_occlusion_culling_strategy(gua::OcclusionCullingStrategy::Coherent_Hierarchical_Culling_PlusPlus);

                std::cout << "Set Occlusion_Culling_Strategy to 'CHC++'" << std::endl;
                //calling touch is necessary for guacamole to notice that the pass has changed
                occlusion_culling_pipeline_description->get_occlusion_culling_tri_mesh_pass()->touch();
            }
            break;
        }

        //scancode for "enter"
        case 36: {
            if(action == 1) {

                auto oc_tri_mesh_pass = occlusion_culling_pipeline_description->get_occlusion_culling_tri_mesh_pass();
                gua::OcclusionQueryType current_oc_query_type = oc_tri_mesh_pass->get_occlusion_query_type();
                
                if(gua::OcclusionQueryType::Any_Samples_Passed == current_oc_query_type) {
                    oc_tri_mesh_pass->set_occlusion_query_type(gua::OcclusionQueryType::Number_Of_Samples_Passed);
                    std::cout << "Set Occlusion Query Type to: Number_Of_Samples_Passed (Approximate)" << std::endl; 

                } else if (gua::OcclusionQueryType::Number_Of_Samples_Passed == current_oc_query_type) {
                    oc_tri_mesh_pass->set_occlusion_query_type(gua::OcclusionQueryType::Any_Samples_Passed);
                    std::cout << "Set Occlusion Query Type to: Any_Samples_Passed (Conservative)" << std::endl; 


                }

                oc_tri_mesh_pass->touch();
            }
            break;
        }





        //scancode for up arrow key
        case 111: {
            if(action == 1) {

                if(show_bounding_boxes) {
                    current_bb_level_to_visualize = std::max(-1, current_bb_level_to_visualize - 1); 
                    was_set_to_show_bounding_boxes = !show_bounding_boxes;
                    std::cout << "Switching BB-Vis level to: " << current_bb_level_to_visualize << std::endl;
                }
            }
            break;
        }

        //scancode for down arrow key
        case 116: {
            if(action == 1) {
                if(show_bounding_boxes) {
                    current_bb_level_to_visualize = current_bb_level_to_visualize + 1; 
                    was_set_to_show_bounding_boxes = !show_bounding_boxes;

                    std::cout << "Switching BB-Vis level to: " << current_bb_level_to_visualize << std::endl;
                }
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

        case 't': {
            //toggle print state on keypress
            if(1 == action) {
                print_times = !print_times;
            }
            break;
        }

        case 'p': {
            //toggle print state on keypress
            if(1 == action) {
                print_scenegraph_once = true;
            }
            break;
        }

        case 'x': {
            //toggle print state on keypress
            if(1 == action) {
                update_camera = !update_camera;
            }
            break;
        }

        case 'v': {
            //toggle print state on keypress
            if(1 == action) {
                // get occlusion culling tri mesh pass and toggle rendering mode
                

#ifdef OC_TRIMESH       
                auto& occlusion_culling_tri_mesh_pass = occlusion_culling_pipeline_description->get_occlusion_culling_tri_mesh_pass();
                bool current_dcv_status = occlusion_culling_tri_mesh_pass->get_enable_depth_complexity_vis();
                bool new_dcv_status = !current_dcv_status;
                occlusion_culling_tri_mesh_pass->set_enable_depth_complexity_vis(new_dcv_status);

                if(true == new_dcv_status) {
                    occlusion_culling_tri_mesh_pass->set_enable_culling_geometry_vis(false);
                }


                // get light visibility pass and toggle pass on and off
                auto& light_visibility_pass = occlusion_culling_pipeline_description->get_light_visibility_pass();
                bool is_light_visibility_pass_enabled = light_visibility_pass->is_enabled();
                light_visibility_pass->enable(!is_light_visibility_pass_enabled);

                // get resolve pass and toggle pass on and off
                auto& resolve_pass = occlusion_culling_pipeline_description->get_resolve_pass();
                bool is_resolve_pass_enabled = resolve_pass->is_enabled();
                resolve_pass->enable(!is_resolve_pass_enabled);

                // get fullscreen color visibility pass and toggle pass on and off
                auto& fullscreen_color_buffer_view_pass = occlusion_culling_pipeline_description->get_full_screen_color_buffer_view_pass();
                bool is_fullscreen_color_view_enabled = fullscreen_color_buffer_view_pass->is_enabled();
                fullscreen_color_buffer_view_pass->enable(!is_fullscreen_color_view_enabled);
                occlusion_culling_tri_mesh_pass->touch();
                

#else
                //for the occlusion culling aware renderer --> changing to tri mesh pass?? but should be for all passes.
                /***************************************************************************************************************************/
                auto& tri_mesh_pass = occlusion_culling_pipeline_description->get_tri_mesh_pass();
                bool current_dcv_status = tri_mesh_pass->get_enable_depth_complexity_vis();
                bool new_dcv_status = !current_dcv_status;
                tri_mesh_pass->set_enable_depth_complexity_vis(new_dcv_status);

                if(true == new_dcv_status) {
                    tri_mesh_pass->set_enable_culling_geometry_vis(false);
                }


                // get light visibility pass and toggle pass on and off
                auto& light_visibility_pass = occlusion_culling_pipeline_description->get_light_visibility_pass();
                bool is_light_visibility_pass_enabled = light_visibility_pass->is_enabled();
                light_visibility_pass->enable(!is_light_visibility_pass_enabled);

                // get resolve pass and toggle pass on and off
                auto& resolve_pass = occlusion_culling_pipeline_description->get_resolve_pass();
                bool is_resolve_pass_enabled = resolve_pass->is_enabled();
                resolve_pass->enable(!is_resolve_pass_enabled);

                // get fullscreen color visibility pass and toggle pass on and off
                auto& fullscreen_color_buffer_view_pass = occlusion_culling_pipeline_description->get_full_screen_color_buffer_view_pass();
                bool is_fullscreen_color_view_enabled = fullscreen_color_buffer_view_pass->is_enabled();
                fullscreen_color_buffer_view_pass->enable(!is_fullscreen_color_view_enabled);
                tri_mesh_pass->touch();
#endif


            }
            break;
        }


        case 'n': {
            //toggle print state on keypress
            if(1 == action) {

#ifdef OC_TRIMESH   
                // get occlusion culling tri mesh pass and toggle occlusion culling vis mode
                auto& occlusion_culling_tri_mesh_pass = occlusion_culling_pipeline_description->get_occlusion_culling_tri_mesh_pass();
                bool current_occlusion_culling_geometry_vis_status = occlusion_culling_tri_mesh_pass->get_enable_culling_geometry_vis();
                occlusion_culling_tri_mesh_pass->set_enable_culling_geometry_vis(!current_occlusion_culling_geometry_vis_status);

                occlusion_culling_tri_mesh_pass->touch();

#else

                //For Occlusion Culling Aware Renderer
                /***********************************************************/
                auto& tri_mesh_pass = occlusion_culling_pipeline_description->get_tri_mesh_pass();
                bool current_occlusion_culling_geometry_vis_status_tri_mesh = tri_mesh_pass->get_enable_culling_geometry_vis();
                tri_mesh_pass->set_enable_culling_geometry_vis(!current_occlusion_culling_geometry_vis_status_tri_mesh);

                tri_mesh_pass->touch();
#endif
            }
            break;
        }

        case 'b': {
            //toggle print state on keypress
            if(1 == action) {
                show_bounding_boxes = !show_bounding_boxes;
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