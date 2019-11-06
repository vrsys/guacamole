/******************************************************************************
 * guacamole - delicious VR                                                   *
 *                                                                            *
 * Copyright: (c) 2011-2013 Bauhaus-Universität Weimar                        *
 * Contact:   felix.lauer@uni-weimar.de / simon.schneegans@uni-weimar.de      *
 *                                                                            *
 * This program is free software: you can redistribute it and/or modify it    *
 * under the terms of the GNU General Public License as published by the Free *
 * Software Foundation, either version 3 of the License, or (at your option)  *
 * any later version.                                                         *
 *                                                                            *
 * This program is distributed in the hope that it will be useful, but        *
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY *
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License   *
 * for more details.                                                          *
 *                                                                            *
 * You should have received a copy of the GNU General Public License along    *
 * with this program. If not, see <http://www.gnu.org/licenses/>.             *
 *                                                                            *
 ******************************************************************************/

#include <functional>

#include <gua/guacamole.hpp>
#include <gua/renderer/TriMeshLoader.hpp>
#include <gua/renderer/ToneMappingPass.hpp>
#include <gua/renderer/DebugViewPass.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/utils/Trackball.hpp>


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

    scm::math::mat4 accumulated_rotation_around_y_world_space = scm::math::mat4::identity();
    //scm::math::vec3f accumulated_translation = scm::math::vec3(0.0, 0.0, 0.0);
    double accumulated_rotation_around_up = 0.0;
    double accumulated_rotation_around_right = 0.0;

    double cam_translation_speed = 5.0;
    double cam_rotation_speed = 50.0;
};

WASD_state cam_navigation_state;

scm::math::mat4 cam_translation;



void update_cam_matrix(std::shared_ptr<gua::node::CameraNode> const& cam_node, std::shared_ptr<gua::node::TransformNode>& nav_node, float elapsed_milliseconds) {


    auto cam_world_matrix = cam_node->get_world_transform();

    /*
    std::cout << "Camera World Transform: " << std::endl;
    std::cout << cam_world_matrix << std::endl;

    auto right_cam_vector = cam_world_matrix.column(0);
    std::cout << "Cam right vector: " << right_cam_vector << std::endl;

    auto up_cam_vector = cam_world_matrix.column(1);
    std::cout << "Cam up vector: " << up_cam_vector << std::endl;

    auto forward_cam_vector = cam_world_matrix.column(2);
    std::cout << "Cam forward vector: " << forward_cam_vector << std::endl;

    auto cam_position = cam_world_matrix.column(3);
    std::cout << "Cam position: " << cam_position << std::endl;
    */

    if(cam_navigation_state.moving_forward ) {
        std::cout << "Moving Upward" << std::endl;

        auto forward_cam_vector = -cam_world_matrix.column(2);

        auto delta = forward_cam_vector * cam_navigation_state.cam_translation_speed * elapsed_milliseconds;

        cam_navigation_state.accumulated_translation_world_space = scm::math::mat4f(scm::math::make_translation( delta[0], delta[1], delta[2])) * cam_navigation_state.accumulated_translation_world_space;
    }

    if(cam_navigation_state.moving_backward ) {
        std::cout << "Moving Upward" << std::endl;

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
        auto down_cam_vector = cam_world_matrix.column(1);

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

        case 's': { //moves the camera downward with respect to the camera vector described in global coordinates
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

        case 'q': { //moves the camera to the right with respect to the camera vector described in global coordinates
            if(1 == action) {
                cam_navigation_state.moving_upward = true;
            } else if(0 == action) {
                cam_navigation_state.moving_upward = false;            
            }

            break;
        }

        case 'e': { //moves the camera to the right with respect to the camera vector described in global coordinates
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

        default: { //no assigned key
            break;
        }
    }

    if(action == 1) {
        std::cout << "On" << std::endl;
    } else if(action == 0) {
        std::cout << "Off" << std::endl;
    }

    if(action == 0)
        return;

    



}



void adjust_arguments(int& argc, char**& argv)
{
    char* argv_tmp[] = {argv[0], NULL};
    int argc_tmp = sizeof(argv_tmp) / sizeof(char*) - 1;
    argc = argc_tmp;
    argv = argv_tmp;
}

std::string parse_model_from_cmd_line(int argc, char** argv)
{
    std::string model_path = "data/objects/teapot.obj";

    std::string log_message_model_string = "";
    if(argc < 2)
    {
        gua::Logger::LOG_MESSAGE << argv[0] << ": Did not provide any model file." << std::endl;
        log_message_model_string = "Using default model path: " + model_path;
    }
    else
    {
        model_path = argv[1];
        log_message_model_string = std::string(argv[0]) + ": Using provided model path:";
    }
    gua::Logger::LOG_MESSAGE << log_message_model_string + model_path << std::endl;

    return model_path;
}

int main(int argc, char** argv)
{
    std::string model_path = parse_model_from_cmd_line(argc, argv);

    adjust_arguments(argc, argv);
    gua::init(argc, argv);

    // setup scene
    gua::SceneGraph graph("main_scenegraph");

    gua::TriMeshLoader loader;

    auto model_mat(gua::MaterialShaderDatabase::instance()->lookup("gua_default_material")->make_new_material());

    model_mat->set_render_wireframe(false);
    model_mat->set_show_back_faces(false);

    auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");
    auto model(
        loader.create_geometry_from_file("model", model_path, model_mat,  gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE ));

    graph.add_node("/transform", model);
    model->set_draw_bounding_box(true);



    auto light = graph.add_node<gua::node::LightNode>("/transform", "light");
    light->data.set_type(gua::node::LightNode::Type::POINT);
    light->data.brightness = 150.0f;
    light->scale(12.f);
    light->translate(-3.f, 5.f, 5.f);



    auto navigation_node = graph.add_node<gua::node::TransformNode>("/", "navigation_node");


    auto screen = graph.add_node<gua::node::ScreenNode>("/navigation_node", "screen");
    screen->data.set_size(gua::math::vec2(1.92f, 1.08f));
    screen->translate(0, 0, -3.0);


    // add mouse interaction
    gua::utils::Trackball trackball(0.01, 0.002, 0.2);

    // setup rendering pipeline and window
    auto resolution = gua::math::vec2ui(960, 540);


    auto resolve_pass = std::make_shared<gua::ResolvePassDescription>();
    resolve_pass->background_mode(gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE);
    resolve_pass->tone_mapping_exposure(1.0f);

    auto camera = graph.add_node<gua::node::CameraNode>("/navigation_node", "cam");
    camera->translate(0, 0, 0);
    camera->config.set_resolution(resolution);
    camera->config.set_screen_path("/navigation_node/screen");
    camera->config.set_scene_graph_name("main_scenegraph");
    camera->config.set_output_window_name("main_window");
    camera->config.set_enable_stereo(false);


    camera->get_pipeline_description()->get_resolve_pass()->tone_mapping_exposure(1.0f);
    camera->get_pipeline_description()->add_pass(std::make_shared<gua::DebugViewPassDescription>());

    auto window = std::make_shared<gua::GlfwWindow>();
    gua::WindowDatabase::instance()->add("main_window", window);

    window->config.set_enable_vsync(false);
    window->config.set_size(resolution);
    window->config.set_resolution(resolution);
    window->config.set_stereo_mode(gua::StereoMode::MONO);

    window->on_resize.connect([&](gua::math::vec2ui const& new_size) {
        window->config.set_resolution(new_size);
        camera->config.set_resolution(new_size);
        screen->data.set_size(gua::math::vec2(0.001 * new_size.x, 0.001 * new_size.y));
    });
    window->on_move_cursor.connect([&](gua::math::vec2 const& pos) { trackball.motion(pos.x, pos.y); });
    window->on_button_press.connect(std::bind(mouse_button, std::ref(trackball), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    window->on_key_press.connect(
        std::bind(key_press, std::ref(*(camera->get_pipeline_description())), std::ref(graph), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));


    gua::Renderer renderer;

    // application loop
    gua::events::MainLoop loop;
    gua::events::Ticker ticker(loop, 1.0 / 500.0);



    ticker.on_tick.connect([&]() {

        std::cout << std::endl;

        // apply trackball matrix to object
        gua::math::mat4 modelmatrix = scm::math::make_translation(trackball.shiftx(), trackball.shifty(), trackball.distance()) * gua::math::mat4(trackball.rotation());




        //void update_navigation_node(cam_world_matrix) {
            
        //}

        //camera = 1;



        transform->set_transform(modelmatrix);
        transform->translate(0.0f, 0.0f, -5.0f);




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



        update_cam_matrix(camera, navigation_node, elapsed_application_time_milliseconds);

        if(window->should_close())
        {
            renderer.stop();
            window->close();
            loop.stop();
        }
        else
        {
            renderer.queue_draw({&graph});
        }
    });

    loop.start();

    return 0;
}