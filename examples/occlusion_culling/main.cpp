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


void update_cam_matrix(std::shared_ptr<gua::node::CameraNode> const& cam_node, std::shared_ptr<gua::node::TransformNode> const& nav_node) {

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

//   7654 3210 
//-------------
// 0bxxxx xxxx
//           ^is moving forwrad?
//          ^is moving left?
//         ^is moving backward?
//        ^is moving right?
//      ^is moving upward?
//     ^is moving downward?
//    ^rotate around local x axis?
//   ^rotate around local y axis?
struct WASD_state {
    uint8_t is_moving = 0x0f;
};

WASD_state cam_navigation_state;

void key_press(gua::PipelineDescription& pipe, gua::SceneGraph& graph, int key, int scancode, int action, int mods)
{

    //action == 1: key was pressed
    //action == 0: key was released

    switch(std::tolower(key)) {
        case 'w': { //moves the camera forward with respect to the camera vector described in global coordinates
            if(1 == action) {
                cam_navigation_state.is_moving |= 0b00000001; //set least significant bit to "true"
            } else if(0 == action) {
                cam_navigation_state.is_moving &= 0b11111110; //set least significant bit to "false"                
            }

            break;
        }

        case 'a': { //moves the camera to the left with respect to the camera vector described in global coordinates
            if(1 == action) {
                cam_navigation_state.is_moving |= 0b00000010; //set least significant bit to "true"
            } else if(0 == action) {
                cam_navigation_state.is_moving &= 0b11111101; //set least significant bit to "false"                
            }

            break;
        }

        case 's': { //moves the camera downward with respect to the camera vector described in global coordinates
            if(1 == action) {
                cam_navigation_state.is_moving |= 0b00000100; //set least significant bit to "true"
            } else if(0 == action) {
                cam_navigation_state.is_moving &= 0b11111011; //set least significant bit to "false"                
            }

            break;
        }

        case 'd': { //moves the camera to the right with respect to the camera vector described in global coordinates
            if(1 == action) {
                cam_navigation_state.is_moving |= 0b00001000; //set least significant bit to "true"
            } else if(0 == action) {
                cam_navigation_state.is_moving &= 0b11110011; //set least significant bit to "false"                
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

    float v = 0.0f;

    /*

    switch(std::tolower(key))
    {
    case 'm': // toggle environment lighting mode

        if(pipe.get_resolve_pass()->environment_lighting_mode() == gua::ResolvePassDescription::EnvironmentLightingMode::AMBIENT_COLOR)
        {
            std::cout << "Setting to gua::ResolvePassDescription::EnvironmentLightingMode::SPHEREMAP" << std::endl;
            pipe.get_resolve_pass()->environment_lighting_mode(gua::ResolvePassDescription::EnvironmentLightingMode::SPHEREMAP);
        }
        else if(pipe.get_resolve_pass()->environment_lighting_mode() == gua::ResolvePassDescription::EnvironmentLightingMode::SPHEREMAP)
        {
            std::cout << "Setting to gua::ResolvePassDescription::EnvironmentLightingMode::CUBEMAP" << std::endl;
            pipe.get_resolve_pass()->environment_lighting_mode(gua::ResolvePassDescription::EnvironmentLightingMode::CUBEMAP);
        }
        else
        {
            std::cout << "Setting to gua::ResolvePassDescription::EnvironmentLightingMode::AMBIENT_COLOR" << std::endl;
            pipe.get_resolve_pass()->environment_lighting_mode(gua::ResolvePassDescription::EnvironmentLightingMode::AMBIENT_COLOR);
        }

        pipe.get_resolve_pass()->touch();
        break;

    case 'b': // toggle background mode

        if(pipe.get_resolve_pass()->background_mode() == gua::ResolvePassDescription::BackgroundMode::COLOR)
        {
            std::cout << "Setting to gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE" << std::endl;
            pipe.get_resolve_pass()->background_mode(gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE);
        }
        else if(pipe.get_resolve_pass()->background_mode() == gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE)
        {
            std::cout << "Setting to gua::ResolvePassDescription::BackgroundMode::SKYMAP_TEXTURE" << std::endl;
            pipe.get_resolve_pass()->background_mode(gua::ResolvePassDescription::BackgroundMode::SKYMAP_TEXTURE);
        }
        else
        {
            std::cout << "Setting to gua::ResolvePassDescription::BackgroundMode::AMBIENT_COLOR" << std::endl;
            pipe.get_resolve_pass()->background_mode(gua::ResolvePassDescription::BackgroundMode::COLOR);
        }

        pipe.get_resolve_pass()->touch();
        break;

    case 'a': // toggle screen space shadows
        pipe.get_resolve_pass()->screen_space_shadows(!pipe.get_resolve_pass()->screen_space_shadows());
        break;

    case 'q':
        pipe.get_resolve_pass()->screen_space_shadow_radius(std::min(10.0f, 1.1f * pipe.get_resolve_pass()->screen_space_shadow_radius()));
        break;

    case 'z':
        pipe.get_resolve_pass()->screen_space_shadow_radius(std::max(0.005f, 0.9f * pipe.get_resolve_pass()->screen_space_shadow_radius()));
        break;

    case 'w':
        pipe.get_resolve_pass()->screen_space_shadow_intensity(std::min(1.0f, 1.1f * pipe.get_resolve_pass()->screen_space_shadow_intensity()));
        break;

    case 'x':
        pipe.get_resolve_pass()->screen_space_shadow_intensity(std::max(0.1f, 0.9f * pipe.get_resolve_pass()->screen_space_shadow_intensity()));
        break;

    case 's': // toggle SSAO
        pipe.get_resolve_pass()->ssao_enable(!pipe.get_resolve_pass()->ssao_enable());
        break;

    case '1':
        pipe.get_resolve_pass()->ssao_intensity(std::min(5.0f, 1.1f * pipe.get_resolve_pass()->ssao_intensity()));
        break;
    case '2':
        pipe.get_resolve_pass()->ssao_intensity(std::max(0.02f, 0.9f * pipe.get_resolve_pass()->ssao_intensity()));
        break;

    case '3':
        pipe.get_resolve_pass()->ssao_radius(std::min(64.0f, 1.1f * pipe.get_resolve_pass()->ssao_radius()));
        break;
    case '4':
        pipe.get_resolve_pass()->ssao_radius(std::max(1.0f, 0.9f * pipe.get_resolve_pass()->ssao_radius()));
        break;

    case '5':
        pipe.get_resolve_pass()->ssao_falloff(std::min(256.0f, 1.1f * pipe.get_resolve_pass()->ssao_falloff()));
        break;
    case '6':
        pipe.get_resolve_pass()->ssao_falloff(std::max(0.1f, 0.9f * pipe.get_resolve_pass()->ssao_falloff()));
        break;

    case 'f':
        if(pipe.get_ssaa_pass()->mode() == gua::SSAAPassDescription::SSAAMode::FXAA311)
        {
            std::cout << "Switching to simple FAST_FXAA\n" << std::endl;
            pipe.get_ssaa_pass()->mode(gua::SSAAPassDescription::SSAAMode::FAST_FXAA);
        }
        else if(pipe.get_ssaa_pass()->mode() == gua::SSAAPassDescription::SSAAMode::FAST_FXAA)
        {
            std::cout << "Switching to No FXAA\n" << std::endl;
            pipe.get_ssaa_pass()->mode(gua::SSAAPassDescription::SSAAMode::DISABLED);
        }
        else
        {
            std::cout << "Switching to FXAA 3.11\n" << std::endl;
            pipe.get_ssaa_pass()->mode(gua::SSAAPassDescription::SSAAMode::FXAA311);
        }
        break;

    case '7':
        v = std::min(1.0f, 1.1f * pipe.get_ssaa_pass()->fxaa_quality_subpix());
        std::cout << "Setting quality_subpix to " << v << std::endl;
        pipe.get_ssaa_pass()->fxaa_quality_subpix(v);
        break;
    case '8':
        v = std::max(0.2f, 0.9f * pipe.get_ssaa_pass()->fxaa_quality_subpix());
        std::cout << "Setting quality_subpix to " << v << std::endl;
        pipe.get_ssaa_pass()->fxaa_quality_subpix(v);
        break;

    case '9':
        v = std::min(0.333f, 1.1f * pipe.get_ssaa_pass()->fxaa_edge_threshold());
        std::cout << "Setting edge_threshold to " << v << std::endl;
        pipe.get_ssaa_pass()->fxaa_edge_threshold(v);
        break;
    case '0':
        v = std::max(0.063f, 0.9f * pipe.get_ssaa_pass()->fxaa_edge_threshold());
        std::cout << "Setting edge_threshold to " << v << std::endl;
        pipe.get_ssaa_pass()->fxaa_edge_threshold(v);
        break;

    case 't':
        pipe.get_resolve_pass()->touch();
        break;

    case ' ':
        animate_light = !animate_light;
        break;

    // recompile shaders
    case 'r':
        pipe.get_resolve_pass()->touch();
        break;

 
    default:
        break;
    }
   */

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
    auto example_model(
        loader.create_geometry_from_file("example_model", model_path, model_mat, 0));

    graph.add_node("/transform", example_model);
    example_model->set_draw_bounding_box(true);

    auto portal = graph.add_node<gua::node::TexturedQuadNode>("/", "portal");
    portal->data.set_size(gua::math::vec2(1.2f, 0.8f));
    portal->data.set_texture("portal");
    portal->translate(0.5f, 0.f, -0.2f);
    portal->rotate(-30, 0.f, 1.f, 0.f);

    auto light2 = graph.add_node<gua::node::LightNode>("/", "light2");
    light2->data.set_type(gua::node::LightNode::Type::POINT);
    light2->data.brightness = 150.0f;
    light2->scale(12.f);
    light2->translate(-3.f, 5.f, 5.f);

    auto screen = graph.add_node<gua::node::ScreenNode>("/", "screen");
    screen->data.set_size(gua::math::vec2(1.92f, 1.08f));
    screen->translate(0, 0, 1.0);


    // add mouse interaction
    gua::utils::Trackball trackball(0.01, 0.002, 0.2);

    // setup rendering pipeline and window
    auto resolution = gua::math::vec2ui(1920, 1080);


    auto resolve_pass = std::make_shared<gua::ResolvePassDescription>();
    resolve_pass->background_mode(gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE);
    resolve_pass->tone_mapping_exposure(1.0f);

    auto camera = graph.add_node<gua::node::CameraNode>("/screen", "cam");
    camera->translate(0, 0, 2.0);
    camera->config.set_resolution(resolution);
    camera->config.set_screen_path("/screen");
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
        // apply trackball matrix to object
        gua::math::mat4 modelmatrix = scm::math::make_translation(trackball.shiftx(), trackball.shifty(), trackball.distance()) * gua::math::mat4(trackball.rotation());


        //void update_navigation_node(cam_world_matrix) {
            
        //}

        //camera = 1;


        auto cam_world_matrix = camera->get_world_transform();

        std::cout << "Camera World Transform: " << cam_world_matrix << std::endl;

        auto right_cam_vector = cam_world_matrix.column(0);
        std::cout << "Cam right vector: " << right_cam_vector << std::endl;

        auto up_cam_vector = cam_world_matrix.column(1);
        std::cout << "Cam up vector: " << up_cam_vector << std::endl;

        auto forward_cam_vector = cam_world_matrix.column(2);
        std::cout << "Cam forward vector: " << forward_cam_vector << std::endl;

        auto cam_position = cam_world_matrix.column(3);
        std::cout << "Cam position: " << cam_position << std::endl;

        transform->set_transform(modelmatrix);





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