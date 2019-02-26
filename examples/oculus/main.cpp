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

#include <memory>
#include <functional>

#include <gua/guacamole.hpp>
#include <gua/renderer/TriMeshLoader.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/OculusWindow.hpp>
#include <gua/renderer/SSAAPass.hpp>

////////////////////////////////////////////////////////////////////////////
// demo configuration
////////////////////////////////////////////////////////////////////////////

// lighting
const unsigned light_count = 4;
const float light_proxy_size = 0.01f;

const gua::utils::Color3f light_color(1.0f, 1.0f, 0.9f);
const float light_size = 4.0f;

const float light_base_y = 2.8f;
const float light_base_x = -0.7f;
const float light_base_z = -1.3f;
const float light_diff_x = 2.2f;
const float light_diff_z = 2.6f;

// user navigation
const float user_height = 1.8f;
const float user_speed_walk = 0.02;
const float user_speed_run = 0.2;
bool run_mode = false;

// visual configuration

// toogle with KEY '1'
bool pipeline_use_ssao = false;
const float pipeline_ssao_intensity = 1.0f;
const float pipeline_ssao_radius = 2.0f;
const float pipeline_ssao_falloff = 0.1;

// toggle with KEY '2'
bool pipeline_show_fps = false;

// toggle with KEY '3'
gua::SSAAPassDescription::SSAAMode antialiasing_mode = gua::SSAAPassDescription::SSAAMode::FXAA311;

// toogle with KEY '4'
bool pipeline_use_shadow_maps = false;
const int pipeline_shadow_map_size = 128;

// toggle with KEY '5'
bool pipeline_enable_alternate_frame_rendering = false;
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
// helper methods for scene construction
////////////////////////////////////////////////////////////////////////////
std::vector<std::shared_ptr<gua::node::TransformNode>> add_lights(gua::SceneGraph& graph)
{
    std::vector<std::shared_ptr<gua::node::TransformNode>> lights(light_count);

    for(int i = 0; i != lights.size(); ++i)
    {
        gua::TriMeshLoader loader;
        auto sphere_geometry(loader.create_geometry_from_file("sphere" + gua::string_utils::to_string(i), "data/objects/light_sphere.obj"));

        sphere_geometry->scale(light_proxy_size);

        lights[i] = graph.add_node("/", std::make_shared<gua::node::TransformNode>("light" + gua::string_utils::to_string(i)));
        lights[i]->add_child(sphere_geometry);
        lights[i]->scale(light_size);
        lights[i]->translate(float(i / 2) * light_diff_x + light_base_x, light_base_y, light_base_z + float(i % 2) * light_diff_z);

        auto light = lights[i]->add_child(std::make_shared<gua::node::LightNode>("light"));
        light->data.set_type(gua::node::LightNode::Type::POINT);
        light->data.set_color(light_color);

        light->data.set_enable_shadows(pipeline_use_shadow_maps);
        light->data.set_shadow_map_size(pipeline_shadow_map_size);
        light->data.set_shadow_near_clipping_in_sun_direction(0.2f);
        light->data.set_shadow_far_clipping_in_sun_direction(5.2f);
    }

    return lights;
}

////////////////////////////////////////////////////////////////////////////
// main application
////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
    // initialize guacamole
    gua::init(argc, argv);

    // setup scene
    gua::SceneGraph graph("main_scenegraph");

    gua::TriMeshLoader loader;

    auto room_geometry(loader.create_geometry_from_file("room", "data/objects/re_room/RE2 Briefing room.obj", gua::TriMeshLoader::LOAD_MATERIALS));

    auto root = graph.add_node("/", room_geometry);

    auto lights = add_lights(graph);

    auto nav = graph.add_node<gua::node::TransformNode>("/", "nav");
    nav->translate(0.0, user_height, 0.0);

    // setup rendering pipeline and window
#if WIN32
    auto window = std::make_shared<gua::OculusWindow>(":0.0");
    gua::WindowDatabase::instance()->add("main_window", window);
    window->config.set_enable_vsync(false);
    window->config.set_fullscreen_mode(false);
    // window->open();
#else
    auto window = std::make_shared<gua::OculusWindow>(":0.0");
    gua::WindowDatabase::instance()->add("main_window", window);
    window->config.set_enable_vsync(false);
    window->config.set_fullscreen_mode(true);
    window->config.set_size(window->get_window_resolution());
    window->config.set_resolution(window->get_window_resolution());
    window->open();
#endif

    // setup pipeline
    auto resolve_pass = std::make_shared<gua::ResolvePassDescription>();
    resolve_pass->tone_mapping_exposure(1.0f);

    // setup camera
    auto camera = graph.add_node<gua::node::CameraNode>("/nav", "cam");

    camera->config.set_resolution(window->get_window_resolution());
    camera->config.set_left_screen_path("/nav/cam/left_screen");
    camera->config.set_right_screen_path("/nav/cam/right_screen");
    camera->config.set_scene_graph_name("main_scenegraph");
    camera->config.set_output_window_name("main_window");
    camera->config.set_enable_stereo(true);
    camera->config.set_eye_dist(window->get_IPD());

    camera->get_pipeline_description()->add_pass(std::make_shared<gua::SSAAPassDescription>());

    camera->get_pipeline_description()->get_resolve_pass()->tone_mapping_exposure(1.0f);

    auto left_screen = graph.add_node<gua::node::ScreenNode>("/nav/cam", "left_screen");
    left_screen->data.set_size(window->get_left_screen_size());
    left_screen->translate(window->get_left_screen_translation());

    auto right_screen = graph.add_node<gua::node::ScreenNode>("/nav/cam", "right_screen");
    right_screen->data.set_size(window->get_right_screen_size());
    right_screen->translate(window->get_right_screen_translation());

    //////////////////////////////////////////////////////////////////////////////////////
    // key press events
    //////////////////////////////////////////////////////////////////////////////////////
    window->on_key_press.connect([&](int key, int scancode, int action, int mods) {
        if(key == 340)
        { // SHIFT
            run_mode = (action != 0);
        }
        float speed = run_mode ? user_speed_run : user_speed_walk;

        if(action == 0)
            return; // only press events

        switch(key)
        {
        case 'W':
            nav->translate(0.0, 0.0, -speed);
            break;
        case 'A':
            nav->translate(-speed, 0.0, 0.0);
            break;
        case 'S':
            nav->translate(0.0, 0.0, speed);
            break;
        case 'D':
            nav->translate(speed, 0.0, 0.0);
            break;
        case '1': // show fps in console
            pipeline_show_fps = !pipeline_show_fps;
            std::cout << "FPS = " << pipeline_show_fps << std::endl;
            break;
        case '2': // screen space ambient occlusion
            pipeline_use_ssao = !pipeline_use_ssao;
            std::cout << "SSAO = " << pipeline_use_ssao << std::endl;
            break;
        case '3': // anti-aliasing
            if(camera->get_pipeline_description()->get_ssaa_pass()->mode() == gua::SSAAPassDescription::SSAAMode::FXAA311)
            {
                std::cout << "Switching to simple FAST_FXAA\n" << std::endl;
                camera->get_pipeline_description()->get_ssaa_pass()->mode(gua::SSAAPassDescription::SSAAMode::FAST_FXAA);
            }
            else if(camera->get_pipeline_description()->get_ssaa_pass()->mode() == gua::SSAAPassDescription::SSAAMode::FAST_FXAA)
            {
                std::cout << "Switching to No FXAA\n" << std::endl;
                camera->get_pipeline_description()->get_ssaa_pass()->mode(gua::SSAAPassDescription::SSAAMode::DISABLED);
            }
            else
            {
                std::cout << "Switching to FXAA 3.11\n" << std::endl;
                camera->get_pipeline_description()->get_ssaa_pass()->mode(gua::SSAAPassDescription::SSAAMode::FXAA311);
            }
            break;
        case '4': // shadow maps
            pipeline_use_shadow_maps = !pipeline_use_shadow_maps;
            std::cout << "Shadow maps = " << pipeline_use_shadow_maps << std::endl;
            break;
        case '5': // alternate frame rendering
            pipeline_enable_alternate_frame_rendering = !pipeline_enable_alternate_frame_rendering;
            std::cout << "pipeline_enable_alternate_frame_rendering maps = " << pipeline_enable_alternate_frame_rendering << std::endl;
            break;
        default:
            break;
        };
    });

    //////////////////////////////////////////////////////////////////////////////////////
    // setup rendering
    //////////////////////////////////////////////////////////////////////////////////////
    double time = 0.0;                            // current time
    long long ctr = 0;                            // frame timer
    const float desired_frame_time = 1.0 / 120.0; // desired application of 120Hz

    // setup application loop
    gua::Renderer renderer;
    gua::events::MainLoop loop;
    gua::events::Ticker ticker(loop, desired_frame_time);

    gua::Timer timer;
    timer.start();

    //////////////////////////////////////////////////////////////////////////////////////
    // mainloop
    //////////////////////////////////////////////////////////////////////////////////////
    ticker.on_tick.connect([&]() {
        double frame_time(timer.get_elapsed());
        time += frame_time;
        timer.reset();

        camera->set_transform(window->get_hmd_sensor_orientation());

        if(pipeline_use_ssao != camera->get_pipeline_description()->get_resolve_pass()->ssao_enable())
        {
            camera->get_pipeline_description()->get_resolve_pass()->ssao_intensity(pipeline_ssao_intensity);
            camera->get_pipeline_description()->get_resolve_pass()->ssao_enable(pipeline_use_ssao);
            camera->get_pipeline_description()->get_resolve_pass()->ssao_falloff(pipeline_ssao_falloff);
            camera->get_pipeline_description()->get_resolve_pass()->ssao_radius(pipeline_ssao_radius);
        }

        if(window->should_close())
        {
            renderer.stop();
            window->close();
            loop.stop();
        }
        else
        {
            renderer.queue_draw({&graph}, pipeline_enable_alternate_frame_rendering);
        }

        for(unsigned i = 0; i != light_count; ++i)
        {
            auto light = std::dynamic_pointer_cast<gua::node::LightNode>(lights[i]->get_children()[1]);

            if(light)
            {
                if(pipeline_use_shadow_maps != light->data.get_enable_shadows())
                {
                    light->data.set_enable_shadows(pipeline_use_shadow_maps);
                }
            }
        }

        if(ctr++ % 150 == 0 && pipeline_show_fps)
        {
            std::cout << "Frame time: " << 1000.f / window->get_rendering_fps() << " ms, fps: " << window->get_rendering_fps() << ", app fps: " << renderer.get_application_fps() << std::endl;
        }
    });

    loop.start();

    return 0;
}
