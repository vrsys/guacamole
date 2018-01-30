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
#include <gua/ViveWindow.hpp>
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
const float user_height = 0.0f;
const float user_speed_walk = 0.02;
const float user_speed_run = 0.2;
bool run_mode = false;

const bool pipeline_enable_alternate_frame_rendering = true;
const bool pipeline_show_fps = true;


////////////////////////////////////////////////////////////////////////////
// main application
////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv) {
    // initialize guacamole
    gua::init(argc, argv);

    // setup scene
    gua::SceneGraph graph("main_scenegraph");

    gua::TriMeshLoader loader;

    auto monkey(loader.create_geometry_from_file(
        "monkey",
        "../data/objects/monkey.obj", gua::TriMeshLoader::LOAD_MATERIALS
    ));

    auto casted(std::dynamic_pointer_cast<gua::node::TriMeshNode>(monkey));
    casted->get_material()->set_uniform("Emissivity", 0.2f);
    casted->get_material()->set_uniform("Color", gua::math::vec4(1.0, 1.0, 1.0, 1));


    monkey->scale(0.2);
    monkey->translate(0.0, 1.75, 0.0);


    auto root = graph.add_node("/", monkey);

    auto nav = graph.add_node<gua::node::TransformNode>("/", "nav");
    nav->translate(0.0, user_height, 0.0);

    // setup rendering pipeline and window
#if WIN32
    auto window = std::make_shared<gua::ViveWindow>(":0.0");
    gua::WindowDatabase::instance()->add("main_window", window);
    window->config.set_enable_vsync(false);
    window->config.set_fullscreen_mode(false);
    //window->open();
#else
    auto window = std::make_shared<gua::ViveWindow>(":0.0");
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
    // setup rendering
    //////////////////////////////////////////////////////////////////////////////////////
    double time = 0.0; // current time
    long long ctr = 0; // frame timer 
    const float desired_frame_time = 1.0 / 1000.0; // desired application of 1000Hz

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

        // tracking update
        camera->set_transform(window->get_hmd_sensor_orientation());

        // window update
        if (window->should_close()) {
            renderer.stop();
            window->close();
            loop.stop();
        } else {
            renderer.queue_draw({ &graph }, pipeline_enable_alternate_frame_rendering);
        }

        if (ctr++ % 150 == 0 && pipeline_show_fps) {
            std::cout << "Frame time: " << 1000.f / window->get_rendering_fps() << " ms, fps: "
                << window->get_rendering_fps() << ", app fps: "
                << renderer.get_application_fps() << std::endl;
        }
    });

    loop.start();

    return 0;
}
