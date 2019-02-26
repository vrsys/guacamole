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
#include <gua/guacamole.hpp>
#include <gua/node/TexturedQuadNode.hpp>
#include <gua/node/TriMeshNode.hpp>

#include <gua/platform.hpp>

#include <gua/renderer/TriMeshLoader.hpp>
#include <gua/renderer/SSAAPass.hpp>
#include <gua/renderer/BBoxPass.hpp>
#include <gua/renderer/TexturedQuadPass.hpp>
#include <gua/renderer/DebugViewPass.hpp>
#include <gua/renderer/TriMeshLoader.hpp>

#include <thread>
#include <chrono>
#include <cmath>

void set_window_default(std::shared_ptr<gua::WindowBase> const& window, gua::math::vec2ui const& res)
{
    window->config.set_size(res);
    window->config.set_resolution(res);
    window->config.set_enable_vsync(true);
}

int main(int argc, char** argv)
{
    // auto resolution = gua::math::vec2ui(800, 600);
    auto resolution = gua::math::vec2ui(3840, 2160);

    // initialize guacamole
    int argc_d = 0;
    char** argv_d = {};
    gua::init(argc_d, argv_d);

    // setup scene
    gua::SceneGraph graph("main_scenegraph");

    auto desc(std::make_shared<gua::MaterialShaderDescription>());
    desc->load_from_file("data/materials/SimpleMaterial.gmd");

    auto shader(std::make_shared<gua::MaterialShader>("simple_mat", desc));
    gua::MaterialShaderDatabase::instance()->add(shader);

    auto mat(shader->make_new_material());

    gua::TriMeshLoader trimeshloader;

    auto teapot_geode(trimeshloader.create_geometry_from_file("teapot_geode", "data/objects/teapot.obj", mat, gua::TriMeshLoader::DEFAULTS));
    auto plate_geode(trimeshloader.create_geometry_from_file("plate_geode", "data/objects/plate.obj", mat, gua::TriMeshLoader::DEFAULTS));

    auto head = graph.add_node<gua::node::TransformNode>("/", "head");
    head->translate(0.0, 0.0, 8.0);

    auto teapot = graph.add_node<gua::node::TransformNode>("/", "teapot");
    teapot->scale(2.0f);
    auto plate = graph.add_node<gua::node::TransformNode>("/", "plate");

    graph.add_node("/teapot", teapot_geode);
    graph.add_node("/plate", plate_geode);

    const float aspect = resolution.x * 1.0f / resolution.y;

    auto pipe = std::make_shared<gua::PipelineDescription>();

    pipe->add_pass(std::make_shared<gua::TriMeshPassDescription>());
    pipe->add_pass(std::make_shared<gua::TexturedQuadPassDescription>());
    // pipe->add_pass(std::make_shared<gua::BBoxPassDescription>());
    pipe->add_pass(std::make_shared<gua::LightVisibilityPassDescription>());
    pipe->add_pass(std::make_shared<gua::ResolvePassDescription>());
    // pipe->add_pass(std::make_shared<gua::SSAAPassDescription>());
    // pipe->add_pass(std::make_shared<gua::DebugViewPassDescription>());

    auto camera_tl = graph.add_node<gua::node::CameraNode>("/head", "camera_tl");
    camera_tl->config.set_output_window_name("window_tl");
    camera_tl->config.set_screen_path("/head/camera_tl/screen_tl");
    camera_tl->config.set_scene_graph_name("main_scenegraph");
    camera_tl->config.set_resolution(resolution);
    camera_tl->config.set_view_id(1);
    camera_tl->set_pipeline_description(pipe);

    auto screen_tl = graph.add_node<gua::node::ScreenNode>("/head/camera_tl", "screen_tl");
    screen_tl->data.set_size(gua::math::vec2(1.6, 0.9));
    screen_tl->translate(-0.8, 0.45, -2.5);

    auto camera_tr = graph.add_node<gua::node::CameraNode>("/head", "camera_tr");
    camera_tr->config.set_output_window_name("window_tr");
    camera_tr->config.set_screen_path("/head/camera_tr/screen_tr");
    camera_tr->config.set_scene_graph_name("main_scenegraph");
    camera_tr->config.set_resolution(resolution);
    camera_tr->config.set_view_id(2);
    camera_tr->set_pipeline_description(pipe);

    auto screen_tr = graph.add_node<gua::node::ScreenNode>("/head/camera_tr", "screen_tr");
    screen_tr->data.set_size(gua::math::vec2(1.6, 0.9));
    screen_tr->translate(0.8f, 0.45, -2.5f);

    auto camera_bl = graph.add_node<gua::node::CameraNode>("/head", "camera_bl");
    camera_bl->config.set_output_window_name("window_bl");
    camera_bl->config.set_screen_path("/head/camera_bl/screen_bl");
    camera_bl->config.set_scene_graph_name("main_scenegraph");
    camera_bl->config.set_resolution(resolution);
    // camera_bl->config.set_view_id(2);
    camera_bl->set_pipeline_description(pipe);

    auto screen_bl = graph.add_node<gua::node::ScreenNode>("/head/camera_bl", "screen_bl");
    screen_bl->data.set_size(gua::math::vec2(1.6, 0.9));
    screen_bl->translate(-0.8, -0.45, -2.5f);

    auto camera_br = graph.add_node<gua::node::CameraNode>("/head", "camera_br");
    camera_br->config.set_output_window_name("window_br");
    camera_br->config.set_screen_path("/head/camera_br/screen_br");
    camera_br->config.set_scene_graph_name("main_scenegraph");
    camera_br->config.set_resolution(resolution);
    camera_br->set_pipeline_description(pipe);

    auto screen_br = graph.add_node<gua::node::ScreenNode>("/head/camera_br", "screen_br");
    screen_br->data.set_size(gua::math::vec2(1.6, 0.9));
    screen_br->translate(0.8, -0.45, -2.5f);

    auto pointlight = graph.add_node<gua::node::LightNode>("/", "pointlight");
    pointlight->data.set_type(gua::node::LightNode::Type::POINT);
    pointlight->scale(30.0f);
    pointlight->rotate(-90, 1, 0, 0);
    pointlight->translate(1.0, 18.0, 1.0);

    pointlight->data.set_falloff(0.1f);
    pointlight->data.set_color({1.0f, 1.0f, 1.0f});
    pointlight->data.set_enable_specular_shading(true);
    pointlight->data.set_enable_diffuse_shading(true);

    auto add_window = [](std::string const& window_name, std::string const& display, std::shared_ptr<gua::node::CameraNode> const& cam_node) {
        auto window = std::make_shared<gua::Window>();
        window->config.set_display_name(display);
        window->join_swap_group(1);
        window->bind_swap_barrier(1);
        gua::WindowDatabase::instance()->add(window_name, window);
        set_window_default(window, cam_node->config.get_resolution());
        cam_node->config.set_output_window_name(window_name);
    };

    add_window("window_tl", ":0.0", camera_tl);

    gua::Renderer renderer;

    // application loop
    gua::events::MainLoop loop;

    gua::events::Ticker ticker(loop, 1.0 / 500.0);

    // gua::Timer frame_timer;
    // frame_timer.start();

    float time_value = 0;

    // application loop
    std::size_t cnt = 0;
    // while (true) {
    // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ticker.on_tick.connect([&]() {
        teapot_geode->rotate(0.3, 0, 1, 0);

        ++cnt;

        if(cnt == 10)
        {
            // add_window("window_tr", ":0.0", camera_tr);
            add_window("window_tr", ":0.1", camera_tr);
        }
        if(cnt == 20)
        {
            // add_window("window_bl", ":0.0", camera_bl);
            add_window("window_bl", ":0.2", camera_bl);
        }
        if(cnt == 30)
        {
            // add_window("window_br", ":0.0", camera_br);
            add_window("window_br", ":0.3", camera_br);
        }

        // GLFW only allows event processing from main thread
        if(gua::WindowDatabase::instance()->lookup("window_tl"))
        {
            gua::WindowDatabase::instance()->lookup("window_tl")->process_events();
        }

        if(gua::WindowDatabase::instance()->lookup("window_tr"))
        {
            gua::WindowDatabase::instance()->lookup("window_tr")->process_events();
        }

        if(gua::WindowDatabase::instance()->lookup("window_bl"))
        {
            gua::WindowDatabase::instance()->lookup("window_bl")->process_events();
        }

        if(gua::WindowDatabase::instance()->lookup("window_br"))
        {
            gua::WindowDatabase::instance()->lookup("window_br")->process_events();
        }

        plate->translate(-plate->get_bounding_box().center());
        plate->rotate(0.04f, 0, 1, 0);
        plate->translate(plate->get_bounding_box().center());

        renderer.queue_draw({&graph});
    });
    //}
    loop.start();
    renderer.stop();

    return 0;
}
