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
    auto resolution = gua::math::vec2ui(800, 600);

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
    // mat.set_uniform("color", gua::math::vec3(1, 0, 0));
    // mat.set_uniform("color", gua::math::vec3(0, 1, 1), 1);
    // mat.set_uniform("color", gua::math::vec3(1, 0, 1), 2);

    gua::TriMeshLoader trimeshloader;
    // gua::NURBSLoader nurbsloader;
    // gua::Video3DLoader videoloader;

    auto teapot_geode(trimeshloader.create_geometry_from_file("teapot_geode", "data/objects/teapot.obj", mat, gua::TriMeshLoader::DEFAULTS));
    auto plate_geode(trimeshloader.create_geometry_from_file("plate_geode", "data/objects/plate.obj", mat, gua::TriMeshLoader::DEFAULTS));

    auto teapot = graph.add_node<gua::node::TransformNode>("/", "teapot");
    auto plate = graph.add_node<gua::node::TransformNode>("/", "plate");

    // auto video = graph.add_node<gua::node::TransformNode>("/", "video");
    // auto nurbs = graph.add_node<gua::node::TransformNode>("/", "nurbs");

    graph.add_node("/teapot", teapot_geode);
    graph.add_node("/plate", plate_geode);
    // graph.add_node("/pig", pig_geode);
    // graph.add_node("/video", video_geode);
    // graph.add_node("/nurbs", nurbs_geode);

    const float aspect = resolution.x * 1.0f / resolution.y;

    auto screen1 = graph.add_node<gua::node::ScreenNode>("/", "screen1");
    screen1->data.set_size(gua::math::vec2(aspect * 2.0, 2.0));
    screen1->translate(0, 0, 6.f);

    auto screen2 = graph.add_node<gua::node::ScreenNode>("/", "screen2");
    screen2->data.set_size(gua::math::vec2(aspect * 2.0, 2.0));
    screen2->translate(0, 0, 6.f);
    screen2->rotate(90, 0, 1, 0.f);

    auto pipe = std::make_shared<gua::PipelineDescription>();

    pipe->add_pass(std::make_shared<gua::TriMeshPassDescription>());
    pipe->add_pass(std::make_shared<gua::TexturedQuadPassDescription>());
    pipe->add_pass(std::make_shared<gua::BBoxPassDescription>());
    pipe->add_pass(std::make_shared<gua::LightVisibilityPassDescription>());
    pipe->add_pass(std::make_shared<gua::ResolvePassDescription>());
    pipe->add_pass(std::make_shared<gua::SSAAPassDescription>());
    pipe->add_pass(std::make_shared<gua::DebugViewPassDescription>());

    auto cam1 = graph.add_node<gua::node::CameraNode>("/screen1", "cam1");
    cam1->translate(0.0, 0.0, 4.0);
    cam1->config.set_output_window_name("window1");
    cam1->config.set_screen_path("/screen1");
    cam1->config.set_scene_graph_name("main_scenegraph");
    cam1->config.set_resolution(resolution);
    cam1->config.set_view_id(1);
    cam1->set_pipeline_description(pipe);

    auto cam2 = graph.add_node<gua::node::CameraNode>("/screen1", "cam2");
    cam2->translate(0.0, 0, 7.5);
    cam2->config.set_output_window_name("window2");
    cam2->config.set_screen_path("/screen1");
    cam2->config.set_scene_graph_name("main_scenegraph");
    cam2->config.set_resolution(resolution);
    cam2->config.set_view_id(2);
    cam2->set_pipeline_description(pipe);

    auto cam3 = graph.add_node<gua::node::CameraNode>("/screen2", "cam3");
    cam3->translate(0.0, 0.5, 16.5);
    cam3->config.set_output_window_name("window3");
    cam3->config.set_screen_path("/screen2");
    cam3->config.set_scene_graph_name("main_scenegraph");
    cam3->config.set_resolution(resolution);
    cam3->config.set_view_id(2);
    cam3->set_pipeline_description(pipe);

    auto cam4 = graph.add_node<gua::node::CameraNode>("/screen2", "cam4");
    cam4->translate(0.0, -0.5, 8.5);
    cam4->config.set_output_window_name("window4");
    cam4->config.set_screen_path("/screen2");
    cam4->config.set_scene_graph_name("main_scenegraph");
    cam4->config.set_resolution(resolution);
    cam4->set_pipeline_description(pipe);

    auto pointlight = graph.add_node<gua::node::LightNode>("/", "pointlight");
    pointlight->data.set_type(gua::node::LightNode::Type::POINT);
    pointlight->scale(30.0f);
    pointlight->rotate(-90, 1, 0, 0);
    pointlight->translate(1.0, 18.0, 1.0);

    // pointlight->data.set_shadow_map_size(1024);
    // pointlight->data.set_shadow_offset(0.005f);
    // pointlight->data.set_enable_shadows(true);
    pointlight->data.set_falloff(0.1f);
    pointlight->data.set_color({1.0f, 1.0f, 1.0f});
    pointlight->data.set_enable_specular_shading(true);
    pointlight->data.set_enable_diffuse_shading(true);

    auto add_window = [](std::string const& window_name, std::shared_ptr<gua::node::CameraNode> const& cam_node) {
        auto window = std::make_shared<gua::GlfwWindow>();
        gua::WindowDatabase::instance()->add(window_name, window);
        set_window_default(window, cam_node->config.get_resolution());
        cam_node->config.set_output_window_name(window_name);
    };

    add_window("window1", cam1);

    gua::Renderer renderer;

    // transform teapot
    auto bbox = teapot_geode->get_bounding_box();
    teapot->translate(-bbox.center());
    teapot->scale(5.0f / std::sqrt(bbox.size(0) * bbox.size(0) + bbox.size(1) * bbox.size(1) + bbox.size(2) * bbox.size(2)));
    teapot->translate(gua::math::vec3{-2.0f, -1.5f, -4.0f});

    // tranform nurbs
    // bbox = nurbs_geode->get_bounding_box();
    // nurbs->translate(-bbox.center());
    // nurbs->scale(5.0f / std::sqrt(bbox.size(0)*bbox.size(0) +
    //   bbox.size(1)*bbox.size(1) +
    //   bbox.size(2)*bbox.size(2)));
    // nurbs->rotate(-90, 1, 0, 0);
    // nurbs->translate(gua::math::vec3{ 3.0f, -1.5f, -4.0f });
    // nurbs_geode->translate(0.0f, 0.0f, 0.0f);

    // transform plate
    plate->scale(0.07);
    plate->translate(0.0f, -4.0f, -4.0f);

    float time_value = 0;

    // application loop
    std::size_t cnt = 0;

    while(true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // scm::math::mat4f ident;
        // scm::math::set_identity(ident);
        // video->set_transform(ident);

        // video->scale(2.0f + std::sin(time_value));
        // video->rotate(10.0f*time_value, 0, 1, 0);
        // video->translate(0, -2.0, -2.0);

        // time_value += 0.01f;

        teapot_geode->rotate(0.3, 0, 1, 0);
        // video_geode->rotate(0.1, 0, 1, 0);
        // nurbs_geode->rotate(0.3, 0, 0, 1);

        ++cnt;

        if(cnt == 1)
        {
            add_window("window2", cam2);
        }
        if(cnt == 2)
        {
            add_window("window3", cam3);
        }
        if(cnt == 3)
        {
            add_window("window4", cam4);
        }

        // GLFW only allows event processing from main thread
        if(gua::WindowDatabase::instance()->lookup("window1"))
        {
            gua::WindowDatabase::instance()->lookup("window1")->process_events();
        }

        if(gua::WindowDatabase::instance()->lookup("window2"))
        {
            gua::WindowDatabase::instance()->lookup("window2")->process_events();
        }

        if(gua::WindowDatabase::instance()->lookup("window3"))
        {
            gua::WindowDatabase::instance()->lookup("window3")->process_events();
        }

        if(gua::WindowDatabase::instance()->lookup("window4"))
        {
            gua::WindowDatabase::instance()->lookup("window4")->process_events();
        }

        plate->translate(-plate->get_bounding_box().center());
        plate->rotate(0.04f, 0, 1, 0);
        plate->translate(plate->get_bounding_box().center());

        renderer.queue_draw({&graph});
    }

    return 0;
}
