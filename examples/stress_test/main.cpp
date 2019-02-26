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

#define COUNT 5

int main(int argc, char** argv)
{
    // initialize guacamole
    gua::init(argc, argv);

    // setup scene
    gua::SceneGraph graph("main_scenegraph");

    auto desc(std::make_shared<gua::MaterialShaderDescription>());
    desc->load_from_file("data/materials/SimpleMaterial.gmd");

    auto shader(std::make_shared<gua::MaterialShader>("simple_mat", desc));
    gua::MaterialShaderDatabase::instance()->add(shader);
    auto mat(shader->make_new_material());

    gua::TriMeshLoader loader;
    auto add_oilrig = [&](int x, int y) {
        auto t = graph.add_node<gua::node::TransformNode>("/", "rig_" + std::to_string(x) + "_" + std::to_string(y));
        t->translate((x - COUNT * 0.5 + 0.5) / 1.5, (y - COUNT * 0.5 + 0.5) / 2, 0);

        auto rig(loader.create_geometry_from_file("rig",
                                                  //"I:/models/Paris/Paris2010_0.obj",
                                                  // "I:/models/Batman/Batman.obj",
                                                  "/opt/3d_models/OIL_RIG_GUACAMOLE/oilrig.obj",
                                                  mat,
                                                  gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE | gua::TriMeshLoader::LOAD_MATERIALS |
                                                      gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::OPTIMIZE_MATERIALS));
        t->add_child(rig);
    };

    for(int x(0); x < COUNT; ++x)
    {
        for(int y(0); y < COUNT; ++y)
        {
            add_oilrig(x, y);
        }
    }

    auto resolution = gua::math::vec2ui(1920, 1080);

    auto light = graph.add_node<gua::node::LightNode>("/", "light");
    light->data.set_type(gua::node::LightNode::Type::POINT);
    light->scale(4.4f);
    light->translate(1.f, 0.f, 2.f);

    auto screen = graph.add_node<gua::node::ScreenNode>("/", "screen");
    screen->data.set_size(gua::math::vec2(0.001 * resolution.x, 0.001 * resolution.y));
    screen->translate(0, 0, 1.0);

    auto camera = graph.add_node<gua::node::CameraNode>("/screen", "cam");
    camera->translate(0, 0, 2.0);
    camera->config.set_resolution(resolution);
    camera->config.set_screen_path("/screen");
    camera->config.set_scene_graph_name("main_scenegraph");
    camera->config.set_output_window_name("main_window");
    camera->config.set_enable_frustum_culling(false);
    camera->config.set_enable_frustum_culling(false);
    camera->get_pipeline_description()->get_pass_by_type<gua::ResolvePassDescription>()->tone_mapping_exposure(0.1f);

    auto window = std::make_shared<gua::GlfwWindow>();
    gua::WindowDatabase::instance()->add("main_window", window);
    window->config.set_enable_vsync(false);
    window->config.set_size(resolution);
    window->config.set_resolution(resolution);

    window->on_resize.connect([&](gua::math::vec2ui const& new_size) {
        window->config.set_resolution(new_size);
        camera->config.set_resolution(new_size);
        screen->data.set_size(gua::math::vec2(0.001 * new_size.x, 0.001 * new_size.y));
    });

    window->open();

    gua::Renderer renderer;

    // application loop
    gua::events::MainLoop loop;
    gua::events::Ticker ticker(loop, 1.0 / 500.0);

    ticker.on_tick.connect([&]() {
        for(int x(0); x < COUNT; ++x)
        {
            for(int y(0); y < COUNT; ++y)
            {
                graph["/rig_" + std::to_string(x) + "_" + std::to_string(y) + "/rig"]->rotate(0.1, 0, 1, 0);
            }
        }

        std::cout << window->get_rendering_fps() << " " << renderer.get_application_fps() << std::endl;

        window->process_events();
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
