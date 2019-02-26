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
#include <gua/renderer/TriMeshLoader.hpp>

int main(int argc, char** argv)
{
    // initialize guacamole
    gua::init(argc, argv);

    // setup scene
    gua::SceneGraph graph("main_scenegraph");

    gua::TriMeshLoader loader;
    auto teapot(loader.create_geometry_from_file("teapot", "data/objects/teapot.obj", gua::Material(), gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));
    graph.add_node("/", teapot);

    teapot->get_tags().add_tags({"red"});

    auto teapot1(loader.create_geometry_from_file("teapot", "data/objects/teapot.obj", gua::Material(), gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));
    teapot1->get_tags().add_tags({"green"});

    auto t1_trans = graph.add_node<gua::node::TransformNode>("/", "t1_trans");
    t1_trans->translate(-1.0, 0.0, 0.0);
    graph.add_node("/t1_trans", teapot1);

    auto teapot2(loader.create_geometry_from_file("teapot", "data/objects/teapot.obj", gua::Material(), gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));
    teapot2->get_tags().add_tags({"blue"});

    auto t2_trans = graph.add_node<gua::node::TransformNode>("/", "t2_trans");
    t2_trans->translate(1.0, 0.0, 0.0);
    graph.add_node("/t2_trans", teapot2);

    gua::Camera cam("/screen/eye", "/screen/eye", "/screen", "/screen", "main_scenegraph");

    // cam.render_mask.whitelist.add_tags({"green", "red"});
    // cam.render_mask.blacklist.add_tags({"red"});

    auto light = graph.add_node<gua::node::LightNode>("/", "light");
    light->data.set_type(gua::node::LightNode::Type::POINT);
    light->scale(5.f);
    light->translate(0, 1.f, 1.f);

    auto screen = graph.add_node<gua::node::ScreenNode>("/", "screen");
    screen->data.set_size(gua::math::vec2(1.6f, 1.2f));

    screen->translate(0.0, 0.0, 2.0);

    auto eye = graph.add_node<gua::node::TransformNode>("/screen", "eye");
    eye->translate(0, 0, 1.5);

    auto pipe = new gua::Pipeline();
    pipe->config.set_camera(cam);
    pipe->config.set_resolution(gua::math::vec2ui(1920, 1080));

    pipe->add_pass<gua::GeometryPass>();
    pipe->add_pass<gua::LightingPass>();
    pipe->add_pass<gua::BackgroundPass>();
    // pipe->add_pass<gua::SSAOPass>().set_radius(10.f).set_intensity(0.5f);

    auto window(new gua::GlfwWindow());
    // pipe->set_window(window);
    gua::Renderer renderer({pipe});

    window->on_resize.connect([&](gua::math::vec2ui const& new_size) {
        window->config.set_left_resolution(new_size);
        pipe->config.set_resolution(new_size);
        screen->data.set_size(gua::math::vec2(0.002 * new_size.x, 0.002 * new_size.y));
    });

#if WIN32
    window->config.set_display_name("\\\\.\\DISPLAY1");
#else
    window->config.set_display_name(":0.0");
#endif

    // application loop
    gua::events::MainLoop loop;
    gua::events::Ticker ticker(loop, 1.0 / 60.0);

    ticker.on_tick.connect([&]() {
        teapot->rotate(0.1, 0, 1, 0);
        teapot1->rotate(0.1, 0, 1, 0);
        teapot2->rotate(0.1, 0, 1, 0);

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
