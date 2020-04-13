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
#include <gua/utils/Trackball.hpp>

#include <gua/renderer/DebugViewPass.hpp>

#define COUNT 5

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

int main(int argc, char** argv)
{
    // initialize guacamole
    gua::init(argc, argv);

    // setup scene
    gua::SceneGraph graph("main_scenegraph");

    gua::TriMeshLoader loader;
    auto add_mesh = [&](int x, int y, int z) {
        auto t = graph.add_node<gua::node::TransformNode>("/", "mesh_" + std::to_string(x) + "_" + std::to_string(y));
        t->scale(0.5);
        t->translate((x - COUNT * 0.5 + 0.5) / 2, (y - COUNT * 0.5 + 0.5) / 2, (z - COUNT * 0.5 + 0.5) / 2);

        auto mesh(loader.create_geometry_from_file("teapot", "data/objects/teapot.obj", gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));
        t->add_child(mesh);
    };

    for(int x(0); x < COUNT; ++x)
    {
        for(int y(0); y < COUNT; ++y)
        {
            for(int z(0); z < COUNT; ++z)
            {
                add_mesh(x, y, z);
            }
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

    auto pipeline_description = camera->get_pipeline_description();
    pipeline_description->get_resolve_pass()->tone_mapping_exposure(0.8f);
    pipeline_description->add_pass(std::make_shared<gua::DebugViewPassDescription>());
    camera->set_pipeline_description(pipeline_description);

    auto plane_transform = graph.add_node<gua::node::TransformNode>("/", "plane_transform");

    auto add_clipping_plane = [&](gua::math::vec3 const& pos, gua::math::vec4 const& rot) {
        auto clipping_plane = graph.add_node<gua::node::ClippingPlaneNode>("/plane_transform", "clipping_plane");
        clipping_plane->rotate(rot.x, rot.y, rot.z, rot.w);
        clipping_plane->translate(pos);
        auto plane(loader.create_geometry_from_file("plane", "data/objects/plane.obj", gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));
        plane->rotate(-90, 1, 0, 0);
        // clipping_plane->add_child(plane);
    };

    add_clipping_plane(gua::math::vec3(0, 0, -0.5), gua::math::vec4(180, 1, 0, 0));
    add_clipping_plane(gua::math::vec3(0, 0, 0.5), gua::math::vec4(0, 0, 0, 0));
    add_clipping_plane(gua::math::vec3(0, 0.5, 0), gua::math::vec4(-90, 1, 0, 0));
    add_clipping_plane(gua::math::vec3(0, -0.5, 0), gua::math::vec4(90, 1, 0, 0));
    add_clipping_plane(gua::math::vec3(-0.5, 0, 0), gua::math::vec4(-90, 0, 1, 0));
    add_clipping_plane(gua::math::vec3(0.5, 0, 0), gua::math::vec4(90, 0, 1, 0));

    // auto clipping_plane_2 = graph.add_node<gua::node::ClippingPlaneNode>("/plane_transform", "clipping_plane_2");
    // clipping_plane_2->rotate(90, 1, 0, 0);
    // clipping_plane_2->translate(0, 0.5, 0);

    // add mouse interaction
    gua::utils::Trackball trackball(0.01, 0.002, 0.2);

    auto window = std::make_shared<gua::GlfwWindow>();
    gua::WindowDatabase::instance()->add("main_window", window);
    window->config.set_enable_vsync(true);
    window->config.set_size(resolution);
    window->config.set_resolution(resolution);

    window->on_resize.connect([&](gua::math::vec2ui const& new_size) {
        window->config.set_resolution(new_size);
        camera->config.set_resolution(new_size);
        screen->data.set_size(gua::math::vec2(0.001 * new_size.x, 0.001 * new_size.y));
    });

    window->on_move_cursor.connect([&](gua::math::vec2 const& pos) { trackball.motion(pos.x, pos.y); });
    window->on_button_press.connect(std::bind(mouse_button, std::ref(trackball), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    window->open();

    gua::Renderer renderer;

    // application loop
    gua::events::MainLoop loop;
    gua::events::Ticker ticker(loop, 1.0 / 500.0);

    ticker.on_tick.connect([&]() {
        // apply trackball matrix to object
        gua::math::mat4 modelmatrix = scm::math::make_translation(gua::math::float_t(trackball.shiftx()), gua::math::float_t(trackball.shifty()), gua::math::float_t(trackball.distance())) *
                                      gua::math::mat4(trackball.rotation());

        plane_transform->set_transform(modelmatrix);

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
