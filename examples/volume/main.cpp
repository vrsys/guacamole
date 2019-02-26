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
#include <gua/volume.hpp>
#include <gua/gui.hpp>
#include <gua/utils/Trackball.hpp>

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

    auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");

    gua::VolumeLoader vloader;
    auto volume(vloader.create_volume_from_file("volume", "/opt/gua_vrgeo_2013/data/objects/head_w256_h256_d225_c1_b8.raw", 0));
    volume->translate(-0.5, -0.5, -0.5);
    graph.add_node("/transform", volume);

    auto vnode = std::dynamic_pointer_cast<gua::node::VolumeNode>(volume);
    vnode->data.alpha_transfer().add_stop(0.5f, 0.1f);

    auto transfer_widget = std::make_shared<gua::GuiResource>();
    transfer_widget->init("transfer_widget", "asset://gua/data/html/transfer_widget.html", gua::math::vec2(500, 300.f));
    auto transfer_widget_quad = graph.add_node<gua::node::TexturedScreenSpaceQuadNode>("/", "transfer_widget_quad");
    transfer_widget_quad->data.texture() = "transfer_widget";
    transfer_widget_quad->data.size() = gua::math::vec2(500, 300.f);
    transfer_widget_quad->data.anchor() = gua::math::vec2(-1.f, 1.f);

    transfer_widget->on_loaded.connect([transfer_widget]() { transfer_widget->add_javascript_callback("set_transfer_function"); });
    transfer_widget->on_javascript_callback.connect([vnode](std::string const&, std::vector<std::string> const& params) {
        vnode->data.alpha_transfer().clear();
        vnode->data.color_transfer().clear();
        std::stringstream sstr(params[0]);

        while(sstr)
        {
            gua::math::vec4 color;
            float pos;

            sstr >> pos >> color;
            vnode->data.alpha_transfer().add_stop(pos, color.w);
            vnode->data.color_transfer().add_stop(pos, gua::math::vec3f(color.x, color.y, color.z));
        }
    });

    std::shared_ptr<gua::GuiResource> focused_element;

    auto screen = graph.add_node<gua::node::ScreenNode>("/", "screen");
    screen->data.set_size(gua::math::vec2(1.92f * 0.25, 1.08f * 0.25));
    screen->translate(0, 0, 2.0);

    // add mouse interaction
    gua::utils::Trackball trackball(0.01, 0.002, 0.2);

    // setup rendering pipeline and window
    auto resolution = gua::math::vec2ui(1920, 1080);

    auto camera = screen->add_child<gua::node::CameraNode>("cam");
    camera->translate(0, 0, 0.5);
    camera->config.set_resolution(resolution);
    camera->config.set_screen_path("/screen");
    camera->config.set_scene_graph_name("main_scenegraph");
    camera->config.set_output_window_name("main_window");
    camera->get_pipeline_description()->add_pass(std::make_shared<gua::VolumePassDescription>());

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
    window->on_move_cursor.connect([&](gua::math::vec2 const& pos) {
        gua::math::vec2 hit_pos;

        if(transfer_widget_quad->pixel_to_texcoords(pos, resolution, hit_pos))
        {
            transfer_widget->inject_mouse_position_relative(hit_pos);
            focused_element = transfer_widget;
        }
        else
        {
            focused_element = nullptr;
            trackball.motion(pos.x, pos.y);
        }
    });
    window->on_button_press.connect([&](int key, int action, int mods) {
        if(focused_element)
        {
            focused_element->inject_mouse_button(gua::Button(key), action, mods);
        }
        else
        {
            mouse_button(trackball, key, action, mods);
        }
    });

    window->open();

    gua::Renderer renderer;

    // application loop
    gua::events::MainLoop loop;
    gua::events::Ticker ticker(loop, 1.0 / 500.0);

    ticker.on_tick.connect([&]() {
        // apply trackball matrix to object
        auto modelmatrix = scm::math::make_translation(trackball.shiftx(), trackball.shifty(), trackball.distance()) * trackball.rotation();
        transform->set_transform(modelmatrix);

        gua::Interface::instance()->update();

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
