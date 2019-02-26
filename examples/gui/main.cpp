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
#include <gua/utils/Trackball.hpp>
#include <gua/gui.hpp>

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

    auto load_mat = [](std::string const& file) {
        auto desc(std::make_shared<gua::MaterialShaderDescription>());
        desc->load_from_file(file);
        auto shader(std::make_shared<gua::MaterialShader>(file, desc));
        gua::MaterialShaderDatabase::instance()->add(shader);
        return shader->make_new_material();
    };

    auto mat1(load_mat("data/materials/SimpleMaterial.gmd"));
    mat1->set_uniform("tex", std::string("google"));

    auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");

    gua::TriMeshLoader loader;
    auto monkey(
        loader.create_geometry_from_file("monkey", "data/objects/monkey.obj", mat1, gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::MAKE_PICKABLE | gua::TriMeshLoader::NORMALIZE_SCALE));
    graph.add_node("/transform", monkey);

    gua::math::vec2 gui_size(1024.f, 1024.f);

    auto gui = std::make_shared<gua::GuiResource>();
    gui->init("google", "https://www.google.com", gua::math::vec2(1024.f, 1024.f));

    gua::math::vec2 fps_size(170.f, 55.f);

    auto fps = std::make_shared<gua::GuiResource>();
    fps->init("fps", "asset://gua/data/html/fps.html", fps_size);

    auto fps_quad = std::make_shared<gua::node::TexturedScreenSpaceQuadNode>("fps_quad");
    fps_quad->data.texture() = "fps";
    fps_quad->data.size() = fps_size;
    fps_quad->data.anchor() = gua::math::vec2(1.f, 1.f);

    graph.add_node("/", fps_quad);

    gua::math::vec2 address_bar_size(340.f, 55.f);

    auto address_bar = std::make_shared<gua::GuiResource>();
    address_bar->init("address_bar", "asset://gua/data/html/address_bar.html", address_bar_size);

    auto address_bar_quad = std::make_shared<gua::node::TexturedScreenSpaceQuadNode>("address_bar_quad");
    address_bar_quad->data.texture() = "address_bar";
    address_bar_quad->data.size() = address_bar_size;
    address_bar_quad->data.anchor() = gua::math::vec2(-1.f, 1.f);

    graph.add_node("/transform/monkey", address_bar_quad);

    address_bar->on_loaded.connect([address_bar]() {
        address_bar->add_javascript_callback("update_address");
        address_bar->add_javascript_callback("address_back");
        address_bar->add_javascript_callback("address_forward");
    });

    address_bar->on_javascript_callback.connect([gui](std::string const& callback, std::vector<std::string> const& params) {
        if(callback == "update_address")
        {
            gui->set_url(params[0]);
        }
        else if(callback == "address_back")
        {
            gui->go_back();
        }
        else if(callback == "address_forward")
        {
            gui->go_forward();
        }
    });

    auto light2 = graph.add_node<gua::node::LightNode>("/", "light2");
    light2->data.set_type(gua::node::LightNode::Type::SUN);
    light2->data.set_brightness(2.f);
    light2->data.set_shadow_cascaded_splits({0.1f, 1.f, 2.f, 5.f});
    light2->data.set_shadow_near_clipping_in_sun_direction(10.0f);
    light2->data.set_shadow_far_clipping_in_sun_direction(10.0f);
    light2->data.set_max_shadow_dist(10.0f);
    light2->data.set_enable_shadows(true);
    light2->data.set_shadow_map_size(1024);
    light2->rotate(-45, 1, 1, 0);

    auto screen = graph.add_node<gua::node::ScreenNode>("/", "screen");
    screen->data.set_size(gua::math::vec2(1.92f * 0.25, 1.08f * 0.25));
    screen->translate(0, 0, 2.0);

    // add mouse interaction
    gua::utils::Trackball trackball(0.01, 0.002, 0.2);

    // setup rendering pipeline and window
    auto resolution = gua::math::vec2ui(1920, 1080);

    std::shared_ptr<gua::GuiResource> focused_element;

    auto camera = screen->add_child<gua::node::CameraNode>("cam");
    camera->translate(0, 0, 0.5);
    camera->config.set_resolution(resolution);
    camera->config.set_screen_path("/screen");
    camera->config.set_scene_graph_name("main_scenegraph");
    camera->config.set_output_window_name("main_window");

    auto window = std::make_shared<gua::GlfwWindow>();
    gua::WindowDatabase::instance()->add("main_window", window);
    window->config.set_enable_vsync(false);
    window->config.set_size(resolution);
    window->config.set_resolution(resolution);
    window->on_char.connect([&](unsigned c) {
        if(focused_element)
        {
            focused_element->inject_char_event(c);
        }
    });
    window->on_key_press.connect([&](int key, int scancode, int action, int mods) {
        if(focused_element)
        {
            focused_element->inject_keyboard_event(gua::Key(key), scancode, action, mods);
        }
    });
    window->on_button_press.connect([&](int key, int action, int mods) {
        if(focused_element)
        {
            focused_element->inject_mouse_button(gua::Button(key), action, mods);
        }
    });
    window->on_scroll.connect([&](gua::math::vec2 const& dir) {
        if(focused_element)
        {
            focused_element->inject_mouse_wheel(dir);
        }
    });
    window->on_resize.connect([&](gua::math::vec2ui const& new_size) {
        window->config.set_resolution(new_size);
        camera->config.set_resolution(new_size);
        screen->data.set_size(gua::math::vec2(0.001 * new_size.x, 0.001 * new_size.y));
    });
    window->on_move_cursor.connect([&](gua::math::vec2 const& pos) {
        gua::math::vec2 hit_pos;

        if(address_bar_quad->pixel_to_texcoords(pos, resolution, hit_pos))
        {
            address_bar->inject_mouse_position_relative(hit_pos);
            focused_element = address_bar;
        }
        else
        {
            auto screen_space_pos(pos / resolution - 0.5);

            gua::math::vec3 origin(screen->get_scaled_world_transform() * gua::math::vec3(screen_space_pos.x, screen_space_pos.y, 0));
            gua::math::vec3 direction(origin - camera->get_world_position());

            gua::Ray ray(origin, direction * 100, 1);

            auto result = graph.ray_test(ray, gua::PickResult::PICK_ONLY_FIRST_OBJECT | gua::PickResult::PICK_ONLY_FIRST_FACE | gua::PickResult::GET_TEXTURE_COORDS);
            if(result.size() > 0)
            {
                for(auto const& r : result)
                {
                    focused_element = gui;
                    focused_element->inject_mouse_position_relative(r.texture_coords);
                }
            }
            else
            {
                focused_element = nullptr;
                trackball.motion(pos.x, pos.y);
            }
        }
    });
    window->on_button_press.connect(std::bind(mouse_button, std::ref(trackball), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    window->open();

    gua::Renderer renderer;

    // application loop
    gua::events::MainLoop loop;
    gua::events::Ticker ticker(loop, 1.0 / 500.0);

    ticker.on_tick.connect([&]() {
        std::stringstream sstr;
        sstr.precision(1);
        sstr.setf(std::ios::fixed, std::ios::floatfield);
        sstr << "FPS: " << renderer.get_application_fps() << " / " << window->get_rendering_fps();
        fps->call_javascript("set_fps_text", sstr.str());

        // ray->rotate(1, 0, 1, 0);
        gua::Interface::instance()->update();
        // apply trackball matrix to object
        auto modelmatrix = scm::math::make_translation(trackball.shiftx(), trackball.shifty(), trackball.distance()) * trackball.rotation();
        transform->set_transform(modelmatrix);

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
