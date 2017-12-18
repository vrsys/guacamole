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
#include <memory>

#include <gua/guacamole.hpp>
#include <gua/renderer/TriMeshLoader.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/utils/Trackball.hpp>

#include <gua/renderer/BBoxPass.hpp>
#include <gua/renderer/DebugViewPass.hpp>
#include <gua/renderer/PBSMaterialFactory.hpp>
#include <gua/renderer/TexturedQuadPass.hpp>
#include <gua/renderer/TexturedScreenSpaceQuadPass.hpp>
// forward mouse interaction to trackball
void mouse_button(gua::utils::Trackball &trackball, int mousebutton, int action, int mods)
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

int main(int argc, char **argv)
{
    gua::Logger::enable_debug = false;
    gua::Logger::enable_message = false;
    gua::Logger::enable_warning = false;
    gua::Logger::enable_error = true;

    gua::init(argc, argv);
    gua::SceneGraph graph("main_scenegraph");

    auto mat(gua::PBSMaterialFactory::create_material(gua::PBSMaterialFactory::ALL));
    mat->set_uniform("Metalness", 0.0f);
    mat->set_uniform("Roughness", 0.7f);
    mat->set_uniform("Emissivity", 0.0f);
    mat->set_show_back_faces(true);
    gua::TriMeshLoader tri_loader;

    auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");
    auto dae_node(tri_loader.create_geometry_from_file("ground", "/home/xaf/NRP/Models/hosta_potted_plant/meshes/plant_coarse.dae", mat, gua::TriMeshLoader::LOAD_MATERIALS));

    graph.add_node("/transform", dae_node);

    auto pointLight = graph.add_node<gua::node::LightNode>("/", "pointLight");
    pointLight->data.set_type(gua::node::LightNode::Type::SUN);
    pointLight->data.color = gua::utils::Color3f(1.0f, 1.0f, 1.0f);
    pointLight->data.brightness = 1.0f; // lm
    pointLight->data.brightness = 1.0f; // lm
    pointLight->data.enable_shadows = true;
    pointLight->rotate(-30.0f, 1.0f, 0.0f, 0.0f);

    auto screen = graph.add_node<gua::node::ScreenNode>("/", "screen");
    screen->data.set_size(gua::math::vec2(1.92f, 1.08f));
    screen->translate(0, 0, 1.0);

    gua::utils::Trackball trackball(0.01, 0.002, 0.2);

    auto resolution = gua::math::vec2ui(1920, 1080);

    auto camera = graph.add_node<gua::node::CameraNode>("/screen", "cam");
    camera->translate(0, 0, 2.0);
    camera->config.set_resolution(resolution);
    camera->config.set_screen_path("/screen");
    camera->config.set_scene_graph_name("main_scenegraph");
    camera->config.set_output_window_name("main_window");
    camera->config.set_enable_stereo(false);
    camera->config.set_near_clip(0.001f);
    camera->config.set_far_clip(100.0f);

    auto pipe = std::make_shared<gua::PipelineDescription>();
    pipe->add_pass(std::make_shared<gua::TriMeshPassDescription>());
    pipe->add_pass(std::make_shared<gua::LightVisibilityPassDescription>());
    pipe->add_pass(std::make_shared<gua::BBoxPassDescription>());
    pipe->add_pass(std::make_shared<gua::ResolvePassDescription>());
    pipe->get_resolve_pass()->background_color(gua::utils::Color3f(0.5f, 0.7f, 1.0f));
    camera->set_pipeline_description(pipe);

    auto window = std::make_shared<gua::GlfwWindow>();
    gua::WindowDatabase::instance()->add("main_window", window);
    window->config.set_enable_vsync(false);
    window->config.set_size(resolution);
    window->config.set_resolution(resolution);
    window->config.set_stereo_mode(gua::StereoMode::MONO);
    window->on_resize.connect([&](gua::math::vec2ui const &new_size) {
        window->config.set_resolution(new_size);
        camera->config.set_resolution(new_size);
        screen->data.set_size(gua::math::vec2(0.001 * new_size.x, 0.001 * new_size.y));
    });
    window->on_move_cursor.connect([&](gua::math::vec2 const &pos) { trackball.motion(pos.x, pos.y); });
    window->on_button_press.connect(std::bind(mouse_button, std::ref(trackball), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    window->open();

    gua::Renderer renderer;

    float i = 0;
    gua::events::MainLoop loop;
    gua::events::Ticker ticker(loop, 1.0 / 500.0);
    ticker.on_tick.connect([&]() {

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
