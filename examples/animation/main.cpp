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
#include <gua/skelanim.hpp>
#include <gua/renderer/TriMeshLoader.hpp>
#include <gua/utils/Trackball.hpp>

#include <gua/renderer/BBoxPass.hpp>
#include <gua/renderer/TexturedQuadPass.hpp>
#include <gua/renderer/TexturedScreenSpaceQuadPass.hpp>
#include <gua/renderer/DebugViewPass.hpp>
#include <gua/renderer/PBSMaterialFactory.hpp>
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

    auto mat1(gua::PBSMaterialFactory::create_material(static_cast<gua::PBSMaterialFactory::Capabilities>(gua::PBSMaterialFactory::ALL)));
    // mat1->set_uniform("Color", gua::math::vec3f(1.0f, 1.0f, 1.0f));
    mat1->set_uniform("Metalness", 0.0f);
    mat1->set_uniform("Roughness", 0.7f);
    mat1->set_uniform("Emissivity", 0.0f);
    gua::SkeletalAnimationLoader loader;
    gua::TriMeshLoader tri_loader;

    auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");
    // load ground
#if WIN32
    auto ground_node(tri_loader.create_geometry_from_file("fbx", "data/objects/plane.obj", mat1, gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));
#else
    auto ground_node(
        tri_loader.create_geometry_from_file("fbx", "/opt/avango/avango-applications/data/objects/plane.obj", mat1, gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));
#endif
    ground_node->set_transform(scm::math::make_translation(0.0, -0.48, 0.0) * scm::math::make_scale(5.0, 5.0, 5.0) * ground_node->get_transform());
    // load character

#if WIN32
    auto character_node(loader.create_geometry_from_file("fbx", "data/objects/HeroTPP.FBX", mat1, gua::SkeletalAnimationLoader::NORMALIZE_POSITION | gua::SkeletalAnimationLoader::NORMALIZE_SCALE));
    character_node->set_transform(scm::math::make_rotation(-90.0, 1.0, 0.0, 0.0) * character_node->get_transform());
    character_node->add_animations("data/objects/Assets/Walk.FBX", "walk");
#else
    auto character_node(
        loader.create_geometry_from_file("fbx", "/opt/project_animation/Assets/HeroTPP.FBX", mat1, gua::SkeletalAnimationLoader::NORMALIZE_POSITION | gua::SkeletalAnimationLoader::NORMALIZE_SCALE));
    character_node->set_transform(scm::math::make_rotation(-90.0, 1.0, 0.0, 0.0) * character_node->get_transform());
    character_node->add_animations("/opt/project_animation/Assets/Walk.FBX", "walk");
#endif

    character_node->set_animation_1("walk");
    character_node->add_animations("/opt/project_animation/Assets/Run.FBX", "run");
    character_node->set_animation_2("run");

    // play only anim nr. 1
    character_node->set_blend_factor(0.5);

    graph.add_node("/transform", ground_node);
    graph.add_node("/transform", character_node);

    auto pointLight = graph.add_node<gua::node::LightNode>("/", "pointLight");
    pointLight->data.set_type(gua::node::LightNode::Type::SUN);
    pointLight->data.color = gua::utils::Color3f(1.0f, 1.0f, 1.0f);
    pointLight->data.brightness = 1.0f; // lm
    pointLight->data.brightness = 1.0f; // lm
    pointLight->data.enable_shadows = true;
    // pointLight->scale(9.f);
    // pointLight->translate(-2.f, 3.f, 5.f);
    pointLight->rotate(-30.0f, 1.0f, 0.0f, 0.0f);

    auto screen = graph.add_node<gua::node::ScreenNode>("/", "screen");
    screen->data.set_size(gua::math::vec2(1.92f, 1.08f));
    screen->translate(0, 0, 1.0);

    // add mouse interaction
    gua::utils::Trackball trackball(0.01, 0.002, 0.2);

    // setup rendering pipeline and window
    auto resolution = gua::math::vec2ui(1920, 1080);

    auto camera = graph.add_node<gua::node::CameraNode>("/screen", "cam");
    camera->translate(0, 0, 2.0);
    camera->config.set_resolution(resolution);
    camera->config.set_screen_path("/screen");
    camera->config.set_scene_graph_name("main_scenegraph");
    camera->config.set_output_window_name("main_window");
    camera->config.set_enable_stereo(false);
    camera->config.set_near_clip(0.001f);
    camera->config.set_far_clip(10.0f);

    auto pipe = std::make_shared<gua::PipelineDescription>();
    pipe->add_pass(std::make_shared<gua::TriMeshPassDescription>());
    pipe->add_pass(std::make_shared<gua::SkeletalAnimationPassDescription>());
    pipe->add_pass(std::make_shared<gua::TexturedQuadPassDescription>());
    pipe->add_pass(std::make_shared<gua::LightVisibilityPassDescription>());
    pipe->add_pass(std::make_shared<gua::BBoxPassDescription>());
    pipe->add_pass(std::make_shared<gua::ResolvePassDescription>());
    pipe->add_pass(std::make_shared<gua::TexturedScreenSpaceQuadPassDescription>());
    // pipe->add_pass(std::make_shared<gua::DebugViewPassDescription>());
    pipe->get_resolve_pass()->background_color(gua::utils::Color3f(0.5f, 0.7f, 1.0f));
    camera->set_pipeline_description(pipe);

    auto window = std::make_shared<gua::GlfwWindow>();
    gua::WindowDatabase::instance()->add("main_window", window);
    window->config.set_enable_vsync(false);
    window->config.set_size(resolution);
    window->config.set_resolution(resolution);
    window->config.set_stereo_mode(gua::StereoMode::MONO);
    window->on_resize.connect([&](gua::math::vec2ui const& new_size) {
        window->config.set_resolution(new_size);
        camera->config.set_resolution(new_size);
        screen->data.set_size(gua::math::vec2(0.001 * new_size.x, 0.001 * new_size.y));
    });
    window->on_move_cursor.connect([&](gua::math::vec2 const& pos) { trackball.motion(pos.x, pos.y); });
    window->on_button_press.connect(std::bind(mouse_button, std::ref(trackball), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    window->open();

    gua::Renderer renderer;

    float i = 0;
    // application loop
    gua::events::MainLoop loop;
    gua::events::Ticker ticker(loop, 1.0 / 500.0);
    ticker.on_tick.connect([&]() {
        // apply trackball matrix to object
        auto modelmatrix = scm::math::make_translation(trackball.shiftx(), trackball.shifty(), trackball.distance()) * trackball.rotation();
        transform->set_transform(modelmatrix);

        // set time variable for animation
        i += 1.0 / 600.0;
        if(i > 1)
            i = 0;
        character_node->set_time_1(i);
        character_node->set_time_2(i);

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
