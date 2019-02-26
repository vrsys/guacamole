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
#include <gua/renderer/TriMeshLoader.hpp>
#include <gua/renderer/DebugViewPass.hpp>
#include <gua/renderer/ToneMappingPass.hpp>
#include <gua/utils/Trackball.hpp>
#include <gua/virtual_texturing/VTBackend.hpp>

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

    auto load_mat = [](std::string const& file) {
        gua::MaterialShaderDescription desc;
        desc.load_from_file(file);
        auto shader(std::make_shared<gua::MaterialShader>(file, std::make_shared<gua::MaterialShaderDescription>(desc)));
        gua::MaterialShaderDatabase::instance()->add(shader);
        return shader->make_new_material();
    };

    // setup scene
    gua::SceneGraph graph("main_scenegraph");

    gua::TriMeshLoader loader;

    auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");

    // load material and create geometry
    // !! take care, this is a modified version of the projective texturing material from the other example
    auto projective_material(load_mat("data/materials/Projective_Virtual_Texture_Material.gmd"));

    // virtual texture path
    std::string vt_texture_path = "data/textures/onepointfive_texture_2048_w2048_h2048.atlas";

    ////// just act as if the vt would be a normal texture and upload it as projective texture
    projective_material->set_uniform("projective_texture", vt_texture_path);

    // turn on virtual texturing for the corresponding material, this helps us keep the projective texturing shader minimal
    projective_material->set_enable_virtual_texturing(true);

    auto monkey(
        loader.create_geometry_from_file("receiving_object", "data/objects/vive_controller.obj", projective_material, gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));

    monkey->rotate(90.0, 1.0, 0.0, 0.0);
    monkey->scale(3);
    monkey->translate(0.0, 0.0, -1.0);
    graph.add_node("/transform", monkey);

    auto light2 = graph.add_node<gua::node::LightNode>("/", "light2");
    light2->data.set_type(gua::node::LightNode::Type::POINT);
    light2->data.brightness = 150.0f;
    light2->scale(12.f);
    light2->translate(-3.f, 5.f, 5.f);

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

    auto pipe = std::make_shared<gua::PipelineDescription>();
    pipe->add_pass(std::make_shared<gua::TriMeshPassDescription>());
    pipe->add_pass(std::make_shared<gua::LightVisibilityPassDescription>());
    auto resolve_pass = std::make_shared<gua::ResolvePassDescription>();
    resolve_pass->background_mode(gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE);
    resolve_pass->tone_mapping_exposure(1.0f);
    pipe->add_pass(resolve_pass);
    camera->set_pipeline_description(pipe);
    camera->get_pipeline_description()->add_pass(std::make_shared<gua::DebugViewPassDescription>());

    // projector transform node, screen and transform node
    auto projector_transform = graph.add_node<gua::node::TransformNode>("/", "projector_transform");
    projector_transform->translate(0.7, 0.0, 0.7);
    projector_transform->rotate(45.0, 0.0, 1.0, 0.0);
    graph.add_node("/", projector_transform);

    auto projector_screen = graph.add_node<gua::node::ScreenNode>("/projector_transform", "projector_screen");
    projector_screen->data.set_size(gua::math::vec2(0.5f, 0.5f));
    projector_screen->translate(0.0, 0.0, -1.0);
    graph.add_node("/projector_transform", projector_screen);

    auto projector_geometry(loader.create_geometry_from_file("projector_geometry", "data/objects/projector.obj", gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::LOAD_MATERIALS));
    projector_geometry->scale(0.1);
    graph.add_node("/projector_transform", projector_geometry);

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

    auto vt_backend = &gua::VTBackend::get_instance();
    vt_backend->add_camera(camera);
    vt_backend->start_backend();

    gua::Renderer renderer;

    // application loop
    gua::events::MainLoop loop;
    gua::events::Ticker ticker(loop, 1.0 / 500.0);

    ticker.on_tick.connect([&]() {
        // apply trackball matrix to object
        gua::math::mat4 modelmatrix = scm::math::make_translation(gua::math::float_t(trackball.shiftx()), gua::math::float_t(trackball.shifty()), gua::math::float_t(trackball.distance())) *
                                      gua::math::mat4(trackball.rotation());

        projector_transform->set_transform(modelmatrix);

        // use the guacamole frustum to calculate a view mat and projection mat for the projection
        auto projection_frustum = gua::Frustum::perspective(projector_transform->get_world_transform(), projector_screen->get_scaled_world_transform(), 0.1f, 1000.0f);

        auto projection_mat = projection_frustum.get_projection();
        auto view_mat = projection_frustum.get_view();

        // set these matrices as uniforms for the projection material
        projective_material->set_uniform("projective_texture_matrix", gua::math::mat4f(projection_mat * view_mat));
        projective_material->set_uniform("view_texture_matrix", gua::math::mat4f(view_mat));

        window->process_events();
        if(window->should_close())
        {
            renderer.stop();
            window->close();
            loop.stop();
            vt_backend->stop_backend();
        }
        else
        {
            renderer.queue_draw({&graph});
        }
    });

    loop.start();

    return 0;
}
