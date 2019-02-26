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
#include <algorithm>

#include <gua/guacamole.hpp>
#include <gua/renderer/TriMeshLoader.hpp>
#include <gua/renderer/DebugViewPass.hpp>
#include <gua/renderer/ToneMappingPass.hpp>
#include <gua/renderer/SSAAPass.hpp>
#include <gua/renderer/BBoxPass.hpp>
#include <gua/renderer/DebugViewPass.hpp>
#include <gua/renderer/LodLoader.hpp>
#include <gua/renderer/PLodPass.hpp>
#include <gua/renderer/MLodPass.hpp>
#include <gua/node/PLodNode.hpp>
#include <gua/node/MLodNode.hpp>
#include <scm/gl_util/manipulators/trackball_manipulator.h>

int main(int argc, char** argv)
{
    // init guacamole
    gua::init(argc, argv);

    // setup scenegraph
    gua::SceneGraph graph("main_scenegraph");

    auto load_mat = [](std::string const& file) {
        auto desc(std::make_shared<gua::MaterialShaderDescription>());
        desc->load_from_file(file);
        auto shader(std::make_shared<gua::MaterialShader>(file, desc));
        gua::MaterialShaderDatabase::instance()->add(shader);
        return shader->make_new_material();
    };

    // create a material which does the texturing using the following texture:
    std::string texture_file(
        "/mnt/pitoti/hallermann_scans/Scherkondetalbruecke2/Pfeiler_nochmal/3D_Modell_Pfeiler_Scherkodetal_midres/lowres/3D_Modell_Pfeiler_Scherkodetal_midres_single_textured.jpg");

    auto mat_textured(load_mat("data/materials/textured.gmd"));
    gua::TextureDatabase::instance()->load(texture_file.c_str());

    mat_textured->set_uniform("ColorMap", texture_file);

    // configure lod backend
    gua::LodLoader lod_loader;
    lod_loader.set_out_of_core_budget_in_mb(4096);
    lod_loader.set_render_budget_in_mb(1024);
    lod_loader.set_upload_budget_in_mb(30);

    auto scene_transform = graph.add_node<gua::node::TransformNode>("/", "transform");

    auto mlod_transform = graph.add_node<gua::node::TransformNode>("/transform", "mlod_transform");

    // load a sample mesh-based lod model
    std::string tri_mesh_file("/opt/3d_models/lamure/mlod/xyzrgb_dragon_7219k.bvh");
    // std::string tri_mesh_file("/mnt/pitoti/hallermann_scans/Scherkondetalbruecke2/Pfeiler_nochmal/3D_Modell_Pfeiler_Scherkodetal_midres/lowres/3D_Modell_Pfeiler_Scherkodetal_midres.bvh");

    auto mlod_node = lod_loader.load_lod_trimesh("tri_mesh", tri_mesh_file.c_str(), mat_textured, gua::LodLoader::NORMALIZE_POSITION | gua::LodLoader::NORMALIZE_SCALE
                                                 /* | gua::LodLoader::MAKE_PICKABLE*/
    );

    mlod_node->set_error_threshold(0.25);
    graph.add_node("/transform/mlod_transform", mlod_node);

    mlod_transform->translate(0.0, 0.0, 0.0);

    // create a lightsource
    auto light_transform = graph.add_node<gua::node::TransformNode>("/transform", "light_transform");
    auto light = graph.add_node<gua::node::LightNode>("/transform/light_transform", "light");
    light->data.set_type(gua::node::LightNode::Type::SPOT);
    light->data.set_enable_shadows(true);

    light->data.set_shadow_map_size(1920);
    light->data.set_shadow_offset(0.001f);
    light->data.set_softness(0.6f);
    light->data.set_shadow_far_clipping_in_sun_direction(2.0f);
    light->data.set_shadow_near_clipping_in_sun_direction(0.1f);

    light->data.brightness = 10.0f;
    light->scale(6.0f);
    light_transform->rotate(-90, 1.0, 0.0, 0.0);
    light_transform->translate(0.f, 3.f, 1.0f);

    auto screen = graph.add_node<gua::node::ScreenNode>("/", "screen");
    screen->data.set_size(gua::math::vec2(1.92f, 1.08f));
    screen->translate(0, 0, 1.0);

    // add mouse interaction
    scm::gl::trackball_manipulator trackball;
    trackball.transform_matrix() * scm::math::make_translation(0.01f, 0.002f, 0.2f);
    trackball.dolly(0.2f);
    float dolly_sens = 1.5f;
    gua::math::vec2 trackball_init_pos(0.f);
    gua::math::vec2 last_mouse_pos(0.f);
    int button_state = -1;

    // setup rendering pipeline and window
    // auto resolution = gua::math::vec2ui(3840, 2160);
    auto resolution = gua::math::vec2ui(1920, 1080);

    auto camera = graph.add_node<gua::node::CameraNode>("/screen", "cam");
    camera->translate(0.0f, 0, 2.5f);
    camera->config.set_resolution(resolution);

    // use close near plane to allow inspection of details
    camera->config.set_near_clip(0.01f);
    camera->config.set_far_clip(20.0f);
    camera->config.set_screen_path("/screen");
    camera->config.set_scene_graph_name("main_scenegraph");
    camera->config.set_output_window_name("main_window");
    // camera->config.set_enable_stereo(true);

    auto pipe = std::make_shared<gua::PipelineDescription>();
    pipe->add_pass(std::make_shared<gua::MLodPassDescription>());
    pipe->add_pass(std::make_shared<gua::BBoxPassDescription>());
    pipe->add_pass(std::make_shared<gua::LightVisibilityPassDescription>());
    pipe->add_pass(std::make_shared<gua::ResolvePassDescription>());
    pipe->add_pass(std::make_shared<gua::DebugViewPassDescription>());
    camera->set_pipeline_description(pipe);

    pipe->get_resolve_pass()->tone_mapping_exposure(1.f);
    pipe->get_resolve_pass()->tone_mapping_method(gua::ResolvePassDescription::ToneMappingMethod::UNCHARTED);

    pipe->get_resolve_pass()->background_mode(gua::ResolvePassDescription::BackgroundMode::SKYMAP_TEXTURE);
    pipe->get_resolve_pass()->background_texture("data/textures/envlightmap.jpg");

    auto& p_desc = camera->get_pipeline_description();
    p_desc->set_enable_abuffer(true);
    p_desc->set_abuffer_size(1500);

    // init window and window behaviour
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

    // trackball controls
    bool drag_mode = false;
    window->on_move_cursor.connect([&](gua::math::vec2 const& pos) {
        float nx = 2.f * float(pos.x - (resolution.x / 2)) / float(resolution.x);
        float ny = -2.f * float(resolution.y - pos.y - (resolution.y / 2)) / float(resolution.y);
        if(button_state != -1)
        {
            if(drag_mode)
            {
                auto ssize = screen->data.get_size();
                gua::math::vec3 trans = gua::math::vec3(ssize.x * (nx - last_mouse_pos.x), ssize.y * (ny - last_mouse_pos.y), 0.f);
                auto object_trans = scm::math::inverse(screen->get_world_transform()) * mlod_transform->get_world_transform();
                mlod_transform->set_world_transform(screen->get_world_transform() * scm::math::make_translation(trans) * object_trans);
            }
            else
            {
                if(button_state == 0)
                { // left
                    trackball.rotation(trackball_init_pos.x, trackball_init_pos.y, nx, ny);
                }
                if(button_state == 1)
                { // right
                    trackball.dolly(dolly_sens * 0.5f * (ny - trackball_init_pos.y));
                }
                if(button_state == 2)
                { // middle
                    float f = dolly_sens < 1.0f ? 0.02f : 0.3f;
                    trackball.translation(f * (nx - trackball_init_pos.x), f * (ny - trackball_init_pos.y));
                }
                trackball_init_pos.x = nx;
                trackball_init_pos.y = ny;
            }
        }
        last_mouse_pos.x = nx;
        last_mouse_pos.y = ny;
    });

    window->on_button_press.connect([&](int mousebutton, int action, int mods) {
        if(action == 1)
        {
            drag_mode = mods == 1;
            trackball_init_pos = last_mouse_pos;
            button_state = mousebutton;
        }
        else
            button_state = -1;
    });

    window->on_key_press.connect(std::bind(
        [&](gua::PipelineDescription& pipe, gua::SceneGraph& graph, int key, int scancode, int action, int mods) {
            if(action == 0)
                return;
            switch(std::tolower(key))
            {
            default:
                break;
            }
        },
        std::ref(*(camera->get_pipeline_description())),
        std::ref(graph),
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3,
        std::placeholders::_4));

    window->open();

    gua::Renderer renderer;

    // application loop
    gua::events::MainLoop loop;
    gua::events::Ticker ticker(loop, 1.0 / 500.0);
    std::size_t framecount = 0;

    ticker.on_tick.connect([&]() {
        screen->set_transform(scm::math::inverse(gua::math::mat4(trackball.transform_matrix())));

        light_transform->rotate(0.1, 0.f, 1.f, 0.f);
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
            if(framecount++ % 200 == 0)
            {
                std::cout << "FPS: " << window->get_rendering_fps() << "  Frametime: " << 1000.f / window->get_rendering_fps() << std::endl;
            }
        }
    });

    loop.start();

    return 0;
}
