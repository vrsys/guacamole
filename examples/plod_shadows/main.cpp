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
#include <gua/renderer/ToneMappingPass.hpp>
#include <gua/renderer/SSAAPass.hpp>
#include <gua/renderer/BBoxPass.hpp>
#include <gua/renderer/DebugViewPass.hpp>
#include <gua/renderer/PLODLoader.hpp>
#include <gua/renderer/PLODPass.hpp>
#include <gua/node/PLODNode.hpp>
#include <scm/gl_util/manipulators/trackball_manipulator.h>

bool animate_light = true;

int main(int argc, char** argv)
{
    gua::init(argc, argv);

    // create simple untextured material shader
    auto plod_keep_input_desc = std::make_shared<gua::MaterialShaderDescription>("./data/materials/PLOD_use_input_color.gmd");
    auto plod_uniform_color_desc = std::make_shared<gua::MaterialShaderDescription>("./data/materials/PLOD_uniform_color.gmd");

    auto plod_keep_color_shader(std::make_shared<gua::MaterialShader>("PLOD_pass_input_color", plod_keep_input_desc));
    auto plod_overwrite_color_shader(std::make_shared<gua::MaterialShader>("PLOD_overwrite_input_color", plod_uniform_color_desc));

    gua::MaterialShaderDatabase::instance()->add(plod_keep_color_shader);
    gua::MaterialShaderDatabase::instance()->add(plod_overwrite_color_shader);

    // create material for pointcloud
    auto plod_rough = plod_keep_color_shader->make_new_material();
    plod_rough->set_uniform("metalness", 0.0f);
    plod_rough->set_uniform("roughness", 0.8f);
    plod_rough->set_uniform("emissivity", 0.0f);

    // simple material for light proxy
    auto rough_white = plod_overwrite_color_shader->make_new_material();
    rough_white->set_uniform("color", gua::math::vec3f(1.0f, 1.0f, 1.0f));
    rough_white->set_uniform("metalness", 0.0f);
    rough_white->set_uniform("roughness", 0.8f);
    rough_white->set_uniform("emissivity", 0.0f);

    // configure plod backend
    gua::PLODLoader plod_loader;
    plod_loader.set_out_of_core_budget_in_mb(2048);
    plod_loader.set_render_budget_in_mb(512);
    plod_loader.set_upload_budget_in_mb(20);
    auto plod_node = plod_loader.load_geometry("pointcloud",
                                               "/opt/3d_models/point_based/plod/pig_pr.bvh",
                                               //"/mnt/pitoti/PitotiTUG/received_18_12_2015/ebee_seradina/ebee_seradina.bvh",
                                               plod_rough,
                                               gua::PLODLoader::NORMALIZE_POSITION | gua::PLODLoader::NORMALIZE_SCALE | gua::PLODLoader::MAKE_PICKABLE);

    // setup scene
    gua::SceneGraph graph("main_scenegraph");
    auto scene_transform = graph.add_node<gua::node::TransformNode>("/", "transform");
    scene_transform->translate(0.f, 0.f, 0.f);

    auto plod_transform = graph.add_node<gua::node::TransformNode>("/transform", "plod_transform");
    plod_transform->rotate(-90.f, 1.f, 0.f, 0.f);
    // plod_transform->rotate(180.f, 0.f, 1.f, 1.f);

    graph.add_node("/transform/plod_transform", plod_node);
    plod_node->set_draw_bounding_box(true);
    plod_node->set_error_threshold(2.5f);
    plod_node->set_radius_scale(1.2f);

    auto light_transform = graph.add_node<gua::node::TransformNode>("/transform", "light_transform");
    auto light = graph.add_node<gua::node::LightNode>("/transform/light_transform", "light");
    light->data.set_type(gua::node::LightNode::Type::SPOT); // SPOT, POINT
    light->data.set_enable_shadows(true);
    light->data.set_shadow_map_size(4096);
    light->data.brightness = 5.0f;
    // light->rotate(90.f, 0.f, 1.f, 0.f);
    light->translate(0.f, 0.2f, 0.f);
    light->scale(2.f);
    light->translate(0.f, 0.f, 1.f);
    light->data.set_shadow_near_clipping_in_sun_direction(0.1f);
    light->data.set_shadow_far_clipping_in_sun_direction(100.f);

    gua::TriMeshLoader trimesh_loader;
    auto light_proxy_geometry(
        trimesh_loader.create_geometry_from_file("light_proxy", "data/objects/sphere.obj", rough_white, gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));

    light_proxy_geometry->scale(0.02);
    light->add_child(light_proxy_geometry);

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
    auto resolution = gua::math::vec2ui(1920, 1080);

    auto camera = graph.add_node<gua::node::CameraNode>("/screen", "cam");
    camera->translate(0, 0, 2.f);
    camera->config.set_resolution(resolution);

    // use close near plane to inspect details
    camera->config.set_near_clip(0.01f);
    camera->config.set_far_clip(20.0f);
    camera->config.set_screen_path("/screen");
    camera->config.set_scene_graph_name("main_scenegraph");
    camera->config.set_output_window_name("main_window");
    camera->config.set_enable_stereo(false);

    auto pipe = std::make_shared<gua::PipelineDescription>();
    pipe->add_pass(std::make_shared<gua::TriMeshPassDescription>());
    pipe->add_pass(std::make_shared<gua::PLODPassDescription>());
    pipe->add_pass(std::make_shared<gua::BBoxPassDescription>());
    pipe->add_pass(std::make_shared<gua::LightVisibilityPassDescription>());
    pipe->add_pass(std::make_shared<gua::ResolvePassDescription>());
    pipe->add_pass(std::make_shared<gua::SSAAPassDescription>());
    // pipe->add_pass(std::make_shared<gua::DebugViewPassDescription>());

    camera->set_pipeline_description(pipe);

    pipe->get_resolve_pass()->tone_mapping_exposure(3.0f);

    pipe->get_resolve_pass()->background_mode(gua::ResolvePassDescription::BackgroundMode::SKYMAP_TEXTURE);
    pipe->get_resolve_pass()->background_texture("data/textures/envlightmap.jpg");

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
                auto object_trans = scm::math::inverse(screen->get_world_transform()) * plod_transform->get_world_transform();
                plod_transform->set_world_transform(screen->get_world_transform() * scm::math::make_translation(trans) * object_trans);
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

            float v = 0.f;

            switch(std::tolower(key))
            {
            case 'm': // toggle environment lighting mode

                if(pipe.get_resolve_pass()->environment_lighting_mode() == gua::ResolvePassDescription::EnvironmentLightingMode::AMBIENT_COLOR)
                {
                    std::cout << "Setting to gua::ResolvePassDescription::EnvironmentLightingMode::SPHEREMAP" << std::endl;
                    pipe.get_resolve_pass()->environment_lighting_mode(gua::ResolvePassDescription::EnvironmentLightingMode::SPHEREMAP);
                }
                else if(pipe.get_resolve_pass()->environment_lighting_mode() == gua::ResolvePassDescription::EnvironmentLightingMode::SPHEREMAP)
                {
                    std::cout << "Setting to gua::ResolvePassDescription::EnvironmentLightingMode::CUBEMAP" << std::endl;
                    pipe.get_resolve_pass()->environment_lighting_mode(gua::ResolvePassDescription::EnvironmentLightingMode::CUBEMAP);
                }
                else
                {
                    std::cout << "Setting to gua::ResolvePassDescription::EnvironmentLightingMode::AMBIENT_COLOR" << std::endl;
                    pipe.get_resolve_pass()->environment_lighting_mode(gua::ResolvePassDescription::EnvironmentLightingMode::AMBIENT_COLOR);
                }

                pipe.get_resolve_pass()->touch();
                break;

            case 'a': // toggle screen space shadows
                pipe.get_resolve_pass()->screen_space_shadows(!pipe.get_resolve_pass()->screen_space_shadows());
                break;

            case 'q':
                pipe.get_resolve_pass()->screen_space_shadow_radius(std::min(10.0f, 1.1f * pipe.get_resolve_pass()->screen_space_shadow_radius()));
                break;

            case 'z':
                pipe.get_resolve_pass()->screen_space_shadow_radius(std::max(0.005f, 0.9f * pipe.get_resolve_pass()->screen_space_shadow_radius()));
                break;

            case 'w':
                pipe.get_resolve_pass()->screen_space_shadow_intensity(std::min(1.0f, 1.1f * pipe.get_resolve_pass()->screen_space_shadow_intensity()));
                break;

            case 'x':
                pipe.get_resolve_pass()->screen_space_shadow_intensity(std::max(0.1f, 0.9f * pipe.get_resolve_pass()->screen_space_shadow_intensity()));
                break;

            case 's': // toggle SSAO
                pipe.get_resolve_pass()->ssao_enable(!pipe.get_resolve_pass()->ssao_enable());
                break;

            case '1':
                pipe.get_resolve_pass()->ssao_intensity(std::min(5.0f, 1.1f * pipe.get_resolve_pass()->ssao_intensity()));
                break;
            case '2':
                pipe.get_resolve_pass()->ssao_intensity(std::max(0.02f, 0.9f * pipe.get_resolve_pass()->ssao_intensity()));
                break;

            case '3':
                pipe.get_resolve_pass()->ssao_radius(std::min(64.0f, 1.1f * pipe.get_resolve_pass()->ssao_radius()));
                break;
            case '4':
                pipe.get_resolve_pass()->ssao_radius(std::max(1.0f, 0.9f * pipe.get_resolve_pass()->ssao_radius()));
                break;

            case '5':
                pipe.get_resolve_pass()->ssao_falloff(std::min(256.0f, 1.1f * pipe.get_resolve_pass()->ssao_falloff()));
                break;
            case '6':
                pipe.get_resolve_pass()->ssao_falloff(std::max(0.1f, 0.9f * pipe.get_resolve_pass()->ssao_falloff()));
                break;

            case 'f':
                if(pipe.get_ssaa_pass()->mode() == gua::SSAAPassDescription::SSAAMode::FXAA311)
                {
                    std::cout << "Switching to simple FAST_FXAA\n" << std::endl;
                    pipe.get_ssaa_pass()->mode(gua::SSAAPassDescription::SSAAMode::FAST_FXAA);
                }
                else if(pipe.get_ssaa_pass()->mode() == gua::SSAAPassDescription::SSAAMode::FAST_FXAA)
                {
                    std::cout << "Switching to No FXAA\n" << std::endl;
                    pipe.get_ssaa_pass()->mode(gua::SSAAPassDescription::SSAAMode::DISABLED);
                }
                else
                {
                    std::cout << "Switching to FXAA 3.11\n" << std::endl;
                    pipe.get_ssaa_pass()->mode(gua::SSAAPassDescription::SSAAMode::FXAA311);
                }
                break;

            case '7':
                v = std::min(1.0f, 1.1f * pipe.get_ssaa_pass()->fxaa_quality_subpix());
                std::cout << "Setting quality_subpix to " << v << std::endl;
                pipe.get_ssaa_pass()->fxaa_quality_subpix(v);
                break;
            case '8':
                v = std::max(0.2f, 0.9f * pipe.get_ssaa_pass()->fxaa_quality_subpix());
                std::cout << "Setting quality_subpix to " << v << std::endl;
                pipe.get_ssaa_pass()->fxaa_quality_subpix(v);
                break;

            case '9':
                v = std::min(0.333f, 1.1f * pipe.get_ssaa_pass()->fxaa_edge_threshold());
                std::cout << "Setting edge_threshold to " << v << std::endl;
                pipe.get_ssaa_pass()->fxaa_edge_threshold(v);
                break;
            case '0':
                v = std::max(0.063f, 0.9f * pipe.get_ssaa_pass()->fxaa_edge_threshold());
                std::cout << "Setting edge_threshold to " << v << std::endl;
                pipe.get_ssaa_pass()->fxaa_edge_threshold(v);
                break;

            case 't':
                pipe.get_resolve_pass()->touch();
                break;

            case ' ':
                animate_light = !animate_light;
                break;

                // recompile shaders
            case 'r':
                pipe.get_resolve_pass()->touch();
                break;

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

    ticker.on_tick.connect([&]() {
        screen->set_transform(scm::math::inverse(gua::math::mat4(trackball.transform_matrix())));

        if(animate_light)
        {
            light->rotate(0.1, 0.0, 1.0, 0.0);
        }
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
