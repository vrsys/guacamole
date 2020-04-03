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
#include <gua/renderer/DebugViewPass.hpp>
#include <gua/utils/Trackball.hpp>

bool animate_light = true;

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

void key_press(gua::PipelineDescription& pipe, gua::SceneGraph& graph, int key, int scancode, int action, int mods)
{
    if(action == 0)
        return;

    float v = 0.0f;

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

    case 'b': // toggle background mode

        if(pipe.get_resolve_pass()->background_mode() == gua::ResolvePassDescription::BackgroundMode::COLOR)
        {
            std::cout << "Setting to gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE" << std::endl;
            pipe.get_resolve_pass()->background_mode(gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE);
        }
        else if(pipe.get_resolve_pass()->background_mode() == gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE)
        {
            std::cout << "Setting to gua::ResolvePassDescription::BackgroundMode::SKYMAP_TEXTURE" << std::endl;
            pipe.get_resolve_pass()->background_mode(gua::ResolvePassDescription::BackgroundMode::SKYMAP_TEXTURE);
        }
        else
        {
            std::cout << "Setting to gua::ResolvePassDescription::BackgroundMode::AMBIENT_COLOR" << std::endl;
            pipe.get_resolve_pass()->background_mode(gua::ResolvePassDescription::BackgroundMode::COLOR);
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
}

int main(int argc, char** argv)
{
    // initialize guacamole
    gua::init(argc, argv);

    // create material
    auto uniform_color_desc = std::make_shared<gua::MaterialShaderDescription>("./data/materials/uniform_color.gmd");
    auto uniform_color_shader(std::make_shared<gua::MaterialShader>("overwrite_color", uniform_color_desc));
    gua::MaterialShaderDatabase::instance()->add(uniform_color_shader);
    auto rough_white = uniform_color_shader->make_new_material();

    rough_white->set_uniform("color", gua::math::vec3f(1.0f, 1.0f, 1.0f));
    rough_white->set_uniform("metalness", 0.0f);
    rough_white->set_uniform("roughness", 1.0f);
    rough_white->set_uniform("emissivity", 0.0f);

    scm::gl::sampler_state_desc const& sampler_state_desc = scm::gl::sampler_state_desc(scm::gl::FILTER_ANISOTROPIC, scm::gl::WRAP_MIRRORED_REPEAT, scm::gl::WRAP_MIRRORED_REPEAT);
    // gua::TextureDatabase::instance()->add("data/textures/envlightmap.jpg", std::make_shared<gua::Texture2D>("data/textures/envlightmap.jpg", true, sampler_state_desc));
    gua::TextureDatabase::instance()->load("data/textures/envlightmap.jpg");
    gua::TextureDatabase::instance()->load("data/textures/envmap.jpg");
    gua::TextureDatabase::instance()->load("data/textures/noise.jpg");
    rough_white->set_uniform("noise_texture", std::string("data/textures/noise.jpg"));

    // setup scene
    gua::SceneGraph graph("main_scenegraph");

    gua::TriMeshLoader loader;

    auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");
    auto city(loader.create_geometry_from_file("city", "data/objects/city.3ds", rough_white, gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));
    // auto city(loader.create_geometry_from_file("city", "data/objects/city.3ds", gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE | gua::TriMeshLoader::LOAD_MATERIALS));
    graph.add_node("/transform", city);
    city->set_draw_bounding_box(true);

    auto light = graph.add_node<gua::node::LightNode>("/transform/city", "light");
    light->data.set_type(gua::node::LightNode::Type::POINT);
    light->data.set_enable_shadows(false);
    // light->data.set_shadow_cascaded_splits({0.3f, 0.7f, 1.0f, 10.0f});
    // light->data.set_max_shadow_dist(100.0f);
    light->data.set_shadow_map_size(1024);
    light->data.set_shadow_near_clipping_in_sun_direction(0.2f);
    light->data.brightness = 5.0f;
    light->scale(9.f);
    light->translate(0.f, 7.f, 8.f);

    auto light_proxy_geometry(loader.create_geometry_from_file("light_proxy", "data/objects/sphere.obj", rough_white, gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));
    light_proxy_geometry->scale(0.02);
    light->add_child(light_proxy_geometry);

    auto screen = graph.add_node<gua::node::ScreenNode>("/", "screen");
    screen->data.set_size(gua::math::vec2(1.92f, 1.08f));
    screen->translate(0, 0, 1.0);

    // add mouse interaction
    gua::utils::Trackball trackball(0.01f, 0.002f, 0.2f);

    // setup rendering pipeline and window
    auto resolution = gua::math::vec2ui(960, 540);

    auto camera = graph.add_node<gua::node::CameraNode>("/screen", "cam");

    camera->translate(0, 0, 2.0);
    camera->config.set_resolution(resolution);
    camera->config.set_left_screen_path("/screen");
    camera->config.set_right_screen_path("/screen");
    camera->config.set_scene_graph_name("main_scenegraph");
    camera->config.set_output_window_name("main_window");
    camera->config.set_enable_stereo(true);

    camera->get_pipeline_description()->get_resolve_pass()->tone_mapping_exposure(1.0f);
    camera->get_pipeline_description()->get_resolve_pass()->ssao_intensity(1.0);
    camera->get_pipeline_description()->get_resolve_pass()->ssao_enable(false);
    camera->get_pipeline_description()->get_resolve_pass()->ssao_falloff(1.0);
    camera->get_pipeline_description()->get_resolve_pass()->ssao_radius(4.0);

    camera->get_pipeline_description()->get_resolve_pass()->screen_space_shadows(false);
    camera->get_pipeline_description()->get_resolve_pass()->screen_space_shadow_radius(1.0);
    camera->get_pipeline_description()->get_resolve_pass()->screen_space_shadow_intensity(0.9);

    camera->get_pipeline_description()->get_resolve_pass()->environment_lighting_mode(gua::ResolvePassDescription::EnvironmentLightingMode::AMBIENT_COLOR);
    camera->get_pipeline_description()->get_resolve_pass()->environment_lighting_texture("data/textures/envlightmap.jpg");

    camera->get_pipeline_description()->get_resolve_pass()->background_mode(gua::ResolvePassDescription::BackgroundMode::SKYMAP_TEXTURE);
    camera->get_pipeline_description()->get_resolve_pass()->background_texture("data/textures/envmap.jpg");

    //camera->get_pipeline_description()->add_pass(std::make_shared<gua::DebugViewPassDescription>());
    //camera->get_pipeline_description()->add_pass(std::make_shared<gua::SSAAPassDescription>());
    camera->config.set_near_clip(0.1f);
    camera->config.set_far_clip(10.0f);


    // define second camera with different stereo mode secondary camera
    auto camera_mvs = graph.add_node<gua::node::CameraNode>("/screen", "cam_mvs");
    camera_mvs->translate(0, 0, 2.0);
    camera_mvs->config.set_resolution(resolution);  
    camera_mvs->config.set_left_screen_path("/screen");
    camera_mvs->config.set_right_screen_path("/screen");
    camera_mvs->config.set_scene_graph_name("main_scenegraph");
    camera_mvs->config.set_output_window_name("software_mvs_window");
    camera_mvs->config.set_enable_stereo(true);
    camera_mvs->set_pipeline_description(camera->get_pipeline_description());
    camera_mvs->config.set_near_clip(0.1f);
    camera_mvs->config.set_far_clip(10.0f);


    auto window = std::make_shared<gua::GlfwWindow>();
    window->config.set_enable_vsync(false);
    window->config.set_size( scm::math::vec2ui(resolution.x * 2, resolution.y) );
    window->config.set_right_position(scm::math::vec2ui(resolution.x , 0));
    window->config.set_left_resolution( scm::math::vec2ui(resolution.x, resolution.y) );
    window->config.set_right_resolution( scm::math::vec2ui(resolution.x, resolution.y) );
    window->config.set_stereo_mode(gua::StereoMode::SIDE_BY_SIDE);

    window->on_resize.connect([&](gua::math::vec2ui const& new_size) {
        window->config.set_resolution(new_size);
        camera->config.set_resolution(new_size);
        screen->data.set_size(gua::math::vec2(0.001 * new_size.x, 0.001 * new_size.y));
    });

    window->on_move_cursor.connect([&](gua::math::vec2 const& pos) { trackball.motion(pos.x, pos.y); });
    window->on_button_press.connect(std::bind(mouse_button, std::ref(trackball), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    window->on_key_press.connect(
        std::bind(key_press, std::ref(*(camera->get_pipeline_description())), std::ref(graph), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));


    auto window_mvs = std::make_shared<gua::GlfwWindow>();

    window_mvs->config.set_enable_vsync(false);
    window_mvs->config.set_size( scm::math::vec2ui(resolution.x * 2, resolution.y) );
    window_mvs->config.set_resolution(scm::math::vec2ui(resolution.x * 2, resolution.y ) );
    window_mvs->config.set_right_position(scm::math::vec2ui(resolution.x , 0));
    window_mvs->config.set_left_resolution( scm::math::vec2ui(resolution.x, resolution.y) );
    window_mvs->config.set_right_resolution( scm::math::vec2ui(resolution.x, resolution.y) );
    window_mvs->config.set_stereo_mode(gua::StereoMode::SIDE_BY_SIDE_SOFTWARE_MULTI_VIEW_RENDERING);

    window_mvs->on_resize.connect([&](gua::math::vec2ui const& new_size) {
        window_mvs->config.set_resolution(new_size);
        camera_mvs->config.set_resolution(new_size);
        screen->data.set_size(gua::math::vec2(0.001 * new_size.x, 0.001 * new_size.y));
    });

    window_mvs->on_move_cursor.connect([&](gua::math::vec2 const& pos) { trackball.motion(pos.x, pos.y); });
    window_mvs->on_button_press.connect(std::bind(mouse_button, std::ref(trackball), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    window_mvs->on_key_press.connect(
        std::bind(key_press, std::ref(*(camera_mvs->get_pipeline_description())), std::ref(graph), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

    //window->open();

    gua::Renderer renderer;

    // application loop
    gua::events::MainLoop loop;
    gua::events::Ticker ticker(loop, 1.0 / 500.0);


    int cnt = 0;

    ticker.on_tick.connect([&]() {
        // apply trackball matrix to object
        gua::math::mat4 modelmatrix = scm::math::make_translation(gua::math::float_t(trackball.shiftx()), gua::math::float_t(trackball.shifty()), gua::math::float_t(trackball.distance())) *
                                      gua::math::mat4(trackball.rotation());

        transform->set_transform(modelmatrix);

        ++cnt;

        if(1 == cnt) {
            gua::WindowDatabase::instance()->add("main_window", window);
        }

        if(200 == cnt) {
            gua::WindowDatabase::instance()->add("software_mvs_window", window_mvs);
        }


        if(window->get_rendering_fps() > 0 && window_mvs->get_rendering_fps() > 0) {
          std::cout << "Speedup: " << (1.0f/window->get_rendering_fps())/(1.0f/window_mvs->get_rendering_fps()) << std::endl;
          std::cout << "draw time ref: " << 1.0 / window->get_rendering_fps() << std::endl;
          std::cout << "draw time 2: " << 1.0 / window_mvs->get_rendering_fps() << std::endl;
        }

        if(animate_light)
        {
            light->rotate(0.1, 0.0, 1.0, 0.0);
        }

        window->process_events();
        window_mvs->process_events();

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
