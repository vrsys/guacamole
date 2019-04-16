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
#include <gua/renderer/DebugViewPass.hpp>
#include <gua/renderer/SSAAPass.hpp>

#include <gua/virtual_texturing/VTBackend.hpp>

#include <gua/utils/Trackball.hpp>
#include <GLFW/glfw3.h>
#include <lamure/vt/VTConfig.h>

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

void set_window_default(std::shared_ptr<gua::WindowBase> const& window, gua::math::vec2ui const& res)
{
    window->config.set_size(res);
    window->config.set_resolution(res);
    window->config.set_enable_vsync(false);
    window->config.set_stereo_mode(gua::StereoMode::MONO);
}

// #define SECOND_WINDOW
// #define SECOND_DATASET
#define A_BUFFER
#define RES_PASS
// #define SCANNED_MODEL_EXPLORATION

int main(int argc, char** argv)
{
    std::string vt_model_path = "/mnt/terabytes_of_textures/FINAL_DEMO_DATA/earth.obj";
    std::string vt_texture_path = "/mnt/terabytes_of_textures/FINAL_DEMO_DATA/earth_colour_86400x43200_256x256_1_rgb.atlas";

#ifndef SCANNED_MODEL_EXPLORATION
    vt::VTConfig::CONFIG_PATH = "/mnt/terabytes_of_textures/FINAL_DEMO_DATA/config_demo_do_not_modify.ini";
    vt::VTConfig::get_instance().define_size_physical_texture(64, 8192);
#else
    vt::VTConfig::CONFIG_PATH = "/mnt/pitoti/3d_pitoti/Vianden/Aussen_gesamt/fullRes/vianden_concat.ini";
    vt::VTConfig::get_instance().define_size_physical_texture(64, 8192);
#endif

#ifdef SCANNED_MODEL_EXPLORATION
    vt_model_path = "/mnt/pitoti/3d_pitoti/Vianden/Aussen_gesamt/VIANDEN_normals_vt.obj";
    vt_texture_path = "/mnt/pitoti/3d_pitoti/Vianden/Aussen_gesamt/fullRes/vianden_concat.atlas";

    // vt_model_path = "/mnt/pitoti/3d_pitoti/Vianden/Innen_gesamt/Innenraeume_Gesamt_vt.obj";
    // vt_texture_path = "/mnt/pitoti/3d_pitoti/Vianden/Innen_gesamt/fullRes/vianden_innen_concat.atlas";
#endif

    if(argc < 3)
    {
        std::cout << "Did not provide object or vt-file. Using default files." << std::endl;
    }
    else
    {
        vt_model_path = argv[1];
        vt_texture_path = argv[2];
    }

    char* argv_tmp[] = {argv[0], NULL};
    int argc_tmp = sizeof(argv_tmp) / sizeof(char*) - 1;

    // initialize guacamole
    gua::init(argc_tmp, argv_tmp);

    // setup scene
    gua::SceneGraph graph("main_scenegraph");

    gua::TriMeshLoader loader;

    auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");

    // VT STEP 1/5: - create a material
    auto earth_vt_mat = gua::MaterialShaderDatabase::instance()->lookup("gua_default_material")->make_new_material();
    // VT STEP 2/5: - load *.atlas-File as uniform
    earth_vt_mat->set_uniform("earth_vt_mat", vt_texture_path);
    earth_vt_mat->set_uniform("metalness", 0.f);
    earth_vt_mat->set_uniform("roughness", 1.f);
    // VT STEP 3/5: - enable virtual texturing for this material
    earth_vt_mat->set_enable_virtual_texturing(true);

    // prepare geometry
    auto earth_1_transform = graph.add_node<gua::node::TransformNode>("/transform", "earth_1_transform");

#ifdef SCANNED_MODEL_EXPLORATION
    // VT STEP 4/5: - load earth with vt material
    auto earth_geode_1(
        loader.create_geometry_from_file("earth_geode", vt_model_path, earth_vt_mat, gua::TriMeshLoader::NORMALIZE_SCALE | gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::MAKE_PICKABLE));
#else
    // VT STEP 4/5: - load earth with vt material
    auto earth_geode_1(loader.create_geometry_from_file("earth_geode", vt_model_path, earth_vt_mat, gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::MAKE_PICKABLE));
#endif

    earth_1_transform->translate(1.5, 0.0, 0.0);
    graph.add_node("/transform/earth_1_transform", earth_geode_1);

#ifdef SECOND_DATASET
    // VT STEP 1/5: - create a material
    auto earth_vt_elevation_mat = gua::MaterialShaderDatabase::instance()->lookup("gua_default_material")->make_new_material();
    // VT STEP 2/5: - load *.atlas-File as uniform

    std::string vt_elevation_texture_path = "/mnt/terabytes_of_textures/FINAL_DEMO_DATA/earth_elevation_43200x21600_256x256_1_rgb.atlas";
    earth_vt_elevation_mat->set_uniform("earth_vt_elevation_mat", vt_elevation_texture_path);
    earth_vt_elevation_mat->set_uniform("metalness", 0.5f);
    earth_vt_elevation_mat->set_uniform("roughness", 0.5f);
    // VT STEP 3/5: - enable virtual texturing for this material
    earth_vt_elevation_mat->set_enable_virtual_texturing(true);

    auto earth_2_transform = graph.add_node<gua::node::TransformNode>("/transform", "earth_2_transform");

    // VT STEP 4*/5: - load second earth with vt material

    auto earth_geode_2(loader.create_geometry_from_file("earth_geode_2", vt_model_path, earth_vt_elevation_mat, gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::MAKE_PICKABLE));

    earth_2_transform->rotate(180.0, 0.0, 1.0, 0.0);
    earth_2_transform->translate(-1.5, 0.0, 0.0);
    graph.add_node("/transform/earth_2_transform", earth_geode_2);
#endif

#ifndef SCANNED_MODEL_EXPLORATION
    auto money_transform = graph.add_node<gua::node::TransformNode>("/transform", "money_transform");
    auto money_geode(loader.create_geometry_from_file(
        "money", "/opt/3d_models/50cent/50Cent.obj", gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));

    graph.add_node("/transform/money_transform", money_geode);
#endif

#ifdef RES_PASS
    // create a lightsource
    auto light_transform = graph.add_node<gua::node::TransformNode>("/transform", "light_transform");
    auto light = graph.add_node<gua::node::LightNode>("/transform/light_transform", "light");
    light->data.set_type(gua::node::LightNode::Type::SUN);
    light->data.set_enable_shadows(false);

    light->data.set_shadow_map_size(1920);
    light->data.set_shadow_offset(0.001f);
    light->data.set_softness(0.6f);
    light->data.set_shadow_far_clipping_in_sun_direction(2.0f);
    light->data.set_shadow_near_clipping_in_sun_direction(0.1f);

    light->data.brightness = 10.0f;
    light->scale(1.0f);
    light_transform->rotate(-90, 1.0, 0.0, 0.0);
    light_transform->translate(0.f, 100.f, 1.0f);
#endif

    auto screen = graph.add_node<gua::node::ScreenNode>("/", "screen");
    screen->data.set_size(gua::math::vec2(1.92f, 1.08f));
    screen->translate(0, 0, 3.0);

#ifdef RES_PASS
    transform->add_child(light);
#endif

    // add mouse interaction
    gua::utils::Trackball trackball(0.01, 0.002, 0.2);

    // setup rendering pipeline and window
    auto resolution = gua::math::vec2ui(3840, 2160);

    auto camera = graph.add_node<gua::node::CameraNode>("/screen", "cam1");
    camera->translate(0, 0, 2.0);
    camera->config.set_resolution(resolution);
    camera->config.set_screen_path("/screen");
    camera->config.set_scene_graph_name("main_scenegraph");
    camera->config.set_output_window_name("Virtual_Texturing_Example_Window_1");
    camera->config.set_enable_stereo(false);

    auto pipe = std::make_shared<gua::PipelineDescription>();
    pipe->add_pass(std::make_shared<gua::TriMeshPassDescription>());
#ifdef RES_PASS
    pipe->add_pass(std::make_shared<gua::LightVisibilityPassDescription>());
    auto resolve_pass = std::make_shared<gua::ResolvePassDescription>();
    resolve_pass->background_mode(gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE);
    resolve_pass->tone_mapping_exposure(1.0f);
    pipe->add_pass(resolve_pass);
#endif
    // pipe->add_pass(std::make_shared<gua::DebugViewPassDescription>());
    pipe->add_pass(std::make_shared<gua::SSAAPassDescription>());
#ifdef A_BUFFER
    pipe->set_enable_abuffer(true);
#endif
    camera->set_pipeline_description(pipe);

    bool should_close = false;

    auto add_window = [&](std::string const& window_name, std::shared_ptr<gua::node::CameraNode> const& cam_node) -> std::shared_ptr<gua::GlfwWindow> {
        auto window = std::make_shared<gua::GlfwWindow>();
        gua::WindowDatabase::instance()->add(window_name, window);
        set_window_default(window, cam_node->config.get_resolution());
        cam_node->config.set_output_window_name(window_name);

        window->on_resize.connect([&](gua::math::vec2ui const& new_size) {
            window->config.set_resolution(new_size);
            cam_node->config.set_resolution(new_size);
            screen->data.set_size(gua::math::vec2(0.001 * new_size.x, 0.001 * new_size.y));
        });

        window->on_move_cursor.connect([&](gua::math::vec2 const& pos) { trackball.motion(pos.x, pos.y); });
        window->on_button_press.connect(std::bind(mouse_button, std::ref(trackball), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        window->on_key_press.connect([&](int key, int scancode, int action, int mods) {
            if(key == GLFW_KEY_ESCAPE)
            {
                should_close = true;
            }
        });
        return window;
    };

    auto main_window = add_window("Virtual_Texturing_Example_Window_1", camera);

#ifdef SECOND_WINDOW
    auto camera2 = graph.add_node<gua::node::CameraNode>("/screen", "cam2");
    camera2->translate(0, 0, 2.0);
    camera2->config.set_resolution(resolution);
    camera2->config.set_screen_path("/screen");
    camera2->config.set_scene_graph_name("main_scenegraph");
    camera2->config.set_output_window_name("Virtual_Texturing_Example_Window_2");
    camera2->config.set_enable_stereo(false);

    auto pipe2 = std::make_shared<gua::PipelineDescription>();
    pipe2->add_pass(std::make_shared<gua::TriMeshPassDescription>());
#ifdef RES_PASS
    pipe2->add_pass(std::make_shared<gua::LightVisibilityPassDescription>());
    auto resolve_pass2 = std::make_shared<gua::ResolvePassDescription>();
    resolve_pass2->background_mode(gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE);
    resolve_pass2->tone_mapping_exposure(1.0f);
    pipe2->add_pass(resolve_pass2);
#endif
#ifdef A_BUFFER
    pipe2->set_enable_abuffer(true);
#endif
    camera2->set_pipeline_description(pipe2);

    auto secondary_window = add_window("Virtual_Texturing_Example_Window_2", camera2);
#endif

    auto vt_backend = &gua::VTBackend::get_instance();
    vt_backend->add_camera(camera);
#ifdef SECOND_WINDOW
    vt_backend->add_camera(camera2);
#endif
    vt_backend->start_backend();

    gua::Renderer renderer;

    double extra_rotation = 0.0;

    while(true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

#ifdef SECOND_WINDOW
        secondary_window->process_events();
#endif
        main_window->process_events();

        extra_rotation += 0.01;
        gua::math::mat4 manipulation_matrix =
            scm::math::make_translation(0.0, 0.0, -3.0) * scm::math::make_translation(trackball.shiftx(), trackball.shifty(), trackball.distance() * 0.15f) * gua::math::mat4(trackball.rotation());

#ifndef SCANNED_MODEL_EXPLORATION
        earth_geode_1->set_transform(scm::math::make_rotation(extra_rotation, 0.0, 1.0, 0.0));
#ifdef SECOND_DATASET
        earth_geode_2->set_transform(scm::math::make_rotation(-extra_rotation, 0.0, 1.0, 0.0));
#endif
        money_transform->set_transform(scm::math::make_rotation(45 * std::sin(extra_rotation), 1.0, 0.0, 0.0));
#endif

        transform->set_transform(manipulation_matrix);

        if(main_window->should_close()
#ifdef SECOND_WINDOW
           || secondary_window->should_close()
#endif
           || should_close)
        {
            vt_backend->stop_backend();
            renderer.stop();
            break;
        }
        else
        {
            renderer.queue_draw({&graph});
        }
        /*
                std::cout << "Frame time: " << 1000.f / main_window->get_rendering_fps()
                          << " ms, fps: "
                          <<
                          main_window->get_rendering_fps() << ", app fps: "
                          << renderer.get_application_fps() << std::endl;
        */
    }

    return 0;
}
