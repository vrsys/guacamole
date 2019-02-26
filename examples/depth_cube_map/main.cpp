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
#include <gua/node/CubemapNode.hpp>
#include <gua/renderer/TriMeshLoader.hpp>

#include <gua/renderer/PLODLoader.hpp>
#include <gua/node/PLODNode.hpp>
#include <gua/renderer/PLODPass.hpp>

#include <gua/renderer/ToneMappingPass.hpp>
#include <gua/renderer/DebugViewPass.hpp>
#include <gua/renderer/DepthCubeMapPass.hpp>
#include <gua/renderer/SSAAPass.hpp>
#include <gua/renderer/TexturedScreenSpaceQuadPass.hpp>
#include <gua/utils/Trackball.hpp>
#include <gua/gui.hpp>

#include <boost/filesystem.hpp>

#include "Navigator.hpp"

#define FULLSCREEN false

// TRIMESH
#define TEAPOT false
#define RECURSIVE_OILRIGS true
#define VIADEN false // requires /mnt/pitoti

// Plod
#define PITOTI false // requires /mnt/pitoti
#define BRIDGE false // requires /mnt/pitoti

#define SKYMAP false

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
    // add interaction
    Navigator nav;
    nav.set_transform(scm::math::make_translation(0.f, 0.f, 4.f));

    // initialize guacamole
    gua::init(argc, argv);

    // setup scene
    gua::SceneGraph graph("main_scenegraph");

    // loaders
    gua::PLODLoader plod_loader;
    gua::TriMeshLoader trimesh_loader;

    plod_loader.set_upload_budget_in_mb(128);
    plod_loader.set_render_budget_in_mb(2048);
    plod_loader.set_out_of_core_budget_in_mb(4096);

    auto navigation = graph.add_node<gua::node::TransformNode>("/", "navigation");
    auto scene = graph.add_node<gua::node::TransformNode>("/", "scene");

    // MODELS
    if(TEAPOT)
    {
        auto teapot(trimesh_loader.create_geometry_from_file("teapot",
                                                             "data/objects/teapot.obj",
                                                             gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::LOAD_MATERIALS |
                                                                 gua::TriMeshLoader::OPTIMIZE_MATERIALS | gua::TriMeshLoader::NORMALIZE_SCALE));

        graph.add_node(scene, teapot);
    }

    if(RECURSIVE_OILRIGS)
    {
        auto much_big_oilrig(trimesh_loader.create_geometry_from_file("much_big_oilrig",
                                                                      "/opt/3d_models/OIL_RIG_GUACAMOLE/oilrig.obj",
                                                                      gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::LOAD_MATERIALS |
                                                                          gua::TriMeshLoader::OPTIMIZE_MATERIALS | gua::TriMeshLoader::NORMALIZE_SCALE));

        much_big_oilrig->rotate(-90.0f, 1.0f, 0.0f, 0.0f);
        much_big_oilrig->scale(100.0);

        graph.add_node(scene, much_big_oilrig);

        auto big_oilrig(trimesh_loader.create_geometry_from_file("big_oilrig",
                                                                 "/opt/3d_models/OIL_RIG_GUACAMOLE/oilrig.obj",
                                                                 gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::LOAD_MATERIALS |
                                                                     gua::TriMeshLoader::OPTIMIZE_MATERIALS | gua::TriMeshLoader::NORMALIZE_SCALE));

        big_oilrig->rotate(-90.0f, 1.0f, 0.0f, 0.0f);
        big_oilrig->scale(10);
        big_oilrig->translate(18.f, 1.0f, 8.f);

        graph.add_node(scene, big_oilrig);

        auto oilrig(trimesh_loader.create_geometry_from_file("oilrig",
                                                             "/opt/3d_models/OIL_RIG_GUACAMOLE/oilrig.obj",
                                                             gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::LOAD_MATERIALS |
                                                                 gua::TriMeshLoader::OPTIMIZE_MATERIALS | gua::TriMeshLoader::NORMALIZE_SCALE));

        oilrig->rotate(-90.0f, 1.0f, 0.0f, 0.0f);
        // oilrig->scale(1.0);
        oilrig->translate(19.8f, 1.1f, 8.8f);

        graph.add_node(scene, oilrig);

        auto small_oilrig(trimesh_loader.create_geometry_from_file("small_oilrig",
                                                                   "/opt/3d_models/OIL_RIG_GUACAMOLE/oilrig.obj",
                                                                   gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::LOAD_MATERIALS |
                                                                       gua::TriMeshLoader::OPTIMIZE_MATERIALS | gua::TriMeshLoader::NORMALIZE_SCALE));

        small_oilrig->rotate(-90.0f, 1.0f, 0.0f, 0.0f);
        small_oilrig->scale(0.1);
        small_oilrig->translate(19.98f, 1.11f, 8.88f);

        graph.add_node(scene, small_oilrig);

        auto elephant(trimesh_loader.create_geometry_from_file("elephant",
                                                               "/opt/3d_models/animals/elephant/elephant.obj",
                                                               gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::LOAD_MATERIALS |
                                                                   gua::TriMeshLoader::OPTIMIZE_MATERIALS | gua::TriMeshLoader::NORMALIZE_SCALE));

        elephant->rotate(-180.0f, 1.0f, 0.0f, 0.0f);
        elephant->rotate(90.0f, 0.0f, 1.0f, 0.0f);
        elephant->scale(0.002);
        elephant->translate(19.957f, 1.1201f, 8.892f);

        graph.add_node(scene, elephant);

        much_big_oilrig->translate(50.0, 0.0, -50.0);
        big_oilrig->translate(50.0, 0.0, -50.0);
        oilrig->translate(50.0, 0.0, -50.0);
        small_oilrig->translate(50.0, 0.0, -50.0);
        elephant->translate(50.0, 0.0, -50.0);

        nav.set_transform(scm::math::make_translation(50.f, 1.f, 9.f) * scm::math::make_rotation(90.0f, 0.f, 1.f, 0.f));
    }

    if(VIADEN)
    {
        auto viaden_outside(trimesh_loader.create_geometry_from_file("viaden_outside",
                                                                     "/mnt/pitoti/3d_pitoti/Vianden/Aussen_gesamt/VIANDEN.obj",
                                                                     gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::OPTIMIZE_MATERIALS));

        viaden_outside->rotate(-90.0f, 1.0f, 0.0f, 0.0f);

        graph.add_node(scene, viaden_outside);

        auto viaden_inside(trimesh_loader.create_geometry_from_file("viaden_inside",
                                                                    "/mnt/pitoti/3d_pitoti/Vianden/Innen_gesamt/Innenraeume_Gesamt.obj",
                                                                    gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::OPTIMIZE_MATERIALS));

        viaden_inside->rotate(-90.0f, 1.0f, 0.0f, 0.0f);

        graph.add_node(scene, viaden_inside);
    }

    if(PITOTI)
    {
        auto rot_offset_mat = gua::math::mat4(0.088, 0.996, -0.007, 0.0, 0.016, 0.005, 1.0, 0.0, 0.996, -0.088, -0.016, 0.0, 0.0, 0.0, 0.0, 1.0);
        rot_offset_mat = scm::math::transpose(rot_offset_mat);

        auto offset_mat = rot_offset_mat * scm::math::make_translation(-604050.0, -5098490.0, -400.0);

        // SERADINA FLYOVER
        std::vector<std::string> model_paths{
            "/mnt/pitoti/3d_pitoti/grundtruth_data/valley/seradina_flyover/seradina_full_las_0_part_00010_knobi_cutout_flagged.kdn",
            "/mnt/pitoti/3d_pitoti/grundtruth_data/valley/seradina_flyover/seradina_full_las_0_part_00011_knobi_cutout_flagged.kdn",

            // "/mnt/pitoti/3d_pitoti/grundtruth_data/valley/seradina_flyover/seradina_full_las_0_part_00001_knobi.kdn",
            // "/mnt/pitoti/3d_pitoti/grundtruth_data/valley/seradina_flyover/seradina_full_las_0_part_00002_knobi.kdn",
            // "/mnt/pitoti/3d_pitoti/grundtruth_data/valley/seradina_flyover/seradina_full_las_0_part_00003_knobi.kdn",
            // "/mnt/pitoti/3d_pitoti/grundtruth_data/valley/seradina_flyover/seradina_full_las_0_part_00004_knobi.kdn",
            // "/mnt/pitoti/3d_pitoti/grundtruth_data/valley/seradina_flyover/seradina_full_las_0_part_00005_knobi.kdn",
            // "/mnt/pitoti/3d_pitoti/grundtruth_data/valley/seradina_flyover/seradina_full_las_0_part_00006_knobi.kdn",
            // "/mnt/pitoti/3d_pitoti/grundtruth_data/valley/seradina_flyover/seradina_full_las_0_part_00007_knobi.kdn",
            // "/mnt/pitoti/3d_pitoti/grundtruth_data/valley/seradina_flyover/seradina_full_las_0_part_00008_knobi.kdn",
            // "/mnt/pitoti/3d_pitoti/grundtruth_data/valley/seradina_flyover/seradina_full_las_0_part_00009_knobi.kdn",
        };

        auto rot_mat = gua::math::mat4(-0.57732352, 0.040792, -0.8154959, 0.0, 0.816437476, 0.042645956, -0.5758569, 0.0, 0.0112872, -0.998257147, -0.057924671, 0.0, 0.0, 0.0, 0.0, 1.0);
        rot_mat = scm::math::transpose(rot_mat);

        auto mat = scm::math::make_translation(603956.727956973, 5098223.502562742, 819.626837676) * rot_mat * scm::math::make_scale(gua::math::vec3(31.261682663898622));
        mat = offset_mat * mat;

        auto seradina_flyover_group = graph.add_node<gua::node::TransformNode>("/scene", "seradina_flyover_group");
        seradina_flyover_group->set_transform(mat);

        for(const auto path : model_paths)
        {
            auto plod_node = plod_loader.load_geometry(path, gua::PLODLoader::DEFAULTS);
            graph.add_node(seradina_flyover_group, plod_node);
        }

        // SERADINA 12C
        model_paths = std::vector<std::string>{
            "/mnt/pitoti/3d_pitoti/grundtruth_data/rocks/seradina12c/TLS_Seradina_Rock-12C_knn_cutout_flagged.kdn",
            "/mnt/pitoti/3d_pitoti/seradina_12c/areas/Area_4_hunter_with_bow_knn.kdn",
        };

        mat = offset_mat * scm::math::make_translation(604050.0, 5098490.0, 400.0);

        auto seradina_12c_group = graph.add_node<gua::node::TransformNode>("/scene", "seradina_12c_group");
        seradina_12c_group->set_transform(mat);

        for(const auto path : model_paths)
        {
            auto plod_node = plod_loader.load_geometry(path, gua::PLODLoader::DEFAULTS);
            graph.add_node(seradina_12c_group, plod_node);
        }
        nav.set_transform(scm::math::make_translation(450.f, 65.f, 115.f));
    }
    if(BRIDGE)
    {
        std::vector<std::string> model_paths{
            "/mnt/pitoti/hallermann_scans/bruecke/bruecke_points_part00001_knobi.kdn",
            "/mnt/pitoti/hallermann_scans/bruecke/bruecke_points_part00002_knobi.kdn",
            "/mnt/pitoti/hallermann_scans/bruecke/bruecke_points_part00003_knobi.kdn",
            "/mnt/pitoti/hallermann_scans/bruecke/bruecke_points_part00004_knobi.kdn",
            "/mnt/pitoti/hallermann_scans/bruecke/bruecke_points_part00005_knobi.kdn",
            "/mnt/pitoti/hallermann_scans/bruecke/bruecke_points_part00006_knobi.kdn",
            "/mnt/pitoti/hallermann_scans/bruecke/bruecke_points_part00007_knobi.kdn",
            "/mnt/pitoti/hallermann_scans/bruecke/bruecke_points_part00008_knobi.kdn",
        };
        for(const auto path : model_paths)
        {
            auto plod_node = plod_loader.load_geometry(path, gua::PLODLoader::DEFAULTS);
            graph.add_node("/scene", plod_node);
        }
        scene->rotate(-90.0, 1.0, 0.0, 0.0);
    }

    auto light = graph.add_node<gua::node::LightNode>("/", "light");
    light->data.set_type(gua::node::LightNode::Type::SUN);
    light->rotate(-60.f, 1.f, 0.f, 0.f);
    light->data.brightness = 0.5f;

    auto screen = graph.add_node<gua::node::ScreenNode>("/navigation", "screen");
    screen->data.set_size(gua::math::vec2(1.92f, 1.08f));
    screen->translate(0, 0, -2.0);

    // setup rendering pipeline and window
    auto resolution = gua::math::vec2ui(2560, 1440);

    auto camera = graph.add_node<gua::node::CameraNode>("/navigation", "cam");
    camera->config.set_resolution(resolution);
    camera->config.set_screen_path("/navigation/screen");
    camera->config.set_scene_graph_name("main_scenegraph");
    camera->config.set_output_window_name("main_window");
    camera->config.set_enable_stereo(false);

    camera->config.set_near_clip(1.0f);
    camera->config.set_far_clip(1000.0f);

    auto pipe = std::make_shared<gua::PipelineDescription>();

    pipe->add_pass(std::make_shared<gua::DepthCubeMapPassDesciption>());
    pipe->add_pass(std::make_shared<gua::TriMeshPassDescription>());
    pipe->add_pass(std::make_shared<gua::PLODPassDescription>());
    pipe->add_pass(std::make_shared<gua::LightVisibilityPassDescription>());
    pipe->add_pass(std::make_shared<gua::ResolvePassDescription>());
    pipe->add_pass(std::make_shared<gua::TexturedScreenSpaceQuadPassDescription>());
    pipe->get_resolve_pass()->tone_mapping_exposure(1.0f);

    if(SKYMAP)
    {
        pipe->get_resolve_pass()->background_mode(gua::ResolvePassDescription::BackgroundMode::SKYMAP_TEXTURE);
        pipe->get_resolve_pass()->background_texture("data/textures/skymap.jpg");
    }

    camera->set_pipeline_description(pipe);

    // CUBEMAP NAVIGATION
    bool adaptive_navigation(true);
    auto cmn(graph.add_node<gua::node::CubemapNode>("/navigation", "test"));
    // graph.remove_node("/navigation/test");
    cmn->config.set_texture_name("navigation_depth_texture");
    cmn->config.set_near_clip(0.005f);
    cmn->config.set_far_clip(500.0f);
    cmn->config.set_resolution(64);
    // cmn->config.set_render_mode(gua::node::CubemapNode::RenderMode::ONE_SIDE_PER_FRAME);
    float motion_speed = 0.01f;

    /* NOT SUPPORTED BY GUACAMOLE RIGHT NOW -> textured_screen_space_quad.frag has to be adapted
     * // DEBUG VIEW
     * bool debug_preview = true;
     * auto cm_preview = std::make_shared<gua::node::TexturedScreenSpaceQuadNode>("cubemap_debug");
     * cm_preview->data.texture() = cmn->config.get_texture_name();
     * gua::math::vec2 preview_size(resolution.x, resolution.x / 6.0f);
     * cm_preview->data.size() = preview_size;
     * cm_preview->data.anchor() = gua::math::vec2(0.f, -1.f);
     * graph.add_node("/", cm_preview);
     */

    auto window = std::make_shared<gua::GlfwWindow>();
    gua::WindowDatabase::instance()->add("main_window", window);
    window->config.set_enable_vsync(false);
    window->config.set_size(resolution);
    window->config.set_fullscreen_mode(FULLSCREEN);
    window->config.set_resolution(resolution);
    window->config.set_stereo_mode(gua::StereoMode::MONO);
    window->on_resize.connect([&](gua::math::vec2ui const& new_size) {
        window->config.set_resolution(new_size);
        camera->config.set_resolution(new_size);
        // cm_preview->data.size() = gua::math::vec2(new_size.x, new_size.x / 6.f);
        screen->data.set_size(gua::math::vec2(0.001 * new_size.x, 0.001 * new_size.y));
    });
    window->on_move_cursor.connect([&](gua::math::vec2 const& pos) {
        // trackball.motion(pos.x, pos.y);
        nav.set_mouse_position(gua::math::vec2i(pos));
    });

    window->on_key_press.connect([&](int key, int scancode, int action, int mods) {
        nav.set_key_press(static_cast<gua::Key>(key), action);
        // std::cout << key << " " << action << std::endl;
        // ENTER
        if((key == 257) && (action == 1))
        {
            adaptive_navigation = !adaptive_navigation;
            if(adaptive_navigation)
            {
                std::cout << "Adaptive locomotion turned on" << std::endl;
                graph.add_node("/navigation", cmn);
            }
            else
            {
                std::cout << "Adaptive locomotion turned off" << std::endl;
                graph.remove_node("/navigation/test");
            }
        }
        // V
        if((key == 86) && (action == 1))
        {
            // debug_preview = !debug_preview;
            // if (debug_preview){
            //   graph.add_node("/", cm_preview);
            // }else{
            //   graph.remove_node("/cubemap_debug");
            // }
        }
    });

    window->on_button_press.connect([&](int key, int action, int mods) { nav.set_mouse_button(key, action); });

    window->open();

    gua::Renderer renderer;

    // application loop
    gua::events::MainLoop loop;
    gua::events::Ticker ticker(loop, 1.0 / 500.0);

    int count(0);

    ticker.on_tick.connect([&]() {
        // adaptive speed & clipping
        float new_motion_speed = motion_speed;
        if(adaptive_navigation)
        {
            float closest_distance = cmn->get_min_distance();

            if((closest_distance != -1.0) && (closest_distance < 1000.0f))
            {
                // adapt motion speed
                new_motion_speed = closest_distance / 1000.0f;

                // adapt clipping planes
                camera->config.set_near_clip(closest_distance / 100.0);
                camera->config.set_far_clip(closest_distance * 100.0);
            }
            nav.set_motion_speed(new_motion_speed);
        }
        else
        {
            nav.set_motion_speed(new_motion_speed);
        }
        nav.update();
        navigation->set_transform(gua::math::mat4(nav.get_transform()));

        count++;
        if(count == 60)
        {
            count = 0;
            std::cout << "FPS: " << window->get_rendering_fps() << "  Frametime: " << 1000.f / window->get_rendering_fps() << std::endl;
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
