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
#include <gua/renderer/TV_3Loader.hpp>
#include <gua/renderer/TV_3VolumePass.hpp>
#include <gua/renderer/TV_3SurfacePass.hpp>
#include <gua/renderer/TriMeshLoader.hpp>
#include <gua/node/TV_3Node.hpp>
#include <gua/renderer/ToneMappingPass.hpp>
#include <gua/renderer/DebugViewPass.hpp>
#include <gua/utils/Trackball.hpp>

bool enable_spatial_linear_filtering = false;
float iso_value = 0.02; // 0.02;

bool render_in_surface_mode = true;
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
    // std::string in_vol_resource_path2 = "***supernova_parts.v_rsc";
    std::string in_vol_resource_path2 = "./data/objects/Bucky_uncertainty_data_w32_h32_d32_c1_b8.raw";

    // initialize guacamole
    gua::init(argc, argv);

    // setup scene
    gua::SceneGraph graph("main_scenegraph");

    auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");
    auto transform2 = graph.add_node<gua::node::TransformNode>("/transform", "transform2");
    auto plane_transform = graph.add_node<gua::node::TransformNode>("/", "plane_transform");
    gua::TriMeshLoader loader;

    auto plane(loader.create_geometry_from_file("plane", "data/objects/plane.obj", gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));
    graph.add_node("/plane_transform", plane);
    plane->scale(10.0f, 10.0, 10.0);
    plane->rotate(90.0f, 1.0, 0.0, 0.0);
    plane->translate(0.0, 0.0, -10.0);
    /*
      teapot->set_draw_bounding_box(true);
    */

    gua::math::vec4 transparent_platinum(0.672, 0.637, 0.585, 0.8);
    gua::math::vec4 transparent_light_green(0.572549, 1.0, 0.286274, 0.8);
    gua::math::vec4 transparent_nickel(0.660, 0.609, 0.526, 0.8);

    gua::math::vec4 supernova_exaggerated_green(0.572549, 1.0, 0.286274, 2.0);
    gua::math::vec4 supernova_exaggerated_red(1.0, 0.0, 0.0, 2.0);

    gua::math::vec4 iron(0.560, 0.570, 0.580, 1);
    gua::math::vec4 silver(0.972, 0.960, 0.915, 1);
    gua::math::vec4 aluminium(0.913, 0.921, 0.925, 1);
    gua::math::vec4 gold(1.000, 0.766, 0.336, 1);
    gua::math::vec4 copper(0.955, 0.637, 0.538, 1);
    gua::math::vec4 chromium(0.550, 0.556, 0.554, 1);
    gua::math::vec4 nickel(0.660, 0.609, 0.526, 1);
    gua::math::vec4 titanium(0.542, 0.497, 0.449, 1);
    gua::math::vec4 cobalt(0.662, 0.655, 0.634, 1);
    gua::math::vec4 platinum(0.672, 0.637, 0.585, 1);

    auto plod_keep_input_desc = std::make_shared<gua::MaterialShaderDescription>("./data/materials/SimpleMaterial.gmd");
    auto plod_keep_color_shader(std::make_shared<gua::MaterialShader>("PLOD_pass_input_color", plod_keep_input_desc));
    gua::MaterialShaderDatabase::instance()->add(plod_keep_color_shader);

    // create material for pointcloud
    auto shiny_green_mat = plod_keep_color_shader->make_new_material();
    shiny_green_mat->set_uniform("color", supernova_exaggerated_green);
    shiny_green_mat->set_uniform("metalness", 1.0f);
    shiny_green_mat->set_uniform("roughness", 0.2f);
    shiny_green_mat->set_uniform("emissivity", 0.0f);

    auto shiny_red_mat = plod_keep_color_shader->make_new_material();
    shiny_red_mat->set_uniform("color", supernova_exaggerated_red);
    shiny_red_mat->set_uniform("metalness", 1.0f);
    shiny_red_mat->set_uniform("roughness", 0.2f);
    shiny_red_mat->set_uniform("emissivity", 0.0f);

    auto plod_keep_color_shader2(std::make_shared<gua::MaterialShader>("PLOD_pass_input_color2", plod_keep_input_desc));
    gua::MaterialShaderDatabase::instance()->add(plod_keep_color_shader2);
    auto plod_rough2 = plod_keep_color_shader2->make_new_material();
    plod_rough2->set_uniform("color", transparent_nickel);
    plod_rough2->set_uniform("metalness", 0.0f);
    plod_rough2->set_uniform("roughness", 0.9f);
    plod_rough2->set_uniform("emissivity", 0.0f);

    auto plod_keep_color_shader3(std::make_shared<gua::MaterialShader>("PLOD_pass_input_color3", plod_keep_input_desc));
    gua::MaterialShaderDatabase::instance()->add(plod_keep_color_shader3);
    auto plod_rough3 = plod_keep_color_shader3->make_new_material();
    plod_rough3->set_uniform("color", silver);
    plod_rough3->set_uniform("metalness", 0.0f);
    plod_rough3->set_uniform("roughness", 0.9f);
    plod_rough3->set_uniform("emissivity", 0.0f);
    /*
      auto pbrMat(gua::Material
                      ->make_new_material());
      pbrMat->set_uniform("Color", chromium);
      pbrMat->set_uniform("Roughness", 0.2f);
      pbrMat->set_uniform("Metalness", 1.0f);
    */

    gua::TV_3Loader tv_3_loader;
    auto test_volume(tv_3_loader.create_geometry_from_file("test_volume",
                                                           in_vol_resource_path2,

                                                           shiny_green_mat,
                                                           gua::TV_3Loader::NORMALIZE_POSITION | gua::TV_3Loader::NORMALIZE_SCALE));

    auto yet_another_test_volume = test_volume->copy();

    graph.add_node("/transform", yet_another_test_volume);
    yet_another_test_volume->translate(1.0, 0.0, 0.0);
    auto test_tv_3_node3 = std::dynamic_pointer_cast<gua::node::TV_3Node>(yet_another_test_volume);
    test_tv_3_node3->set_render_mode(gua::node::TV_3Node::RenderMode::VOL_MAX_INTENSITY);
    test_tv_3_node3->set_iso_value(0.5);
    test_tv_3_node3->set_material(shiny_red_mat);

    auto yet_another_test_volume2 = test_volume->copy();

    // graph.add_node("/transform", yet_another_test_volume2);
    yet_another_test_volume2->translate(-1.0, 0.0, 0.0);
    auto test_tv_3_node4 = std::dynamic_pointer_cast<gua::node::TV_3Node>(yet_another_test_volume2);
    test_tv_3_node4->set_render_mode(gua::node::TV_3Node::RenderMode::VOL_AVG_INTENSITY);
    test_tv_3_node4->set_iso_value(0.5);

    auto test_volume2(tv_3_loader.create_geometry_from_file("test_volume2", in_vol_resource_path2, plod_rough3, gua::TV_3Loader::NORMALIZE_POSITION | gua::TV_3Loader::NORMALIZE_SCALE));
    auto test_tv_3_node2 = std::dynamic_pointer_cast<gua::node::TV_3Node>(test_volume2);
    test_tv_3_node2->set_iso_value(0.5);
    test_tv_3_node2->set_render_mode(gua::node::TV_3Node::RenderMode::VOL_MAX_INTENSITY);

    auto test_tv_3_node = std::dynamic_pointer_cast<gua::node::TV_3Node>(test_volume);
    test_tv_3_node->set_iso_value(iso_value);
    test_tv_3_node->set_render_mode(gua::node::TV_3Node::RenderMode::SUR_PBR);

    // test_volume->set_draw_bounding_box(true);
    // test_volume->rotate(90, 1.0, 0.0, 0.0);
    // test_volume->rotate(180, 0.0, 1.0, 0.0);
    auto head_scale = scm::math::make_scale(1.0, 1.0, 1.0);
    auto head_rotation = scm::math::make_rotation(180.0, 0.0, 1.0, 0.0) * scm::math::make_rotation(90.0, 1.0, 0.0, 0.0);
    auto head_translation = scm::math::make_translation(0.0, 0.0, 5.0);

    graph.add_node("/transform", test_volume);

    // reinterpret_cast<gua::node::TV_3Node*>(test_volume)->iso_value(0.2);
    // test_volume->translate(0.0, 0.0, 2.0);
    //  reinterpret_cast<gua::node::TV_3Node*>(test_volume.get())->register_clipping_geometry(std::shared_ptr<gua::node::TriMeshNode>(reinterpret_cast<gua::node::TriMeshNode*>(teapot.get()) ) );

    /*
      auto portal = graph.add_node<gua::node::TexturedQuadNode>("/", "portal");
      portal->data.set_size(gua::math::vec2(1.2f, 0.8f));
      portal->data.set_texture("portal");
      portal->translate(0.5f, 0.f, -0.2f);
      portal->rotate(-30, 0.f, 1.f, 0.f);
    */
    auto light2 = graph.add_node<gua::node::LightNode>("/", "light2");
    light2->data.set_type(gua::node::LightNode::Type::SPOT);
    light2->data.brightness = 3.0f;
    light2->scale(100.f);
    // light2->rotate(-90.0f, 1.0f, 0.0f, 0.0f);
    light2->translate(2.0f, 0.0f, 12.0f);
    light2->rotate(10.0f, 0.0f, 1.0f, 0.0f);
    light2->data.set_enable_shadows(true);
    light2->data.set_shadow_map_size(4096);

    light2->data.set_shadow_near_clipping_in_sun_direction(0.01f);
    light2->data.set_shadow_far_clipping_in_sun_direction(100.f);

    auto screen = graph.add_node<gua::node::ScreenNode>("/", "screen");
    screen->data.set_size(gua::math::vec2(1.92f, 1.08f));
    screen->translate(0, 0, 1.0);
    /*
      auto portal_screen =
          graph.add_node<gua::node::ScreenNode>("/", "portal_screen");
      portal_screen->translate(0.0, 0.0, 5.0);
      portal_screen->rotate(90, 0.0, 1.0, 0.0);
      portal_screen->data.set_size(gua::math::vec2(1.2f, 0.8f));
    */
    // add mouse interaction
    gua::utils::Trackball trackball(0.01, 0.002, 0.2);

    // setup rendering pipeline and window
    auto resolution = gua::math::vec2ui(1920, 1080);
    /*
      auto portal_camera =
          graph.add_node<gua::node::CameraNode>("/portal_screen", "portal_cam");
      portal_camera->translate(0, 0, 2.0);
      portal_camera->config.set_resolution(gua::math::vec2ui(1200, 800));
      portal_camera->config.set_screen_path("/portal_screen");
      portal_camera->config.set_scene_graph_name("main_scenegraph");
      portal_camera->config.set_output_texture_name("portal");
      portal_camera->config.set_enable_stereo(false);
    */
    auto portal_pipe = std::make_shared<gua::PipelineDescription>();
    portal_pipe->add_pass(std::make_shared<gua::TriMeshPassDescription>());
    portal_pipe->add_pass(std::make_shared<gua::TV_3SurfacePassDescription>());
    portal_pipe->add_pass(std::make_shared<gua::LightVisibilityPassDescription>());
    auto resolve_pass = std::make_shared<gua::ResolvePassDescription>();
    resolve_pass->background_mode(gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE);
    resolve_pass->tone_mapping_exposure(1.0f);

    portal_pipe->add_pass(resolve_pass);
    // portal_pipe->add_pass(std::make_shared<gua::TV_3VolumePassDescription>());
    portal_pipe->add_pass(std::make_shared<gua::TV_3VolumePassDescription>());
    // portal_pipe->add_pass(std::make_shared<gua::DebugViewPassDescription>());
    portal_pipe->set_enable_abuffer(false);
    portal_pipe->set_abuffer_size(2000);
    // portal_camera->set_pipeline_description(portal_pipe);

    auto camera = graph.add_node<gua::node::CameraNode>("/screen", "cam");
    camera->translate(0, 0, 10.0);
    camera->config.set_resolution(resolution);
    camera->config.set_screen_path("/screen");
    camera->config.set_scene_graph_name("main_scenegraph");
    camera->config.set_output_window_name("main_window");
    camera->config.set_enable_stereo(false);
    camera->set_pipeline_description(portal_pipe);
    // camera->set_pre_render_cameras({portal_camera});

    camera->get_pipeline_description()->get_resolve_pass()->tone_mapping_exposure(1.0f);
    // camera->get_pipeline_description()->add_pass(
    //  std::make_shared<gua::DebugViewPassDescription>());

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

    //////////////////////////////////////////////////////////////////////////////////////
    // key press events
    //////////////////////////////////////////////////////////////////////////////////////
    window->on_key_press.connect([&](int key, int scancode, int action, int mods) {
        if(key == 340)
        { // SHIFT
        }

        if(action == 0)
            return; // only press events

        switch(key)
        {
        case 'W':
            iso_value = std::min(1.0f, iso_value + 0.01f);
            break;
        // case 'A': nav->translate(-speed, 0.0, 0.0); break;
        case 'S':
            iso_value = std::max(0.0f, iso_value - 0.01f);
            break;

        case 'K':
            enable_spatial_linear_filtering = !enable_spatial_linear_filtering;
            test_tv_3_node->enable_spatial_linear_filter(enable_spatial_linear_filtering);
            break;

        case '1':
            test_tv_3_node->set_render_mode(gua::node::TV_3Node::RenderMode::SUR_PBR);
            break;
        case '2':
            test_tv_3_node->set_render_mode(gua::node::TV_3Node::RenderMode::VOL_MAX_INTENSITY);
            break;
        case '3':
            test_tv_3_node->set_render_mode(gua::node::TV_3Node::RenderMode::VOL_AVG_INTENSITY);
            break;
        case '4':
            test_tv_3_node->set_render_mode(gua::node::TV_3Node::RenderMode::VOL_COMPOSITING);
            break;
        case '5':
            test_tv_3_node->set_render_mode(gua::node::TV_3Node::RenderMode::VOL_ISOSURFACE);
            break;

        case 'P':
            if(gua::node::TV_3Node::PlaybackMode::NONE == test_tv_3_node->get_playback_mode())
            {
                test_tv_3_node->set_playback_mode(gua::node::TV_3Node::PlaybackMode::FORWARD);
            }
            else
            {
                test_tv_3_node->set_playback_mode(gua::node::TV_3Node::PlaybackMode::NONE);
            }
            break;
        default:
            break;
        };

        test_tv_3_node->set_iso_value(iso_value);
    });

    gua::Renderer renderer;

    // application loop
    gua::events::MainLoop loop;
    gua::events::Ticker ticker(loop, 1.0 / 500.0);

    size_t ctr{};

    ticker.on_tick.connect([&]() {
        if(++ctr % 150 == 0)
            std::cout << "Frame time: " << 1000.f / window->get_rendering_fps() << " ms, fps: " << window->get_rendering_fps() << ", app fps: " << renderer.get_application_fps() << "\n";

        // apply trackball matrix to object
        gua::math::mat4 modelmatrix =
            head_translation * scm::math::make_translation(trackball.shiftx(), trackball.shifty(), trackball.distance()) * gua::math::mat4(trackball.rotation()) * head_rotation * head_scale;

        transform->set_transform(modelmatrix);

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
