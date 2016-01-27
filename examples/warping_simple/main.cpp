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

#include "Navigator.hpp"

const float aspect = 1.0f/1.6f;
const float screen_width = 4.f;
const float screen_dist = 2.5f;
const int   window_size = 1600;
const float eye_dist = 0.1f;
const bool fullscreen = false;
const bool vsync = false;


int main(int argc, char** argv) {

  // initialize guacamole
  gua::init(argc, argv);

  // ---------------------------------------------------------------------------
  // ---------------------------- setup scene ----------------------------------
  // ---------------------------------------------------------------------------

  gua::SceneGraph graph("main_scenegraph");

  #if WIN32
    //std::string opt_prefix("D:/guacamole/");
    std::string opt_prefix("C:/Users/localadmin/Desktop/Simon/opt/");
	#else
    std::string opt_prefix("/opt/");
	#endif

  gua::TriMeshLoader loader;
  auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");
  transform->get_tags().add_tag("scene");

  auto sun_light = graph.add_node<gua::node::LightNode>("/", "sun_light");
  sun_light->data.set_type(gua::node::LightNode::Type::SUN);
  sun_light->data.set_color(gua::utils::Color3f(1.5f, 1.2f, 1.f));
  sun_light->data.set_shadow_cascaded_splits({0.1f, 1.5, 5.f, 10.f});
  sun_light->data.set_shadow_near_clipping_in_sun_direction(100.0f);
  sun_light->data.set_shadow_far_clipping_in_sun_direction(100.0f);
  sun_light->data.set_max_shadow_dist(30.0f);
  sun_light->data.set_shadow_offset(0.0004f);
  sun_light->data.set_enable_shadows(true);
  sun_light->data.set_shadow_map_size(512);
  sun_light->rotate(-65, 1, 0, 0);
  sun_light->rotate(-100, 0, 1, 0);

  auto light2 = graph.add_node<gua::node::LightNode>("/", "light2");
  light2->data.set_type(gua::node::LightNode::Type::SUN);
  light2->data.set_color(gua::utils::Color3f(0.8f, 1.0f, 1.5f));
  light2->data.set_brightness(1.5f);
  light2->data.set_enable_specular_shading(false);
  light2->rotate(-45, 1, 0, 0);
  light2->rotate(-120, 0, 1, 0);

  auto light3 = graph.add_node<gua::node::LightNode>("/", "light3");
  light3->data.set_type(gua::node::LightNode::Type::SUN);
  light3->data.set_color(gua::utils::Color3f(0.9f, 1.3f, 1.1f));
  light3->data.set_brightness(1.0f);
  light3->data.set_enable_specular_shading(false);
  light3->rotate(45, 1, 0, 0);
  light3->rotate(80, 0, 1, 0);

  auto sponza(loader.create_geometry_from_file("sponza", opt_prefix + "3d_models/sponza_with_windows/sponza.obj",
    gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION |
    gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::OPTIMIZE_MATERIALS |
    gua::TriMeshLoader::NORMALIZE_SCALE));
  sponza->rotate(40, 0, 1, 0);
  sponza->scale(20);
  sponza->translate(0, 2, 0);
  graph.get_root()->add_child(sponza);


  // ---------------------------------------------------------------------------
  // ------------------------ setup rendering pipeline -------------------------
  // ---------------------------------------------------------------------------

  auto resolution = gua::math::vec2ui(window_size, window_size*aspect);

  auto pipe = gua::PipelineFactory::make_pipeline(
    gua::PipelineFactory::DEFAULT | 
    gua::PipelineFactory::ABUFFER |
    gua::PipelineFactory::WARPING
  );

  pipe->get_resolve_pass()->
    background_mode(gua::ResolvePassDescription::BackgroundMode::SKYMAP_TEXTURE).
    environment_lighting_texture(opt_prefix + "guacamole/resources/skymaps/DH206SN.png").
    environment_lighting(gua::utils::Color3f(0.4, 0.4, 0.5)).
    environment_lighting_mode(gua::ResolvePassDescription::EnvironmentLightingMode::AMBIENT_COLOR).
    tone_mapping_method(gua::ResolvePassDescription::ToneMappingMethod::HEJL).
    tone_mapping_exposure(1.5f).horizon_fade(0.2f).
    background_texture(opt_prefix + "guacamole/resources/skymaps/cycles_island.jpg");

  // ---------------------------------------------------------------------------
  // --------------------------- camera / navigation ---------------------------
  // ---------------------------------------------------------------------------

  Navigator navigator;
  navigator.set_transform(scm::math::mat4f(0.637, 0.067, -0.768, -4.160,
                                           0.000, 0.996, 0.086, 2.715,
                                           0.771, -0.055, 0.635, 2.682,
                                           0.000, 0.000, 0.000, 1.000));

  auto navigation = graph.add_node<gua::node::TransformNode>("/", "navigation");
  auto warp_navigation = graph.add_node<gua::node::TransformNode>("/navigation", "warp");

  auto screen = graph.add_node<gua::node::ScreenNode>("/navigation", "screen");
  auto cam = graph.add_node<gua::node::CameraNode>("/navigation", "cam");
  screen->data.set_size(gua::math::vec2(screen_width,screen_width*aspect));
  screen->translate(0, 0, -screen_dist);
  cam->config.set_resolution(resolution);
  cam->config.set_screen_path("/navigation/screen");
  cam->config.set_scene_graph_name("main_scenegraph");
  cam->config.set_eye_dist(eye_dist);
  cam->config.set_enable_stereo(true);
  cam->config.set_output_window_name("window");
  cam->set_pipeline_description(pipe);

  // ---------------------------------------------------------------------------
  // ----------------------------- setup window --------------------------------
  // ---------------------------------------------------------------------------

  auto window = std::make_shared<gua::GlfwWindow>();
  window->config.set_fullscreen_mode(fullscreen);
  window->config.set_enable_vsync(vsync);
  window->config.set_size(resolution);
  window->config.set_resolution(resolution);
  window->config.set_stereo_mode(gua::StereoMode::ANAGLYPH_RED_CYAN);
  window->cursor_mode(gua::GlfwWindow::CursorMode::HIDDEN);
  window->open();

  gua::WindowDatabase::instance()->add("window", window);

  window->on_resize.connect([&](gua::math::vec2ui const& new_size) {
    resolution = new_size;
    window->config.set_resolution(new_size);
    cam->config.set_resolution(new_size);
    screen->data.set_size(gua::math::vec2(screen_width, screen_width * new_size.y / new_size.x));
  });

  window->on_button_press.connect([&](int key, int action, int mods) {
    navigator.set_mouse_button(key, action);
  });

  window->on_key_press.connect([&](int key, int scancode, int action, int mods) {
    navigator.set_key_press(key, action);
    if (action >= 1) {
      if (key == 49) {
        std::cout << "RENDER_TWICE" << std::endl;
        cam->config.set_stereo_type(gua::StereoType::RENDER_TWICE);
      }
      if (key == 50) {
        std::cout << "SPATIAL_WARP" << std::endl;
        cam->config.set_stereo_type(gua::StereoType::SPATIAL_WARP);
      }
      if (key == 51) {
        std::cout << "TEMPORAL_WARP" << std::endl;
        cam->config.set_stereo_type(gua::StereoType::TEMPORAL_WARP);
      }
    }
  });

  window->on_move_cursor.connect([&](gua::math::vec2 const& pos) {
    navigator.set_mouse_position(gua::math::vec2i(pos));
  });

  // ---------------------------------------------------------------------------
  // ---------------------------- application loop -----------------------------
  // ---------------------------------------------------------------------------

  gua::Renderer renderer;

  gua::events::MainLoop loop;
  gua::events::Ticker ticker(loop, 1.0/60.0);

  ticker.on_tick.connect([&]() {
    navigator.update();
    navigation->set_transform(gua::math::mat4(navigator.get_transform()));

    std::cout << window->get_rendering_fps() << std::endl;

    window->process_events();

    if (window->should_close()) {
      renderer.stop();
      window->close();
      loop.stop();
    } else {
      renderer.queue_draw({&graph});
    }
  });

  loop.start();

  return 0;
}
