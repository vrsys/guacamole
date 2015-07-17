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
#include <gua/renderer/ToneMappingPass.hpp>
#include <gua/renderer/DebugViewPass.hpp>
#include <gua/renderer/DepthCubeMapPass.hpp>
#include <gua/renderer/SSAAPass.hpp>
#include <gua/renderer/TexturedScreenSpaceQuadPass.hpp>
#include <gua/utils/Trackball.hpp>
#include <gua/gui.hpp>
 
#include "Navigator.hpp"

 #define FULLSCREEN false
 #define RECURSIVE_OILRIGS true
 #define ELEPHANT true
 #define SKYMAP true

// forward mouse interaction to trackball
void mouse_button (gua::utils::Trackball& trackball, int mousebutton, int action, int mods)
{
  gua::utils::Trackball::button_type button;
  gua::utils::Trackball::state_type state;

  switch (mousebutton) {
    case 0: button = gua::utils::Trackball::left; break;
    case 2: button = gua::utils::Trackball::middle; break;
    case 1: button = gua::utils::Trackball::right; break;
  };

  switch (action) {
    case 0: state = gua::utils::Trackball::released; break;
    case 1: state = gua::utils::Trackball::pressed; break;
  };

  trackball.mouse(button, state, trackball.posx(), trackball.posy());
}

int main(int argc, char** argv) {

  // add interaction
  // gua::utils::Trackball object_trackball(0.01, 0.002, 0, 0.2);
  Navigator nav;
  nav.set_transform(scm::math::make_translation(0.f, 0.f, 60.f));

  // initialize guacamole
  gua::init(argc, argv);

  // setup scene
  gua::SceneGraph graph("main_scenegraph");

  gua::TriMeshLoader loader;


  auto navigation = graph.add_node<gua::node::TransformNode>("/", "navigation");

  auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");


  
  // MODELS
  if (RECURSIVE_OILRIGS)
  {
    auto much_big_oilrig(loader.create_geometry_from_file("much_big_oilrig", "/opt/3d_models/OIL_RIG_GUACAMOLE/oilrig.obj",
      gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION |
      gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::OPTIMIZE_MATERIALS |
      gua::TriMeshLoader::NORMALIZE_SCALE));

    much_big_oilrig->rotate(-90.0f, 1.0f, 0.0f, 0.0f);
    much_big_oilrig->scale(100.0);

    graph.add_node(transform, much_big_oilrig);


    auto big_oilrig(loader.create_geometry_from_file("big_oilrig", "/opt/3d_models/OIL_RIG_GUACAMOLE/oilrig.obj",
      gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION |
      gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::OPTIMIZE_MATERIALS |
      gua::TriMeshLoader::NORMALIZE_SCALE));

    big_oilrig->rotate(-90.0f, 1.0f, 0.0f, 0.0f);
    big_oilrig->scale(10);
    big_oilrig->translate(18.f, 1.0f, 8.f);

    graph.add_node(transform, big_oilrig);


    auto oilrig(loader.create_geometry_from_file("oilrig", "/opt/3d_models/OIL_RIG_GUACAMOLE/oilrig.obj",
      gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION |
      gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::OPTIMIZE_MATERIALS |
      gua::TriMeshLoader::NORMALIZE_SCALE));

    oilrig->rotate(-90.0f, 1.0f, 0.0f, 0.0f);
    // oilrig->scale(0.1);
    oilrig->translate(19.8f, 1.1f, 8.8f);

    graph.add_node(transform, oilrig);


    auto small_oilrig(loader.create_geometry_from_file("small_oilrig", "/opt/3d_models/OIL_RIG_GUACAMOLE/oilrig.obj",
      gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION |
      gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::OPTIMIZE_MATERIALS |
      gua::TriMeshLoader::NORMALIZE_SCALE));

    small_oilrig->rotate(-90.0f, 1.0f, 0.0f, 0.0f);
    small_oilrig->scale(0.1);
    small_oilrig->translate(19.98f, 1.11f, 8.88f);

    graph.add_node(transform, small_oilrig);
  }

  if (ELEPHANT)
  {
    auto elephant(loader.create_geometry_from_file("elephant", "/opt/3d_models/animals/elephant/elephant.obj",
      gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION |
      gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::OPTIMIZE_MATERIALS |
      gua::TriMeshLoader::NORMALIZE_SCALE));

    elephant->rotate(-180.0f, 1.0f, 0.0f, 0.0f);
    elephant->rotate(90.0f, 0.0f, 1.0f, 0.0f);
    elephant->scale(0.002);
    elephant->translate(19.957f, 1.1201f, 8.892f);

    graph.add_node(transform, elephant);
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

  camera->config.set_near_clip(0.002f);
  camera->config.set_far_clip(200.0f);

  camera->get_pipeline_description()->get_resolve_pass()->tone_mapping_exposure(1.0f);

  if (SKYMAP)
  {
  camera->get_pipeline_description()->get_resolve_pass()->background_mode(gua::ResolvePassDescription::BackgroundMode::SKYMAP_TEXTURE);
  camera->get_pipeline_description()->get_resolve_pass()->background_texture("data/textures/skymap.jpg");
  }

  camera->get_pipeline_description()->add_pass(std::make_shared<gua::DepthCubeMapPassDesciption>());
  camera->get_pipeline_description()->add_pass(std::make_shared<gua::SSAAPassDescription>());

  // GUI
  // gua::math::vec2 gui_size(150.f, 80.f);
  // auto gui = std::make_shared<gua::GuiResource>();
  // gui->init("gui", "asset://gua/data/gui/fps.html", gui_size);
  
  // auto gui_quad = std::make_shared<gua::node::TexturedScreenSpaceQuadNode>("gui_quad");
  // gui_quad->data.texture() = "gui";
  // gui_quad->data.size() = gui_size;
  // gui_quad->data.anchor() = gua::math::vec2(1.f, 1.f);

  // graph.add_node("/", gui_quad);
  
  // CUBEMAP NAVIGATION
  bool adaptive_navigation(true);
  auto cmn(graph.add_node<gua::node::CubemapNode>("/navigation", "test"));
  cmn->set_texture_name("navigation_depth_texture");
  float motion_speed = 0.03f;

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
    screen->data.set_size(gua::math::vec2(0.001 * new_size.x, 0.001 * new_size.y));
  });
  window->on_move_cursor.connect([&](gua::math::vec2 const& pos) {
    // trackball.motion(pos.x, pos.y);
    nav.set_mouse_position(gua::math::vec2i(pos));
  });

  window->on_key_press.connect([&](int key, int scancode, int action, int mods) {
    nav.set_key_press(static_cast<gua::Key>(key), action);
    // std::cout << key << " " << action << std::endl;       
    if ((key == 257) && (action == 1)){
      adaptive_navigation = !adaptive_navigation;
      if (adaptive_navigation){
        std::cout << "Adaptive locomotion turned on" << std::endl;       
      }else{
        std::cout << "Adaptive locomotion turned off" << std::endl;       
      }
    }
  });  

  window->on_button_press.connect([&](int key, int action, int mods) {
    nav.set_mouse_button(key, action);
  });

  window->open();


  gua::Renderer renderer;

  // application loop
  gua::events::MainLoop loop;
  gua::events::Ticker ticker(loop, 1.0/500.0);

  int count(0);

  ticker.on_tick.connect([&]() {
  //   std::stringstream sstr;
  //   sstr.precision(1);
  //   sstr.setf(std::ios::fixed, std::ios::floatfield);
  //   sstr << "FPS: " << renderer.get_application_fps()
  //        << " / " << window->get_rendering_fps();
  //   // std::cout << sstr.str() << std::endl;
  //   gui->call_javascript("set_fps_text", sstr.str());


    // apply trackball matrix to object
    float new_motion_speed = motion_speed;
    if (adaptive_navigation) {

      float closest_distance = cmn->get_closest_distance();
      if ((closest_distance != -1.0) && (closest_distance < 30.0f)){
        new_motion_speed = closest_distance / 1000.0f;
      }
      nav.set_motion_speed(new_motion_speed);

      // std::cout << motion_speed << std::endl;
    } else {
      nav.set_motion_speed(new_motion_speed);
    }
    nav.update();
    navigation->set_transform(gua::math::mat4(nav.get_transform()));
    // std::cout << nav.get_transform() << std::endl;
    
    count++;
    if(count == 60){
      count = 0;
      std::cout << new_motion_speed*60 << std::endl;
    }

    // gua::math::mat4 modelmatrix = scm::math::make_translation(gua::math::float_t(trackball.shiftx()),
                                                              // gua::math::float_t(trackball.shifty()),
                                                              // gua::math::float_t(trackball.distance())) * gua::math::mat4(trackball.rotation());

    // transform->set_transform(modelmatrix);

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
