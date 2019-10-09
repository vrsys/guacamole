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

#include <gua/ViveWindow.hpp>
#include <gua/guacamole.hpp>
#include <gua/renderer/SSAAPass.hpp>
#include <gua/renderer/TriMeshLoader.hpp>
#include <gua/utils/Logger.hpp>

////////////////////////////////////////////////////////////////////////////
// demo configuration
////////////////////////////////////////////////////////////////////////////

// lighting
const unsigned light_count = 4;
const float light_proxy_size = 0.01f;

const gua::utils::Color3f light_color(1.0f, 1.0f, 0.9f);
const float light_size = 4.0f;

const float light_base_y = 2.8f;
const float light_base_x = -0.7f;
const float light_base_z = -1.3f;
const float light_diff_x = 2.2f;
const float light_diff_z = 2.6f;

// user navigation
const float user_height = 0.0f;
const float user_speed_walk = 0.02;
const float user_speed_run = 0.2;
bool run_mode = false;

const bool pipeline_enable_alternate_frame_rendering = true;
const bool pipeline_show_fps = true;

////////////////////////////////////////////////////////////////////////////
// main application
////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv) {

  // initialize guacamole
  gua::init(argc, argv);

  // setup scene
  gua::SceneGraph graph("main_scenegraph");

  gua::TriMeshLoader trimesh_loader;

  // another model
  auto model2_material(gua::MaterialShaderDatabase::instance()
                           ->lookup("gua_default_material")
                           ->make_new_material());
  model2_material->set_uniform("Color", gua::math::vec4(1.0, 0.0, 0.0, 1));
  model2_material->set_uniform("Roughness", 0.2f);
  model2_material->set_uniform("Metalness", 0.5f);
  model2_material->set_uniform("Emissivity", 0.3f);

  auto model2(trimesh_loader.create_geometry_from_file(
      "model2", "data/objects/monkey.obj", model2_material));
  model2->scale(0.3, 0.3, 0.3);
  model2->translate(0.5, 1.5, 0.5);

  auto root2 = graph.add_node("/", model2);

  auto nav = graph.add_node<gua::node::TransformNode>("/", "nav");

  auto light2 = graph.add_node<gua::node::LightNode>("/nav", "light2");
  light2->data.set_type(gua::node::LightNode::Type::POINT);
  light2->data.brightness = 10.0f;

  // setup rendering pipeline and window
#if WIN32
  auto window = std::make_shared<gua::ViveWindow>(":0.0");
  gua::WindowDatabase::instance()->add("main_window", window);
  window->config.set_enable_vsync(false);
  window->config.set_fullscreen_mode(false);
  window->adjust_clipping(0.1, 100.0);
#else
  auto window = std::make_shared<gua::ViveWindow>(":0.0");
  gua::WindowDatabase::instance()->add("main_window", window);
  window->config.set_enable_vsync(false);
  window->config.set_fullscreen_mode(false);
  window->config.set_size(window->get_window_resolution());
  window->config.set_resolution(window->get_window_resolution());
  window->open();
#endif
  
  // setup pipeline
  auto resolve_pass = std::make_shared<gua::ResolvePassDescription>();
  resolve_pass->tone_mapping_exposure(1.0f);

  auto vive_controller_0(trimesh_loader.create_geometry_from_file(
      "Vive-HMD-Controller_0",
      "./data/objects/vive_controller/vive_controller.obj",
      gua::TriMeshLoader::LOAD_MATERIALS));
  auto vive_controller_1(trimesh_loader.create_geometry_from_file(
      "Vive-HMD-Controller_1",
      "./data/objects/vive_controller/vive_controller.obj",
      gua::TriMeshLoader::LOAD_MATERIALS));
  auto vive_lighthouse_0(trimesh_loader.create_geometry_from_file(
      "Vive-HMD-Lighthouse_0",
      "./data/objects/vive_lighthouse/vive_lighthouse.obj",
      gua::TriMeshLoader::LOAD_MATERIALS));
  auto vive_lighthouse_1(trimesh_loader.create_geometry_from_file(
      "Vive-HMD-Lighthouse_0",
      "./data/objects/vive_lighthouse/vive_lighthouse.obj",
      gua::TriMeshLoader::LOAD_MATERIALS));

  graph.add_node("/nav", vive_controller_0);
  window->register_node(vive_controller_0, gua::ViveWindow::DeviceID::CONTROLLER_0);
  graph.add_node("/nav", vive_controller_1);
  window->register_node(vive_controller_1, gua::ViveWindow::DeviceID::CONTROLLER_1);
  graph.add_node("/nav", vive_lighthouse_0);
  window->register_node(vive_lighthouse_0, gua::ViveWindow::DeviceID::TRACKING_REFERENCE_0);
  graph.add_node("/nav", vive_lighthouse_1);
  window->register_node(vive_lighthouse_1, gua::ViveWindow::DeviceID::TRACKING_REFERENCE_1);


  
  auto model3(trimesh_loader.create_geometry_from_file(
      "model3", "data/objects/monkey.obj", model2_material));
  model3->scale(0.1, 0.1, 0.1);
  graph.add_node("/nav/Vive-HMD-Controller_0", model3);

  graph.add_node<gua::node::TransformNode>("/nav/Vive-HMD-Controller_1", "test_trans");

  auto model4(trimesh_loader.create_geometry_from_file(
      "model4", "data/objects/monkey.obj", model2_material));
  model4->scale(0.1, 0.1, 0.1);
  graph.add_node("/nav/Vive-HMD-Controller_1/test_trans", model4); 

  // setup camera
  auto camera = graph.add_node<gua::node::CameraNode>("/nav", "Vive-HMD-User_0");
  camera->config.set_resolution(window->get_window_resolution());
  camera->config.set_left_screen_path("/nav/Vive-HMD-User_0/left_screen");
  camera->config.set_right_screen_path("/nav/Vive-HMD-User_0/right_screen");
  camera->config.set_scene_graph_name("main_scenegraph");
  camera->config.set_output_window_name("main_window");
  camera->config.set_enable_stereo(true);
  camera->config.set_eye_dist(window->get_IPD());

  camera->get_pipeline_description()->add_pass(
      std::make_shared<gua::SSAAPassDescription>());

  camera->get_pipeline_description()->get_resolve_pass()->tone_mapping_exposure(
      1.0f);
  camera->get_pipeline_description()->get_resolve_pass()->background_mode(
      gua::ResolvePassDescription::BackgroundMode::SKYMAP_TEXTURE);
  camera->get_pipeline_description()->get_resolve_pass()->background_texture(
      "data/textures/skymap.jpg");

  auto left_screen = graph.add_node<gua::node::ScreenNode>(
      "/nav/Vive-HMD-User_0", "left_screen");
  left_screen->data.set_size(window->get_left_screen_size());
  left_screen->translate(window->get_left_screen_translation());

  auto right_screen = graph.add_node<gua::node::ScreenNode>(
      "/nav/Vive-HMD-User_0", "right_screen");
  right_screen->data.set_size(window->get_right_screen_size());
  right_screen->translate(window->get_right_screen_translation());

  //////////////////////////////////////////////////////////////////////////////////////
  // setup rendering
  //////////////////////////////////////////////////////////////////////////////////////
  double time = 0.0;                             // current time
  long long ctr = 0;                             // frame timer
  const float desired_frame_time = 1.0 / 240.0;  // desired application of 180Hz

  // setup application loop
  gua::Renderer renderer;
  gua::events::MainLoop loop;
  gua::events::Ticker ticker(loop, desired_frame_time);

  gua::Timer timer;
  timer.start();
   
  float latest_trigger_value = 0.0f;

  //////////////////////////////////////////////////////////////////////////////////////
  // mainloop
  //////////////////////////////////////////////////////////////////////////////////////
  ticker.on_tick.connect([&]() {
    double frame_time(timer.get_elapsed());
    time += frame_time;
    timer.reset();

    // animation
    model2->set_transform(scm::math::make_translation(0.0, 5.0, 0.0) * scm::math::make_rotation(time * 5.0, 0.0, 1.0, 0.0));
    model2->scale(0.2f, 0.2f, 0.2f);

    light2->set_transform(camera->get_transform() *
                          scm::math::make_scale(2.0, 2.0, 2.0));

   /*
	//for the retrieval of one of the binary states use get_controller_button_active
	if(window->get_controller_button_active(gua::ViveWindow::DeviceID::CONTROLLER_0,
	gua::ViveWindow::ControllerBinaryStates::TRIGGER_BUTTON)) { std::cout <<
	"Controller 0 Pressed: TRIGGER_BUTTON\n";
	}

	//for the retrieval of one of the continuous states PAD_X_VALUE, PAD_Y_VALUE,
	TRIGGER_VALUE use get_controller_value float trigger_value =
	window->get_controller_value(gua::ViveWindow::DeviceID::CONTROLLER_0,
	gua::ViveWindow::ControllerContinuousStates::TRIGGER_VALUE);


	if(trigger_value != latest_trigger_value) {
		latest_trigger_value = trigger_value;
		std::cout << "Controller 0 Trigger Value Changed: " << trigger_value <<
	"\n";
	}
  */

   // window update
   if (window->should_close()) {
      renderer.stop();
      window->close();
      loop.stop();
   }
   else {
      renderer.queue_draw({&graph}, pipeline_enable_alternate_frame_rendering);
      // renderer.draw_single_threaded({&graph});
   }

   if (ctr++ % 500 == 0 && pipeline_show_fps) {
      std::cout << "Frame time: " << 1000.f / window->get_rendering_fps()
                << " ms, fps: " << window->get_rendering_fps()
                << ", app fps: " << renderer.get_application_fps() << std::endl;
   }


  });

  loop.start();

  return 0;
}
