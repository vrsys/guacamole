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

#include <gua/guacamole.hpp>
#include <gua/renderer/TriMeshLoader.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/OculusSDK2Window.hpp>

const std::string geometry("data/objects/monkey.obj");

std::vector<std::shared_ptr<gua::node::TransformNode>> add_lights(gua::SceneGraph& graph,
                                                  int count) {

  std::vector<std::shared_ptr<gua::node::TransformNode>> lights(count);

  for (int i(0); i < count; ++i) {
    scm::math::vec3 randdir(gua::math::random::get(-10.f, 10.f),
                            gua::math::random::get(-10.f, 10.f),
                            gua::math::random::get(-10.f, 10.f));
    scm::math::normalize(randdir);

    gua::TriMeshLoader loader;
    auto sphere_geometry(
      loader.create_geometry_from_file(
      "sphere" + gua::string_utils::to_string(i),
      "data/objects/light_sphere.obj"
    ));

    sphere_geometry->scale(0.4, 0.4, 0.4);

    lights[i] = graph.add_node("/", std::make_shared<gua::node::TransformNode>("light" + gua::string_utils::to_string(i)));
    lights[i]->add_child(sphere_geometry);
    lights[i]->translate(randdir[0], randdir[1], randdir[2]);

    auto light = lights[i]->add_child(std::make_shared<gua::node::LightNode>("light"));
    light->data.set_type(gua::node::LightNode::Type::POINT);
    light->data.set_color(gua::utils::Color3f::random());
    light->scale(20);
  }

  return lights;
}

int main(int argc, char** argv) {
  // initialize guacamole
  gua::init(argc, argv);

  // initialize Oculus SDK
  gua::OculusSDK2Window::initialize_oculus_environment();

  // setup scene
  gua::SceneGraph graph("main_scenegraph");

  gua::TriMeshLoader loader;

#if WIN32
  //std::string opt_prefix("D:/guacamole/");
  std::string opt_prefix("C:/Users/localadmin/Desktop/Simon/opt/");
#else
  std::string opt_prefix("/opt/");
#endif

  auto add_oilrig = [&](std::string const& parent) {
    auto t = graph.add_node<gua::node::TransformNode>(parent, "t");
    auto oilrig(loader.create_geometry_from_file("oilrig", opt_prefix + "3d_models/OIL_RIG_GUACAMOLE/oilrig.obj",
      gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION |
      gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::OPTIMIZE_MATERIALS |
      gua::TriMeshLoader::NORMALIZE_SCALE));
    t->add_child(oilrig);

  };

  auto scene_root = graph.add_node<gua::node::TransformNode>("/", "model");
  scene_root->scale(70);
  scene_root->rotate(-90, 1, 0, 0);
  scene_root->translate(12, -1, -20);
  add_oilrig("/model");

  auto lights = add_lights(graph, 5);

  auto nav = graph.add_node<gua::node::TransformNode>("/", "nav");
  nav->translate(0.0, 0.0, 1.0);

  // setup rendering pipeline and window
  auto window = std::make_shared<gua::OculusSDK2Window>(":0.0");
  gua::WindowDatabase::instance()->add("main_window", window);
  window->config.set_enable_vsync(false);
  window->config.set_fullscreen_mode(true);

  window->open();

  auto camera = graph.add_node<gua::node::CameraNode>("/nav", "cam");

  //camera->translate(0, 0, 2.0);
  camera->config.set_resolution(window->config.get_left_resolution());
  camera->config.set_left_screen_path("/nav/cam/left_screen");
  camera->config.set_right_screen_path("/nav/cam/right_screen");
  camera->config.set_scene_graph_name("main_scenegraph");
  camera->config.set_output_window_name("main_window");
  camera->config.set_enable_stereo(true);
  camera->config.set_eye_dist(0.0635);

  camera->get_pipeline_description()->get_resolve_pass()->tone_mapping_exposure(1.0f);
  camera->get_pipeline_description()->get_resolve_pass()->background_mode(gua::ResolvePassDescription::BackgroundMode::SKYMAP_TEXTURE);
  camera->get_pipeline_description()->get_resolve_pass()->background_texture(opt_prefix + "guacamole/resources/skymaps/cycles_island.jpg");

  auto left_screen = graph.add_node<gua::node::ScreenNode>("/nav/cam", "left_screen");
  left_screen->data.set_size(gua::math::vec2(0.17074, 0.21));
  left_screen->translate(-0.03175, 0, -0.08f);

  auto right_screen = graph.add_node<gua::node::ScreenNode>("/nav/cam", "right_screen");
  right_screen->data.set_size(gua::math::vec2(0.17074, 0.21));
  right_screen->translate(0.03175, 0, -0.08f);

  gua::Renderer renderer;

  gua::Timer timer;
  timer.start();

  double time(0);

  // application loop
  while (true) {
    double frame_time(timer.get_elapsed());
    time += frame_time;
    timer.reset();

    camera->set_transform(window->get_oculus_sensor_orientation());

    std::cout << window->get_rendering_fps() << " " << renderer.get_application_fps() << std::endl;

    window->process_events();
    if (window->should_close()) {
      renderer.stop();
      window->close();
      break;
    }
    else {
      renderer.draw_single_threaded({ &graph });
    }
  }

  gua::OculusSDK2Window::shutdown_oculus_environment();
  return 0;
}
