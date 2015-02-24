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
#include <gua/utils/Trackball.hpp>

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

void key_press(gua::PipelineDescription& pipe, gua::SceneGraph& graph, int key, int scancode, int action, int mods)
{
  if (action == 0) return;

  switch (std::tolower(key))
  {
  case 'm': // toggle environment lighting mode
    
    if (pipe.get_resolve_pass()->environment_lighting_mode() == gua::ResolvePassDescription::EnvironmentLightingMode::AMBIENT_COLOR) {
      std::cout << "Setting to gua::ResolvePassDescription::EnvironmentLightingMode::SPHEREMAP" << std::endl;
      pipe.get_resolve_pass()->environment_lighting_mode(gua::ResolvePassDescription::EnvironmentLightingMode::SPHEREMAP);
    }
    else if (pipe.get_resolve_pass()->environment_lighting_mode() == gua::ResolvePassDescription::EnvironmentLightingMode::SPHEREMAP) {
      std::cout << "Setting to gua::ResolvePassDescription::EnvironmentLightingMode::CUBEMAP" << std::endl;
      pipe.get_resolve_pass()->environment_lighting_mode(gua::ResolvePassDescription::EnvironmentLightingMode::CUBEMAP);
    }
    else {
      std::cout << "Setting to gua::ResolvePassDescription::EnvironmentLightingMode::AMBIENT_COLOR" << std::endl;
      pipe.get_resolve_pass()->environment_lighting_mode(gua::ResolvePassDescription::EnvironmentLightingMode::AMBIENT_COLOR);
    }

    pipe.get_resolve_pass()->touch();
    break;

  case 'b': // toggle background mode

    if (pipe.get_resolve_pass()->background_mode() == gua::ResolvePassDescription::BackgroundMode::COLOR) {
      std::cout << "Setting to gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE" << std::endl;
      pipe.get_resolve_pass()->background_mode(gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE);
    }
    else if (pipe.get_resolve_pass()->background_mode() == gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE) {
      std::cout << "Setting to gua::ResolvePassDescription::BackgroundMode::SKYMAP_TEXTURE" << std::endl;
      pipe.get_resolve_pass()->background_mode(gua::ResolvePassDescription::BackgroundMode::SKYMAP_TEXTURE);
    }
    else {
      std::cout << "Setting to gua::ResolvePassDescription::BackgroundMode::AMBIENT_COLOR" << std::endl;
      pipe.get_resolve_pass()->background_mode(gua::ResolvePassDescription::BackgroundMode::COLOR);
    }

    pipe.get_resolve_pass()->touch();
    break;

  case 's':  // toggle SSAO
    pipe.get_resolve_pass()->ssao_enable(!pipe.get_resolve_pass()->ssao_enable());
    break;

  case '1': 
    pipe.get_resolve_pass()->ssao_intensity(std::min(10.0, 1.1 * pipe.get_resolve_pass()->ssao_intensity()));
    break;
  case '2':
    pipe.get_resolve_pass()->ssao_intensity(std::max(0.02, 0.9 * pipe.get_resolve_pass()->ssao_intensity()));
    break;

  case '3':
    pipe.get_resolve_pass()->ssao_radius(std::min(256.0, 1.1 * pipe.get_resolve_pass()->ssao_radius()));
    break;
  case '4':
    pipe.get_resolve_pass()->ssao_radius(std::max(1.0, 0.9 * pipe.get_resolve_pass()->ssao_radius()));
    break;

  case 't':

    pipe.get_resolve_pass()->touch();
    break;

  default:
    break;
  }

}



int main(int argc, char** argv) {

  // initialize guacamole
  gua::init(argc, argv);

  // setup scene
  gua::SceneGraph graph("main_scenegraph");

  gua::TriMeshLoader loader;

  auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");
  auto monkey(loader.create_geometry_from_file("teapot", "data/objects/Shrek.obj", gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE | gua::TriMeshLoader::LOAD_MATERIALS));
  graph.add_node("/transform", monkey);
  monkey->set_draw_bounding_box(true);

  auto light = graph.add_node<gua::node::PointLightNode>("/", "light2");
  light->data.brightness = 40.0f;
  light->scale(9.f);
  light->translate(-3.f, 5.f, 5.f);

  auto screen = graph.add_node<gua::node::ScreenNode>("/", "screen");
  screen->data.set_size(gua::math::vec2(1.92f, 1.08f));
  screen->translate(0, 0, 1.0);

  // add mouse interaction
  gua::utils::Trackball trackball(0.01, 0.002, 0.2);

  // setup rendering pipeline and window
  auto resolution = gua::math::vec2ui(1920, 1080);

  auto camera = graph.add_node<gua::node::CameraNode>("/screen", "cam");

  camera->translate(0, 0, 2.0);

  camera->config.set_resolution(resolution);
  camera->config.set_screen_path("/screen");
  camera->config.set_scene_graph_name("main_scenegraph");
  camera->config.set_output_window_name("main_window");
  camera->config.set_enable_stereo(false);

  camera->get_pipeline_description()->get_resolve_pass()->tone_mapping_exposure(1.0f);

  camera->get_pipeline_description()->get_resolve_pass()->ssao_intensity(2.0);
  camera->get_pipeline_description()->get_resolve_pass()->ssao_enable(true);
  camera->get_pipeline_description()->get_resolve_pass()->ssao_radius(16.0);

  camera->get_pipeline_description()->get_resolve_pass()->environment_lighting_mode(gua::ResolvePassDescription::EnvironmentLightingMode::SPHEREMAP);
  camera->get_pipeline_description()->get_resolve_pass()->environment_lighting_spheremap("data/textures/envlightmap.jpg");

  camera->get_pipeline_description()->get_resolve_pass()->background_mode(gua::ResolvePassDescription::BackgroundMode::SKYMAP_TEXTURE);
  camera->get_pipeline_description()->get_resolve_pass()->background_texture("data/textures/envmap.jpg");

  camera->get_pipeline_description()->add_pass(std::make_shared<gua::DebugViewPassDescription>());

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

  window->on_move_cursor.connect([&](gua::math::vec2 const& pos) {
    trackball.motion(pos.x, pos.y);
  });

  window->on_button_press.connect(std::bind(mouse_button, std::ref(trackball), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  window->on_key_press.connect(std::bind(key_press,
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
  gua::events::Ticker ticker(loop, 1.0/500.0);

  ticker.on_tick.connect([&]() {

    // apply trackball matrix to object
    auto modelmatrix = scm::math::make_translation(trackball.shiftx(), trackball.shifty(), trackball.distance()) * trackball.rotation();
    transform->set_transform(modelmatrix);

    window->process_events();
    if (window->should_close()) {
      renderer.stop();
      window->close();
      loop.stop();
    } else {
      renderer.queue_draw({&graph}, {camera});
    }
  });

  loop.start();

  return 0;
}
