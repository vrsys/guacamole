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

#include <gua/renderer/NURBSPass.hpp>
#include <gua/renderer/BBoxPass.hpp>
#include <gua/renderer/TexturedQuadPass.hpp>
#include <gua/renderer/TexturedScreenSpaceQuadPass.hpp>
#include <gua/renderer/NURBSLoader.hpp>
#include <gua/utils/Trackball.hpp>
#include <gua/node/NURBSNode.hpp>

// forward mouse interaction to trackball
void mouse_button(gua::utils::Trackball& trackball, int mousebutton, int action, int mods)
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

void key_press(gua::PipelineDescription& pipe, int key, int scancode, int action, int mods)
{
  char k = std::tolower(key);
  switch (k)
  {
  case 'r' :
    auto& trimesh_desc = pipe.get_pass<gua::NURBSPassDescription>();
    std::cout << "setting dirty flag " << std::endl;
    trimesh_desc.touch();
    break;
  }
}

int main(int argc, char** argv) {

  // initialize guacamole
  gua::init(argc, argv);

  // setup scene
  gua::SceneGraph graph("main_scenegraph");

  gua::MaterialShaderDescription desc;
  desc.load_from_file("data/materials/SimpleMaterial.gmd");

  auto shader(std::make_shared<gua::MaterialShader>("simple_mat", desc));
  gua::MaterialShaderDatabase::instance()->add(shader);

  gua::NURBSLoader nurbs_loader;
  gua::TriMeshLoader loader;

  auto input_transform = graph.add_node<gua::node::TransformNode>("/", "nurbs_transform");

  auto nurbs_object (nurbs_loader.load_geometry(
    "nurbs_object",
    //"I:/models/Paris/Paris2010_0.obj",
    "data/objects/teapot.igs",
    shader->get_default_material(),
    gua::NURBSLoader::NORMALIZE_POSITION |
    gua::NURBSLoader::NORMALIZE_SCALE// |
    //gua::NURBSLoader::WIREFRAME
    ));
  input_transform->add_child(nurbs_object);

  auto resolution = gua::math::vec2ui(1920, 1080);

  auto light = graph.add_node<gua::node::PointLightNode>("/", "light");
  light->scale(4.4f);
  light->translate(1.f, 0.f, 2.f);

  auto light2 = graph.add_node<gua::node::PointLightNode>("/", "light2");
  light2->data.color = gua::utils::Color3f(1.0f, 1.0f, 1.0f);
  light2->scale(3.4f);
  light2->translate(-2.f, 1.f, 2.f);

  auto screen = graph.add_node<gua::node::ScreenNode>("/", "screen");
  screen->data.set_size(gua::math::vec2(0.001 * resolution.x, 0.001 * resolution.y));
  screen->translate(0, 0, 1.0);

  auto camera = graph.add_node<gua::node::CameraNode>("/screen", "cam");
  camera->translate(0, 0, 2.0);
  camera->config.set_resolution(resolution);
  camera->config.set_screen_path("/screen");
  camera->config.set_scene_graph_name("main_scenegraph");
  camera->config.set_output_window_name("main_window");
  camera->config.set_enable_frustum_culling(false);

  gua::PipelineDescription pipe;
  pipe.add_pass<gua::TriMeshPassDescription>();
  pipe.add_pass<gua::NURBSPassDescription>();
  pipe.add_pass<gua::TexturedQuadPassDescription>();
  pipe.add_pass<gua::SSAOPassDescription>();
  pipe.add_pass<gua::EmissivePassDescription>();
  pipe.add_pass<gua::LightingPassDescription>();
  pipe.add_pass<gua::BBoxPassDescription>();
  pipe.add_pass<gua::BackgroundPassDescription>();
  pipe.add_pass<gua::TexturedScreenSpaceQuadPassDescription>();

  camera->config.set_pipeline_description(pipe);

  gua::utils::Trackball trackball(0.01, 0.002, 0.2);

  auto window = std::make_shared<gua::GlfwWindow>();
  gua::WindowDatabase::instance()->add("main_window", window);
  window->config.set_enable_vsync(false);
  window->config.set_size(resolution);
  window->config.set_resolution(resolution);

  window->on_resize.connect([&](gua::math::vec2ui const& new_size) {
    window->config.set_resolution(new_size);
    camera->config.set_resolution(new_size);
    screen->data.set_size(gua::math::vec2(0.001 * new_size.x, 0.001 * new_size.y));
  });

  window->on_move_cursor.connect([&](gua::math::vec2 const& pos) {
    trackball.motion(pos.x, pos.y);
  });

  window->on_button_press.connect(std::bind(mouse_button, 
                                  std::ref(trackball), 
                                  std::placeholders::_1, 
                                  std::placeholders::_2, 
                                  std::placeholders::_3));

  window->on_key_press.connect(std::bind(key_press, 
                               std::ref(pipe), 
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

    // apply trackball matrix to object
    auto modelmatrix = scm::math::make_translation(trackball.shiftx(), trackball.shifty(), trackball.distance()) * trackball.rotation();
    input_transform->set_transform(modelmatrix);

    window->process_events();
    if (window->should_close()) {
      renderer.stop();
      window->close();
      loop.stop();
    }
    else {
      renderer.queue_draw({ &graph }, { camera });
    }
  });

  loop.start();

  return 0;
}
