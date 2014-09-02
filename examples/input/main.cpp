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


int main(int argc, char** argv) {

  // initialize guacamole
  gua::init(argc, argv);

  // setup scene
  gua::SceneGraph graph("main_scenegraph");

  gua::TriMeshLoader loader;

  auto teapot(loader.create_geometry_from_file("teapot", "data/objects/teapot.obj", "data/materials/Red.gmd", gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));

  graph.add_node("/", teapot);

  auto light = graph.add_node<gua::node::PointLightNode>("/", "light");
  light->scale(20.f);
  light->translate(0,0,12);

  auto screen = graph.add_node<gua::node::ScreenNode>("/", "screen");
  screen->data.set_size(gua::math::vec2(16.0f, 12.0f));
  screen->translate(0, 0, 5);

  auto eye = graph.add_node<gua::node::TransformNode>("/screen", "eye");
  eye->translate(0, 0, 7);

  // setup rendering pipeline and window
  auto resolution = gua::math::vec2ui(1600, 1200);

  gua::Camera cam("/screen/eye", "/screen/eye", "/screen", "/screen", "main_scenegraph");
  auto pipe = new gua::Pipeline();
  pipe->config.set_camera(cam);
  // pipe->config.set_enable_fps_display(true);

  auto window(new gua::GlfwWindow());
  // pipe->set_window(window);
  gua::Renderer renderer({pipe});

  // add mouse interaction
  gua::utils::Trackball trackball;

  window->on_resize.connect([&](gua::math::vec2ui const& new_size) {
    window->config.set_left_resolution(new_size);
    pipe->config.set_left_resolution(new_size);
    screen->data.set_size(gua::math::vec2(0.002 * new_size.x, 0.002 * new_size.y));
  });

  window->on_move_cursor.connect([&](gua::math::vec2 const& pos) {
    trackball.motion(pos.x, pos.y);
  });

  window->on_button_press.connect(std::bind(mouse_button, std::ref(trackball), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  window->on_key_press.connect([&](int key, int scancode, int action, int mods) {
    if (action) {
      std::cout << char(key) << " pressed." << std::endl;
    } else {
      std::cout << char(key) << " released." << std::endl;
    }
  });

#if WIN32
  window->config.set_display_name("\\\\.\\DISPLAY1");
  window->config.set_left_resolution(resolution);
  window->config.set_right_resolution(resolution);
  window->config.set_size(resolution);
#else
  window->config.set_display_name(":0.0");
#endif

  // application loop
  gua::events::MainLoop loop;
  gua::events::Ticker ticker(loop, 1.0/60.0);

  ticker.on_tick.connect([&]() {

    // apply trackball matrix to object
    auto modelmatrix = scm::math::make_translation(trackball.shiftx(), trackball.shifty(), trackball.distance()) * trackball.rotation();
    teapot->set_transform(modelmatrix);

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
