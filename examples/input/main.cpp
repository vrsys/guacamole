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

int main(int argc, char** argv) {

  // initialize guacamole
  gua::init(argc, argv);

  // setup scene
  gua::SceneGraph graph("main_scenegraph");

  gua::TriMeshLoader loader;
  auto teapot_geometry(loader.create_geometry_from_file("teapot", "data/objects/teapot.obj", "data/materials/Red.gmd", gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));

  auto teapot = graph.add_node("/", teapot_geometry);

  auto light = graph.add_node<gua::node::PointLightNode>("/", "light");
  light->scale(5.f);
  light->translate(0, 1.f, 1.f);

  auto screen = graph.add_node<gua::node::ScreenNode>("/", "screen");
  screen->data.set_size(gua::math::vec2(1.6f, 1.2f));

  auto eye = graph.add_node<gua::node::TransformNode>("/screen", "eye");
  eye->translate(0, 0, 1.5);

  auto pipe = new gua::Pipeline();
  pipe->config.set_camera(gua::Camera("/screen/eye", "/screen/eye", "/screen", "/screen", "main_scenegraph"));
  pipe->config.set_enable_fps_display(true);

  auto window(new gua::GlfwWindow());
  pipe->set_window(window);
  gua::Renderer renderer({pipe});

  window->on_resize.connect([&](gua::math::vec2ui const& new_size) {
    window->config.set_left_resolution(new_size);
    pipe->config.set_left_resolution(new_size);
    screen->data.set_size(gua::math::vec2(0.002 * new_size.x, 0.002 * new_size.y));
  });

  window->on_move_cursor.connect([&](gua::math::vec2 const& pos) {
    std::cout << "Cursor: " << pos << std::endl;
  });

  window->on_button_press.connect([&](int button, int action, int mods) {
    if (action == 0) std::cout << "Mouse button " << button << " up" << std::endl;
    else             std::cout << "Mouse button " << button << " down" << std::endl;
  });

  // application loop
  gua::events::MainLoop loop;
  gua::events::Ticker ticker(loop, 1.0/60.0);

  ticker.on_tick.connect([&]() {

	  teapot->rotate(0.1, 0, 1, 0);

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
