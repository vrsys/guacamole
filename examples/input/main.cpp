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

  gua::MaterialDescription desc;
  desc.load_from_file("data/materials/SimpleMaterial.gmd");

  auto mat(std::make_shared<gua::Material>("simple_mat", desc));
  gua::MaterialDatabase::instance()->add(mat);

  gua::TriMeshLoader loader;
  auto teapot(loader.create_geometry_from_file("teapot", "data/objects/teapot.obj", "simple_mat", gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));

  graph.add_node("/", teapot);

  auto light = graph.add_node<gua::node::PointLightNode>("/", "light");
  light->scale(1.4f);
  light->translate(1.f, 0.f, 0.f);

  auto light2 = graph.add_node<gua::node::PointLightNode>("/", "light2");
  light2->data.color = gua::utils::Color3f(1.f, 0.5f, 0.0f);
  light2->scale(3.4f);
  light2->translate(-1.f, 1.f, 0.f);

  auto screen = graph.add_node<gua::node::ScreenNode>("/", "screen");
  screen->data.set_size(gua::math::vec2(1.6f, 1.2f));

  auto eye = graph.add_node<gua::node::TransformNode>("/screen", "eye");
  eye->translate(0, 0, 1.5);

  gua::Camera cam("/screen/eye", "/screen/eye", "/screen", "/screen", "main_scenegraph");
  auto pipe = new gua::Pipeline();
  pipe->config.set_camera(cam);

  pipe->add_pass<gua::GBufferPass>();
  // pipe->add_pass<gua::LightingPass>();
  // pipe->add_pass<gua::BackgroundPass>();
  // pipe->add_pass<gua::SSAOPass>().set_radius(10.f).set_intensity(0.5f);

  auto window(new gua::GlfwWindow());
  pipe->set_output_window(window);
  gua::Renderer renderer({pipe});

  window->on_resize.connect([&](gua::math::vec2ui const& new_size) {
    window->config.set_left_resolution(new_size);
    pipe->config.set_resolution(new_size);
    screen->data.set_size(gua::math::vec2(0.002 * new_size.x, 0.002 * new_size.y));
  });

  window->on_move_cursor.connect([&](gua::math::vec2 const& pos) {
    std::cout << "Cursor: " << pos << std::endl;
  });

  window->on_button_press.connect([&](int button, int action, int mods) {
    if (action == 0) std::cout << "Mouse button " << button << " up" << std::endl;
    else             std::cout << "Mouse button " << button << " down" << std::endl;
  });

#if WIN32
  window->config.set_display_name("\\\\.\\DISPLAY1");
#else
  window->config.set_display_name(":0.0");
#endif

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
