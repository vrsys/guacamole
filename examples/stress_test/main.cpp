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

#define COUNT 5

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

  auto add_oilrig = [&](int x, int y) {

    auto t = graph.add_node<gua::node::TransformNode>("/", "rig_" + std::to_string(x) + "_" + std::to_string(y));
    t->translate((x - COUNT*0.5 + 0.5)/1.5, (y - COUNT*0.5 + 0.5)/3, 0);

    auto rig(loader.create_geometry_from_file(
      "rig",
      "/opt/3d_models/OIL_RIG_GUACAMOLE/oilrig.obj",
      mat->get_default_instance(),
      gua::TriMeshLoader::NORMALIZE_POSITION |
      gua::TriMeshLoader::NORMALIZE_SCALE |
      gua::TriMeshLoader::LOAD_MATERIALS |
      gua::TriMeshLoader::OPTIMIZE_GEOMETRY
    ));
    t->add_child(rig);
  };

  for (int x(0); x<COUNT; ++x) {
    for (int y(0); y<COUNT; ++y) {
      add_oilrig(x, y);
    }
  }

  auto light = graph.add_node<gua::node::PointLightNode>("/", "light");
  light->scale(4.4f);
  light->translate(1.f, 0.f, 2.f);

  auto light2 = graph.add_node<gua::node::PointLightNode>("/", "light2");
  light2->data.color = gua::utils::Color3f(1.0f, 1.0f, 1.0f);
  light2->scale(3.4f);
  light2->translate(-2.f, 1.f, 2.f);

  auto screen = graph.add_node<gua::node::ScreenNode>("/", "screen");
  screen->data.set_size(gua::math::vec2(1.92f, 1.08f));
  screen->translate(0, 0, 1.0);

  auto eye = graph.add_node<gua::node::TransformNode>("/screen", "eye");
  eye->translate(0, 0, 2);

  gua::Camera cam("/screen/eye", "/screen/eye", "/screen", "/screen", "main_scenegraph");
  auto pipe = new gua::Pipeline();
  pipe->config.set_camera(cam);
  pipe->config.set_resolution(gua::math::vec2ui(1920, 1080));

  pipe->add_pass<gua::GBufferPass>();
  pipe->add_pass<gua::LightingPass>();
  pipe->add_pass<gua::BackgroundPass>();
  // pipe->add_pass<gua::SSAOPass>().set_radius(10.f).set_intensity(0.5f);

  auto window(new gua::GlfwWindow());
  pipe->set_output_window(window);
  gua::Renderer renderer({pipe});

  window->config.set_enable_vsync(false);
  window->config.set_size(gua::math::vec2ui(1920, 1080));
  window->config.set_left_resolution(gua::math::vec2ui(1920, 1080));

  window->on_resize.connect([&](gua::math::vec2ui const& new_size) {
    window->config.set_left_resolution(new_size);
    pipe->config.set_resolution(new_size);
    screen->data.set_size(gua::math::vec2(0.001 * new_size.x, 0.001 * new_size.y));
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
  gua::events::Ticker ticker(loop, 1.0/500.0);

  ticker.on_tick.connect([&]() {

    // for (int x(0); x<COUNT; ++x) {
    //   for (int y(0); y<COUNT; ++y) {
    //     graph["/rig_" + std::to_string(x) + "_" + std::to_string(y) + "/rig"]->rotate(0.1, 0, 1, 0);
    //   } 
    // }

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
