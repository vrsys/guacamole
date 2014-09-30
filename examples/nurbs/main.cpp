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
#include <gua/renderer/NURBSLoader.hpp>
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

  gua::NURBSLoader loader; 
  auto teapot_geometry(loader.create_geometry_from_file("teapot_geometry", "data/objects/teapot.igs", "data/materials/Red.gmd", gua::NURBSLoader::NORMALIZE_SCALE | gua::NURBSLoader::NORMALIZE_POSITION | gua::NURBSLoader::WIREFRAME));
  auto teapot2_geometry(loader.create_geometry_from_file("teapot2_geometry", "data/objects/part3.igs", "data/materials/White.gmd", gua::NURBSLoader::NORMALIZE_SCALE | gua::NURBSLoader::NORMALIZE_POSITION ));
  auto teapot3_geometry(loader.create_geometry_from_file("teapot3_geometry", "data/objects/part3.igs", "data/materials/Yellow.gmd", gua::NURBSLoader::NORMALIZE_SCALE | gua::NURBSLoader::NORMALIZE_POSITION | gua::NURBSLoader::RAYCASTING));

  //std::cout << "[ " << teapot_geometry->get_bounding_box().min << " , " << teapot_geometry->get_bounding_box().max << " ]" << std::endl;
  //teapot_geometry->scale(10.0f);

  float const model_size = 20.0f;
  teapot_geometry->scale(model_size / scm::math::length(teapot_geometry->get_bounding_box().max - teapot_geometry->get_bounding_box().min));
  teapot_geometry->translate(-teapot_geometry->get_bounding_box().center());
  teapot_geometry->translate(-8, 0, 0);
  
  float const model2_size = 20.0f;
  teapot2_geometry->translate(-teapot2_geometry->get_bounding_box().center());
  teapot2_geometry->translate(8,0,0);
  teapot2_geometry->scale(model2_size / scm::math::length(teapot2_geometry->get_bounding_box().max - teapot2_geometry->get_bounding_box().min));
  
  float const model3_size = 20.0f;
  teapot3_geometry->translate(-teapot3_geometry->get_bounding_box().center());
  teapot3_geometry->translate(4, 0, 8);
  teapot3_geometry->scale(model3_size / scm::math::length(teapot3_geometry->get_bounding_box().max - teapot3_geometry->get_bounding_box().min));

  auto input = graph.add_node<gua::node::TransformNode>("/", "input"); 
   
  auto teapot = graph.add_node<gua::node::TransformNode>("/input", "teapot");
  graph.add_node("/input/teapot", teapot_geometry);
  graph.add_node("/input/teapot", teapot2_geometry);
  //graph.add_node("/input/teapot", teapot3_geometry);

  auto light = graph.add_node<gua::node::SpotLightNode>("/", "light");
  light->scale(500.f);
  light->translate(0.0f, 10.0f, 160.0f);

  auto screen = graph.add_node<gua::node::ScreenNode>("/", "screen");
  screen->data.set_size(gua::math::vec2(60.0f, 34.0f));
  screen->translate(0.0f, 0.0f, 30.0f);

  auto eye = graph.add_node<gua::node::TransformNode>("/screen", "eye");
  eye->translate(0.0f, 0.0f, 60.0f);

  // setup rendering pipeline and window
  auto resolution = gua::math::vec2ui(1600, 1200);

  auto pipe = new gua::Pipeline();
  pipe->config.set_camera(gua::Camera("/screen/eye", "/screen/eye", "/screen", "/screen", "main_scenegraph"));
  pipe->config.set_enable_fps_display(true);
  pipe->config.set_left_resolution(resolution);
  pipe->config.set_right_resolution(resolution);

  auto window(new gua::GlfwWindow());
  pipe->set_window(window);
  pipe->config.set_near_clip(1.0f);
  pipe->config.set_far_clip(1000.0f);

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
    if (mods == 0) key = std::tolower(key);

    if (action) {
      // don't do something on press event
    }
    else {
      // execute only on release 
      switch (key)
      {
      case 'p': teapot_geometry->max_pre_tesselation(teapot_geometry->max_pre_tesselation() - 1.0f);  break;
      case 'P': teapot_geometry->max_pre_tesselation(teapot_geometry->max_pre_tesselation() + 1.0f);  break;
      case 'f': teapot_geometry->max_final_tesselation(teapot_geometry->max_final_tesselation() - 1.0f);  break;
      case 'F': teapot_geometry->max_final_tesselation(teapot_geometry->max_final_tesselation() + 1.0f);  break;
      }
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
    input->set_transform(modelmatrix);

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
