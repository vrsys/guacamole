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

const bool SHOW_FRAME_RATE = true;
const bool CLIENT_SERVER = false;
const bool RENDER_BACKFACES = false;

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

void show_backfaces(std::shared_ptr<gua::node::Node> const& node) {
  auto casted(std::dynamic_pointer_cast<gua::node::TriMeshNode>(node));
  if (casted) {
    casted->get_material()->set_show_back_faces(true);
  }

  for (auto& child: node->get_children()) {
    show_backfaces(child);
  }
}

int main(int argc, char** argv) {

  // initialize guacamole
  gua::init(argc, argv);

  // setup scene
  gua::SceneGraph graph("main_scenegraph");

  gua::TriMeshLoader loader;

  auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");
  transform->get_tags().add_tag("scene");
  auto teapot(loader.create_geometry_from_file("teapot", "/opt/3d_models/OIL_RIG_GUACAMOLE/oilrig.obj",  
  // auto teapot(loader.create_geometry_from_file("teapot", "data/objects/teapot.obj",  
    gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));
  graph.add_node("/transform", teapot);

  if (RENDER_BACKFACES) {
    show_backfaces(teapot);
  }

  auto light2 = graph.add_node<gua::node::LightNode>("/", "light2");
  light2->data.set_type(gua::node::LightNode::Type::POINT);
  light2->data.brightness = 150.0f;
  light2->scale(12.f);
  light2->translate(-3.f, 5.f, 5.f);

  auto fast_screen = graph.add_node<gua::node::ScreenNode>("/", "fast_screen");
  fast_screen->data.set_size(gua::math::vec2(1.92f, 1.08f));

  auto slow_screen = graph.add_node<gua::node::ScreenNode>("/", "slow_screen");
  slow_screen->data.set_size(gua::math::vec2(1.92f, 1.08f));

  // add mouse interaction
  gua::utils::Trackball trackball(0.01, 0.002, 0.2);

  // setup rendering pipeline and window
  auto resolution = gua::math::vec2ui(1920, 1080);

  std::shared_ptr<gua::WindowBase> window;
  std::shared_ptr<gua::WindowBase> hidden_window;

  // slow client ---------------------------------------------------------------
  auto slow_cam = graph.add_node<gua::node::CameraNode>("/slow_screen", "slow_cam");
  slow_cam->translate(0, 0, 2.0);
  slow_cam->config.set_resolution(resolution);
  slow_cam->config.set_screen_path("/slow_screen");
  slow_cam->config.set_scene_graph_name("main_scenegraph");
  
  // fast client ---------------------------------------------------------------
  auto fast_cam = graph.add_node<gua::node::CameraNode>("/fast_screen", "fast_cam");
  fast_cam->translate(0, 0, 2.0);
  fast_cam->config.set_resolution(resolution);
  fast_cam->config.set_screen_path("/fast_screen");
  fast_cam->config.set_scene_graph_name("main_scenegraph");
  
  auto warp_pass(std::make_shared<gua::WarpPassDescription>());

  if (CLIENT_SERVER) {
    slow_cam->config.set_output_window_name("hidden_window");
    fast_cam->config.set_output_window_name("window");
    
    auto slow_pipe = std::make_shared<gua::PipelineDescription>();
    slow_pipe->set_enable_abuffer(true);
    slow_pipe->add_pass(std::make_shared<gua::TriMeshPassDescription>());
    slow_cam->set_pipeline_description(slow_pipe);
    
    warp_pass->use_abuffer_from_window("hidden_window");

    auto fast_pipe = std::make_shared<gua::PipelineDescription>();
    fast_pipe->set_enable_abuffer(true);
    fast_pipe->add_pass(warp_pass);
    fast_cam->set_pipeline_description(fast_pipe);
    fast_cam->config.mask().blacklist.add_tag("scene");

  } else {

    slow_cam->config.set_output_window_name("window");

    auto pipe = std::make_shared<gua::PipelineDescription>();
    pipe->set_enable_abuffer(true);
    pipe->add_pass(std::make_shared<gua::TriMeshPassDescription>());
    pipe->add_pass(warp_pass);
    slow_cam->set_pipeline_description(pipe);
  }


  // create windows
  if (CLIENT_SERVER) {
    window = std::make_shared<gua::Window>();
    
    hidden_window = std::make_shared<gua::HeadlessSurface>();
    hidden_window->config.set_size(resolution);
    hidden_window->config.set_resolution(resolution);
    hidden_window->config.set_enable_vsync(false);
    hidden_window->config.set_context_share("window");

    gua::WindowDatabase::instance()->add("hidden_window", hidden_window);
  } else {
    auto glfw = std::make_shared<gua::GlfwWindow>();
    glfw->on_move_cursor.connect([&](gua::math::vec2 const& pos) {
      trackball.motion(pos.x, pos.y);
    });
    glfw->on_button_press.connect(std::bind(mouse_button, std::ref(trackball), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    window = glfw;
  }
  
  window->config.set_size(resolution);
  window->config.set_resolution(resolution);
  window->config.set_enable_vsync(false);
  gua::WindowDatabase::instance()->add("window", window);
  window->open();
  



  // render setup --------------------------------------------------------------
  gua::Renderer renderer;

  // application loop
  gua::events::MainLoop loop;
  gua::events::Ticker ticker(loop, 1.0/500.0);

  int ctr=0;

  ticker.on_tick.connect([&]() {

    // apply trackball matrix to object
    gua::math::mat4 modelmatrix = scm::math::make_translation(gua::math::float_t(trackball.shiftx()),
                                                              gua::math::float_t(trackball.shifty()),
                                                              gua::math::float_t(trackball.distance())) * gua::math::mat4(trackball.rotation());

    gua::Frustum warp_frustum;

    if (CLIENT_SERVER) {
      fast_screen->rotate(0.1, 0, 1, 0);
      if (SHOW_FRAME_RATE && ctr++ % 300 == 0) {
        slow_screen->set_transform(fast_screen->get_transform());
        std::cout << "Slow fps: " << hidden_window->get_rendering_fps() 
                  << ", Fast fps: " << window->get_rendering_fps() 
                  << ", App fps: " << renderer.get_application_fps() << std::endl;
      }
      warp_frustum = slow_cam->get_rendering_frustum(graph, gua::CameraMode::CENTER);
    } else {
      fast_screen->set_transform(modelmatrix);
      if (SHOW_FRAME_RATE && ctr++ % 300 == 0) {
        std::cout << "Render fps: " << window->get_rendering_fps() 
                  << ", App fps: " << renderer.get_application_fps() << std::endl;
      }
      warp_frustum = fast_cam->get_rendering_frustum(graph, gua::CameraMode::CENTER);
    }

    gua::math::mat4f projection(warp_frustum.get_projection());
    gua::math::mat4f view(warp_frustum.get_view());
    warp_pass->original_inverse_projection_view_matrix(scm::math::inverse(projection * view));

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
