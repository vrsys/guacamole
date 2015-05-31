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
#include <gua/renderer/TexturedScreenSpaceQuadPass.hpp>
#include <gua/renderer/TriMeshLoader.hpp>
#include <gua/renderer/ToneMappingPass.hpp>
#include <gua/renderer/DebugViewPass.hpp>
#include <gua/utils/Trackball.hpp>

#include <gua/gui.hpp>

#define COUNT 6

const bool SHOW_FRAME_RATE = false;
const bool CLIENT_SERVER = false;

int depth_layers      = 2;
bool depth_test       = true;
bool backface_culling = true;
bool orthographic     = false;
gua::WarpPassDescription::DisplayMode display_mode = gua::WarpPassDescription::DisplayMode::POINTS;

// forward mouse interaction to trackball
void mouse_button (gua::utils::Trackball& trackball, int mousebutton, int action, int mods) {
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
    casted->get_material()->set_show_back_faces(!backface_culling);
  }

  for (auto& child: node->get_children()) {
    show_backfaces(child);
  }
}

int main(int argc, char** argv) {

  auto resolution = gua::math::vec2ui(1920, 1080);

  // add mouse interaction
  gua::utils::Trackball trackball(0.01, 0.002, 0.2);

  // initialize guacamole
  gua::init(argc, argv);

  gua::SceneGraph graph("main_scenegraph");

  // ---------------------------------------------------------------------------
  // ---------------------------- setup scene ----------------------------------
  
  gua::TriMeshLoader loader;
  auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");
  transform->get_tags().add_tag("scene");

  // many oilrigs scene --------------------------------------------------------
  auto scene_root = graph.add_node<gua::node::TransformNode>("/transform", "many_oilrigs");
  auto add_oilrig = [&](int x, int y, int c, std::string const& parent) {
    auto t = graph.add_node<gua::node::TransformNode>(parent, "t");
    t->translate((x - c*0.5 + 0.5)/1.5, (y - c*0.5 + 0.8)/2, 0);
    auto teapot(loader.create_geometry_from_file("teapot", "/opt/3d_models/OIL_RIG_GUACAMOLE/oilrig.obj",  
      gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION | 
      gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::OPTIMIZE_MATERIALS |
      gua::TriMeshLoader::NORMALIZE_SCALE));
    t->add_child(teapot);

  };

  for (int x(0); x<COUNT; ++x) {
    for (int y(0); y<COUNT; ++y) {
      add_oilrig(x, y, COUNT, "/transform/many_oilrigs");
    }
  }

  // one oilrig ----------------------------------------------------------------
  scene_root = graph.add_node<gua::node::TransformNode>("/transform", "one_oilrig");
  scene_root->scale(3);
  add_oilrig(0, 0, 1, "/transform/one_oilrig");

  // textured quads scene ------------------------------------------------------
  scene_root = graph.add_node<gua::node::TransformNode>("/transform", "textured_quads");
  for (int x(0); x<10; ++x) {
    auto node = graph.add_node<gua::node::TexturedQuadNode>("/transform/textured_quads", "node" + std::to_string(x));
    node->translate(0, 0, -x);
    node->data.set_size(gua::math::vec2(1.92f*2, 1.08f*2));
  }

  // teapot --------------------------------------------------------------------
  scene_root = graph.add_node<gua::node::TransformNode>("/transform", "teapot");
  auto teapot(loader.create_geometry_from_file("teapot", "data/objects/teapot.obj",  
    gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION | 
    gua::TriMeshLoader::NORMALIZE_SCALE));
  scene_root->add_child(teapot);


  show_backfaces(transform);

  auto set_scene = [&](std::string const& name) {
    graph["/transform/many_oilrigs"]->get_tags().add_tag("invisible");
    graph["/transform/one_oilrig"]->get_tags().add_tag("invisible");
    graph["/transform/textured_quads"]->get_tags().add_tag("invisible");
    graph["/transform/teapot"]->get_tags().add_tag("invisible");

    if (name == "set_scene_many_oilrigs") 
      graph["/transform/many_oilrigs"]->get_tags().remove_tag("invisible");
    if (name == "set_scene_one_oilrig") 
      graph["/transform/one_oilrig"]->get_tags().remove_tag("invisible");
    if (name == "set_scene_textured_quads") 
      graph["/transform/textured_quads"]->get_tags().remove_tag("invisible");
    if (name == "set_scene_teapot") 
      graph["/transform/teapot"]->get_tags().remove_tag("invisible");
  };

  set_scene("set_scene_one_oilrig");

  // ---------------------------------------------------------------------------
  // ------------------------ setup rendering pipelines ------------------------
  
  // slow client ---------------------------------------------------------------
  auto slow_screen = graph.add_node<gua::node::ScreenNode>("/", "slow_screen");
  slow_screen->data.set_size(gua::math::vec2(1.92f*2, 1.08f*2));

  auto slow_cam = graph.add_node<gua::node::CameraNode>("/slow_screen", "slow_cam");
  slow_cam->translate(0, 0, 2.0);
  if (orthographic) slow_cam->config.set_mode(gua::node::CameraNode::ProjectionMode::ORTHOGRAPHIC);
  slow_cam->config.set_resolution(resolution);
  slow_cam->config.set_screen_path("/slow_screen");
  slow_cam->config.set_scene_graph_name("main_scenegraph");
  slow_cam->config.mask().blacklist.add_tag("invisible");
  
  // fast client ---------------------------------------------------------------
  auto fast_screen = graph.add_node<gua::node::ScreenNode>("/", "fast_screen");
  fast_screen->data.set_size(gua::math::vec2(1.92f*2, 1.08f*2));

  auto fast_cam = graph.add_node<gua::node::CameraNode>("/fast_screen", "fast_cam");
  fast_cam->translate(0, 0, 2.0);
  if (orthographic) fast_cam->config.set_mode(gua::node::CameraNode::ProjectionMode::ORTHOGRAPHIC);
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
    pipe->add_pass(std::make_shared<gua::TriMeshPassDescription>());
    pipe->add_pass(std::make_shared<gua::TexturedQuadPassDescription>());
    pipe->add_pass(std::make_shared<gua::GenerateWarpGridPassDescription>());
    pipe->add_pass(warp_pass);
    pipe->add_pass(std::make_shared<gua::TexturedScreenSpaceQuadPassDescription>());
    pipe->set_enable_abuffer(true);
    slow_cam->set_pipeline_description(pipe);
  }


  // ---------------------------------------------------------------------------
  // ----------------------------- setup gui -----------------------------------

  auto gui = std::make_shared<gua::GuiResource>();
  auto gui_quad = std::make_shared<gua::node::TexturedScreenSpaceQuadNode>("gui_quad");
  
  if (!CLIENT_SERVER) {
    gui->init("gui", "asset://gua/data/gui/gui.html", resolution);

    gui->on_loaded.connect([&]() {
      gui->add_javascript_getter("get_depth_layers", [&](){ return std::to_string(depth_layers);});
      gui->add_javascript_getter("get_depth_test", [&](){ return std::to_string(depth_test);});
      gui->add_javascript_getter("get_backface_culling", [&](){ return std::to_string(backface_culling);});
      gui->add_javascript_getter("get_orthographic", [&](){ return std::to_string(orthographic);});

      gui->add_javascript_callback("set_depth_layers");
      gui->add_javascript_callback("set_depth_test");
      gui->add_javascript_callback("set_backface_culling");
      gui->add_javascript_callback("set_orthographic");
      gui->add_javascript_callback("set_type_points");
      gui->add_javascript_callback("set_type_quads");
      gui->add_javascript_callback("set_type_scaled_points");
      gui->add_javascript_callback("set_scene_one_oilrig");
      gui->add_javascript_callback("set_scene_many_oilrigs");
      gui->add_javascript_callback("set_scene_teapot");
      gui->add_javascript_callback("set_scene_textured_quads");

      gui->add_javascript_callback("reset_view");

      gui->call_javascript("init");
    });

    gui->on_javascript_callback.connect([&](std::string const& callback, std::vector<std::string> const& params) {
      if (callback == "set_depth_layers") {
        std::stringstream str(params[0]);
        str >> depth_layers;
        warp_pass->max_layers(depth_layers);
      } else if (callback == "set_depth_test") {
        std::stringstream str(params[0]);
        str >> depth_test;
        warp_pass->depth_test(depth_test);
      } else if (callback == "set_backface_culling") {
        std::stringstream str(params[0]);
        str >> backface_culling;
        show_backfaces(transform);
      } else if (callback == "set_orthographic") {
        std::stringstream str(params[0]);
        str >> orthographic;
        if (orthographic) {
          fast_cam->config.set_mode(gua::node::CameraNode::ProjectionMode::ORTHOGRAPHIC);
          slow_cam->config.set_mode(gua::node::CameraNode::ProjectionMode::ORTHOGRAPHIC);
        } else {
          fast_cam->config.set_mode(gua::node::CameraNode::ProjectionMode::PERSPECTIVE);
          slow_cam->config.set_mode(gua::node::CameraNode::ProjectionMode::PERSPECTIVE);
        }
        
      } else if (callback == "reset_view") {
        trackball.reset();
      } else if (callback == "set_type_points") {
        std::stringstream str(params[0]);
        bool checked;
        str >> checked;
        if (checked) {
          warp_pass->display_mode(gua::WarpPassDescription::DisplayMode::POINTS);
        }
      } else if (callback == "set_type_quads") {
        std::stringstream str(params[0]);
        bool checked;
        str >> checked;
        if (checked) {
          warp_pass->display_mode(gua::WarpPassDescription::DisplayMode::QUADS);
        }
      } else if (callback == "set_type_scaled_points") {
        std::stringstream str(params[0]);
        bool checked;
        str >> checked;
        if (checked) {
          warp_pass->display_mode(gua::WarpPassDescription::DisplayMode::SCALED_POINTS);
        }
      } else if (callback == "set_scene_one_oilrig" || 
                 callback == "set_scene_many_oilrigs" ||
                 callback == "set_scene_teapot" ||
                 callback == "set_scene_textured_quads") {
        set_scene(callback);
      } 
    });

    gui_quad->data.texture() = "gui";
    gui_quad->data.size() = resolution;
    gui_quad->data.anchor() = gua::math::vec2(1.f, 1.f);

    graph.add_node("/", gui_quad);
  }
  

  // ---------------------------------------------------------------------------
  // ----------------------------- setup windows -------------------------------

  std::shared_ptr<gua::WindowBase> window;
  std::shared_ptr<gua::WindowBase> hidden_window;


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
    glfw->on_resize.connect([&](gua::math::vec2ui const& new_size) {
      glfw->config.set_resolution(new_size);
      slow_cam->config.set_resolution(new_size);
      // slow_screen->data.set_size(gua::math::vec2(1.08*2 * new_size.x / new_size.y, 1.08*2));
      // fast_screen->data.set_size(gua::math::vec2(1.08*2 * new_size.x / new_size.y, 1.08*2));
    });
    glfw->on_button_press.connect([&](int key, int action, int mods) {
      gui->inject_mouse_button(gua::Button(key), action, mods);
    });
    glfw->on_key_press.connect([&](int key, int scancode, int action, int mods) {
      gui->inject_keyboard_event(gua::Key(key), scancode, action, mods);
    });
    glfw->on_move_cursor.connect([&](gua::math::vec2 const& pos) {
      gui->inject_mouse_position(pos);
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

    if (!CLIENT_SERVER) {
      gua::Interface::instance()->update();
    }

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
      if (ctr++ % 100 == 0) {
        if (SHOW_FRAME_RATE) {
          std::cout << "Render fps: " << window->get_rendering_fps() 
                    << ", App fps: " << renderer.get_application_fps() << std::endl;
        }

        double trimesh_time(0); 
        double warp_time(0); 
        int primitives(0);

        for (auto const& result: window->get_context()->time_query_results) {
          if (result.first.find("Trimesh") != std::string::npos) trimesh_time = result.second;
          if (result.first.find("Warp") != std::string::npos) warp_time = result.second;
        }

        for (auto const& result: window->get_context()->primitive_query_results) {
          if (result.first.find("Warp") != std::string::npos) primitives = result.second.first;
        }

        gui->call_javascript("set_stats", renderer.get_application_fps(), 
                             window->get_rendering_fps(), trimesh_time, 
                             warp_time, primitives);
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
