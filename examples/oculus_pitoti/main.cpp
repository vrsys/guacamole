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
#include <gua/OculusWindow.hpp>
#include <gua/renderer/PLODLoader.hpp>
#include <gua/node/PLODNode.hpp>
#include <gua/renderer/PLODPass.hpp>
#include <gua/renderer/PLODLoader.hpp>
#include <gua/renderer/SSAAPass.hpp>

#include <gua/renderer/BBoxPass.hpp>
#include <gua/renderer/DebugViewPass.hpp>

#include <gua/renderer/TexturedQuadPass.hpp>

#include <iomanip>

#define NB_DISABLE 0
#define NB_ENABLE 1

int kbhit() {
  struct timeval tv;
  fd_set fds;
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  FD_ZERO(&fds);
  FD_SET(STDIN_FILENO, &fds); //STDIN_FILENO is 0
  select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
  return FD_ISSET(STDIN_FILENO, &fds);
}
 
void nonblock( int state ) {
  struct termios ttystate;

  //get the terminal state
  tcgetattr(STDIN_FILENO, &ttystate);

  if (state==NB_ENABLE) {
    //turn off canonical mode
    ttystate.c_lflag &= ~ICANON;
    //minimum of number input read.
    ttystate.c_cc[VMIN] = 1;
  }
  else if (state==NB_DISABLE) {
    //turn on canonical mode
    ttystate.c_lflag |= ICANON;
  }
  //set the terminal attributes.
  tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
}


int main(int argc, char** argv) {
  // initialize guacamole
  gua::init(argc, argv);

  /////////////////////////////////////////////////////////////////////////////
  // create a set of materials
  /////////////////////////////////////////////////////////////////////////////

  // create simple untextured material shader
  auto pbr_keep_input_desc    = std::make_shared<gua::MaterialShaderDescription>("./data/materials/PBR_use_input_color.gmd");
  auto pbr_uniform_color_desc = std::make_shared<gua::MaterialShaderDescription>("./data/materials/PBR_uniform_color.gmd");

  //use this material for models where shading does not make sense
  auto pbr_keep_color_shader(std::make_shared<gua::MaterialShader>("PBR_pass_input_color", pbr_keep_input_desc));
  auto pbr_overwrite_color_shader(std::make_shared<gua::MaterialShader>("PBR_overwrite_input_color", pbr_uniform_color_desc));

  gua::MaterialShaderDatabase::instance()->add(pbr_keep_color_shader);
  gua::MaterialShaderDatabase::instance()->add(pbr_overwrite_color_shader);

  // create different materials for point-based input
  auto plod_glossy      = pbr_keep_color_shader->make_new_material();
  auto plod_rough       = pbr_keep_color_shader->make_new_material();
  auto plod_passthrough = pbr_keep_color_shader->make_new_material();

  // glossy has high metalness, low roughness and no emissivity (=no passthrough)
  plod_glossy->set_uniform("metalness", 1.0f);
  plod_glossy->set_uniform("roughness", 0.3f);
  plod_glossy->set_uniform("emissivity", 0.0f);

  // rough has no metalness, high roughness and no emissivity (=no passthrough)
  plod_rough->set_uniform("metalness", 0.0f);
  plod_rough->set_uniform("roughness", 0.8f);
  plod_rough->set_uniform("emissivity", 0.0f);

  // passthrough has emissivity of 1.0
  plod_passthrough->set_uniform("emissivity", 1.0f);

  auto rough_white = pbr_overwrite_color_shader->make_new_material();
  rough_white->set_uniform("color", gua::math::vec3f(1.0f, 1.0f, 1.0f));
  rough_white->set_uniform("metalness", 0.0f);
  rough_white->set_uniform("roughness", 0.8f);
  rough_white->set_uniform("emissivity", 0.0f);

  auto rough_red = pbr_overwrite_color_shader->make_new_material();
  rough_red->set_uniform("color", gua::math::vec3f(0.8f, 0.0f, 0.0f));
  rough_red->set_uniform("metalness", 0.0f);
  rough_red->set_uniform("roughness", 0.8f);
  rough_red->set_uniform("emissivity", 0.0f);


  // setup scene
  gua::SceneGraph graph("main_scenegraph");

  gua::PLODLoader plodLoader;

  plodLoader.set_upload_budget_in_mb(32);
  plodLoader.set_render_budget_in_mb(2048);
  plodLoader.set_out_of_core_budget_in_mb(4096);

  auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");
  auto model_xf = graph.add_node<gua::node::TransformNode>("/transform", "model_xf");

  auto setup_plod_node = [](std::shared_ptr<gua::node::PLODNode> const& node) {
    node->set_radius_scale(1.0f);
    node->set_enable_backface_culling_by_normal(false);
    node->set_draw_bounding_box(true);
  };


  auto plod_pig_node = plodLoader.load_geometry("plod_pig", "data/objects/pig.kdn", plod_rough, gua::PLODLoader::NORMALIZE_POSITION | gua::PLODLoader::NORMALIZE_SCALE | gua::PLODLoader::MAKE_PICKABLE);
  plod_pig_node->scale(5.0, 5.0, 5.0);

  gua::TriMeshLoader loader;

  auto root_plod_pig_model = graph.add_node("/", plod_pig_node);

  auto nav = graph.add_node<gua::node::TransformNode>("/", "nav");
  nav->translate(0.0, 0.0, 2.0);


  auto window = std::make_shared<gua::OculusWindow>(":0.0");

  //get the resolution from the oculus window
  gua::math::vec2ui res(window->get_window_resolution());

  gua::math::vec2ui eye_res(window->get_eye_resolution());

  gua::WindowDatabase::instance()->add("main_window", window);
  window->config.set_enable_vsync(false);
  window->config.set_fullscreen_mode(true);
  window->config.set_size(res);
  window->config.set_resolution( res );
  window->open();


  auto resolve_pass = std::make_shared<gua::ResolvePassDescription>();
  resolve_pass->background_mode(gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE);
  resolve_pass->tone_mapping_exposure(1.0f);

  auto neck = graph.add_node<gua::node::TransformNode>("/nav", "neck");

  auto camera = graph.add_node<gua::node::CameraNode>("/nav/neck", "cam");

  float camera_trans_y = 0.0;
  float camera_trans_z = 0.0;

  camera->translate(0.0, camera_trans_y, camera_trans_z);

  camera->config.set_resolution( res );
  camera->config.set_left_screen_path("/nav/neck/cam/left_screen");
  camera->config.set_right_screen_path("/nav/neck/cam/right_screen");
  camera->config.set_scene_graph_name("main_scenegraph");
  camera->config.set_output_window_name("main_window");
  camera->config.set_enable_stereo(true);
  camera->config.set_eye_dist(window->get_IPD());

  auto pipe = std::make_shared<gua::PipelineDescription>();

  pipe->add_pass(std::make_shared<gua::PLODPassDescription>());
  pipe->add_pass(std::make_shared<gua::LightVisibilityPassDescription>());
  pipe->add_pass(std::make_shared<gua::ResolvePassDescription>());

  camera->set_pipeline_description(pipe); 
  
 
  // retreive the complete viewing setup from the oculus window
  //**************
  auto left_screen = graph.add_node<gua::node::ScreenNode>("/nav/neck/cam", "left_screen");
  left_screen->data.set_size(window->get_left_screen_size());
  left_screen->translate(window->get_left_screen_translation());

  auto right_screen = graph.add_node<gua::node::ScreenNode>("/nav/neck/cam", "right_screen");
  right_screen->data.set_size(window->get_right_screen_size());
  right_screen->translate(window->get_right_screen_translation());

  //****************/

  gua::Renderer renderer;

  gua::Timer timer;
  timer.start();

  double time(0);
  float desired_frame_time(1.0 / 60.0);
  gua::events::MainLoop loop;

 nonblock(NB_ENABLE);

 char c = '0';
 int i = 0;
 char prev_character = '\n';
  // application loop

  // application loop
  gua::events::Ticker ticker(loop, desired_frame_time);

  ticker.on_tick.connect([&]() {
    double frame_time(timer.get_elapsed());
    time += frame_time;
    timer.reset();

    std::function<void (std::shared_ptr<gua::node::Node>, int)> rotate;
    rotate = [&](std::shared_ptr<gua::node::Node> node, int depth) {
      node->rotate(frame_time * (1+depth) * 0.5, 1, 1, 0);
      for (auto child: node->get_children()) {
        rotate(child, ++depth);
      }
    };

    i=kbhit();

    float frame_offset = 1.0 * frame_time;

    if (i!=0){
      prev_character = c;
      c=fgetc(stdin);
        
      if(c == 'w' || c == 'W') {
          camera->translate(0.0, 0.01, 0.0);
          camera_trans_y += 0.01;
          std::cout << "Increased y to: " << camera_trans_y << "  \n";
      } else if(c == 's' || c == 'S') {
          camera->translate(0.0, -0.01, 0.0);
          camera_trans_y += -0.01;  
          std::cout << "Decreased y to: " << camera_trans_y << "  \n";
      } else if(c == 'a' || c == 'A') {
          camera->translate(0.0, 0.0, -0.01);
          camera_trans_z += -0.01;
          std::cout << "Decreased z to: " << camera_trans_z << " \n";
      } else if(c == 'd' || c == 'D') {
          camera->translate(0.0, 0.0, 0.01);
          camera_trans_z += 0.01;
          std::cout << "Increased z to: " << camera_trans_z << " \n";
      } else if( c == 'u' || c == 'U') {
        nav->translate(0.0, 0.0, -frame_offset);
      } else if( c == 'j' || c == 'J') {
        nav->translate(0.0, 0.0, frame_offset);
      } else if( c == 'h' || c == 'H') {
        nav->translate(-frame_offset, 0.0, 0.0);
      } else if( c == 'k' || c == 'K') {
        nav->translate(frame_offset, 0.0, 0.0);
      }
      else {
        i=0;
      }
    }

    neck->set_transform(window->get_oculus_sensor_orientation());

    renderer.queue_draw({&graph});
  });

  loop.start();

  return 0;
}

