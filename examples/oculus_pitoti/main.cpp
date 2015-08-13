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

const std::string geometry("data/objects/monkey.obj");
// const std::string geometry("data/objects/cube.obj");

std::vector<std::shared_ptr<gua::node::TransformNode>> add_lights(gua::SceneGraph& graph,
                                                  int count) {

  std::vector<std::shared_ptr<gua::node::TransformNode>> lights(count);

  for (int i(0); i < count; ++i) {
    scm::math::vec3 randdir(gua::math::random::get(-1.f, 1.f),
                            gua::math::random::get(-1.f, 1.f),
                            gua::math::random::get(-1.f, 1.f));
    scm::math::normalize(randdir);

    gua::TriMeshLoader loader;
    auto sphere_geometry(
      loader.create_geometry_from_file(
      "sphere" + gua::string_utils::to_string(i),
      "data/objects/light_sphere.obj"
    ));

    sphere_geometry->scale(0.04, 0.04, 0.04);

    lights[i] = graph.add_node("/", std::make_shared<gua::node::TransformNode>("light" + gua::string_utils::to_string(i)));
    lights[i]->add_child(sphere_geometry);
    lights[i]->scale(300.f);
    lights[i]->translate(5.0, 5.0, 10.0);

    auto light = lights[i]->add_child(std::make_shared<gua::node::LightNode>("light"));
    light->data.set_brightness(300.f);    
    light->data.set_type(gua::node::LightNode::Type::POINT);
    light->data.set_color(gua::utils::Color3f(1.0, 1.0, 1.0));
    light->data.set_max_shadow_dist(8000.0f);
    light->data.set_shadow_offset(0.002f);
    light->data.set_enable_shadows(true);
    light->data.set_shadow_map_size(2048);

  }

  return lights;
}

void setup_scene(gua::SceneGraph& graph,
                 std::shared_ptr<gua::node::Node> const& root_monkey,
                 int depth_count) {

  if (depth_count > 0) {
    gua::TriMeshLoader loader;

    float offset(2.f);
    std::vector<gua::math::vec3> directions = {
      gua::math::vec3(0, offset, 0),
      gua::math::vec3(0, -offset, 0),
      gua::math::vec3(offset, 0, 0),
      gua::math::vec3(-offset, 0, 0),
      gua::math::vec3(0, 0, offset),
      gua::math::vec3(0, 0, -offset)
    };

    for (auto direction: directions) {
      auto monkey_geometry(loader.create_geometry_from_file(
        "monkey",
        geometry
      ));

      auto monkey = root_monkey->add_child(monkey_geometry);
      monkey->scale(0.5, 0.5, 0.5);
      monkey->translate(direction[0], direction[1], direction[2]);

      setup_scene(graph, monkey, depth_count - 1);
    }
  }
}


int main(int argc, char** argv) {

  // initialize guacamole
  gua::init(argc, argv);


  // initialize Oculus SDK
  gua::OculusWindow::initialize_oculus_environment();
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
  

  auto lights = add_lights(graph, 2);

  auto nav = graph.add_node<gua::node::TransformNode>("/", "nav");
  nav->translate(0.0, 0.0, 2.0);

  auto window = std::make_shared<gua::OculusWindow>(":0.0");
  gua::WindowDatabase::instance()->add("main_window", window);
  window->config.set_enable_vsync(false);

  window->open();

  // setup rendering pipeline and window
  auto resolve_pass = std::make_shared<gua::ResolvePassDescription>();
  //resolve_pass->background_mode(gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE);
  resolve_pass->tone_mapping_exposure(1.0f);

  auto camera = graph.add_node<gua::node::CameraNode>("/nav", "cam");

  //camera->translate(0, 0, 2.0);
  camera->config.set_resolution(window->get_resolution());
  camera->config.set_left_screen_path("/nav/cam/left_screen");
  camera->config.set_right_screen_path("/nav/cam/right_screen");
  camera->config.set_scene_graph_name("main_scenegraph");
  camera->config.set_output_window_name("main_window");
  camera->config.set_enable_stereo(true);
  camera->config.set_eye_dist(0.064);


  auto pipe = std::make_shared<gua::PipelineDescription>();

  pipe->add_pass(std::make_shared<gua::PLODPassDescription>());
  pipe->add_pass(std::make_shared<gua::LightVisibilityPassDescription>());
  pipe->add_pass(std::make_shared<gua::ResolvePassDescription>());


  pipe->get_resolve_pass()->background_mode(gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE);
  pipe->get_resolve_pass()->background_texture("data/images/skymap.jpg");

  float eye_screen_distance = 0.08f;

  camera->set_pipeline_description(pipe);
  auto left_screen = graph.add_node<gua::node::ScreenNode>("/nav/cam", "left_screen");
  left_screen->data.set_size(window->get_screen_size_per_eye());
  left_screen->translate(-0.5 * window->get_screen_size_per_eye().x, 0, -eye_screen_distance);

  auto right_screen = graph.add_node<gua::node::ScreenNode>("/nav/cam", "right_screen");
  right_screen->data.set_size(window->get_screen_size_per_eye());
  right_screen->translate(0.5 * window->get_screen_size_per_eye().x, 0, -eye_screen_distance);



  gua::Renderer renderer;

  gua::Timer timer;
  timer.start();

  double time(0);
  float desired_frame_time(1.0 / 60.0);
  gua::events::MainLoop loop;

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


    camera->set_transform(window->get_sensor_orientation());
    

    renderer.queue_draw({&graph});
  });

  loop.start();

  return 0;
}

