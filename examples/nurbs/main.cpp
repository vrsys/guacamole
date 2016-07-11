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
#include <gua/renderer/DebugViewPass.hpp>
#include <gua/renderer/TexturedScreenSpaceQuadPass.hpp>
#include <gua/renderer/NURBSLoader.hpp>

#include <gua/databases/MaterialShaderDatabase.hpp>
#include <gua/utils/Trackball.hpp>
#include <gua/node/NURBSNode.hpp>

void toggle_raycasting(std::shared_ptr<gua::node::Node> const& node)
{
  auto nurbs_node = std::dynamic_pointer_cast<gua::node::NURBSNode>(node);
  if (nurbs_node) {
    nurbs_node->raycasting(!nurbs_node->raycasting());
    std::cout << "Raycasting = " << nurbs_node->raycasting() << std::endl;
  }

  for (auto const& node : node->get_children()) {
    toggle_raycasting(node);
  }
}

void increase_error(std::shared_ptr<gua::node::Node> const& node)
{
  auto nurbs_node = std::dynamic_pointer_cast<gua::node::NURBSNode>(node);
  if (nurbs_node) {
    nurbs_node->max_tesselation_error(nurbs_node->max_tesselation_error() + 1.0);
    std::cout << "Error = " << nurbs_node->max_tesselation_error() << std::endl;
  }

  for (auto const& node : node->get_children()) {
    increase_error(node);
  }
}

void decrease_error(std::shared_ptr<gua::node::Node> const& node)
{
  auto nurbs_node = std::dynamic_pointer_cast<gua::node::NURBSNode>(node);
  if (nurbs_node) {
    nurbs_node->max_tesselation_error(std::max(1.0, nurbs_node->max_tesselation_error() - 1.0));
    std::cout << "Error = " << nurbs_node->max_tesselation_error() << std::endl;
  }

  for (auto const& node : node->get_children()) {
    decrease_error(node);
  }
}


void toggle_backface_culling(std::shared_ptr<gua::node::Node> const& node)
{
  auto nurbs_node = std::dynamic_pointer_cast<gua::node::NURBSNode>(node);
  if (nurbs_node)
    nurbs_node->render_backfaces(!nurbs_node->render_backfaces());

  for (auto const& node : node->get_children()) {
    toggle_backface_culling(node);
  }
}


void toggle_trimming(std::shared_ptr<gua::node::Node> const& node)
{
  auto nurbs_node = std::dynamic_pointer_cast<gua::node::NURBSNode>(node);
  if (nurbs_node) {
    switch (nurbs_node->trimming_mode())
    {
    case gua::node::NURBSNode::classic :
      nurbs_node->trimming_mode(gua::node::NURBSNode::double_binary_contours);
      break;
    case gua::node::NURBSNode::double_binary_contours:
      nurbs_node->trimming_mode(gua::node::NURBSNode::classic);
      break;
    }
  }

  for (auto const& node : node->get_children()) {
    toggle_trimming(node);
  }
}

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

void key_press(gua::PipelineDescription& pipe, gua::SceneGraph& graph, int key, int scancode, int action, int mods)
{
  if (action == 0) return;

  auto& d_r = pipe.get_resolve_pass();

  switch (std::tolower(key))
  {
  case 'r' :
    pipe.get_pass_by_type<gua::NURBSPassDescription>()->touch();
    break;
  case 't' : 
    toggle_raycasting(graph.get_root());
    break;
  case 'b':
    toggle_backface_culling(graph.get_root());
    break;
  case 'y':
    toggle_trimming(graph.get_root());
    break;
  case 'u':
    increase_error(graph.get_root());
    break;
  case 'j':
    decrease_error(graph.get_root());
    break;
  case 'q':
    d_r->debug_tiles(!d_r->debug_tiles());
    d_r->touch();
    break;
  default : 
    break;
  }

}

/////////////////////////////////////////////////////////////////////////////
// create node
/////////////////////////////////////////////////////////////////////////////
std::shared_ptr<gua::node::Node> create_node_from_igs_file(std::string const& nodename, std::string const& filepath, std::shared_ptr<gua::Material> const& material)
{
  gua::NURBSLoader nurbs_loader;
  auto node = nurbs_loader.load_geometry(nodename, filepath, material, gua::NURBSLoader::DEFAULTS);
  node->max_tesselation_error(8.0f);
  return node;
}

/////////////////////////////////////////////////////////////////////////////
// application
/////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  // some global constants
  gua::math::vec4f iron(0.560, 0.570, 0.580, 1);
  gua::math::vec4f silver(0.972, 0.960, 0.915, 1);
  gua::math::vec4f aluminium(0.913, 0.921, 0.925, 1);
  gua::math::vec4f gold(1.0, 0.766, 0.336, 1);
  gua::math::vec4f copper(0.955, 0.637, 0.538, 1);
  gua::math::vec4f chromium(0.550, 0.556, 0.554, 1);
  gua::math::vec4f nickel(0.660, 0.609, 0.526, 1);
  gua::math::vec4f titanium(0.542, 0.497, 0.449, 1);
  gua::math::vec4f cobalt(0.662, 0.655, 0.634, 1);
  gua::math::vec4f platinum(0.672, 0.637, 0.585, 1);
  gua::math::vec4f water(0.2, 0.2, 0.2, 1);

  // initialize guacamole
  gua::init(argc, argv);

  // setup scene
  gua::SceneGraph graph("main_scenegraph");

  // create simple untextured material shader
  auto desc = std::make_shared<gua::MaterialShaderDescription>();
  desc->load_from_file("./data/materials/SimpleMaterial.gmd");
  auto material_shader(std::make_shared<gua::MaterialShader>("simple_material", desc));

  // create new material configurations for material shader
  gua::MaterialShaderDatabase::instance()->add(material_shader);
  auto lack = material_shader->make_new_material();
  auto glass = material_shader->make_new_material();
  auto gum = material_shader->make_new_material();

  // configure materials 
  lack->set_uniform("Color", copper);
  lack->set_uniform("Roughness", 0.3f);
  lack->set_uniform("Metalness", 0.0f);
  lack->set_uniform("Opacity", 1.0f);

  glass->set_uniform("Color", water);
  glass->set_uniform("Roughness", 0.3f);
  glass->set_uniform("Metalness", 1.0f);
  glass->set_uniform("Opacity", 0.3f);

  gum->set_uniform("Color", gua::math::vec4f(0.1, 0.1, 0.1, 1.0));
  gum->set_uniform("Roughness", 1.0f);
  gum->set_uniform("Metalness", 0.0f);
  gum->set_uniform("Opacity", 1.0f);

  /////////////////////////////////////////////////////////////////////////////
  // setup scene 
  /////////////////////////////////////////////////////////////////////////////
  auto input_transform = graph.add_node<gua::node::TransformNode>("/", "nurbs_transform");
  auto model_transform = graph.add_node<gua::node::TransformNode>("/nurbs_transform", "model_transform");

  // add light proxy
  //gua::TriMeshLoader triloader;
  //auto center(triloader.create_geometry_from_file("center", "./data/objects/teapot.obj", gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));
  //center->scale(1.1f);
  //input_transform->add_child(center);

  auto count = 0;
  model_transform->add_child(create_node_from_igs_file("igs" + std::to_string(count++), "./data/motor.igs", lack));
  //model_transform->add_child(create_node_from_igs_file("igs" + std::to_string(count++), "./data/objects/audi/antenne.igs", lack));
  //model_transform->add_child(create_node_from_igs_file("igs" + std::to_string(count++), "./data/objects/audi/frontblinker.igs", lack));
  //model_transform->add_child(create_node_from_igs_file("igs" + std::to_string(count++), "./data/objects/audi/frontscheinwerfer.igs", glass));
  //model_transform->add_child(create_node_from_igs_file("igs" + std::to_string(count++), "./data/objects/audi/grill_innen.igs", lack));
  //model_transform->add_child(create_node_from_igs_file("igs" + std::to_string(count++), "./data/objects/audi/frontscheinwerfer_innen.igs", lack));
  //model_transform->add_child(create_node_from_igs_file("igs" + std::to_string(count++), "./data/objects/audi/karosserie.igs", lack));
  //model_transform->add_child(create_node_from_igs_file("igs" + std::to_string(count++), "./data/objects/audi/kuehlergrill.igs", lack));
  //model_transform->add_child(create_node_from_igs_file("igs" + std::to_string(count++), "./data/objects/audi/nebelscheinwerfer.igs", lack));
  //model_transform->add_child(create_node_from_igs_file("igs" + std::to_string(count++), "./data/objects/audi/seitenscheiben.igs", glass));
  //model_transform->add_child(create_node_from_igs_file("igs" + std::to_string(count++), "./data/objects/audi/tuergriffe.igs", lack));
  //model_transform->add_child(create_node_from_igs_file("igs" + std::to_string(count++), "./data/objects/audi/windschutzscheibe.igs", glass));

  //input_transform->add_child(create_node_from_igs_file("igs" + std::to_string(count++), "./data/objects/vw/lack/schweller_links.igs", lack));
  //input_transform->add_child(create_node_from_igs_file("igs" + std::to_string(count++), "./data/objects/vw/lack/schweller_rechts.igs", lack));
  //input_transform->add_child(create_node_from_igs_file("igs" + std::to_string(count++), "./data/objects/vw/lack/seitenteil_hinten_links.igs", lack));
  //input_transform->add_child(create_node_from_igs_file("igs" + std::to_string(count++), "./data/objects/vw/lack/seitenteil_hinten_rechts.igs", lack));
  //input_transform->add_child(create_node_from_igs_file("igs" + std::to_string(count++), "./data/objects/vw/lack/seitenteil_hinten_links.igs", lack));
  //input_transform->add_child(create_node_from_igs_file("igs" + std::to_string(count++), "./data/objects/vw/lack/seitenteil_hinten_rechts.igs", lack));
  //input_transform->add_child(create_node_from_igs_file("igs" + std::to_string(count++), "./data/objects/vw/lack/kotfluegel_vorn_links.igs", lack));
  //input_transform->add_child(create_node_from_igs_file("igs" + std::to_string(count++), "./data/objects/vw/lack/kotfluegel_vorn_rechts.igs", lack));
  //input_transform->add_child(create_node_from_igs_file("igs" + std::to_string(count++), "./data/objects/vw/lack/stossfaenger_vorn.igs", lack));
  //input_transform->add_child(create_node_from_igs_file("igs" + std::to_string(count++), "./data/objects/vw/lack/a_saeule_closed.igs", lack));
  //                                                           
  //input_transform->add_child(create_node_from_igs_file("igs" + std::to_string(count++), "./data/objects/vw/scheiben/heckscheibe.igs", glass));
  //input_transform->add_child(create_node_from_igs_file("igs" + std::to_string(count++), "./data/objects/vw/scheiben/tuerseitenscheibe_hinten_links.igs", glass));
  //input_transform->add_child(create_node_from_igs_file("igs" + std::to_string(count++), "./data/objects/vw/scheiben/tuerseitenscheibe_vorn_links.igs", glass));
  //input_transform->add_child(create_node_from_igs_file("igs" + std::to_string(count++), "./data/objects/vw/scheiben/windschutzscheibe.igs", glass));
  //input_transform->add_child(create_node_from_igs_file("igs" + std::to_string(count++), "./data/objects/vw/scheiben/tuerseitenscheibe_hinten_rechts.igs", glass));
  //input_transform->add_child(create_node_from_igs_file("igs" + std::to_string(count++), "./data/objects/vw/scheiben/tuerseitenscheibe_vorn_rechts.igs", glass));


  graph.update_cache();

  auto bbox = model_transform->get_bounding_box();
  std::cout << "Original size of loaded model : [" << bbox.min << " - " << bbox.max << "]" << std::endl;
  model_transform->scale(0.01f);

  graph.update_cache();

  bbox = model_transform->get_bounding_box();
  std::cout << "Model bbox after scale: [" << bbox.min << " - " << bbox.max << "]" << std::endl;

  graph.update_cache();

  bbox = model_transform->get_bounding_box();
  model_transform->translate(-bbox.center());

  graph.update_cache();
  bbox = model_transform->get_bounding_box();
  std::cout << "Model bbox after translation: [" << bbox.min << " - " << bbox.max << "]" << std::endl;

  auto scene_center = model_transform->get_bounding_box().center();
  float scene_size = scm::math::length(bbox.max - bbox.min);
  scene_size = std::max(scene_size, 1.0f);
  
  std::cout << "Size of loaded model : " << scene_size << std::endl;

  unsigned const max_lights = 10;
  unsigned const max_light_intensity = 100.0f;
  unsigned const min_light_intensity = 10.0f;
  float const light_scale = 5.0f;

  for (unsigned i = 0; i != max_lights; ++i)
  {
    // create random light
    float relative_intensity = float(std::rand()) / float(RAND_MAX);

    gua::math::vec3 light_pos = (i % 2) ? bbox.min : bbox.max;

    auto dim = std::rand() % 3;
    light_pos[dim] = float(std::rand() % unsigned(scene_size)) - scene_size / 2;

    std::string lightname = std::string("light") + std::to_string(i);
    auto light = graph.add_node<gua::node::LightNode>("/", lightname);
    light->data.set_type(gua::node::LightNode::Type::POINT);

    light->data.color = gua::utils::Color3f(1.0f, 1.0f, 1.0f);
    light->scale(light_scale * scene_size * relative_intensity);
    light->data.brightness = min_light_intensity + relative_intensity * (max_light_intensity - min_light_intensity);
    light->translate(light_pos);

#if 0
    // add light proxy
    gua::TriMeshLoader loader;
    auto light_proxy(loader.create_geometry_from_file("light_proxy", "./data/objects/sphere.obj", gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));
    light_proxy->scale(0.1f * (1.0f / light_scale));
    light->add_child(light_proxy);
#endif
  }

  float screen_height = 0.4;
  float viewer_screen_distance = 0.8;

  auto resolution   = gua::math::vec2ui(2560, 1600);
  auto aspect_ratio = float(resolution.x) / float(resolution.y);
  auto screen = graph.add_node<gua::node::ScreenNode>("/", "screen");
  screen->data.set_size(gua::math::vec2(screen_height * aspect_ratio, screen_height));
  screen->translate(0, 0, scene_size);

  auto camera = graph.add_node<gua::node::CameraNode>("/screen", "cam");
  camera->translate(0, 0, viewer_screen_distance);
  camera->config.set_resolution(resolution);
  camera->config.set_screen_path("/screen");
  camera->config.set_scene_graph_name("main_scenegraph");
  camera->config.set_output_window_name("main_window");
  camera->config.set_enable_frustum_culling(false);
  camera->config.set_near_clip(0.01f);
  camera->config.set_far_clip(100.0f);

  std::cout << "Setting near / far clip to : " << 0.01f * scene_size << " - " << 100.0f * scene_size << std::endl;

  auto pipe = std::make_shared<gua::PipelineDescription>();
  pipe->add_pass(std::make_shared<gua::TriMeshPassDescription>());
  pipe->add_pass(std::make_shared<gua::NURBSPassDescription>());

  auto light_visibility_pass = std::make_shared<gua::LightVisibilityPassDescription>();
  pipe->add_pass(light_visibility_pass);
  
  auto resolve_pass = std::make_shared<gua::ResolvePassDescription>();
  resolve_pass->background_mode(gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE);
  resolve_pass->tone_mapping_exposure(1.0f);
  pipe->add_pass(resolve_pass);

  pipe->add_pass(std::make_shared<gua::DebugViewPassDescription>());

  pipe->set_enable_abuffer(true);
  
  camera->set_pipeline_description(pipe);

  gua::utils::Trackball trackball(0.1, 0.02, 0.2);

  auto window = std::make_shared<gua::GlfwWindow>();
  gua::WindowDatabase::instance()->add("main_window", window);
  window->config.set_enable_vsync(false);
  window->config.set_size(resolution);
  window->config.set_resolution(resolution);

  window->on_resize.connect([&](gua::math::vec2ui const& new_size) {
    window->config.set_resolution(new_size);
    camera->config.set_resolution(new_size);

    auto new_aspect_ratio = float(new_size.x) / float(new_size.y);
    screen->data.set_size(gua::math::vec2(scene_size * new_aspect_ratio, scene_size));
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
                               std::ref(*(camera->get_pipeline_description())),
                               std::ref(graph),
                               std::placeholders::_1, 
                               std::placeholders::_2,
                               std::placeholders::_3,
                               std::placeholders::_4));
  window->open();

  gua::Renderer renderer;

  // application loop
  gua::events::MainLoop loop;
  gua::events::Ticker ticker(loop, 1.0 / 500.0);

  std::size_t frame_counter = 0;
  ticker.on_tick.connect([&]() {

    // apply trackball matrix to object
    gua::math::mat4 modelmatrix = scm::math::make_translation(gua::math::float_t(trackball.shiftx()),
      gua::math::float_t(trackball.shifty()),
      gua::math::float_t(trackball.distance())) * gua::math::mat4(trackball.rotation());

    input_transform->set_transform(modelmatrix);

    if (frame_counter++ % 500 == 0)
     std::cout << window->get_rendering_fps() << std::endl;

    window->process_events();
    if (window->should_close()) {
      renderer.stop();
      window->close();
      loop.stop();
    }
    else {
      renderer.queue_draw({ &graph });
    }
  });

  loop.start();

  return 0;
}
 
