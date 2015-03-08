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

#include <gua/renderer/PLODLoader.hpp>
#include <gua/node/PLODNode.hpp>
#include <gua/renderer/PLODPass.hpp>
#include <gua/renderer/TriMeshPass.hpp>

#include <gua/renderer/BBoxPass.hpp>
#include <gua/renderer/DebugViewPass.hpp>

#include <gua/renderer/TexturedQuadPass.hpp>

bool rotate_light;

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

/////////////////////////////////////////////////////////////////////////////
void increase_radius(std::shared_ptr<gua::node::Node> const& node) {
  auto plodnode = std::dynamic_pointer_cast<gua::node::PLODNode>(node);
  if (plodnode) {
    auto radius_scale = plodnode->get_radius_scale();
    plodnode->set_radius_scale(std::min(2.0, 1.1 * radius_scale));
    std::cout << "Setting radius scale to " << plodnode->get_radius_scale() << std::endl;
  }
  for (auto const& c : node->get_children()) {
    increase_radius(c);
  }
}

/////////////////////////////////////////////////////////////////////////////
void decrease_radius(std::shared_ptr<gua::node::Node> const& node) {
  auto plodnode = std::dynamic_pointer_cast<gua::node::PLODNode>(node);
  if (plodnode) {
    auto radius_scale = plodnode->get_radius_scale();
    plodnode->set_radius_scale(std::max(0.1, 0.9 * radius_scale));
    std::cout << "Setting radius scale to " << plodnode->get_radius_scale() << std::endl;
  }
  for (auto const& c : node->get_children()) {
    decrease_radius(c);
  }
}

/////////////////////////////////////////////////////////////////////////////
void increase_error_threshold(std::shared_ptr<gua::node::Node> const& node) {
  auto plodnode = std::dynamic_pointer_cast<gua::node::PLODNode>(node);
  if (plodnode) {
    auto radius_scale = plodnode->get_error_threshold();
    plodnode->set_error_threshold(std::min(16.0, 1.1 * radius_scale));
    std::cout << "Setting error threshold to " << plodnode->get_error_threshold() << std::endl;
  }
  for (auto const& c : node->get_children()) {
    increase_error_threshold(c);
  }
}

/////////////////////////////////////////////////////////////////////////////
void decrease_error_threshold(std::shared_ptr<gua::node::Node> const& node) {
  auto plodnode = std::dynamic_pointer_cast<gua::node::PLODNode>(node);
  if (plodnode) {
    auto radius_scale = plodnode->get_error_threshold();
    plodnode->set_error_threshold(std::max(1.0, 0.9 * radius_scale));
    std::cout << "Setting  error threshold to " << plodnode->get_error_threshold() << std::endl;
  }
  for (auto const& c : node->get_children()) {
    decrease_error_threshold(c);
  }
}

/////////////////////////////////////////////////////////////////////////////
// keyboard callback 
//    -> use 'r' to force shader recompilation
//    -> use 'u' to increase splat radius
//    -> use 'j' to decrease splat radius

void key_press(gua::PipelineDescription& pipe, gua::SceneGraph& graph, int key, int scancode, int action, int mods)
{
  if (action == 0) return;

  switch (std::tolower(key))
  {
  case 'r':
    pipe.get_resolve_pass()->touch();
    break;
  case 'u':
    increase_radius(graph.get_root());
    break;
  case 'j':
    decrease_radius(graph.get_root());
    break;
  case 'i':
    increase_error_threshold(graph.get_root());
    break;
  case 'k':
    decrease_error_threshold(graph.get_root());
    break;

  case ' ':
    rotate_light = !rotate_light;
    break;
  default:
    break;
  }
}

/////////////////////////////////////////////////////////////////////////////
// example configuration
/////////////////////////////////////////////////////////////////////////////
#define RENDER_SINGLE_PIG_MODEL 0
#define RENDER_PITOTI_HUNTING_SCENE 1
#define RENDER_ADDITIONAL_TRIMESH_MODEL 0

int main(int argc, char** argv) {
  /////////////////////////////////////////////////////////////////////////////
  // initialize guacamole
  /////////////////////////////////////////////////////////////////////////////

  gua::init(argc, argv);

  gua::Logger::enable_debug = false;
  rotate_light = true;

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

  /////////////////////////////////////////////////////////////////////////////
  // create scene
  /////////////////////////////////////////////////////////////////////////////

  // create scene graph object
  gua::SceneGraph graph("main_scenegraph");

  // configure plod-renderer and create point-based objects
  gua::PLODLoader plodLoader;

  plodLoader.set_upload_budget_in_mb(32);
  plodLoader.set_render_budget_in_mb(2048);
  plodLoader.set_out_of_core_budget_in_mb(4096);

  auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");

#if RENDER_PITOTI_HUNTING_SCENE  
  #if WIN32
    //auto plod_geometry(plodLoader.load_geometry("hunter", "\\GRANDMOTHER/pitoti/XYZ_ALL/new_pitoti_sampling/objects/Area_4_hunter_with_bow.kdn", plod_passthrough, gua::PLODLoader::NORMALIZE_POSITION));
  auto plod_geometry(plodLoader.load_geometry("plod_pig", "data/objects/Area-1_Warrior-scene_P01-1_transformed.kdn", plod_passthrough, gua::PLODLoader::NORMALIZE_POSITION ));
  #else
  auto plod_geometry(plodLoader.load_geometry("plod_pig", "/mnt/pitoti/precision_tests/001/Area-1_Warrior-scene_P01-1_transformed.kdn", plod_passthrough, gua::PLODLoader::NORMALIZE_POSITION ));
    //auto plod_geometry(plodLoader.load_geometry("/mnt/pitoti/XYZ_ALL/new_pitoti_sampling/Area_4_hunter_with_bow.kdn", gua::PLODLoader::NORMALIZE_POSITION  ));
  #endif
#endif

  //auto plod_geometry(plodLoader.load_geometry("plod_pig", "/opt/3d_models/point_based/plod/pig.kdn", plod_passthrough, gua::PLODLoader::NORMALIZE_POSITION | gua::PLODLoader::NORMALIZE_SCALE ));
  //auto plod_geometry(plodLoader.load_geometry("plod_pig", "/mnt/pitoti/Seradina_FULL_SCAN/sera_fixed/sera_part_01.kdn", plod_passthrough, gua::PLODLoader::NORMALIZE_POSITION | gua::PLODLoader::NORMALIZE_SCALE ));
  //auto plod_geometry2(plodLoader.load_geometry("plod_pig2", "/mnt/pitoti/KDN_LOD/PITOTI_KDN_LOD/_seradina.kdn", plod_passthrough, gua::PLODLoader::NORMALIZE_POSITION | gua::PLODLoader::NORMALIZE_SCALE ) );

#if RENDER_SINGLE_PIG_MODEL
  auto plod_geometry(plodLoader.load_geometry("plod_pig", "data/objects/pig.kdn", plod_passthrough, gua::PLODLoader::NORMALIZE_POSITION | gua::PLODLoader::NORMALIZE_SCALE));
  //auto plod_geometry2(plodLoader.load_geometry("plod_pig2", "data/objects/pig2.kdn", plod_glossy, gua::PLODLoader::NORMALIZE_POSITION));
  //auto plod_geometry3(plodLoader.load_geometry("plod_pig3", "data/objects/pig3.kdn", plod_passthrough, gua::PLODLoader::NORMALIZE_POSITION));
#endif

  auto setup_plod_node = [] ( std::shared_ptr<gua::node::PLODNode> const& node) {
    node->set_radius_scale(0.7f);
    node->set_enable_backface_culling_by_normal(false);
    node->set_draw_bounding_box(true);
  };

  
  setup_plod_node(plod_geometry);
  //setup_plod_node(plod_geometry2);
  //setup_plod_node(plod_geometry3);

  // create trimesh geometry
  gua::TriMeshLoader loader;

  auto light_proxy_geometry(loader.create_geometry_from_file("light", "data/objects/sphere.obj", rough_white, gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));
  auto camera_proxy_geometry(loader.create_geometry_from_file("camera_proxy", "data/objects/sphere.obj", rough_white, gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));
  light_proxy_geometry->scale(0.02);

  // connect scene graph
  transform->add_child(plod_geometry);  
  //transform->add_child(plod_geometry2);

#if RENDER_ADDITIONAL_TRIMESH_MODEL
  auto teapot_geometry(loader.create_geometry_from_file("teapot", "data/objects/teapot.obj", rough_white, gua::TriMeshLoader::NORMALIZE_POSITION));
  transform->add_child(teapot_geometry);
#endif
  
  //transform->add_child(plod_geometry2);
  //transform->add_child(plod_geometry3);

  //plod_geometry2->translate(0.0, 2.5, 0.0);
  //plod_geometry3->translate(0.0, -2.5, 0.0);
  
  /////////////////////////////////////////////////////////////////////////////
  // create lighting
  /////////////////////////////////////////////////////////////////////////////

  auto light_center = graph.add_node<gua::node::TransformNode>("/", "light_center");
  light_center->translate(0.f, 0.f, 2.f);

  auto light = graph.add_node<gua::node::PointLightNode>("/light_center", "light");
  light->data.set_enable_shadows(true);
  light->scale(3.f);
  light->translate(0.f, 0.f, 3.7f);
  light->add_child(light_proxy_geometry);

  /////////////////////////////////////////////////////////////////////////////
  // create viewing setup
  /////////////////////////////////////////////////////////////////////////////

  auto portal = graph.add_node<gua::node::TexturedQuadNode>("/", "portal");
  portal->data.set_size(gua::math::vec2(1.2f, 0.8f));
  portal->data.set_texture("portal");
  portal->scale(5.0f);
  portal->translate(4.5f, 1.0, -0.2f);
  portal->rotate(-60, 0.f, 1.f, 0.f);
  portal->translate(0.0f, 0.f, 5.0f);

  auto screen = graph.add_node<gua::node::ScreenNode>("/", "screen");
  //screen->data.set_size(gua::math::vec2(1.92f, 1.08f));
  //screen->data.set_size(gua::math::vec2(1.6f, 0.9f));
  screen->data.set_size(gua::math::vec2(0.45f, 0.25f));
  screen->translate(0, 0, 30.0);

  auto portal_screen = graph.add_node<gua::node::ScreenNode>("/", "portal_screen");
  portal_screen->data.set_size(gua::math::vec2(1.2f, 0.8f));
  portal_screen->translate(0.0, 0, 15.0);
  portal_screen->rotate(-90, 0.0, 1.0, 0.0);

  // add mouse interaction
  gua::utils::Trackball trackball(0.01, 0.002, 0.2);

  // setup rendering pipeline and window
  //auto resolution = gua::math::vec2ui(2560, 1440);
  auto resolution = gua::math::vec2ui(1920, 1080);
  
  /////////////////////////////////////////////////////////////////////////////
  // create portal camera and pipeline
  /////////////////////////////////////////////////////////////////////////////

  auto portal_camera = graph.add_node<gua::node::CameraNode>("/portal_screen", "portal_cam");
  portal_camera->translate(0.0, 0, 3.0);
  portal_camera->config.set_resolution(gua::math::vec2ui(1200, 800));
  //portal_camera->config.set_resolution(resolution);
  portal_camera->config.set_screen_path("/portal_screen");
  portal_camera->config.set_scene_graph_name("main_scenegraph");
  portal_camera->config.set_output_texture_name("portal");
  portal_camera->config.set_enable_stereo(false);
  portal_camera->config.set_far_clip(200.0);
  portal_camera->config.set_near_clip(0.1);

  auto portal_pipe = std::make_shared<gua::PipelineDescription>();
  portal_pipe->add_pass(std::make_shared<gua::TriMeshPassDescription>());
  portal_pipe->add_pass(std::make_shared<gua::PLODPassDescription>());
  portal_pipe->add_pass(std::make_shared<gua::LightVisibilityPassDescription>());
  portal_pipe->add_pass(std::make_shared<gua::ResolvePassDescription>());
  portal_pipe->get_pass_by_type<gua::ResolvePassDescription>()->background_mode(gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE);
  portal_pipe->get_pass_by_type<gua::ResolvePassDescription>()->background_texture("data/images/skymap.jpg");
  portal_pipe->add_pass(std::make_shared<gua::DebugViewPassDescription>());
  portal_camera->set_pipeline_description(portal_pipe);
  
  /////////////////////////////////////////////////////////////////////////////
  // create scene camera and pipeline
  /////////////////////////////////////////////////////////////////////////////

  auto camera = graph.add_node<gua::node::CameraNode>("/screen", "cam");
  camera->translate(0, 0, 1.0);
  camera->config.set_resolution(resolution);
  camera->config.set_screen_path("/screen");
  camera->config.set_eye_dist(0.06);
  camera->config.set_left_screen_path("/screen");
  camera->config.set_right_screen_path("/screen");
  camera->config.set_scene_graph_name("main_scenegraph");
  camera->config.set_output_window_name("main_window");
  camera->config.set_enable_stereo(false);
  camera->config.set_far_clip(100.0);
  camera->config.set_near_clip(0.01);
  camera->add_child(camera_proxy_geometry);
 //camera->set_pre_render_cameras({portal_camera});

  auto pipe = std::make_shared<gua::PipelineDescription>();

  pipe->add_pass(std::make_shared<gua::TriMeshPassDescription>());
  pipe->add_pass(std::make_shared<gua::TexturedQuadPassDescription>());
  //pipe->add_pass(std::make_shared<gua::BBoxPassDescription>());
  pipe->add_pass(std::make_shared<gua::PLODPassDescription>());
  pipe->add_pass(std::make_shared<gua::LightVisibilityPassDescription>());
  pipe->add_pass(std::make_shared<gua::ResolvePassDescription>());
  pipe->add_pass(std::make_shared<gua::DebugViewPassDescription>());

  pipe->get_resolve_pass()->background_mode(gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE);
  pipe->get_resolve_pass()->background_texture("data/images/skymap.jpg");

  //pipe->get_pass_by_type<gua::ResolvePassDescription>()->ssao_enable(true);
  //pipe->get_pass_by_type<gua::ResolvePassDescription>()->ssao_radius(16.0);
  //pipe->get_pass_by_type<gua::ResolvePassDescription>()->ssao_enable(2.5);

  camera->set_pipeline_description(pipe);
  
  /////////////////////////////////////////////////////////////////////////////
  // create window and callback setup
  /////////////////////////////////////////////////////////////////////////////

  auto window = std::make_shared<gua::GlfwWindow>();
  gua::WindowDatabase::instance()->add("main_window", window);
  window->config.set_enable_vsync(false);
  window->config.set_size(resolution);
  window->config.set_resolution(resolution);
  //window->config.set_stereo_mode(gua::StereoMode::ANAGLYPH_RED_CYAN);
  window->config.set_stereo_mode(gua::StereoMode::MONO);

  window->on_resize.connect([&](gua::math::vec2ui const& new_size) {
    window->config.set_resolution(new_size);
    camera->config.set_resolution(new_size);
    screen->data.set_size(gua::math::vec2(0.001 * new_size.x, 0.001 * new_size.y));
  });

  window->on_move_cursor.connect([&](gua::math::vec2 const& pos) {
    trackball.motion(pos.x, pos.y);
  });

  window->on_button_press.connect(std::bind(mouse_button, std::ref(trackball), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  window->on_key_press.connect(std::bind(key_press, std::ref(*(camera->get_pipeline_description())), std::ref(graph), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

  window->open();

  gua::Renderer renderer;

  // application loop
  gua::events::MainLoop loop;
  gua::events::Ticker ticker(loop, 1.0/500.0);

  std::size_t ctr = 0;

  ticker.on_tick.connect([&]() {
    gua::math::mat4 modelmatrix = scm::math::make_translation(gua::math::float_t(trackball.shiftx()), 
                                                              gua::math::float_t(trackball.shifty()), 
                                                              gua::math::float_t(trackball.distance())) * gua::math::mat4(trackball.rotation());
    transform->set_transform(modelmatrix);
    static unsigned framecounter = 0;
    ++framecounter;

    if (rotate_light) {
      // modify scene
      light->rotate(0.1, 0.0, 1.0, 0.0);
    }

    if (ctr++ % 150 == 0)
      std::cout << "Frame time: " << 1000.f / camera->get_rendering_fps() << " ms, fps: "
      << camera->get_rendering_fps() << ", app fps: "
      << camera->get_application_fps() << std::endl;

    // apply trackball matrix to object
    window->process_events();
    if (window->should_close()) {
      renderer.stop();
      window->close();
      loop.stop();
    } else { 
      renderer.queue_draw({&graph}, {camera});
    }
  });

  loop.start();

  return 0;
}
