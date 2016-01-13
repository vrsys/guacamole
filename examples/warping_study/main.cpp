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

#define POWER_WALL      0
#define SHADOWS         1

#include <functional>

#include <scm/input/tracking/art_dtrack.h>
#include <scm/input/tracking/target.h>

#include <gua/guacamole.hpp>
#include <gua/renderer/TexturedScreenSpaceQuadPass.hpp>
#include <gua/renderer/TriMeshLoader.hpp>
#include <gua/renderer/DebugViewPass.hpp>
#include <gua/utils/Trackball.hpp>
#include <gua/renderer/PLODPass.hpp>
#include <gua/renderer/PLODLoader.hpp>
#include <gua/node/PLODNode.hpp>
#include <gua/gui.hpp>
#include <gua/physics.hpp>

#include "Navigator.hpp"

bool vsync                  = false;
float eye_offset            = 0.f;
int current_scene           = 0;
int current_mode            = 0;

std::vector<std::vector<gua::StereoType>> stereo_types(3);
std::vector<std::string> scenes(3);

const float aspect = 1.0f/1.6f;
const float screen_width = 4.f;
const float screen_dist = 2.5f;
const int   window_size = 1600;

float eye_dist = 0.1f;

gua::math::mat4 current_tracking_matrix(gua::math::mat4::identity());
std::string     current_transparency_mode("set_transparency_type_raycasting");

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

void show_backfaces(std::shared_ptr<gua::node::Node> const& node, bool show) {
  auto casted(std::dynamic_pointer_cast<gua::node::TriMeshNode>(node));
  if (casted) {
    casted->get_material()->set_show_back_faces(show);
  }

  for (auto& child: node->get_children()) {
    show_backfaces(child, show);
  }
}

int main(int argc, char** argv) {

  if (argc == 2) {
    std::string permutation(argv[1]);
    if (permutation.length() == 5) {
      if (permutation[1] == '1') scenes = {"pitoti", "oilrig", "physics"};
      if (permutation[1] == '2') scenes = {"pitoti", "physics", "oilrig"};
      if (permutation[1] == '3') scenes = {"oilrig", "pitoti", "physics"};
      if (permutation[1] == '4') scenes = {"oilrig", "physics", "pitoti"};
      if (permutation[1] == '5') scenes = {"physics", "oilrig", "pitoti"};
      if (permutation[1] == '6') scenes = {"physics", "pitoti", "oilrig"};

      for (int i(0); i<3; ++i) {
        if (permutation[i+2] == '1') stereo_types[i] = {gua::StereoType::RENDER_TWICE, gua::StereoType::SPATIAL_WARP, gua::StereoType::TEMPORAL_WARP};
        if (permutation[i+2] == '2') stereo_types[i] = {gua::StereoType::RENDER_TWICE, gua::StereoType::TEMPORAL_WARP, gua::StereoType::SPATIAL_WARP};
        if (permutation[i+2] == '3') stereo_types[i] = {gua::StereoType::SPATIAL_WARP, gua::StereoType::TEMPORAL_WARP, gua::StereoType::RENDER_TWICE};
        if (permutation[i+2] == '4') stereo_types[i] = {gua::StereoType::SPATIAL_WARP, gua::StereoType::RENDER_TWICE, gua::StereoType::TEMPORAL_WARP};
        if (permutation[i+2] == '5') stereo_types[i] = {gua::StereoType::TEMPORAL_WARP, gua::StereoType::RENDER_TWICE, gua::StereoType::SPATIAL_WARP};
        if (permutation[i+2] == '6') stereo_types[i] = {gua::StereoType::TEMPORAL_WARP, gua::StereoType::SPATIAL_WARP, gua::StereoType::RENDER_TWICE};
      }
    } else return 0;
  } else return 0;

  // initialize guacamole
  gua::init(argc, argv);

  gua::Logger::enable_debug = false;

  auto physics = std::make_shared<gua::physics::Physics>();
  auto window = std::make_shared<gua::GlfwWindow>();

  #if POWER_WALL
    bool fullscreen = true;
    auto resolution = gua::math::vec2ui(1780, 1185);
  #else
    bool fullscreen = false;
    auto resolution = gua::math::vec2ui(window_size, window_size*aspect);
  #endif

  // add mouse interaction
  gua::utils::Trackball object_trackball(0.01, 0.002, 0, 0.2);
  Navigator nav;
  Navigator warp_nav;
  nav.set_transform(scm::math::make_translation(0.f, 0.f, 3.f));

  gua::SceneGraph graph("main_scenegraph");

  // ---------------------------------------------------------------------------
  // ---------------------------- setup scene ----------------------------------
  // ---------------------------------------------------------------------------

  #if WIN32
    //std::string opt_prefix("D:/guacamole/");
    std::string opt_prefix("C:/Users/localadmin/Desktop/Simon/opt/");
	#else
    std::string opt_prefix("/opt/");
	#endif

  gua::TriMeshLoader loader;
  auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");
  transform->get_tags().add_tag("scene");

  auto sun_light = graph.add_node<gua::node::LightNode>("/", "sun_light");
  sun_light->data.set_type(gua::node::LightNode::Type::SUN);
  sun_light->data.set_color(gua::utils::Color3f(1.5f, 1.2f, 1.f));
  sun_light->data.set_shadow_cascaded_splits({0.1f, 1.5, 5.f, 10.f});
  sun_light->data.set_shadow_near_clipping_in_sun_direction(100.0f);
  sun_light->data.set_shadow_far_clipping_in_sun_direction(100.0f);
  sun_light->data.set_max_shadow_dist(30.0f);
  sun_light->data.set_shadow_offset(0.0004f);
  sun_light->data.set_enable_shadows(SHADOWS);
  sun_light->data.set_shadow_map_size(512);
  sun_light->data.set_brightness(3.f);
  sun_light->rotate(-65, 1, 0, 0);
  sun_light->rotate(-100, 0, 1, 0);

  auto light2 = graph.add_node<gua::node::LightNode>("/", "light2");
  light2->data.set_type(gua::node::LightNode::Type::SUN);
  light2->data.set_color(gua::utils::Color3f(0.8f, 1.0f, 1.5f));
  light2->data.set_brightness(0.5f);
  light2->data.set_enable_specular_shading(false);
  light2->rotate(45, 1, 0, 0);
  light2->rotate(-120, 0, 1, 0);

  auto light3 = graph.add_node<gua::node::LightNode>("/", "light3");
  light3->data.set_type(gua::node::LightNode::Type::SUN);
  light3->data.set_color(gua::utils::Color3f(0.9f, 1.3f, 1.1f));
  light3->data.set_brightness(1.0f);
  light3->data.set_enable_specular_shading(false);
  light3->rotate(45, 1, 0, 0);
  light3->rotate(80, 0, 1, 0);


  // one oilrig ----------------------------------------------------------------
  auto scene_root = graph.add_node<gua::node::TransformNode>("/transform", "one_oilrig");
  scene_root->scale(20);
  scene_root->rotate(-90, 1, 0, 0);

  auto oilrig(loader.create_geometry_from_file("oilrig", opt_prefix + "3d_models/OIL_RIG_GUACAMOLE/oilrig.obj",
    gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION |
    gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::OPTIMIZE_MATERIALS |
    gua::TriMeshLoader::NORMALIZE_SCALE));
  scene_root->add_child(oilrig);

  // pitoti --------------------------------------------------------------------
  scene_root = graph.add_node<gua::node::TransformNode>("/transform", "pitoti");
  gua::PLODLoader plodloader;
  plodloader.set_out_of_core_budget_in_mb(5000);
  plodloader.set_render_budget_in_mb(1000);
  plodloader.set_upload_budget_in_mb(20);
  // for (int i(1); i<=2; ++i) {
  //   // auto pitoti(plodloader.load_geometry("/mnt/pitoti/hallermann_scans/bruecke/bruecke_points_part0000" + std::to_string(i) + "_knobi.kdn",
  //   auto pitoti(plodloader.load_geometry("/mnt/pitoti/hallermann_scans/Pointcloud_Stuetzwand_part_0000" + std::to_string(i) + "_knobi.kdn",
  //     gua::PLODLoader::NORMALIZE_POSITION | gua::PLODLoader::NORMALIZE_SCALE));
  //   scene_root->add_child(pitoti);
  //   pitoti->set_radius_scale(1.2f);
  //   pitoti->set_error_threshold(2.0f);
  // }
  // auto pitoti(plodloader.load_geometry("/mnt/pitoti/misc_scans/UDK/udk_9.bvh",
  auto pitoti(plodloader.load_geometry("/mnt/pitoti/3d_pitoti/seradina_12c/rock/TLS_Seradina_Rock-12C_knn.kdn",
      gua::PLODLoader::NORMALIZE_POSITION | gua::PLODLoader::NORMALIZE_SCALE));
  scene_root->add_child(pitoti);
  pitoti->set_radius_scale(1.2f);
  pitoti->set_error_threshold(2.0f);
  scene_root->rotate(-90, 1, 0, 0);
  scene_root->scale(10);

  // physics --------------------------------------------------------------------
  scene_root = graph.add_node<gua::node::TransformNode>("/transform", "physics");

  auto plane(loader.create_geometry_from_file("plane", "data/objects/plane.obj",
    gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION |
    gua::TriMeshLoader::NORMALIZE_SCALE));
  plane->scale(10);
  plane->translate(0, 1, 0);
  auto casted(std::dynamic_pointer_cast<gua::node::TriMeshNode>(plane));
  if (casted) {
    casted->get_material()->set_show_back_faces(true);
    casted->get_material()->set_uniform("Metalness", 0.1f);
    casted->get_material()->set_uniform("Roughness", 0.5f);
    casted->get_material()->set_uniform("RoughnessMap", std::string("data/textures/tiles_specular.jpg"));
    casted->get_material()->set_uniform("ColorMap", std::string("data/textures/tiles_diffuse.jpg"));
    casted->get_material()->set_uniform("NormalMap", std::string("data/textures/tiles_normal.jpg"));
  }

  scene_root->add_child(plane);

  gua::physics::CollisionShapeDatabase::add_shape("sphere", new gua::physics::SphereShape(0.25));
  gua::physics::CollisionShapeDatabase::add_shape("box", new gua::physics::BoxShape(gua::math::vec3(5, 1, 5)));

  auto floor_body = std::make_shared<gua::physics::RigidBodyNode>("floor_body", 0, 0.5, 0.7);
  auto floor_shape = std::make_shared<gua::physics::CollisionShapeNode>("floor_shape");
  floor_shape->data.set_shape("box");
  graph.get_root()->add_child(floor_body);
  floor_body->add_child(floor_shape);
  physics->add_rigid_body(floor_body);

  std::list<std::shared_ptr<gua::physics::RigidBodyNode>> balls;

  auto add_sphere = [&](){
    auto sphere_body = std::make_shared<gua::physics::RigidBodyNode>("sphere_body", 5, 0.5, 0.7, scm::math::make_translation(1.0-2.0*std::rand()/RAND_MAX, 5.0, 1.0-2.0*std::rand()/RAND_MAX));
    auto sphere_shape = std::make_shared<gua::physics::CollisionShapeNode>("sphere_shape");
    sphere_shape->data.set_shape("sphere");
    graph.get_root()->add_child(sphere_body);
    sphere_body->add_child(sphere_shape);

    auto sphere_geometry(loader.create_geometry_from_file("sphere_geometry", "data/objects/sphere.obj",
      gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION |
      gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::OPTIMIZE_MATERIALS |
      gua::TriMeshLoader::NORMALIZE_SCALE));
    sphere_shape->add_child(sphere_geometry);
    sphere_geometry->scale(0.5);

    physics->add_rigid_body(sphere_body);

    balls.push_back(sphere_body);
  };




  auto pipe = gua::PipelineFactory::make_pipeline(
    gua::PipelineFactory::DEFAULT | 
    gua::PipelineFactory::DRAW_PLODS |
    gua::PipelineFactory::ABUFFER |
    gua::PipelineFactory::DEBUG_WARPING |
    gua::PipelineFactory::WARPING
  );

  pipe->get_resolve_pass()->
    background_mode(gua::ResolvePassDescription::BackgroundMode::SKYMAP_TEXTURE).
    background_texture(opt_prefix + "guacamole/resources/skymaps/DH206SN.png").
    background_texture(opt_prefix + "guacamole/resources/skymaps/cycles_island.jpg").
    background_color(gua::utils::Color3f(0,0,0)).
    environment_lighting(gua::utils::Color3f(0.4, 0.4, 0.5)).
    environment_lighting_mode(gua::ResolvePassDescription::EnvironmentLightingMode::AMBIENT_COLOR).
    // ssao_enable(true).
    tone_mapping_method(gua::ResolvePassDescription::ToneMappingMethod::HEJL).
    tone_mapping_exposure(1.5f).
    horizon_fade(0.2f).
    ssao_intensity(1.5f).
    ssao_radius(2.f);

  auto res_pass(pipe->get_resolve_pass());
  auto warp_pass(pipe->get_pass_by_type<gua::WarpPassDescription>());
  auto grid_pass(pipe->get_pass_by_type<gua::GenerateWarpGridPassDescription>());
  auto render_grid_pass(pipe->get_pass_by_type<gua::RenderWarpGridPassDescription>());

  warp_pass->interpolation_mode(gua::WarpPassDescription::INTERPOLATION_MODE_ADAPTIVE);
  warp_pass->gbuffer_warp_mode(gua::WarpPassDescription::GBUFFER_GRID_NON_UNIFORM_SURFACE_ESTIMATION);
  grid_pass->mode(gua::WarpPassDescription::GBUFFER_GRID_NON_UNIFORM_SURFACE_ESTIMATION);
  warp_pass->hole_filling_mode(gua::WarpPassDescription::HOLE_FILLING_BLUR);
  render_grid_pass->mode(gua::WarpPassDescription::GBUFFER_GRID_NON_UNIFORM_SURFACE_ESTIMATION);

  auto set_scene = [&](std::string const& name) {
    graph["/transform/physics"]->get_tags().add_tag("invisible");
    graph["/transform/one_oilrig"]->get_tags().add_tag("invisible");
    graph["/transform/pitoti"]->get_tags().add_tag("invisible");

    sun_light->data.set_enable_shadows(true);

    if (name == "oilrig") {
      graph["/transform/one_oilrig"]->get_tags().remove_tag("invisible");
      nav.set_transform(scm::math::mat4f(0.001, -0.018, 1.000, 2.351,
                                         0.000, 1.000, 0.018, 0.151,
                                         -1.000, -0.000, 0.001, 3.742,
                                         0.000, 0.000, 0.000, 1.000));
    }
    if (name == "physics") {
      graph["/transform/physics"]->get_tags().remove_tag("invisible");

      nav.set_transform(scm::math::mat4f(0.001, -0.006, -1.000, -4.940,
                                         0.000, 1.000, -0.006, 0.556,
                                         1.000, 0.000, 0.001, 2.416,
                                         0.000, 0.000, 0.000, 1.000));
    }               
    if (name == "pitoti") {
      sun_light->data.set_enable_shadows(false);
      graph["/transform/pitoti"]->get_tags().remove_tag("invisible");

      nav.set_transform(scm::math::mat4f(-0.188, 0.015, 0.982, 1.020,
                                         0.000, 1.000, -0.015, -1.774,
                                         -0.982, -0.003, -0.188, 1.475,
                                         0.000, 0.000, 0.000, 1.000));                     
    }
  };


  // ---------------------------------------------------------------------------
  // ------------------------ setup rendering pipelines ------------------------
  // ---------------------------------------------------------------------------
  auto navigation = graph.add_node<gua::node::TransformNode>("/", "navigation");
  auto warp_navigation = graph.add_node<gua::node::TransformNode>("/navigation", "warp");

  // normal camera -------------------------------------------------------------
  auto normal_screen = graph.add_node<gua::node::ScreenNode>("/navigation", "normal_screen");
  auto normal_cam = graph.add_node<gua::node::CameraNode>("/navigation", "normal_cam");
  #if POWER_WALL
    normal_screen->data.set_size(gua::math::vec2(3, 2));
    normal_screen->translate(0, 1.42, -1.6);
    normal_cam->translate(0, 1.42, -1.6);
  #else
    normal_screen->data.set_size(gua::math::vec2(screen_width,screen_width*aspect));
    normal_screen->translate(0, 0, -screen_dist);
  #endif
  normal_cam->config.set_resolution(resolution);
  normal_cam->config.set_screen_path("/navigation/normal_screen");

  normal_cam->config.set_scene_graph_name("main_scenegraph");
  normal_cam->config.mask().blacklist.add_tag("invisible");
  normal_cam->config.set_far_clip(350.f);
  normal_cam->config.set_near_clip(0.1f);
  normal_cam->config.set_eye_dist(eye_dist);

  // warping camera ------------------------------------------------------------
  auto warp_screen = graph.add_node<gua::node::ScreenNode>("/navigation/warp", "warp_screen");
  auto warp_cam = graph.add_node<gua::node::CameraNode>("/navigation/warp", "warp_cam");
  #if POWER_WALL
    warp_screen->data.set_size(gua::math::vec2(3, 2));
    warp_screen->translate(0, 1.42, -1.6);
    warp_cam->translate(0, 1.42, -1.6);
  #else
    warp_screen->data.set_size(gua::math::vec2(screen_width,screen_width*aspect));
    warp_screen->translate(0, 0, -screen_dist);
  #endif
  warp_cam->config.set_resolution(resolution);
  warp_cam->config.set_screen_path("/navigation/warp/warp_screen");

  warp_cam->config.set_scene_graph_name("main_scenegraph");
  warp_cam->config.set_far_clip(normal_cam->config.get_far_clip()*1.5);
  warp_cam->config.set_near_clip(normal_cam->config.get_near_clip());
  warp_cam->config.set_eye_dist(eye_dist);

  normal_cam->config.set_output_window_name("window");
  normal_cam->set_pipeline_description(pipe);

  // pipe->set_enable_abuffer(true);
  res_pass->write_abuffer_depth(false);
  normal_cam->config.set_enable_stereo(true);
  window->config.set_stereo_mode(POWER_WALL ? gua::StereoMode::SIDE_BY_SIDE : gua::StereoMode::ANAGLYPH_RED_CYAN);
  warp_pass->abuffer_warp_mode(gua::WarpPassDescription::ABUFFER_NONE);
  // warp_pass->abuffer_warp_mode(gua::WarpPassDescription::ABUFFER_RAYCASTING);
  
  set_scene(scenes[current_scene]);
  normal_cam->config.set_stereo_type(stereo_types[current_scene][current_mode]);

  // ---------------------------------------------------------------------------
  // ----------------------------- setup gui -----------------------------------
  // ---------------------------------------------------------------------------

  auto mouse = std::make_shared<gua::GuiResource>();
  mouse->init("mouse", "asset://gua/data/gui/mouse.html", gua::math::vec2ui(50, 50));

  auto gui_quad = std::make_shared<gua::node::TexturedScreenSpaceQuadNode>("gui_quad");
  gui_quad->data.texture() = "gui";
  gui_quad->data.size() = gua::math::vec2ui(550, 160);
  graph.add_node("/", gui_quad);

  auto mouse_quad = std::make_shared<gua::node::TexturedScreenSpaceQuadNode>("mouse_quad");
  mouse_quad->data.texture() = "mouse";
  mouse_quad->data.size() = gua::math::vec2ui(50, 50);
  graph.add_node("/", mouse_quad);

  mouse_quad->data.anchor() = gua::math::vec2(-1.f, -1.f);
  gui_quad->data.anchor() = gua::math::vec2(0.f, 1.f);
  gui_quad->data.offset() = gua::math::vec2(0.f, -20.f);

  gua::Interface::instance()->on_cursor_change.connect([&](gua::Cursor pointer){
    mouse->call_javascript("set_active", pointer == gua::Cursor::HAND);
    return true;
  });

  // right side gui ----------------------------------------------------------
  auto gui = std::make_shared<gua::GuiResource>();
  gui->init("gui", "asset://gua/data/gui/gui.html", gua::math::vec2ui(550, 160));

  gui->on_loaded.connect([&]() {
    gui->add_javascript_callback("set_warp_mode_1");
    gui->add_javascript_callback("set_warp_mode_2");
    gui->add_javascript_callback("set_warp_mode_3");
    gui->add_javascript_callback("set_scene_1");
    gui->add_javascript_callback("set_scene_2");
    gui->add_javascript_callback("set_scene_3");

    gui->call_javascript("init");
  });

  gui->on_javascript_callback.connect([&](std::string const& callback, std::vector<std::string> const& params) {

    if      (callback == "set_scene_1") current_scene = 0;
    else if (callback == "set_scene_2") current_scene = 1;
    else if (callback == "set_scene_3") current_scene = 2;

    if      (callback == "set_warp_mode_1") current_mode = 0;
    else if (callback == "set_warp_mode_2") current_mode = 1;
    else if (callback == "set_warp_mode_3") current_mode = 2;
    
    set_scene(scenes[current_scene]);
    normal_cam->config.set_stereo_type(stereo_types[current_scene][current_mode]);
  });

  // ---------------------------------------------------------------------------
  // ----------------------------- setup windows -------------------------------
  // ---------------------------------------------------------------------------
  window->config.set_fullscreen_mode(fullscreen);
  window->cursor_mode(gua::GlfwWindow::CursorMode::HIDDEN);

  #if (!POWER_WALL)
    window->on_resize.connect([&](gua::math::vec2ui const& new_size) {
      resolution = new_size;
      window->config.set_resolution(new_size);
      normal_cam->config.set_resolution(new_size);
      normal_screen->data.set_size(gua::math::vec2(screen_width, screen_width * new_size.y / new_size.x));
      warp_screen->data.set_size(gua::math::vec2(screen_width, screen_width * new_size.y / new_size.x));
    });
  #endif

  window->on_button_press.connect([&](int key, int action, int mods) {
    gui->inject_mouse_button(gua::Button(key), action, mods);
    nav.set_mouse_button(key, action);
    warp_nav.set_mouse_button(key, action);

  });

  window->on_key_press.connect([&](int key, int scancode, int action, int mods) {
    nav.set_key_press(key, action);

    if (action >= 1) {
      if (key == 49) {
        gui->call_javascript("select_1");
      }
      if (key == 50) {
        gui->call_javascript("select_2");
      }
      if (key == 51) {
        gui->call_javascript("select_3");
      } 
    }
  });

  window->on_move_cursor.connect([&](gua::math::vec2 const& pos) {
    bool hit_gui(false);
    gua::math::vec2 hit_pos;
    mouse_quad->data.offset() = pos + gua::math::vec2i(-2, -45);
    hit_gui = gui_quad->pixel_to_texcoords(pos, resolution, hit_pos) && !gui_quad->get_tags().has_tag("invisible");

    if (hit_gui) {
      gui->inject_mouse_position_relative(hit_pos);
    } else {
      nav.set_mouse_position(gua::math::vec2i(pos));
    }
  });

  window->on_button_press.connect(std::bind(mouse_button, std::ref(object_trackball), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  #if POWER_WALL
    window->config.set_size(gua::math::vec2ui(1920*2, 1200));
    window->config.set_right_position(gua::math::vec2ui(1920, 0));
    window->config.set_right_resolution(gua::math::vec2ui(1780, 1185));
    window->config.set_left_position(gua::math::vec2ui(140, 0));
    window->config.set_left_resolution(gua::math::vec2ui(1780, 1185));
  #else
    window->config.set_size(resolution);
    window->config.set_resolution(resolution);
  #endif
  window->config.set_enable_vsync(vsync);
  gua::WindowDatabase::instance()->add("window", window);
  window->open();

  // tracking ------------------------------------------------------------------
  warp_pass->get_warp_state([&](){
    #if POWER_WALL
      warp_cam->set_transform(current_tracking_matrix);
    #endif

    gua::WarpPassDescription::WarpState state;

    gua::Frustum frustum = warp_cam->get_rendering_frustum(graph, gua::CameraMode::CENTER);
    state.projection_view_center = frustum.get_projection() * frustum.get_view();

    frustum = warp_cam->get_rendering_frustum(graph, gua::CameraMode::LEFT);
    state.projection_view_left = frustum.get_projection() * frustum.get_view();

    frustum = warp_cam->get_rendering_frustum(graph, gua::CameraMode::RIGHT);
    state.projection_view_right = frustum.get_projection() * frustum.get_view();

    return state;
  });

  #if POWER_WALL
    std::thread tracking_thread([&]() {
      scm::inp::tracker::target_container targets;
      targets.insert(scm::inp::tracker::target_container::value_type(5, scm::inp::target(5)));

      scm::inp::art_dtrack* dtrack(new scm::inp::art_dtrack(5000));
      if (!dtrack->initialize()) {
        std::cerr << std::endl << "Tracking System Fault" << std::endl;
        return;
      }
      while (true) {
        dtrack->update(targets);
        auto t = targets.find(5)->second.transform();
        t[12] /= 1000.f; t[13] /= 1000.f; t[14] /= 1000.f;
        current_tracking_matrix = t;
      }
    });
  #endif

  // render setup --------------------------------------------------------------
  gua::Renderer renderer;

  gua::Timer frame_timer;
  frame_timer.start();

  // application loop
  while(true) {

    bool spawn_balls(!graph["/transform/physics"]->get_tags().has_tag("invisible"));

    if (spawn_balls && frame_timer.get_elapsed() > 0.02f) {
      frame_timer.reset();
      add_sphere();
    }

    auto b(balls.begin());
    while (b != balls.end()) {
      if (!spawn_balls || (*b)->get_world_position().y < -5.0) {
        physics->remove_rigid_body(*b);
        graph.get_root()->remove_child(*b);
        b = balls.erase(b);
      } else {
        ++b;
      }
    }

    physics->synchronize(true);

    // std::cout << nav.get_transform() << std::endl;

    #if POWER_WALL
      normal_cam->set_transform(current_tracking_matrix);
    #endif

    nav.update();
    warp_nav.update();

    #if POWER_WALL
      navigation->set_transform(normal_screen->get_transform() * gua::math::mat4(nav.get_transform()) * scm::math::inverse(normal_screen->get_transform()));
      warp_navigation->set_transform(normal_screen->get_transform() * gua::math::mat4(warp_nav.get_transform()) * scm::math::inverse(normal_screen->get_transform()));
    #else
      navigation->set_transform(gua::math::mat4(nav.get_transform()));
      warp_navigation->set_transform(gua::math::mat4(warp_nav.get_transform()));
    #endif

    gua::Interface::instance()->update();

    gua::math::mat4 modelmatrix = scm::math::make_translation(gua::math::float_t(object_trackball.shiftx()),
                                                              gua::math::float_t(object_trackball.shifty()),
                                                              gua::math::float_t(object_trackball.distance())) * gua::math::mat4(object_trackball.rotation());
    transform->set_transform(modelmatrix);

    window->process_events();
    if (window->should_close()) {
      renderer.stop();
      window->close();
      break;
    } else {
      renderer.draw_single_threaded({&graph});
    }
  }

  return 0;
}
