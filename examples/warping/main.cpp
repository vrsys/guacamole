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

#define POWER_WALL      false
#define OCULUS          false
#define STEREO_MONITOR  false

#define SSAO            true
#define LOAD_CAR        true
#define LOAD_PITOTI     false
#define LOAD_MOUNTAINS  true

#include <functional>

#include <scm/input/tracking/art_dtrack.h>
#include <scm/input/tracking/target.h>

#include <gua/guacamole.hpp>
#include <gua/renderer/TexturedScreenSpaceQuadPass.hpp>
#include <gua/renderer/TriMeshLoader.hpp>
#include <gua/renderer/DebugViewPass.hpp>
#include <gua/utils/Trackball.hpp>
#include <gua/gui.hpp>

#if OCULUS
#include <OVR.h>
#include <gua/OculusWindow.hpp>
#endif

#if LOAD_PITOTI
#include <gua/renderer/PLODPass.hpp>
#include <gua/renderer/PLODLoader.hpp>
#include <gua/node/PLODNode.hpp>
#endif

#include "Navigator.hpp"

bool depth_test             = true;
bool backface_culling       = true;
bool manipulation_navigator = true;
bool manipulation_camera    = false;
bool manipulation_object    = false;
bool warping                = true;
bool stereo                 = false;
bool warp_perspective       = false;

gua::math::mat4 current_tracking_matrix(gua::math::mat4::identity());
std::string     current_transparency_mode("set_transparency_type_gbuffer");

#if OCULUS
  OVR::SensorFusion* init_oculus() {
    OVR::System::Init(OVR::Log::ConfigureDefaultLog(OVR::LogMask_All));
    OVR::DeviceManager* device_manager  = OVR::DeviceManager::Create();
    OVR::SensorDevice*  sensor_device   = device_manager->EnumerateDevices<OVR::SensorDevice>().CreateDevice();
    if (sensor_device) {
      OVR::SensorFusion* sensor_fusion = new OVR::SensorFusion();
      sensor_fusion->AttachToSensor(sensor_device);
      return sensor_fusion;
    }
    return nullptr;
  }

  gua::math::mat4 const get_oculus_transform(OVR::SensorFusion* sensor) {
    OVR::Quatf orient = sensor->GetPredictedOrientation();
    OVR::Matrix4f mat(orient.Inverted());
    return gua::math::mat4( mat.M[0][0], mat.M[0][1], mat.M[0][2], mat.M[0][3],
                            mat.M[1][0], mat.M[1][1], mat.M[1][2], mat.M[1][3],
                            mat.M[2][0], mat.M[2][1], mat.M[2][2], mat.M[2][3],
                            mat.M[3][0], mat.M[3][1], mat.M[3][2], mat.M[3][3]);
  }
#endif

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

  #if POWER_WALL
    bool fullscreen = true;
    auto resolution = gua::math::vec2ui(1780, 1185);
  #elif OCULUS
    bool fullscreen = true;
    auto resolution = gua::math::vec2ui(640, 800);

    // initialize Oculus SDK
    OVR::SensorFusion* oculus_sensor = init_oculus();
    if (!oculus_sensor) {
      gua::Logger::LOG_WARNING << "Could not connect to Oculus Rift! " << std::endl;
      return -1;
    }
  #elif STEREO_MONITOR
    bool fullscreen = true;
    auto resolution = gua::math::vec2ui(2560, 1440);
  #else
    bool fullscreen = (argc == 2);
    auto resolution = gua::math::vec2ui(1600, 1000);
  #endif

  // add mouse interaction
  gua::utils::Trackball object_trackball(0.01, 0.002, 0, 0.2);
  Navigator nav;
  Navigator warp_nav;
  nav.set_transform(scm::math::make_translation(0.f, 0.f, 3.f));

  // initialize guacamole
  gua::init(argc, argv);

  gua::SceneGraph graph("main_scenegraph");

  // ---------------------------------------------------------------------------
  // ---------------------------- setup scene ----------------------------------
  // ---------------------------------------------------------------------------

  gua::TriMeshLoader loader;
  auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");
  transform->get_tags().add_tag("scene");

  auto light = graph.add_node<gua::node::LightNode>("/", "light");
  light->data.set_type(gua::node::LightNode::Type::SUN);
  light->data.set_brightness(3.f);
  light->data.set_shadow_cascaded_splits({0.1f, 0.5f, 1.5f, 5.f, 20.f});
  light->data.set_shadow_near_clipping_in_sun_direction(10.0f);
  light->data.set_shadow_far_clipping_in_sun_direction(10.0f);
  light->data.set_max_shadow_dist(50.0f);
  light->data.set_shadow_offset(0.0005f);
  light->data.set_enable_shadows(true);
  light->data.set_shadow_map_size(2048);
  light->rotate(-95, 1, 0.5, 0);

  // floor
  auto plane(loader.create_geometry_from_file("plane", "data/objects/plane.obj",
    gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION |
    gua::TriMeshLoader::NORMALIZE_SCALE));
  plane->translate(0, -0.26, 0);
  plane->scale(2);
  auto casted(std::dynamic_pointer_cast<gua::node::TriMeshNode>(plane));
  if (casted) {
    casted->get_material()->set_show_back_faces(true);
    casted->get_material()->set_uniform("Metalness", 0.1f);
    casted->get_material()->set_uniform("Roughness", 0.5f);
    casted->get_material()->set_uniform("RoughnessMap", std::string("data/textures/tiles_specular.jpg"));
    casted->get_material()->set_uniform("ColorMap", std::string("data/textures/tiles_diffuse.jpg"));
    casted->get_material()->set_uniform("NormalMap", std::string("data/textures/tiles_normal.jpg"));
  }

  // many oilrigs scene --------------------------------------------------------
  auto scene_root = graph.add_node<gua::node::TransformNode>("/transform", "many_oilrigs");
  scene_root->rotate(-90, 1, 0, 0);
  auto add_oilrig = [&](int x, int y, int c, std::string const& parent) {
    auto t = graph.add_node<gua::node::TransformNode>(parent, "t");
    t->translate((x - c*0.5 + 0.5)/1.5, (y - c*0.5 + 0.8)/2, 0);
    auto oilrig(loader.create_geometry_from_file("oilrig", "/opt/3d_models/OIL_RIG_GUACAMOLE/oilrig.obj",
      gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION |
      gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::OPTIMIZE_MATERIALS |
      gua::TriMeshLoader::NORMALIZE_SCALE));
    t->add_child(oilrig);

  };

  for (int x(0); x<6; ++x) {
    for (int y(0); y<6; ++y) {
      add_oilrig(x, y, 6, "/transform/many_oilrigs");
    }
  }

  // one oilrig ----------------------------------------------------------------
  scene_root = graph.add_node<gua::node::TransformNode>("/transform", "one_oilrig");
  scene_root->scale(3);
  scene_root->rotate(-90, 1, 0, 0);
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
  scene_root->add_child(plane);

  // car --------------------------------------------------------------------
  scene_root = graph.add_node<gua::node::TransformNode>("/transform", "car");
  #if LOAD_CAR
    auto car(loader.create_geometry_from_file("car", "data/objects/car/car.dae",
      gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION |
      gua::TriMeshLoader::LOAD_MATERIALS |
      gua::TriMeshLoader::NORMALIZE_SCALE));
    scene_root->scale(2);
    scene_root->add_child(car);

    for (auto& child: car->get_children()) {
      auto casted(std::dynamic_pointer_cast<gua::node::TriMeshNode>(child));
      if (casted) {
        casted->get_material()->set_show_back_faces(true);
        casted->get_material()->set_uniform("Metalness", 0.5f);
        casted->get_material()->set_uniform("Roughness", 0.5f);
      }
    }
  #endif
  // scene_root->add_child(plane);

  // pitoti --------------------------------------------------------------------
  scene_root = graph.add_node<gua::node::TransformNode>("/transform", "pitoti");
  #if LOAD_PITOTI
    gua::PLODLoader plodloader;
    for (int i(1); i<=8; ++i) {
      auto pitoti(plodloader.load_geometry("/mnt/pitoti/hallermann_scans/bruecke/bruecke_points_part0000" + std::to_string(i) + "_knobi.kdn",
        gua::PLODLoader::DEFAULTS));
      scene_root->add_child(pitoti);
      pitoti->set_radius_scale(1.5f);
      pitoti->set_error_threshold(1.0f);
    }
    scene_root->rotate(-90, 1, 0, 0);
    scene_root->scale(0.5);
  #endif

  // bottle --------------------------------------------------------------------
  auto load_mat = [](std::string const& file){
    auto desc(std::make_shared<gua::MaterialShaderDescription>());
    desc->load_from_file(file);
    auto shader(std::make_shared<gua::MaterialShader>(file, desc));
    gua::MaterialShaderDatabase::instance()->add(shader);
    return shader->make_new_material();
  };

  scene_root = graph.add_node<gua::node::TransformNode>("/transform", "bottle");
  auto mat_bottle(load_mat("../transparency/data/materials/Bottle.gmd"));
  mat_bottle->set_uniform("ColorMap",     std::string("../transparency/data/objects/bottle/albedo.png"))
             .set_uniform("RoughnessMap", std::string("../transparency/data/objects/bottle/roughness.jpg"))
             .set_show_back_faces(true);

  for (int i(-1); i<=1; ++i) {
    auto bottle(loader.create_geometry_from_file("bottle", "../transparency/data/objects/bottle/bottle.obj", mat_bottle,
                                                 gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));
    bottle->translate(i*0.75, 0, 0);
    scene_root->add_child(bottle);
  }
  scene_root->add_child(plane);

  // mountains --------------------------------------------------------------------
  scene_root = graph.add_node<gua::node::TransformNode>("/transform", "mountains");
  #if LOAD_MOUNTAINS
    auto mountains(loader.create_geometry_from_file("mountains", "/home/rufu1194/Desktop/island/guacamole-restricted/vr_hyperspace/data/objects/terrain/lod0.obj",
      gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION |
      gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::OPTIMIZE_MATERIALS |
      gua::TriMeshLoader::NORMALIZE_SCALE));
    auto clouds(std::dynamic_pointer_cast<gua::node::TriMeshNode>(loader.create_geometry_from_file("clouds", "data/objects/plane.obj",
      gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION |
      gua::TriMeshLoader::NORMALIZE_SCALE)));
    clouds->translate(0, 0.05, 0);
    clouds->get_material()->set_uniform("ColorMap", std::string("data/textures/clouds.png"));
    clouds->get_material()->set_uniform("Roughness", 1.f);
    clouds->get_material()->set_uniform("Metalness", 0.f);
    clouds->get_material()->set_show_back_faces(false);
    clouds->set_shadow_mode(gua::ShadowMode::HIGH_QUALITY);
    scene_root->scale(30);
    scene_root->add_child(mountains);
    scene_root->add_child(clouds);
  #endif

  // spheres -------------------------------------------------------------------
  scene_root = graph.add_node<gua::node::TransformNode>("/transform", "sphere");
  auto sphere(loader.create_geometry_from_file("sphere", "data/objects/sphere.obj",
    gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION |
    gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::OPTIMIZE_MATERIALS |
    gua::TriMeshLoader::NORMALIZE_SCALE));
  scene_root->add_child(sphere);
  scene_root->add_child(plane);

  // buddha --------------------------------------------------------------------
  scene_root = graph.add_node<gua::node::TransformNode>("/transform", "buddha");
  auto buddha = loader.create_geometry_from_file("buddha", "data/objects/buddha.dae",
    gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION |
    gua::TriMeshLoader::NORMALIZE_SCALE);
  buddha->translate(0, -0.16, 0);
  for (auto c: buddha->get_children()) {
    auto node = std::dynamic_pointer_cast<gua::node::TriMeshNode>(c);
    node->get_material()->set_uniform("Color", gua::math::vec4(1.f, 0.7f, 0.f, 1.f));
    node->get_material()->set_uniform("Roughness", 0.2f);
    node->get_material()->set_uniform("Metalness", 1.0f);
  }
  scene_root->add_child(buddha);
  // scene_root->add_child(plane);

  // dragon --------------------------------------------------------------------
  scene_root = graph.add_node<gua::node::TransformNode>("/transform", "dragon");
  auto dragon = std::dynamic_pointer_cast<gua::node::TriMeshNode>(loader.create_geometry_from_file("dragon", "data/objects/dragon.dae",
    gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION |
    gua::TriMeshLoader::NORMALIZE_SCALE));
  auto transp_dragon = std::dynamic_pointer_cast<gua::node::TriMeshNode>(loader.create_geometry_from_file("dragon", "data/objects/dragon.dae",
    gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION |
    gua::TriMeshLoader::NORMALIZE_SCALE));
  dragon->get_material()->set_uniform("Color", gua::math::vec4(0.8f, 0.8f, 0.8f, 1.0f));
  transp_dragon->get_material()->set_uniform("Color", gua::math::vec4(1.f, 1.f, 1.f, 0.3f));

  dragon->translate(0.6, -0.17, 0);
  transp_dragon->translate(-0.6, 0, 0);
  transp_dragon->scale(1.4);
  scene_root->add_child(dragon);
  scene_root->add_child(transp_dragon);
  scene_root->add_child(plane);

  // sponza --------------------------------------------------------------------
  scene_root = graph.add_node<gua::node::TransformNode>("/transform", "sponza");
  auto sponza(loader.create_geometry_from_file("sponza", "/opt/3d_models/Sponza/sponza.obj",
    gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION |
    gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::OPTIMIZE_MATERIALS |
    gua::TriMeshLoader::NORMALIZE_SCALE));
  sponza->scale(20);
  sponza->translate(0, 3, 0);
  scene_root->add_child(transp_dragon);
  scene_root->add_child(sponza);

  // hairball --------------------------------------------------------------------
  scene_root = graph.add_node<gua::node::TransformNode>("/transform", "hairball");
  // auto hairball(loader.create_geometry_from_file("hairball", "data/objects/hairball.dae",
  //   gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION |
  //   gua::TriMeshLoader::NORMALIZE_SCALE));
  // scene_root->add_child(hairball);

  // engine --------------------------------------------------------------------
  scene_root = graph.add_node<gua::node::TransformNode>("/transform", "engine");
  scene_root->scale(0.05);

  auto light2 = std::make_shared<gua::node::LightNode>("light2");
  light2->data.set_type(gua::node::LightNode::Type::POINT);
  light2->data.set_brightness(10.0f);
  light2->scale(50.f);
  light2->translate(14.f, 20.f, -25.f);
  scene_root->add_child(light2);

  auto light3 = std::make_shared<gua::node::LightNode>("light3");
  light3->data.set_type(gua::node::LightNode::Type::POINT);
  light3->data.set_brightness(10.0f);
  light3->scale(50.f);
  light3->translate(-10.f, -20.f, 20.f);
  scene_root->add_child(light3);

  // material
  auto desc(std::make_shared<gua::MaterialShaderDescription>());
  desc->load_from_file("data/materials/engine.gmd");
  auto shader(std::make_shared<gua::MaterialShader>("data/materials/engine.gmd", desc));
  gua::MaterialShaderDatabase::instance()->add(shader);

  std::map<std::string, int> part_names {
    {"part_inner_04",2},   
    {"part_inner_03",2},  
    {"part_inner_02",2}, 
    {"part_08",2      }, 
    {"part_10",1      },  
    {"part_09",1      }, 
    {"part_04",2      },  
    {"part_05",1      },  
    {"part_06",2      },  
    {"part_07",2      }, 
    {"part_02",2      },  
    {"part_03",2      }, 
    {"part_inner_01",1},   
    {"part_01",0      }    
  };

  const std::string engine_directory = "/opt/3d_models/engine/";

  // prepare engine parts
  for(const auto& p : part_names) {
    auto mat = shader->make_new_material();
    mat->set_uniform("style", p.second)
     .set_uniform("opacity", 1.f)
     .set_uniform("cut_rad", 0.3f)
     .set_uniform("cut_n", gua::math::vec3(0.5, 1, 0))
     .set_uniform("roughness_map", std::string("data/textures/roughness.jpg"))
     .set_uniform("reflection_texture", std::string("/opt/guacamole/resources/skymaps/uffizi.jpg"))
     .set_show_back_faces(true);

    // derived from http://www.turbosquid.com/3d-models/speculate-3ds-free/302188
    // Royalty Free License
    auto part(loader.create_geometry_from_file(p.first,
                                               engine_directory + p.first + ".obj",
                                               mat,
                                               gua::TriMeshLoader::DEFAULTS));
    scene_root->add_child(part);
  }

  show_backfaces(transform);

  auto set_scene = [&](std::string const& name) {
    graph["/transform/many_oilrigs"]->get_tags().add_tag("invisible");
    graph["/transform/sponza"]->get_tags().add_tag("invisible");
    graph["/transform/one_oilrig"]->get_tags().add_tag("invisible");
    graph["/transform/textured_quads"]->get_tags().add_tag("invisible");
    graph["/transform/teapot"]->get_tags().add_tag("invisible");
    graph["/transform/car"]->get_tags().add_tag("invisible");
    graph["/transform/pitoti"]->get_tags().add_tag("invisible");
    graph["/transform/bottle"]->get_tags().add_tag("invisible");
    graph["/transform/mountains"]->get_tags().add_tag("invisible");
    graph["/transform/sphere"]->get_tags().add_tag("invisible");
    graph["/transform/engine"]->get_tags().add_tag("invisible");
    graph["/transform/dragon"]->get_tags().add_tag("invisible");
    graph["/transform/buddha"]->get_tags().add_tag("invisible");
    graph["/transform/hairball"]->get_tags().add_tag("invisible");

    if (name == "set_scene_many_oilrigs")
      graph["/transform/many_oilrigs"]->get_tags().remove_tag("invisible");
    if (name == "set_scene_one_oilrig")
      graph["/transform/one_oilrig"]->get_tags().remove_tag("invisible");
    if (name == "set_scene_sponza")
      graph["/transform/sponza"]->get_tags().remove_tag("invisible");
    if (name == "set_scene_textured_quads")
      graph["/transform/textured_quads"]->get_tags().remove_tag("invisible");
    if (name == "set_scene_teapot")
      graph["/transform/teapot"]->get_tags().remove_tag("invisible");
    if (name == "set_scene_car")
      graph["/transform/car"]->get_tags().remove_tag("invisible");
    if (name == "set_scene_pitoti")
      graph["/transform/pitoti"]->get_tags().remove_tag("invisible");
    if (name == "set_scene_bottle")
      graph["/transform/bottle"]->get_tags().remove_tag("invisible");
    if (name == "set_scene_mountains")
      graph["/transform/mountains"]->get_tags().remove_tag("invisible");
    if (name == "set_scene_sphere")
      graph["/transform/sphere"]->get_tags().remove_tag("invisible");
    if (name == "set_scene_engine")
      graph["/transform/engine"]->get_tags().remove_tag("invisible");
    if (name == "set_scene_dragon")
      graph["/transform/dragon"]->get_tags().remove_tag("invisible");
    if (name == "set_scene_hairball")
      graph["/transform/hairball"]->get_tags().remove_tag("invisible");
    if (name == "set_scene_buddha")
      graph["/transform/buddha"]->get_tags().remove_tag("invisible");
  };

  set_scene("set_scene_one_oilrig");

  // ---------------------------------------------------------------------------
  // ------------------------ setup rendering pipelines ------------------------
  // ---------------------------------------------------------------------------
  auto navigation = graph.add_node<gua::node::TransformNode>("/", "navigation");
  auto warp_navigation = graph.add_node<gua::node::TransformNode>("/navigation", "warp");

  // normal camera -------------------------------------------------------------
  #if OCULUS
    auto slow_cam = graph.add_node<gua::node::CameraNode>("/navigation", "slow_cam");
    auto slow_screen_left = graph.add_node<gua::node::ScreenNode>("/navigation/slow_cam", "slow_screen_left");
    auto slow_screen_right = graph.add_node<gua::node::ScreenNode>("/navigation/slow_cam", "slow_screen_right");
    slow_screen_left->data.set_size(gua::math::vec2(0.08, 0.1));
    slow_screen_right->data.set_size(gua::math::vec2(0.08, 0.1));
    slow_screen_right->translate(0.04, 0, -0.05f);
    slow_screen_left->set_transform(gua::math::mat4(scm::math::make_translation(0.f, 0.f, -0.05f)));
    slow_cam->config.set_eye_dist(0.f);
    slow_cam->config.set_resolution(resolution);
    slow_cam->config.set_left_screen_path("/navigation/slow_cam/slow_screen_left");
    slow_cam->config.set_right_screen_path("/navigation/slow_cam/slow_screen_right");
  #else
    auto slow_screen = graph.add_node<gua::node::ScreenNode>("/navigation", "slow_screen");
    auto slow_cam = graph.add_node<gua::node::CameraNode>("/navigation", "slow_cam");
    slow_screen->data.set_size(gua::math::vec2(3, 2));
    slow_screen->translate(0, 0, -1.7);
    #if POWER_WALL
      slow_screen->translate(0, 1.5, 0);
      slow_cam->translate(0, 1.5, 0);
    #endif
    slow_cam->config.set_eye_dist(0.f);
    slow_cam->config.set_resolution(resolution);
    slow_cam->config.set_screen_path("/navigation/slow_screen");
  #endif

  slow_cam->config.set_scene_graph_name("main_scenegraph");
  slow_cam->config.mask().blacklist.add_tag("invisible");
  slow_cam->config.set_near_clip(0.1f);

  // warping camera ------------------------------------------------------------
  #if OCULUS
    auto fast_cam = graph.add_node<gua::node::CameraNode>("/navigation", "fast_cam");
    auto fast_screen_left = graph.add_node<gua::node::ScreenNode>("/navigation/fast_cam", "fast_screen_left");
    auto fast_screen_right = graph.add_node<gua::node::ScreenNode>("/navigation/fast_cam", "fast_screen_right");
    fast_screen_left->data.set_size(gua::math::vec2(0.08, 0.1));
    fast_screen_right->data.set_size(gua::math::vec2(0.08, 0.1));
    fast_screen_left->set_transform(gua::math::mat4(scm::math::make_translation(0.f, 0.f, -0.05f)));
    fast_screen_right->translate(0.04, 0, -0.05f);
    fast_cam->config.set_eye_dist(0.f);
    fast_cam->config.set_resolution(resolution);
    fast_cam->config.set_left_screen_path("/navigation/fast_cam/fast_screen_left");
    fast_cam->config.set_right_screen_path("/navigation/fast_cam/fast_screen_right");
  #else
    auto fast_screen = graph.add_node<gua::node::ScreenNode>("/navigation/warp", "fast_screen");
    auto fast_cam = graph.add_node<gua::node::CameraNode>("/navigation/warp", "fast_cam");
    fast_screen->data.set_size(gua::math::vec2(3, 2));
    fast_screen->translate(0, 0, -1.7);
    #if POWER_WALL
      fast_screen->translate(0, 1.5, 0);
      fast_cam->translate(0, 1.5, 0);
    #endif
    fast_cam->config.set_eye_dist(0.f);
    fast_cam->config.set_resolution(resolution);
    fast_cam->config.set_screen_path("/navigation/warp/fast_screen");
  #endif

  fast_cam->config.set_scene_graph_name("main_scenegraph");
  fast_cam->config.set_far_clip(slow_cam->config.get_far_clip()*1.5);
  fast_cam->config.set_near_clip(0.1f);

  auto clear_pass(std::make_shared<gua::ClearPassDescription>());
  auto tex_quad_pass(std::make_shared<gua::TexturedQuadPassDescription>());
  auto light_pass(std::make_shared<gua::LightVisibilityPassDescription>());
  auto warp_pass(std::make_shared<gua::WarpPassDescription>());
  auto grid_pass(std::make_shared<gua::GenerateWarpGridPassDescription>());
  auto render_grid_pass(std::make_shared<gua::RenderWarpGridPassDescription>());
  auto trimesh_pass(std::make_shared<gua::TriMeshPassDescription>());
  #if LOAD_PITOTI
  auto plod_pass(std::make_shared<gua::PLODPassDescription>());
  #endif
  auto res_pass(std::make_shared<gua::ResolvePassDescription>());
  res_pass->background_mode(gua::ResolvePassDescription::BackgroundMode::SKYMAP_TEXTURE).
            background_texture("/opt/guacamole/resources/skymaps/uffizi.jpg").
            environment_lighting_texture("/opt/guacamole/resources/skymaps/uffizi.jpg").
            background_color(gua::utils::Color3f(0, 0, 0)).
            environment_lighting_mode(gua::ResolvePassDescription::EnvironmentLightingMode::SKYMAP_TEXTURE).
            ssao_enable(SSAO).
            horizon_fade(0.2f).
            compositing_enable(false).
            ssao_intensity(3.f).
            ssao_radius(2.f);

  slow_cam->config.set_output_window_name("window");

  auto warp_pipe = std::make_shared<gua::PipelineDescription>();
  warp_pipe->add_pass(clear_pass);
  warp_pipe->add_pass(trimesh_pass);
  warp_pipe->add_pass(tex_quad_pass);
  #if LOAD_PITOTI
  warp_pipe->add_pass(plod_pass);
  #endif
  warp_pipe->add_pass(light_pass);
  warp_pipe->add_pass(res_pass);
  warp_pipe->add_pass(grid_pass);
  warp_pipe->add_pass(warp_pass);
  warp_pipe->add_pass(render_grid_pass);
  warp_pipe->add_pass(std::make_shared<gua::TexturedScreenSpaceQuadPassDescription>());
  warp_pipe->set_enable_abuffer(true);
  slow_cam->set_pipeline_description(warp_pipe);

  auto normal_pipe = std::make_shared<gua::PipelineDescription>();
  normal_pipe->add_pass(std::make_shared<gua::ClearPassDescription>());
  normal_pipe->add_pass(trimesh_pass);
  normal_pipe->add_pass(tex_quad_pass);
  #if LOAD_PITOTI
  normal_pipe->add_pass(plod_pass);
  #endif
  normal_pipe->add_pass(light_pass);
  normal_pipe->add_pass(res_pass);
  normal_pipe->add_pass(std::make_shared<gua::TexturedScreenSpaceQuadPassDescription>());
  normal_pipe->set_enable_abuffer(true);

  #if OCULUS
    auto window = std::make_shared<gua::OculusWindow>();
  #else
    auto window = std::make_shared<gua::GlfwWindow>();
  #endif

  auto update_view_mode([&](){

    // set transparency mode
    if (warping) {
      warp_pipe->set_enable_abuffer(true);
      res_pass->compositing_enable(false);

      if (current_transparency_mode == "set_transparency_type_raycasting")    warp_pass->abuffer_warp_mode(gua::WarpPassDescription::ABUFFER_RAYCASTING);
      if (current_transparency_mode == "set_transparency_type_none") {
        warp_pass->abuffer_warp_mode(gua::WarpPassDescription::ABUFFER_NONE);
        warp_pipe->set_enable_abuffer(false);
      }
      if (current_transparency_mode == "set_transparency_type_gbuffer") {
        warp_pass->abuffer_warp_mode(gua::WarpPassDescription::ABUFFER_NONE);
        res_pass->compositing_enable(true);
      }
    } else {
      res_pass->compositing_enable(true);
    }

    // set stereo options
    if (stereo) {

      slow_cam->config.set_enable_stereo(true);

      #if OCULUS
        fast_screen_left->set_transform(gua::math::mat4(scm::math::make_translation(-0.04f, 0.f, -0.05f)));
        fast_screen_right->set_transform(gua::math::mat4(scm::math::make_translation(0.04f, 0.f, -0.05f)));
      #else
        window->config.set_stereo_mode(POWER_WALL ? gua::StereoMode::SIDE_BY_SIDE : gua::StereoMode::ANAGLYPH_RED_CYAN);
      #endif

      clear_pass->set_enable_for_right_eye(!warping);
      trimesh_pass->set_enable_for_right_eye(!warping);
      #if LOAD_PITOTI
        plod_pass->set_enable_for_right_eye(!warping);
      #endif
      tex_quad_pass->set_enable_for_right_eye(!warping);
      light_pass->set_enable_for_right_eye(!warping);
      res_pass->set_enable_for_right_eye(!warping);
      grid_pass->set_enable_for_right_eye(!warping);
      render_grid_pass->set_enable_for_right_eye(!warping);

      if (warping) {
        slow_cam->config.set_eye_dist(0.f);
        fast_cam->config.set_eye_dist(0.064f);
        
        #if OCULUS
          slow_screen_left->set_transform(gua::math::mat4(scm::math::make_translation(0.f, 0.f, -0.05f)));
          slow_screen_right->set_transform(gua::math::mat4(scm::math::make_translation(0.f, 0.f, -0.05f)));
        #endif
        
        slow_cam->set_pipeline_description(warp_pipe);

      } else {
        slow_cam->config.set_eye_dist(0.064f);
      
        #if OCULUS
          slow_screen_left->set_transform(gua::math::mat4(scm::math::make_translation(-0.04f, 0.f, -0.05f)));
          slow_screen_right->set_transform(gua::math::mat4(scm::math::make_translation(0.04f, 0.f, -0.05f)));
        #endif
        
        slow_cam->set_pipeline_description(normal_pipe);
      }

    } else {

      slow_cam->config.set_enable_stereo(false);
      slow_cam->config.set_eye_dist(0.f);
      fast_cam->config.set_eye_dist(0.f);
      clear_pass->set_enable_for_right_eye(true);

      #if OCULUS
        slow_screen_left->set_transform(gua::math::mat4(scm::math::make_translation(0.f, 0.f, -0.05f)));
        fast_screen_left->set_transform(gua::math::mat4(scm::math::make_translation(0.f, 0.f, -0.05f)));
      #else
        window->config.set_stereo_mode(gua::StereoMode::MONO);
      #endif

      if (warping) {
        slow_cam->set_pipeline_description(warp_pipe);
      } else {
        slow_cam->set_pipeline_description(normal_pipe);
      }
    }

  });

  update_view_mode();

  // ---------------------------------------------------------------------------
  // ----------------------------- setup gui -----------------------------------
  // ---------------------------------------------------------------------------

  auto gui = std::make_shared<gua::GuiResource>();
  auto gui_quad = std::make_shared<gua::node::TexturedScreenSpaceQuadNode>("gui_quad");

  auto stats = std::make_shared<gua::GuiResource>();
  auto stats_quad = std::make_shared<gua::node::TexturedScreenSpaceQuadNode>("stats_quad");

  auto mouse = std::make_shared<gua::GuiResource>();
  auto mouse_quad = std::make_shared<gua::node::TexturedScreenSpaceQuadNode>("mouse_quad");
  mouse->init("mouse", "asset://gua/data/gui/mouse.html", gua::math::vec2ui(50, 50));
  mouse_quad->data.texture() = "mouse";
  mouse_quad->data.size() = gua::math::vec2ui(50, 50);
  mouse_quad->data.anchor() = gua::math::vec2(-1.f, -1.f);

  gua::Interface::instance()->on_cursor_change.connect([&](gua::Cursor pointer){
    mouse->call_javascript("set_active", pointer == gua::Cursor::HAND);
    return true;
  });

  // right side gui ----------------------------------------------------------
  gui->init("gui", "asset://gua/data/gui/gui.html", gua::math::vec2ui(330, 800));

  gui->on_loaded.connect([&]() {
    gui->add_javascript_getter("get_depth_layers", [&](){ return std::to_string(warp_pass->max_layers());});
    gui->add_javascript_getter("get_split_threshold", [&](){ return gua::string_utils::to_string(grid_pass->split_threshold());});
    gui->add_javascript_getter("get_max_split_depth", [&](){ return gua::string_utils::to_string(grid_pass->max_split_depth());});
    gui->add_javascript_getter("get_cell_size", [&](){ return gua::string_utils::to_string(std::log2(grid_pass->cell_size()));});
    gui->add_javascript_getter("get_depth_test", [&](){ return std::to_string(depth_test);});
    gui->add_javascript_getter("get_warp_perspective", [&](){ return std::to_string(warp_perspective);});
    gui->add_javascript_getter("get_warping", [&](){ return std::to_string(warping);});
    gui->add_javascript_getter("get_stereo", [&](){ return std::to_string(stereo);});
    gui->add_javascript_getter("get_backface_culling", [&](){ return std::to_string(backface_culling);});
    gui->add_javascript_getter("get_background", [&](){ return std::to_string(res_pass->background_mode() == gua::ResolvePassDescription::BackgroundMode::SKYMAP_TEXTURE);});
    gui->add_javascript_getter("get_show_warp_grid", [&](){ return std::to_string(render_grid_pass->show_warp_grid());});
    gui->add_javascript_getter("get_adaptive_entry_level", [&](){ return std::to_string(warp_pass->adaptive_entry_level());});
    gui->add_javascript_getter("get_debug_cell_colors", [&](){ return std::to_string(warp_pass->debug_cell_colors());});
    gui->add_javascript_getter("get_debug_sample_count", [&](){ return std::to_string(warp_pass->debug_sample_count());});
    gui->add_javascript_getter("get_debug_bounding_volumes", [&](){ return std::to_string(warp_pass->debug_bounding_volumes());});
    gui->add_javascript_getter("get_debug_sample_ray", [&](){ return std::to_string(warp_pass->debug_sample_ray());});
    gui->add_javascript_getter("get_pixel_size", [&](){ return gua::string_utils::to_string(warp_pass->pixel_size()+0.5);});
    gui->add_javascript_getter("get_adaptive_abuffer", [&](){ return std::to_string(trimesh_pass->adaptive_abuffer());});

    gui->add_javascript_callback("set_depth_layers");
    gui->add_javascript_callback("set_split_threshold");
    gui->add_javascript_callback("set_max_split_depth");
    gui->add_javascript_callback("set_cell_size");
    gui->add_javascript_callback("set_depth_test");
    gui->add_javascript_callback("set_warp_perspective");
    gui->add_javascript_callback("set_warping");
    gui->add_javascript_callback("set_stereo");
    gui->add_javascript_callback("set_backface_culling");
    gui->add_javascript_callback("set_background");
    gui->add_javascript_callback("set_gbuffer_type_none");
    gui->add_javascript_callback("set_gbuffer_type_points");
    gui->add_javascript_callback("set_gbuffer_type_scaled_points");
    gui->add_javascript_callback("set_gbuffer_type_quads_screen_aligned");
    gui->add_javascript_callback("set_gbuffer_type_quads_normal_aligned");
    gui->add_javascript_callback("set_gbuffer_type_quads_depth_aligned");
    gui->add_javascript_callback("set_gbuffer_type_grid_depth_theshold");
    gui->add_javascript_callback("set_gbuffer_type_grid_surface_estimation");
    gui->add_javascript_callback("set_gbuffer_type_grid_advanced_surface_estimation");
    gui->add_javascript_callback("set_gbuffer_type_grid_non_uniform_surface_estimation");
    gui->add_javascript_callback("set_transparency_type_none");
    gui->add_javascript_callback("set_transparency_type_gbuffer");
    gui->add_javascript_callback("set_transparency_type_raycasting");
    gui->add_javascript_callback("set_hole_filling_type_none");
    gui->add_javascript_callback("set_hole_filling_type_inpaint");
    gui->add_javascript_callback("set_adaptive_abuffer");
    gui->add_javascript_callback("set_scene_one_oilrig");
    gui->add_javascript_callback("set_scene_many_oilrigs");
    gui->add_javascript_callback("set_scene_sponza");
    gui->add_javascript_callback("set_scene_teapot");
    gui->add_javascript_callback("set_scene_pitoti");
    gui->add_javascript_callback("set_scene_car");
    gui->add_javascript_callback("set_scene_bottle");
    gui->add_javascript_callback("set_scene_mountains");
    gui->add_javascript_callback("set_scene_sphere");
    gui->add_javascript_callback("set_scene_textured_quads");
    gui->add_javascript_callback("set_scene_dragon");
    gui->add_javascript_callback("set_scene_buddha");
    gui->add_javascript_callback("set_scene_hairball");
    gui->add_javascript_callback("set_scene_engine");
    gui->add_javascript_callback("set_manipulation_navigator");
    gui->add_javascript_callback("set_manipulation_camera");
    gui->add_javascript_callback("set_manipulation_object");
    gui->add_javascript_callback("set_show_warp_grid");
    gui->add_javascript_callback("set_adaptive_entry_level");
    gui->add_javascript_callback("set_debug_cell_colors");
    gui->add_javascript_callback("set_debug_sample_count");
    gui->add_javascript_callback("set_debug_bounding_volumes");
    gui->add_javascript_callback("set_debug_sample_ray");
    gui->add_javascript_callback("set_pixel_size");
    gui->add_javascript_callback("set_bg_tex");
    gui->add_javascript_callback("set_view_mono_warped");
    gui->add_javascript_callback("set_view_stereo_warped");
    gui->add_javascript_callback("set_view_mono");
    gui->add_javascript_callback("set_view_stereo");

    gui->add_javascript_callback("reset_view");
    gui->add_javascript_callback("reset_object");

    gui->call_javascript("init");
  });

  gui->on_javascript_callback.connect([&](std::string const& callback, std::vector<std::string> const& params) {
    if (callback == "set_depth_layers") {
      std::stringstream str(params[0]);
      int depth_layers;
      str >> depth_layers;
      warp_pass->max_layers(depth_layers);

    } else if (callback == "set_split_threshold") {
      std::stringstream str(params[0]);
      float split_threshold;
      str >> split_threshold;
      grid_pass->split_threshold(split_threshold);
    } else if (callback == "set_max_split_depth") {
      std::stringstream str(params[0]);
      float max_split_depth;
      str >> max_split_depth;
      grid_pass->max_split_depth(max_split_depth);
    } else if (callback == "set_pixel_size") {
      std::stringstream str(params[0]);
      float pixel_size;
      str >> pixel_size;
      warp_pass->pixel_size(pixel_size-0.5);
    } else if (callback == "set_bg_tex") {
      res_pass->background_texture(params[0]);
    } else if (callback == "set_cell_size") {
      std::stringstream str(params[0]);
      int cell_size;
      str >> cell_size;
      grid_pass->cell_size(std::pow(2, cell_size));
    } else if (callback == "set_depth_test") {
      std::stringstream str(params[0]);
      str >> depth_test;
      warp_pass->depth_test(depth_test);
    } else if (callback == "set_warp_perspective") {
      std::stringstream str(params[0]);
      str >> warp_perspective;
    } else if (callback == "set_warping") {
      std::stringstream str(params[0]);
      str >> warping;
      update_view_mode();
    } else if (callback == "set_stereo") {
      std::stringstream str(params[0]);
      str >> stereo;
      update_view_mode();
    } else if (callback == "set_backface_culling") {
      std::stringstream str(params[0]);
      str >> backface_culling;
      show_backfaces(transform);
    } else if (callback == "set_background") {
      std::stringstream str(params[0]);
      bool checked;
      str >> checked;
      res_pass->background_mode(checked ? gua::ResolvePassDescription::BackgroundMode::SKYMAP_TEXTURE : gua::ResolvePassDescription::BackgroundMode::COLOR);
    } else if (callback == "set_show_warp_grid") {
      std::stringstream str(params[0]);
      bool checked;
      str >> checked;
      render_grid_pass->show_warp_grid(checked);
    } else if (callback == "set_adaptive_entry_level") {
      std::stringstream str(params[0]);
      bool checked;
      str >> checked;
      warp_pass->adaptive_entry_level(checked);
    } else if (callback == "set_debug_cell_colors") {
      std::stringstream str(params[0]);
      bool checked;
      str >> checked;
      warp_pass->debug_cell_colors(checked);
    } else if (callback == "set_debug_sample_count") {
      std::stringstream str(params[0]);
      bool checked;
      str >> checked;
      warp_pass->debug_sample_count(checked);
    } else if (callback == "set_debug_bounding_volumes") {
      std::stringstream str(params[0]);
      bool checked;
      str >> checked;
      warp_pass->debug_bounding_volumes(checked);
    } else if (callback == "set_debug_sample_ray") {
      std::stringstream str(params[0]);
      bool checked;
      str >> checked;
      warp_pass->debug_sample_ray(checked);
    } else if (callback == "set_adaptive_abuffer") {
      std::stringstream str(params[0]);
      bool checked;
      str >> checked;
      trimesh_pass->adaptive_abuffer(checked);
    } else if (callback == "reset_view") {
      warp_nav.reset();
    } else if (callback == "reset_object") {
      object_trackball.reset();
    } else if (callback == "set_gbuffer_type_points"
             | callback == "set_gbuffer_type_scaled_points"
             | callback == "set_gbuffer_type_quads_screen_aligned"
             | callback == "set_gbuffer_type_quads_normal_aligned"
             | callback == "set_gbuffer_type_quads_depth_aligned"
             | callback == "set_gbuffer_type_grid_depth_theshold"
             | callback == "set_gbuffer_type_grid_surface_estimation"
             | callback == "set_gbuffer_type_grid_advanced_surface_estimation"
             | callback == "set_gbuffer_type_grid_non_uniform_surface_estimation"
             | callback == "set_gbuffer_type_none") {
      std::stringstream str(params[0]);
      bool checked;
      str >> checked;
      if (checked) {

        gua::WarpPassDescription::GBufferWarpMode mode(gua::WarpPassDescription::GBUFFER_NONE);

        if (callback == "set_gbuffer_type_points")
          mode = gua::WarpPassDescription::GBUFFER_POINTS;
        if (callback == "set_gbuffer_type_scaled_points")
          mode = gua::WarpPassDescription::GBUFFER_SCALED_POINTS;
        if (callback == "set_gbuffer_type_quads_screen_aligned")
          mode = gua::WarpPassDescription::GBUFFER_QUADS_SCREEN_ALIGNED;
        if (callback == "set_gbuffer_type_quads_normal_aligned")
          mode = gua::WarpPassDescription::GBUFFER_QUADS_NORMAL_ALIGNED;
        if (callback == "set_gbuffer_type_quads_depth_aligned")
          mode = gua::WarpPassDescription::GBUFFER_QUADS_DEPTH_ALIGNED;
        if (callback == "set_gbuffer_type_grid_depth_theshold")
          mode = gua::WarpPassDescription::GBUFFER_GRID_DEPTH_THRESHOLD;
        if (callback == "set_gbuffer_type_grid_surface_estimation")
          mode = gua::WarpPassDescription::GBUFFER_GRID_SURFACE_ESTIMATION;
        if (callback == "set_gbuffer_type_grid_advanced_surface_estimation")
          mode = gua::WarpPassDescription::GBUFFER_GRID_ADVANCED_SURFACE_ESTIMATION;
        if (callback == "set_gbuffer_type_grid_non_uniform_surface_estimation")
          mode = gua::WarpPassDescription::GBUFFER_GRID_NON_UNIFORM_SURFACE_ESTIMATION;
        if (callback == "set_gbuffer_type_none")
          mode = gua::WarpPassDescription::GBUFFER_NONE;

        warp_pass->gbuffer_warp_mode(mode);
        grid_pass->mode(mode);
        render_grid_pass->mode(mode);
      }
    } else if (callback == "set_transparency_type_gbuffer"
             | callback == "set_transparency_type_raycasting"
             | callback == "set_transparency_type_none") {

      current_transparency_mode = callback;
      update_view_mode();

    } else if (callback == "set_hole_filling_type_none"
             | callback == "set_hole_filling_type_inpaint") {
      std::stringstream str(params[0]);
      bool checked;
      str >> checked;
      if (checked) {

        gua::WarpPassDescription::HoleFillingMode mode(gua::WarpPassDescription::HOLE_FILLING_NONE);

        if (callback == "set_hole_filling_type_inpaint")
          mode = gua::WarpPassDescription::HOLE_FILLING_INPAINT;

        warp_pass->hole_filling_mode(mode);
      }

    } else if (callback == "set_manipulation_object"
            || callback == "set_manipulation_camera"
            || callback == "set_manipulation_navigator") {
      std::stringstream str(params[0]);
      bool checked;
      str >> checked;
      if (checked) {
        manipulation_object = false;
        manipulation_camera = false;
        manipulation_navigator = false;

        if (callback == "set_manipulation_camera") manipulation_camera = true;
        if (callback == "set_manipulation_object") manipulation_object = true;
        if (callback == "set_manipulation_navigator") manipulation_navigator = true;
      }
    } else if (callback == "set_scene_one_oilrig"        ||
               callback == "set_scene_many_oilrigs"      ||
               callback == "set_scene_sponza"            ||
               callback == "set_scene_teapot"            ||
               callback == "set_scene_pitoti"            ||
               callback == "set_scene_car"               ||
               callback == "set_scene_bottle"            ||
               callback == "set_scene_mountains"         ||
               callback == "set_scene_sphere"            ||
               callback == "set_scene_dragon"            ||
               callback == "set_scene_hairball"          ||
               callback == "set_scene_engine"           ||
               callback == "set_scene_buddha"            ||
               callback == "set_scene_textured_quads") {
      set_scene(callback);
    }
  });

  gui_quad->data.texture() = "gui";
  gui_quad->data.size() = gua::math::vec2ui(330, 800);
  gui_quad->data.anchor() = gua::math::vec2(1.f, 1.f);

  graph.add_node("/", gui_quad);

  // bottom gui --------------------------------------------------------------
  stats->init("stats", "asset://gua/data/gui/statistics.html", gua::math::vec2ui(resolution.x, 60));

  stats->on_loaded.connect([&]() {
    stats->call_javascript("init");
  });

  stats_quad->data.texture() = "stats";
  stats_quad->data.size() = gua::math::vec2ui(resolution.x, 60);
  stats_quad->data.anchor() = gua::math::vec2(0.f, -1.f);

  graph.add_node("/", stats_quad);
  graph.add_node("/", mouse_quad);


  // ---------------------------------------------------------------------------
  // ----------------------------- setup windows -------------------------------
  // ---------------------------------------------------------------------------

  window->config.set_fullscreen_mode(fullscreen);
  window->cursor_mode(gua::GlfwWindow::CursorMode::HIDDEN);

  #if (!POWER_WALL && !OCULUS && !STEREO_MONITOR)
    window->on_resize.connect([&](gua::math::vec2ui const& new_size) {
      resolution = new_size;
      window->config.set_resolution(new_size);
      slow_cam->config.set_resolution(new_size);
      slow_screen->data.set_size(gua::math::vec2(1.08*2 * new_size.x / new_size.y, 1.08*2));
      fast_screen->data.set_size(gua::math::vec2(1.08*2 * new_size.x / new_size.y, 1.08*2));
    });
  #endif

  window->on_button_press.connect([&](int key, int action, int mods) {
    gui->inject_mouse_button(gua::Button(key), action, mods);

    nav.set_mouse_button(key, action);
    warp_nav.set_mouse_button(key, action);

  });

  window->on_key_press.connect([&](int key, int scancode, int action, int mods) {
    if (manipulation_navigator) {
      nav.set_key_press(static_cast<gua::Key>(key), action);
    } else if (manipulation_camera) {
      warp_nav.set_key_press(static_cast<gua::Key>(key), action);
    }
    gui->inject_keyboard_event(gua::Key(key), scancode, action, mods);
    if (key == 72 && action == 1) {
      // hide gui
      if (gui_quad->get_tags().has_tag("invisible")) {
        gui_quad->get_tags().remove_tag("invisible");
        stats_quad->get_tags().remove_tag("invisible");
      } else {
        gui_quad->get_tags().add_tag("invisible");
        stats_quad->get_tags().add_tag("invisible");
      }
    }
  });

  window->on_move_cursor.connect([&](gua::math::vec2 const& pos) {
    mouse_quad->data.offset() = pos + gua::math::vec2i(-2, -45);
    gua::math::vec2 hit_pos;
    if (gui_quad->pixel_to_texcoords(pos, resolution, hit_pos) && !gui_quad->get_tags().has_tag("invisible")) {
      gui->inject_mouse_position_relative(hit_pos);
    } else {
      if (manipulation_object) {
        object_trackball.motion(pos.x, pos.y);
      } else if (manipulation_navigator) {
        nav.set_mouse_position(gua::math::vec2i(pos));
      } else {
        warp_nav.set_mouse_position(gua::math::vec2i(pos));
      }
    }
  });

  window->on_button_press.connect(std::bind(mouse_button, std::ref(object_trackball), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  #if POWER_WALL
    window->config.set_size(gua::math::vec2ui(1920*2, 1200));
    window->config.set_right_position(gua::math::vec2ui(1920, 0));
    window->config.set_right_resolution(gua::math::vec2ui(1780, 1185));
    window->config.set_left_position(gua::math::vec2ui(140, 0));
    window->config.set_left_resolution(gua::math::vec2ui(1780, 1185));
  #elif !OCULUS
    window->config.set_size(resolution);
    window->config.set_resolution(resolution);
  #endif
  window->config.set_enable_vsync(STEREO_MONITOR);
  gua::WindowDatabase::instance()->add("window", window);
  window->open();

  // tracking ------------------------------------------------------------------
  warp_pass->get_warp_state([&](){
    #if POWER_WALL
      fast_cam->set_transform(current_tracking_matrix);
    #elif OCULUS
      fast_cam->set_transform(get_oculus_transform(oculus_sensor));
    #endif

    gua::WarpPassDescription::WarpState state;

    gua::Frustum frustum = fast_cam->get_rendering_frustum(graph, gua::CameraMode::CENTER);
    state.projection_view_center = frustum.get_projection() * frustum.get_view();

    frustum = fast_cam->get_rendering_frustum(graph, gua::CameraMode::LEFT);
    state.projection_view_left = frustum.get_projection() * frustum.get_view();

    frustum = fast_cam->get_rendering_frustum(graph, gua::CameraMode::RIGHT);
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

  // application loop
  gua::events::MainLoop loop;
  gua::events::Ticker ticker(loop, 1.0/120.0);

  int ctr=0;

  ticker.on_tick.connect([&]() {

    gua::Interface::instance()->update();

    #if POWER_WALL
      slow_cam->set_transform(current_tracking_matrix);
    #elif OCULUS
      slow_cam->set_transform(get_oculus_transform(oculus_sensor));
    #endif

    nav.update();
    warp_nav.update();

    #if POWER_WALL
      if (warp_perspective && !warping) {
        navigation->set_transform(slow_screen->get_transform() * gua::math::mat4(warp_nav.get_transform()) * scm::math::inverse(slow_screen->get_transform()));
      } else {
        navigation->set_transform(slow_screen->get_transform() * gua::math::mat4(nav.get_transform()) * scm::math::inverse(slow_screen->get_transform()));
      }
      warp_navigation->set_transform(slow_screen->get_transform() * gua::math::mat4(warp_nav.get_transform()) * scm::math::inverse(slow_screen->get_transform()));
    #else
      if (warp_perspective && !warping) {
        navigation->set_transform(gua::math::mat4(nav.get_transform() * warp_nav.get_transform()));
      } else {
        navigation->set_transform(gua::math::mat4(nav.get_transform()));
      }
      warp_navigation->set_transform(gua::math::mat4(warp_nav.get_transform()));
    #endif

    gua::math::mat4 modelmatrix = scm::math::make_translation(gua::math::float_t(object_trackball.shiftx()),
                                                              gua::math::float_t(object_trackball.shifty()),
                                                              gua::math::float_t(object_trackball.distance())) * gua::math::mat4(object_trackball.rotation());
    transform->set_transform(modelmatrix);



    if (ctr++ % 100 == 0) {
      double trimesh_time(0);
      double gbuffer_warp_time(0);
      double abuffer_warp_time(0);
      double gbuffer_grid_time(0);
      double abuffer_grid_time(0);
      int gbuffer_primitives(0);
      int abuffer_primitives(0);

      for (auto const& result: window->get_context()->time_query_results) {
        if (result.first.find("GPU") != std::string::npos) {
          if (result.first.find("Trimesh") != std::string::npos) trimesh_time += result.second;
          if (result.first.find("Resolve") != std::string::npos) trimesh_time += result.second;
          if (result.first.find("WarpPass GBuffer") != std::string::npos) gbuffer_warp_time += result.second;
          if (result.first.find("WarpPass ABuffer") != std::string::npos) abuffer_warp_time += result.second;
          if (result.first.find("WarpGridGenerator") != std::string::npos) gbuffer_grid_time += result.second;
          if (result.first.find("MinMaxMap") != std::string::npos) abuffer_grid_time += result.second;
        }
      }

      for (auto const& result: window->get_context()->primitive_query_results) {
        if (result.first.find("WarpPass GBuffer") != std::string::npos) gbuffer_primitives += result.second.first;
        if (result.first.find("WarpPass ABuffer") != std::string::npos) abuffer_primitives += result.second.first;
      }

      // std::cout << resolution.x*resolution.y << " " << 1000.f / window->get_rendering_fps() << " " << gbuffer_warp_time+abuffer_warp_time << " " << gbuffer_grid_time+abuffer_grid_time << std::endl;

      stats->call_javascript("set_stats", 1000.f / window->get_rendering_fps(),
                           window->get_rendering_fps(), trimesh_time, gbuffer_grid_time,
                           abuffer_grid_time, gbuffer_warp_time, abuffer_warp_time, 
                           gbuffer_primitives, abuffer_primitives);
    }

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
