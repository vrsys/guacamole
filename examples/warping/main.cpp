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


#define GUI_SUPPORT     1

#define POWER_WALL      0
#define OCULUS1         0
#define OCULUS2         0
#define STEREO_MONITOR  0
#define USE_SIDE_BY_SIDE 0

#define SHADOWS         1
#define LOAD_PITOTI     1
#define LOAD_SPONZA     1
#define LOAD_HAIRBALL   1

#include <functional>

#include <scm/input/tracking/art_dtrack.h>
#include <scm/input/tracking/target.h>

#include <gua/guacamole.hpp>
#include <gua/renderer/TexturedScreenSpaceQuadPass.hpp>
#include <gua/renderer/TriMeshLoader.hpp>
#include <gua/renderer/DebugViewPass.hpp>
#include <gua/utils/Trackball.hpp>

#if GUI_SUPPORT
  #include <gua/gui.hpp>
#endif

#if OCULUS1
  #include <OVR.h>
  #include <gua/OculusWindow.hpp>
#elif OCULUS2
  #include <gua/OculusSDK2Window.hpp>
#endif

#if LOAD_PITOTI
  #include <gua/renderer/PLODPass.hpp>
  #include <gua/renderer/PLODLoader.hpp>
  #include <gua/node/PLODNode.hpp>
#endif

#include "Navigator.hpp"

bool manipulation_navigator = true;
bool manipulation_camera    = false;
bool manipulation_object    = false;
bool stereotype_spatial     = true;
bool stereotype_temporal    = false;
bool warping                = true;
bool stereo                 = false;
bool warp_perspective       = false;
bool vsync                  = false;
float eye_offset            = 0.f;

const float aspect = 1.0f/1.6f;
const float screen_width = 4.f;
const float screen_dist = 2.5f;
const int   window_size = 1600;


#if OCULUS1 || OCULUS2
  float eye_dist = 0.0635f;
#else
  float eye_dist = 0.1f;
#endif

gua::math::mat4 current_tracking_matrix(gua::math::mat4::identity());
std::string     current_transparency_mode("set_transparency_type_raycasting");


#if OCULUS1
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
#elif OCULUS2
  void init_oculus() {
    gua::OculusSDK2Window::initialize_oculus_environment();
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

  // initialize guacamole
  gua::init(argc, argv);

  gua::Logger::enable_debug = false;

  #if OCULUS1
    auto window = std::make_shared<gua::OculusWindow>();
  #elif OCULUS2
    init_oculus();
    auto window = std::make_shared<gua::OculusSDK2Window>(":0.0");
  #else
    auto window = std::make_shared<gua::GlfwWindow>();
  #endif

  #if POWER_WALL
    bool fullscreen = true;
    auto resolution = gua::math::vec2ui(1780, 1185);
  #elif OCULUS1
    bool fullscreen = true;
    auto resolution = gua::math::vec2ui(1280/2, 800);

    OVR::SensorFusion* oculus_sensor = init_oculus();
    if (!oculus_sensor) {
      gua::Logger::LOG_WARNING << "Could not connect to Oculus Rift! " << std::endl;
      return -1;
    }
  #elif OCULUS2
    bool fullscreen = true;
    auto resolution = window->get_eye_resolution();
  #elif STEREO_MONITOR
    bool fullscreen = true;
    auto resolution = gua::math::vec2ui(2560, 1440);
  #else
    bool fullscreen = (argc == 2);

    auto resolution = gua::math::vec2ui(window_size, window_size*aspect);
    if (fullscreen) {
      resolution = gua::math::vec2ui(1920, 1080);
    }
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
  sun_light->rotate(-65, 1, 0, 0);
  sun_light->rotate(-100, 0, 1, 0);

  auto light2 = graph.add_node<gua::node::LightNode>("/", "light2");
  light2->data.set_type(gua::node::LightNode::Type::SUN);
  light2->data.set_color(gua::utils::Color3f(0.8f, 1.0f, 1.5f));
  light2->data.set_brightness(1.5f);
  light2->data.set_enable_specular_shading(false);
  light2->rotate(-45, 1, 0, 0);
  light2->rotate(-120, 0, 1, 0);

  auto light3 = graph.add_node<gua::node::LightNode>("/", "light3");
  light3->data.set_type(gua::node::LightNode::Type::SUN);
  light3->data.set_color(gua::utils::Color3f(0.9f, 1.3f, 1.1f));
  light3->data.set_brightness(1.0f);
  light3->data.set_enable_specular_shading(false);
  light3->rotate(45, 1, 0, 0);
  light3->rotate(80, 0, 1, 0);


  // many oilrigs scene --------------------------------------------------------
  auto add_oilrig = [&](int x, int y, int c, std::string const& parent) {
    auto t = graph.add_node<gua::node::TransformNode>(parent, "t");
    t->translate((x - c*0.5 + 0.5)/1.5, (y - c*0.5 + 0.8)/2, 0);
    auto oilrig(loader.create_geometry_from_file("oilrig", opt_prefix + "3d_models/OIL_RIG_GUACAMOLE/oilrig.obj",
      gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION |
      gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::OPTIMIZE_MATERIALS |
      gua::TriMeshLoader::NORMALIZE_SCALE));
    t->add_child(oilrig);
  };

  // one oilrig ----------------------------------------------------------------
  auto scene_root = graph.add_node<gua::node::TransformNode>("/transform", "oilrig");
  scene_root->scale(20);
  scene_root->rotate(-90, 1, 0, 0);
  add_oilrig(0, 0, 1, "/transform/oilrig");


  // teapot --------------------------------------------------------------------
  scene_root = graph.add_node<gua::node::TransformNode>("/transform", "teapot");
  auto teapot(loader.create_geometry_from_file("teapot", "data/objects/teapot.obj",
    gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION |
    gua::TriMeshLoader::NORMALIZE_SCALE));
  scene_root->add_child(teapot);

  // pitoti --------------------------------------------------------------------
  scene_root = graph.add_node<gua::node::TransformNode>("/transform", "pitoti");
  #if LOAD_PITOTI
    gua::PLODLoader plodloader;
    plodloader.set_out_of_core_budget_in_mb(5000);
    plodloader.set_render_budget_in_mb(1000);
    plodloader.set_upload_budget_in_mb(20);
    auto pitoti(plodloader.load_geometry("/mnt/pitoti/3d_pitoti/groundtruth_data/rocks/seradina12c/TLS_Seradina_Rock-12C_knn.kdn",
        gua::PLODLoader::NORMALIZE_POSITION | gua::PLODLoader::NORMALIZE_SCALE));
    scene_root->add_child(pitoti);
    pitoti->set_radius_scale(1.0f);
    pitoti->set_error_threshold(2.5f);
    scene_root->rotate(-90, 1, 0, 0);
    scene_root->scale(20);
  #endif

  // bottle --------------------------------------------------------------------
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
  auto load_mat = [](std::string const& file){
    auto desc(std::make_shared<gua::MaterialShaderDescription>());
    desc->load_from_file(file);
    auto shader(std::make_shared<gua::MaterialShader>(file, desc));
    gua::MaterialShaderDatabase::instance()->add(shader);
    return shader->make_new_material();
  };

  scene_root = graph.add_node<gua::node::TransformNode>("/transform", "bottle");
  auto mat_bottle(load_mat("data/materials/Bottle.gmd"));
  mat_bottle->set_uniform("ColorMap",     std::string("data/objects/bottle/albedo.png"))
             .set_uniform("RoughnessMap", std::string("data/objects/bottle/roughness.jpg"))
             .set_show_back_faces(true);

  for (int i(-1); i<=1; ++i) {
    auto bottle(loader.create_geometry_from_file("bottle", "data/objects/bottle/bottle.obj", mat_bottle,
                                                 gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));
    bottle->translate(i*0.75, 0, 0);
    scene_root->add_child(bottle);
  }
  scene_root->add_child(plane);

  // glasses --------------------------------------------------------------------
  scene_root = graph.add_node<gua::node::TransformNode>("/transform", "glasses");
  auto mat_glasses(load_mat("data/materials/Glasses.gmd"));
  mat_glasses->set_uniform("ReflectionMap", std::string(opt_prefix + "/guacamole/resources/skymaps/DH206SN.png"))
              .set_show_back_faces(true);
  auto glasses(loader.create_geometry_from_file("glasses", "data/objects/glasses.dae", mat_glasses,
    gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION |
    gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::OPTIMIZE_MATERIALS |
    gua::TriMeshLoader::NORMALIZE_SCALE));
  std::dynamic_pointer_cast<gua::node::TriMeshNode>(glasses->get_children()[0])->set_material(mat_glasses);
  // std::dynamic_pointer_cast<gua::node::TriMeshNode>(glasses->get_children()[1])->set_material(mat_glasses);
  scene_root->add_child(glasses);
  scene_root->scale(10);
  scene_root->rotate(180, 0, 1, 0);
  scene_root->scale(2);


  // sponza --------------------------------------------------------------------
  scene_root = graph.add_node<gua::node::TransformNode>("/transform", "sponza");
  scene_root->rotate(40, 0, 1, 0);
  #if LOAD_SPONZA 
  auto sponza(loader.create_geometry_from_file("sponza","data/objects/sponza/sponza.obj",
  // auto sponza(loader.create_geometry_from_file("sponza", opt_prefix + "3d_models/SponzaPBR/sponza.obj",
    gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION |
    gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::OPTIMIZE_MATERIALS |
    gua::TriMeshLoader::NORMALIZE_SCALE));
  sponza->scale(20);
  sponza->translate(0, 2, 0);
  scene_root->add_child(sponza);

  auto sponza_light_05 = std::make_shared<gua::node::TransformNode>("sponza_light_05");
  auto light = std::make_shared<gua::node::LightNode>("light");
  light->data.set_type(gua::node::LightNode::Type::POINT);
  light->data.set_brightness(10.f);
  light->data.set_falloff(2.f);
  light->data.set_color(gua::utils::Color3f(1.5f, 0.5f, 0.3f));
  light->translate(2.1, 0.2, 0.8);
  light->scale(3.0);
  scene_root->add_child(sponza_light_05);
  sponza_light_05->add_child(light);

  auto sponza_light_06 = std::make_shared<gua::node::TransformNode>("sponza_light_06");
  light = std::make_shared<gua::node::LightNode>("light");
  light->data.set_type(gua::node::LightNode::Type::POINT);
  light->data.set_brightness(10.f);
  light->data.set_falloff(2.f);
  light->data.set_color(gua::utils::Color3f(1.5f, 0.5f, 0.3f));
  light->translate(-2.1, 0.2, 0.8);
  light->scale(3.0);
  scene_root->add_child(sponza_light_06);
  sponza_light_06->add_child(light);

  auto sponza_light_07 = std::make_shared<gua::node::TransformNode>("sponza_light_07");
  light = std::make_shared<gua::node::LightNode>("light");
  light->data.set_type(gua::node::LightNode::Type::POINT);
  light->data.set_brightness(10.f);
  light->data.set_falloff(2.f);
  light->data.set_color(gua::utils::Color3f(1.5f, 0.5f, 0.3f));
  light->translate(-2.1, 0.2, -0.8);
  light->scale(3.0);
  scene_root->add_child(sponza_light_07);
  sponza_light_07->add_child(light);

  auto sponza_light_08 = std::make_shared<gua::node::TransformNode>("sponza_light_08");
  light = std::make_shared<gua::node::LightNode>("light");
  light->data.set_type(gua::node::LightNode::Type::POINT);
  light->data.set_brightness(10.f);
  light->data.set_falloff(2.f);
  light->data.set_color(gua::utils::Color3f(1.5f, 0.5f, 0.3f));
  light->translate(2.1, 0.2, -0.8);
  light->scale(3.0);
  scene_root->add_child(sponza_light_08);
  sponza_light_08->add_child(light);
  #endif

  // hairball --------------------------------------------------------------------
  scene_root = graph.add_node<gua::node::TransformNode>("/transform", "hairball");
  #if LOAD_HAIRBALL
    auto hairball(loader.create_geometry_from_file("hairball", "data/objects/hairball.dae",
      gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION |
      gua::TriMeshLoader::NORMALIZE_SCALE));
    hairball->scale(5.0);
    hairball->translate(0, 0, -7);
    scene_root->add_child(hairball);

    hairball = loader.create_geometry_from_file("hairball", "data/objects/hairball.dae",
      gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION |
      gua::TriMeshLoader::NORMALIZE_SCALE);
    for (auto c: hairball->get_children()) {
      auto node = std::dynamic_pointer_cast<gua::node::TriMeshNode>(c);
      node->get_material()->set_uniform("Color", gua::math::vec4(1.f, 1.f, 1.f, 0.5f));
    }
    hairball->scale(5.0);
    hairball->translate(0, 0, 7);
    scene_root->add_child(hairball);
  #endif

  auto pipe = gua::PipelineFactory::make_pipeline(
    gua::PipelineFactory::DEFAULT | 
    #if LOAD_PITOTI
      gua::PipelineFactory::DRAW_PLODS |
    #endif
    gua::PipelineFactory::ABUFFER |
    gua::PipelineFactory::WARPING
  );

  pipe->get_resolve_pass()->
    background_mode(gua::ResolvePassDescription::BackgroundMode::SKYMAP_TEXTURE).
    background_texture(opt_prefix + "guacamole/resources/skymaps/DH206SN.png").
    environment_lighting_texture(opt_prefix + "guacamole/resources/skymaps/DH206SN.png").
    background_color(gua::utils::Color3f(0,0,0)).
    environment_lighting(gua::utils::Color3f(0.4, 0.4, 0.5)).
    environment_lighting_mode(gua::ResolvePassDescription::EnvironmentLightingMode::AMBIENT_COLOR).
    ssao_enable(true).
    tone_mapping_method(gua::ResolvePassDescription::ToneMappingMethod::HEJL).
    tone_mapping_exposure(1.5f).
    horizon_fade(0.2f).
    ssao_intensity(1.5f).
    ssao_radius(2.f);

  auto res_pass(pipe->get_resolve_pass());
  auto warp_pass(pipe->get_pass_by_type<gua::WarpPassDescription>());
  auto grid_pass(pipe->get_pass_by_type<gua::GenerateWarpGridPassDescription>());

  auto set_scene = [&](std::string const& name) {
    graph["/transform/sponza"]->get_tags().add_tag("invisible");
    graph["/transform/oilrig"]->get_tags().add_tag("invisible");
    graph["/transform/teapot"]->get_tags().add_tag("invisible");
    graph["/transform/pitoti"]->get_tags().add_tag("invisible");
    graph["/transform/bottle"]->get_tags().add_tag("invisible");
    graph["/transform/hairball"]->get_tags().add_tag("invisible");
    graph["/transform/glasses"]->get_tags().add_tag("invisible");

    sun_light->data.set_brightness(3.f);
    sun_light->data.set_enable_shadows(true);
    res_pass->ssao_enable(true);
    res_pass->ssao_intensity(1.5f);
    res_pass->ssao_radius(2.0f);

    if (name == "set_scene_oilrig") {
      graph["/transform/oilrig"]->get_tags().remove_tag("invisible");
      res_pass->background_texture(opt_prefix + "guacamole/resources/skymaps/cycles_island.jpg");
      nav.set_transform(scm::math::mat4f(-0.228, -0.031, 0.973, 2.404,
                                         0.000, 1.000, 0.031, 1.568,
                                         -0.974, 0.007, -0.228, -1.231,
                                         0.000, 0.000, 0.000, 1.000));
    }
    if (name.find("set_scene_sponza") != std::string::npos) {
      graph["/transform/sponza"]->get_tags().remove_tag("invisible");
      sun_light->data.set_brightness(15.f);
      //res_pass->ssao_enable(false);
      res_pass->ssao_intensity(5.0f);
      res_pass->ssao_radius(5.0f);
      res_pass->background_texture(opt_prefix + "guacamole/resources/skymaps/cycles_island.jpg");

      if (name == "set_scene_sponza1")
        nav.set_transform(scm::math::mat4f(0.637, 0.067, -0.768, -4.160,
                                           0.000, 0.996, 0.086, 2.715,
                                           0.771, -0.055, 0.635, 2.682,
                                           0.000, 0.000, 0.000, 1.000));
      if (name == "set_scene_sponza2")
        nav.set_transform(scm::math::mat4f(-0.750, -0.126, -0.649, -3.217,
                                           0.000, 0.982, -0.191, 2.467,
                                           0.661, -0.143, -0.736, -0.661,
                                           0.000, 0.000, 0.000, 1.000));
      if (name == "set_scene_sponza3")
        nav.set_transform(scm::math::mat4f(-0.978, 0.051, 0.204, 4.450,
                                           0.000, 0.970, -0.241, -0.268,
                                           -0.210, -0.236, -0.949, -4.290,
                                           0.000, 0.000, 0.000, 1.000));
    }
    if (name == "set_scene_teapot")
      graph["/transform/teapot"]->get_tags().remove_tag("invisible");
    if (name == "set_scene_pitoti") {
      sun_light->data.set_enable_shadows(false);
      graph["/transform/pitoti"]->get_tags().remove_tag("invisible");
    }
    if (name == "set_scene_bottle")
      graph["/transform/bottle"]->get_tags().remove_tag("invisible");
    if (name == "set_scene_hairball") {
      graph["/transform/hairball"]->get_tags().remove_tag("invisible");
      res_pass->background_texture(opt_prefix + "guacamole/resources/skymaps/uffizi.jpg");
      res_pass->ssao_enable(false);
      nav.set_transform(scm::math::mat4f(1.000, 0.007, -0.031, -0.269,
                                         0.000, 0.974, 0.228, 0.758,
                                         0.031, -0.228, 0.973, -3.367,
                                         0.000, 0.000, 0.000, 1.000));
    }
    if (name == "set_scene_glasses")
      graph["/transform/glasses"]->get_tags().remove_tag("invisible");
  };

  set_scene("set_scene_oilrig");

  // ---------------------------------------------------------------------------
  // ------------------------ setup rendering pipelines ------------------------
  // ---------------------------------------------------------------------------
  auto navigation = graph.add_node<gua::node::TransformNode>("/", "navigation");
  auto warp_navigation = graph.add_node<gua::node::TransformNode>("/navigation", "warp");

  // normal camera -------------------------------------------------------------
  #if OCULUS1 || OCULUS2

    auto normal_cam = graph.add_node<gua::node::CameraNode>("/navigation", "normal_cam");
    auto normal_screen_left = graph.add_node<gua::node::ScreenNode>("/navigation/normal_cam", "normal_screen_left");
    auto normal_screen_right = graph.add_node<gua::node::ScreenNode>("/navigation/normal_cam", "normal_screen_right");
    normal_cam->config.set_resolution(resolution);
    normal_cam->config.set_left_screen_path("/navigation/normal_cam/normal_screen_left");
    normal_cam->config.set_right_screen_path("/navigation/normal_cam/normal_screen_right");

    #if OCULUS1
      normal_screen_left->data.set_size(gua::math::vec2(0.08, 0.1));
      normal_screen_right->data.set_size(gua::math::vec2(0.08, 0.1));
    #else
      normal_screen_left->data.set_size(gua::math::vec2(0.17074, 0.21));
      normal_screen_right->data.set_size(gua::math::vec2(0.17074, 0.21));
    #endif
  #else
    auto normal_screen = graph.add_node<gua::node::ScreenNode>("/navigation", "normal_screen");
    auto normal_cam = graph.add_node<gua::node::CameraNode>("/navigation", "normal_cam");
    #if POWER_WALL
      normal_screen->data.set_size(gua::math::vec2(3, 2));
      normal_screen->translate(0, 1.42, -1.6);
      normal_cam->translate(0, 1.42, -1.6);
    #else
      #if USE_SIDE_BY_SIDE
        normal_screen->data.set_size(gua::math::vec2(screen_width/2,screen_width*aspect));
      #else
        normal_screen->data.set_size(gua::math::vec2(screen_width,screen_width*aspect));
      #endif
      normal_screen->translate(0, 0, -screen_dist);
    #endif
    normal_cam->config.set_resolution(resolution);
    normal_cam->config.set_screen_path("/navigation/normal_screen");
  #endif

  normal_cam->config.set_scene_graph_name("main_scenegraph");
  normal_cam->config.mask().blacklist.add_tag("invisible");
  normal_cam->config.set_far_clip(350.f);
  normal_cam->config.set_near_clip(0.1f);
  normal_cam->config.set_eye_dist(eye_dist);

  // warping camera ------------------------------------------------------------
  #if OCULUS1 || OCULUS2
    auto warp_cam = graph.add_node<gua::node::CameraNode>("/navigation", "warp_cam");
    auto warp_screen_left = graph.add_node<gua::node::ScreenNode>("/navigation/warp_cam", "warp_screen_left");
    auto warp_screen_right = graph.add_node<gua::node::ScreenNode>("/navigation/warp_cam", "warp_screen_right");
    warp_cam->config.set_resolution(resolution);
    warp_cam->config.set_left_screen_path("/navigation/warp_cam/warp_screen_left");
    warp_cam->config.set_right_screen_path("/navigation/warp_cam/warp_screen_right");

    #if OCULUS1
      warp_screen_left->data.set_size(gua::math::vec2(0.08, 0.1));
      warp_screen_right->data.set_size(gua::math::vec2(0.08, 0.1));
    #else
      warp_screen_left->data.set_size(gua::math::vec2(0.17074, 0.21));
      warp_screen_right->data.set_size(gua::math::vec2(0.17074, 0.21));
    #endif
  #else
    auto warp_screen = graph.add_node<gua::node::ScreenNode>("/navigation/warp", "warp_screen");
    auto warp_cam = graph.add_node<gua::node::CameraNode>("/navigation/warp", "warp_cam");
    #if POWER_WALL
      warp_screen->data.set_size(gua::math::vec2(3, 2));
      warp_screen->translate(0, 1.42, -1.6);
      warp_cam->translate(0, 1.42, -1.6);
    #else
      #if USE_SIDE_BY_SIDE
        warp_screen->data.set_size(gua::math::vec2(screen_width/2,screen_width*aspect));
      #else
        warp_screen->data.set_size(gua::math::vec2(screen_width,screen_width*aspect));
      #endif
      warp_screen->translate(0, 0, -screen_dist);
    #endif
    warp_cam->config.set_resolution(resolution);
    warp_cam->config.set_screen_path("/navigation/warp/warp_screen");
  #endif

  warp_cam->config.set_scene_graph_name("main_scenegraph");
  warp_cam->config.set_far_clip(normal_cam->config.get_far_clip()*1.5);
  warp_cam->config.set_near_clip(normal_cam->config.get_near_clip());
  warp_cam->config.set_eye_dist(eye_dist);

  normal_cam->config.set_output_window_name("window");
  normal_cam->set_pipeline_description(pipe);

  auto update_view_mode([&](){

    // set transparency mode
    pipe->set_enable_abuffer(current_transparency_mode != "set_transparency_type_none");
    res_pass->write_abuffer_depth(false);

    // set stereo options
    if (stereo) {

      normal_cam->config.set_enable_stereo(true);

      #if OCULUS1
        warp_screen_left->set_transform(gua::math::mat4(scm::math::make_translation(-0.04f, 0.f, -0.05f)));
        warp_screen_right->set_transform(gua::math::mat4(scm::math::make_translation(0.04f, 0.f, -0.05f)));
      #elif OCULUS2
        warp_screen_left->set_transform(gua::math::mat4(scm::math::make_translation(-0.03175f, 0.f, -0.08f)));
        warp_screen_right->set_transform(gua::math::mat4(scm::math::make_translation(0.03175f, 0.f, -0.08f)));
      #else
        window->config.set_stereo_mode(POWER_WALL || USE_SIDE_BY_SIDE ? gua::StereoMode::SIDE_BY_SIDE : gua::StereoMode::ANAGLYPH_RED_CYAN);
      #endif


      if (warping) {

        if (stereotype_spatial) {
          normal_cam->config.set_stereo_type(gua::StereoType::SPATIAL_WARP);

          #if OCULUS1
            normal_screen_left->set_transform(gua::math::mat4(scm::math::make_translation(0.f, 0.f, -0.05f)));
            normal_screen_right->set_transform(gua::math::mat4(scm::math::make_translation(0.f, 0.f, -0.05f)));
          #elif OCULUS2
            normal_screen_left->set_transform(gua::math::mat4(scm::math::make_translation(0.f, 0.f, -0.08f)));
            normal_screen_right->set_transform(gua::math::mat4(scm::math::make_translation(0.f, 0.f, -0.08f)));
          #endif

        } else if (stereotype_temporal) {
          normal_cam->config.set_stereo_type(gua::StereoType::TEMPORAL_WARP);
        }

      } else {
        normal_cam->config.set_stereo_type(gua::StereoType::RENDER_TWICE);
      } 

    } else {

      normal_cam->config.set_enable_stereo(false);

      #if OCULUS1
        normal_screen_left->set_transform(gua::math::mat4(scm::math::make_translation(0.f, 0.f, -0.05f)));
        warp_screen_left->set_transform(gua::math::mat4(scm::math::make_translation(0.f, 0.f, -0.05f)));
      #elif OCULUS2
        normal_screen_left->set_transform(gua::math::mat4(scm::math::make_translation(0.f, 0.f, -0.08f)));
        warp_screen_left->set_transform(gua::math::mat4(scm::math::make_translation(0.f, 0.f, -0.08f)));
      #else
        window->config.set_stereo_mode(gua::StereoMode::MONO);
      #endif

      if (warping) {
        normal_cam->config.set_stereo_type(gua::StereoType::SPATIAL_WARP);
        normal_cam->config.set_eye_offset(0.f);
        warp_cam->config.set_eye_offset(eye_offset);
      } else {
        normal_cam->config.set_stereo_type(gua::StereoType::RENDER_TWICE);
        if (warp_perspective) {
          normal_cam->config.set_eye_offset(eye_offset);
        } else {
          normal_cam->config.set_eye_offset(0.f);
        }
      }
    }

  });

  update_view_mode();

  // ---------------------------------------------------------------------------
  // ----------------------------- setup gui -----------------------------------
  // ---------------------------------------------------------------------------

  #if GUI_SUPPORT

    auto mouse = std::make_shared<gua::GuiResource>();
    mouse->init("mouse", "asset://gua/data/gui/mouse.html", gua::math::vec2ui(50, 50));

    auto gui_quad = std::make_shared<gua::node::TexturedScreenSpaceQuadNode>("gui_quad");
    gui_quad->data.texture() = "gui";
    gui_quad->data.size() = gua::math::vec2ui(330, 830);
    graph.add_node("/", gui_quad);

    auto mouse_quad = std::make_shared<gua::node::TexturedScreenSpaceQuadNode>("mouse_quad");
    mouse_quad->data.texture() = "mouse";
    mouse_quad->data.size() = gua::math::vec2ui(50, 50);
    graph.add_node("/", mouse_quad);

    #if OCULUS1 || OCULUS2
      mouse_quad->data.fake_parallax() = -0.01f;
      gui_quad->data.fake_parallax() = -0.01f;
      mouse_quad->data.anchor() = gua::math::vec2(-1.f, -1.f);
      gui_quad->data.anchor() = gua::math::vec2(0.7f, 0.f);
    #else
      mouse_quad->data.anchor() = gua::math::vec2(-1.f, -1.f);
      gui_quad->data.anchor() = gua::math::vec2(1.f, 1.f);
    #endif

    gua::Interface::instance()->on_cursor_change.connect([&](gua::Cursor pointer){
      mouse->call_javascript("set_active", pointer == gua::Cursor::HAND);
      return true;
    });

    auto toggle_gui = [&](){
      if (gui_quad->get_tags().has_tag("invisible")) {
        gui_quad->get_tags().remove_tag("invisible");
        mouse_quad->get_tags().remove_tag("invisible");
      } else {
        gui_quad->get_tags().add_tag("invisible");
        mouse_quad->get_tags().add_tag("invisible");
      }
    };

    // right side gui ----------------------------------------------------------
    auto gui = std::make_shared<gua::GuiResource>();
    gui->init("gui", "asset://gua/data/gui/gui.html", gua::math::vec2ui(330, 830));

    gui->on_loaded.connect([&]() {
      gui->add_javascript_getter("get_max_raysteps", [&](){ return std::to_string(warp_pass->max_raysteps());});
      gui->add_javascript_getter("get_split_threshold", [&](){ return gua::string_utils::to_string(grid_pass->split_threshold());});
      gui->add_javascript_getter("get_cell_size", [&](){ return gua::string_utils::to_string(std::log2(grid_pass->cell_size()));});
      gui->add_javascript_getter("get_vsync", [&](){ return std::to_string(vsync);});
      gui->add_javascript_getter("get_warp_perspective", [&](){ return std::to_string(warp_perspective);});
      gui->add_javascript_getter("get_warping", [&](){ return std::to_string(warping);});
      gui->add_javascript_getter("get_stereo", [&](){ return std::to_string(stereo);});
      gui->add_javascript_getter("get_adaptive_entry_level", [&](){ return std::to_string(warp_pass->adaptive_entry_level());});
      gui->add_javascript_getter("get_debug_cell_colors", [&](){ return std::to_string(warp_pass->debug_cell_colors());});
      gui->add_javascript_getter("get_debug_sample_count", [&](){ return std::to_string(warp_pass->debug_sample_count());});
      gui->add_javascript_getter("get_debug_bounding_volumes", [&](){ return std::to_string(warp_pass->debug_bounding_volumes());});
      gui->add_javascript_getter("get_pixel_size", [&](){ return gua::string_utils::to_string(warp_pass->pixel_size()+0.5);});

      gui->add_javascript_callback("set_max_raysteps");
      gui->add_javascript_callback("set_split_threshold");
      gui->add_javascript_callback("set_cell_size");
      gui->add_javascript_callback("set_vsync");
      gui->add_javascript_callback("set_warp_perspective");
      gui->add_javascript_callback("set_warping");
      gui->add_javascript_callback("set_stereo");
      gui->add_javascript_callback("set_hole_filling_color");
      gui->add_javascript_callback("set_hole_filling_type_none");
      gui->add_javascript_callback("set_hole_filling_type_epipolar_search");
      gui->add_javascript_callback("set_hole_filling_type_epipolar_mirror");
      gui->add_javascript_callback("set_hole_filling_type_blur");
      gui->add_javascript_callback("set_scene_oilrig");
      gui->add_javascript_callback("set_scene_sponza1");
      gui->add_javascript_callback("set_scene_sponza2");
      gui->add_javascript_callback("set_scene_sponza3");
      gui->add_javascript_callback("set_scene_teapot");
      gui->add_javascript_callback("set_scene_pitoti");
      gui->add_javascript_callback("set_scene_bottle");
      gui->add_javascript_callback("set_scene_glasses");
      gui->add_javascript_callback("set_scene_hairball");
      gui->add_javascript_callback("set_manipulation_navigator");
      gui->add_javascript_callback("set_manipulation_camera");
      gui->add_javascript_callback("set_manipulation_object");
      gui->add_javascript_callback("set_stereotype_spatial");
      gui->add_javascript_callback("set_stereotype_temporal");
      gui->add_javascript_callback("set_adaptive_entry_level");
      gui->add_javascript_callback("set_debug_cell_colors");
      gui->add_javascript_callback("set_debug_sample_count");
      gui->add_javascript_callback("set_debug_bounding_volumes");
      gui->add_javascript_callback("set_pixel_size");
      gui->add_javascript_callback("set_view_mono_warped");
      gui->add_javascript_callback("set_view_stereo_warped");
      gui->add_javascript_callback("set_view_mono");
      gui->add_javascript_callback("set_view_stereo");

      gui->add_javascript_callback("reset_view");
      gui->add_javascript_callback("set_left_view");
      gui->add_javascript_callback("set_right_view");

      gui->call_javascript("init");
    });

    gui->on_javascript_callback.connect([&](std::string const& callback, std::vector<std::string> const& params) {
      if (callback == "set_max_raysteps") {
        std::stringstream str(params[0]);
        int max_raysteps;
        str >> max_raysteps;
        warp_pass->max_raysteps(max_raysteps);

      } else if (callback == "set_split_threshold") {
        std::stringstream str(params[0]);
        float split_threshold;
        str >> split_threshold;
        grid_pass->split_threshold(split_threshold);
      } else if (callback == "set_pixel_size") {
        std::stringstream str(params[0]);
        float pixel_size;
        str >> pixel_size;
        warp_pass->pixel_size(pixel_size-0.5);
      } else if (callback == "set_cell_size") {
        std::stringstream str(params[0]);
        int cell_size;
        str >> cell_size;
        grid_pass->cell_size(std::pow(2, cell_size));
      } else if (callback == "set_vsync") {
        std::stringstream str(params[0]);
        str >> vsync;
        window->config.set_enable_vsync(vsync);
      } else if (callback == "set_warp_perspective") {
        std::stringstream str(params[0]);
        str >> warp_perspective;
        update_view_mode();
      } else if (callback == "set_warping") {
        std::stringstream str(params[0]);
        str >> warping;
        update_view_mode();
      } else if (callback == "set_stereo") {
        std::stringstream str(params[0]);
        str >> stereo;
        update_view_mode();
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
      } else if (callback == "set_hole_filling_color") {
        std::stringstream str(params[0]);
        gua::math::vec3f color;
        str >> color;
        warp_pass->hole_filling_color(color/255.f);
      } else if (callback == "reset_view") {
        warp_nav.reset();
        eye_offset = 0;
        update_view_mode();
      } else if (callback == "set_left_view") {
        eye_offset = eye_dist*-0.5;
        update_view_mode();
      } else if (callback == "set_right_view") {
        eye_offset = eye_dist*0.5;
        update_view_mode();
      } else if (callback == "set_hole_filling_type_none"
               | callback == "set_hole_filling_type_epipolar_search"
               | callback == "set_hole_filling_type_epipolar_mirror"
               | callback == "set_hole_filling_type_blur") {
        std::stringstream str(params[0]);
        bool checked;
        str >> checked;
        if (checked) {

          gua::WarpPassDescription::HoleFillingMode mode(gua::WarpPassDescription::HOLE_FILLING_NONE);

          if (callback == "set_hole_filling_type_epipolar_search")
            mode = gua::WarpPassDescription::HOLE_FILLING_EPIPOLAR_SEARCH;
          if (callback == "set_hole_filling_type_epipolar_mirror")
            mode = gua::WarpPassDescription::HOLE_FILLING_EPIPOLAR_MIRROR;
          if (callback == "set_hole_filling_type_blur")
            mode = gua::WarpPassDescription::HOLE_FILLING_BLUR;

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
      } else if (callback == "set_stereotype_spatial"
              || callback == "set_stereotype_temporal") {
        std::stringstream str(params[0]);
        bool checked;
        str >> checked;
        if (checked) {
          stereotype_spatial = false;
          stereotype_temporal = false;

          if (callback == "set_stereotype_spatial") stereotype_spatial = true;
          if (callback == "set_stereotype_temporal") stereotype_temporal = true;
        }
        update_view_mode();
      } else if (callback == "set_scene_oilrig"            ||
                 callback == "set_scene_sponza1"           ||
                 callback == "set_scene_sponza2"           ||
                 callback == "set_scene_sponza3"           ||
                 callback == "set_scene_teapot"            ||
                 callback == "set_scene_pitoti"            ||
                 callback == "set_scene_bottle"            ||
                 callback == "set_scene_hairball"          ||
                 callback == "set_scene_glasses") {
        set_scene(callback);
      }
    });
  #endif

  // ---------------------------------------------------------------------------
  // ----------------------------- setup windows -------------------------------
  // ---------------------------------------------------------------------------

  window->config.set_fullscreen_mode(fullscreen);
  window->cursor_mode(gua::GlfwWindow::CursorMode::HIDDEN);

  #if (!POWER_WALL && !OCULUS1 && !OCULUS2 && !STEREO_MONITOR)
    window->on_resize.connect([&](gua::math::vec2ui const& new_size) {
      resolution = new_size;
      window->config.set_resolution(new_size);
      normal_cam->config.set_resolution(new_size);

      #if USE_SIDE_BY_SIDE
        normal_screen->data.set_size(gua::math::vec2(screen_width/2, screen_width * new_size.y / new_size.x));
        warp_screen->data.set_size(gua::math::vec2(screen_width/2, screen_width * new_size.y / new_size.x));
      #else
        normal_screen->data.set_size(gua::math::vec2(screen_width, screen_width * new_size.y / new_size.x));
        warp_screen->data.set_size(gua::math::vec2(screen_width, screen_width * new_size.y / new_size.x));
      #endif
    });
  #endif

  window->on_button_press.connect([&](int key, int action, int mods) {
    #if GUI_SUPPORT
      gui->inject_mouse_button(gua::Button(key), action, mods);
    #endif

    nav.set_mouse_button(key, action);
    warp_nav.set_mouse_button(key, action);

  });

  window->on_key_press.connect([&](int key, int scancode, int action, int mods) {
    if (manipulation_navigator) {
      nav.set_key_press(key, action);
    } else if (manipulation_camera) {
      warp_nav.set_key_press(key, action);
    }
    #if GUI_SUPPORT
      gui->inject_keyboard_event(gua::Key(key), scancode, action, mods);
      if (action >= 1) {
        if (key == 72) toggle_gui();
        if (key == 93) {
          pitoti->set_error_threshold(std::max(0.0, pitoti->get_error_threshold()-0.5));
          std::cout << pitoti->get_error_threshold() << std::endl;
        }
        if (key == 47) {
          pitoti->set_error_threshold(pitoti->get_error_threshold()+0.5);
          std::cout << pitoti->get_error_threshold() << std::endl;
        }
      }
    #endif
  });

  window->on_move_cursor.connect([&](gua::math::vec2 const& pos) {
    #if GUI_SUPPORT
      bool hit_gui(false);
      gua::math::vec2 hit_pos;
      mouse_quad->data.offset() = pos + gua::math::vec2i(-2, -45);
      hit_gui = gui_quad->pixel_to_texcoords(pos, resolution, hit_pos) && !gui_quad->get_tags().has_tag("invisible");

      if (hit_gui) {
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
    #else
      if (manipulation_object) {
        object_trackball.motion(pos.x, pos.y);
      } else if (manipulation_navigator) {
        nav.set_mouse_position(gua::math::vec2i(pos));
      } else {
        warp_nav.set_mouse_position(gua::math::vec2i(pos));
      }
    #endif
  });

  window->on_button_press.connect(std::bind(mouse_button, std::ref(object_trackball), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  #if POWER_WALL
    window->config.set_size(gua::math::vec2ui(1920*2, 1200));
    window->config.set_right_position(gua::math::vec2ui(1920, 0));
    window->config.set_right_resolution(gua::math::vec2ui(1780, 1185));
    window->config.set_left_position(gua::math::vec2ui(140, 0));
    window->config.set_left_resolution(gua::math::vec2ui(1780, 1185));
  #elif !OCULUS1 && !OCULUS2
    
    window->config.set_size(resolution);

    #if USE_SIDE_BY_SIDE 
      window->config.set_left_resolution(gua::math::vec2ui(resolution.x/2, resolution.y));
      window->config.set_right_resolution(gua::math::vec2ui(resolution.x/2, resolution.y));
      window->config.set_right_position(gua::math::vec2ui(resolution.x/2, 0));
    #else
      window->config.set_resolution(resolution);
    #endif    
  #endif
  window->config.set_enable_vsync(vsync);
  gua::WindowDatabase::instance()->add("window", window);
  window->open();

  // tracking ------------------------------------------------------------------
  warp_pass->get_warp_state([&](){
    #if POWER_WALL
      warp_cam->set_transform(current_tracking_matrix);
    #elif OCULUS1
      warp_cam->set_transform(get_oculus_transform(oculus_sensor));
    #elif OCULUS2
      warp_cam->set_transform(window->get_oculus_sensor_orientation());
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

  // stat display update counter
  int ctr=0;

  gua::Timer frame_timer;
  frame_timer.start();

  // application loop
  while(true) {

    #if POWER_WALL
      normal_cam->set_transform(current_tracking_matrix);
    #elif OCULUS1
      normal_cam->set_transform(get_oculus_transform(oculus_sensor));
    #elif OCULUS2
      normal_cam->set_transform(window->get_oculus_sensor_orientation());
    #endif

    nav.update();
    warp_nav.update();

    #if POWER_WALL
      if (warp_perspective && !warping) {
        navigation->set_transform(normal_screen->get_transform() * gua::math::mat4(warp_nav.get_transform()) * scm::math::inverse(normal_screen->get_transform()));
      } else {
        navigation->set_transform(normal_screen->get_transform() * gua::math::mat4(nav.get_transform()) * scm::math::inverse(normal_screen->get_transform()));
      }
      warp_navigation->set_transform(normal_screen->get_transform() * gua::math::mat4(warp_nav.get_transform()) * scm::math::inverse(normal_screen->get_transform()));
    #else
      if (warp_perspective && !warping) {
        navigation->set_transform(gua::math::mat4(nav.get_transform() * warp_nav.get_transform()));
      } else {
        navigation->set_transform(gua::math::mat4(nav.get_transform()));
      }
      warp_navigation->set_transform(gua::math::mat4(warp_nav.get_transform()));
    #endif

    #if GUI_SUPPORT
      gua::Interface::instance()->update();
    #endif

    gua::math::mat4 modelmatrix = scm::math::make_translation(gua::math::float_t(object_trackball.shiftx()),
                                                              gua::math::float_t(object_trackball.shifty()),
                                                              gua::math::float_t(object_trackball.distance())) * gua::math::mat4(object_trackball.rotation());
    transform->set_transform(modelmatrix);

    if (ctr++ % 100 == 0) {
      std::cout << window->get_rendering_fps() << std::endl;
    }

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
