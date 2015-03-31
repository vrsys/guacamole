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
#include <gua/node/RayNode.hpp>
#include <gua/renderer/PLODPass.hpp>
#include <gua/renderer/TriMeshPass.hpp>
#include <gua/renderer/SSAAPass.hpp>

#include <gua/renderer/BBoxPass.hpp>
#include <gua/renderer/DebugViewPass.hpp>

#include <gua/renderer/TexturedQuadPass.hpp>

bool rotate_light = false;

/////////////////////////////////////////////////////////////////////////////
// keyboard callbacks:
/////////////////////////////////////////////////////////////////////////////
//
//    general
//    -> 'r' to force shader recompilation
//    -> 'm' toggle ambient lighting vis_mode
//    -> 'b' toggle background
//    -> SPACE toggle light rotation on/off
//    -> use SHIFT + Mouse to pick lens center

//    splatting configuration
//    -> 'u' to increase splat radius
//    -> 'j' to decrease splat radius
//    -> 'i' to increase error threshold
//    -> 'k' to decrease error threshold

//    lens effects
//    -> 'y' toggle lens vis_mode 
//        0 = off
//        1 = distance to lens plane 
//        2 = normal in object space
//    -> 't' to increase lens radius
//    -> 'g' to decrease lens radius

//    screen space shadows
//    -> 'a' toggle on/off
//    -> 'q' to increase shadow radius
//    -> 'z' to decrease shadow radius
//    -> 'w' to increase shadow intensity
//    -> 'x' to decrease shadow intensity

//    SSAO
//    -> 's' toggle on/off
//    -> '1' to increase SSAO intensity
//    -> '2' to decrease SSAO intensity
//    -> '3' to increase SSAO radius
//    -> '4' to decrease SSAO radius
//    -> '5' to increase SSAO falloff
//    -> '6' to decrease SSAO falloff

//    FXAA
//    -> 'f' toggle FXAA: off/fast/FXAA311
//    -> '7' to increase FXAA subpix quality
//    -> '8' to decrease FXAA subpix quality
//    -> '9' to increase FXAA edge threshold
//    -> '0' to increase FXAA edge threshold




///////////////////////////////////////////////////////////////////////////////

// 0 = off
// 1 = distance to plane
// 2 = normal visualization

struct LensConfig {
    enum LensVisMode{ off = 0x0,
                    distance, 
                    normals,
                    first_derivation,
                    second_derivation};

    enum LensGeoMode { sphere_os = 0x0,
        sphere_ss,
        box_ss,
        //box_ws
    };

  gua::math::vec3 world_position;
  gua::math::vec2 screen_position;
  gua::math::vec3 world_normal;
  LensVisMode vis_mode;
  LensGeoMode geo_mode;
  float radius;
  gua::math::vec2 square_ss_min;
  gua::math::vec2 square_ss_max;
  gua::math::vec3 square_ws_min;
  gua::math::vec3 square_ws_max;
  bool dirty_flag;
};

///////////////////////////////////////////////////////////////////////////////
gua::math::vec3 compute_ray(std::shared_ptr<gua::node::ScreenNode> const& screen, std::shared_ptr<gua::node::CameraNode> const& camera, double x, double y) {
  auto sz = screen->data.get_size();
  gua::math::vec4 t = screen->get_world_transform() * gua::math::vec4(x*(sz.x / 2.0), y*(sz.y / 2.0), 0.0, 1.0);
  return normalize(gua::math::vec3(t) - gua::math::get_translation(camera->get_world_transform()));
};

///////////////////////////////////////////////////////////////////////////////
void mouse_button(gua::utils::Trackball& trackball, 
                  gua::SceneGraph& graph, 
                  std::shared_ptr<gua::node::ScreenNode> const& screen, 
                  std::shared_ptr<gua::node::CameraNode> const& camera, 
                  gua::math::vec2ui const& resolution, 
                  LensConfig& lens,
                  int mousebutton, 
                  int action, 
                  int mods)
{
  gua::utils::Trackball::button_type button;
  gua::utils::Trackball::state_type state;

  switch (mousebutton) {
    case 0: button = gua::utils::Trackball::left; break;
    case 1: button = gua::utils::Trackball::right; break;
    case 2: button = gua::utils::Trackball::middle; break;
  };

  switch (action) {
    case 0: state = gua::utils::Trackball::released; break;
    case 1: state = gua::utils::Trackball::pressed; break;
  };

  if (mods) {
    if (state == gua::utils::Trackball::released) {
      // current mouse pos
      double nx = 2.0 * (double(int(trackball.posx()) - int(resolution.x / 2)) / double(resolution.x));
      double ny = 2.0 * (double(int(trackball.posy()) - int(resolution.y / 2)) / double(resolution.y));

      const double ray_dist = 2000.0;
      auto origin = gua::math::get_translation(camera->get_world_transform());
      auto direction = compute_ray(screen, camera, nx, ny) * ray_dist;

      std::set<gua::PickResult> picks = graph.ray_test(
        gua::Ray(origin, direction, 1.0),
        gua::PickResult::PICK_ONLY_FIRST_OBJECT |
        gua::PickResult::PICK_ONLY_FIRST_FACE |
        gua::PickResult::GET_WORLD_POSITIONS |
        gua::PickResult::GET_POSITIONS);

      for (auto const& r : picks)
      {          
        auto frustrum = camera->get_frustum(graph, gua::CameraMode::CENTER);
        auto pick_centre_ss = frustrum.get_projection() * frustrum.get_view() * gua::math::vec4(r.world_position, 1.0f);
        auto pick_centre_ss_norm = (gua::math::vec2(pick_centre_ss.x, pick_centre_ss.y) / pick_centre_ss.w) * 0.5f + gua::math::vec2(0.5f);
        
        lens.screen_position = pick_centre_ss_norm;
        lens.world_position = r.world_position;
        lens.world_normal = r.normal;
        lens.dirty_flag = true;

        lens.square_ss_min = lens.screen_position - gua::math::vec2(lens.radius);
        lens.square_ss_max = lens.screen_position + gua::math::vec2(lens.radius);

        std::cout << "Picked Name: " << r.object->get_name() << "Position: " <<  r.position << " Normal: " << r.normal<< std::endl;
      }
    }
  }
  else {
    trackball.mouse(button, state, trackball.posx(), trackball.posy());
  }
}

/////////////////////////////////////////////////////////////////////////////
void increase_radius(std::shared_ptr<gua::node::Node> const& node) {
  auto plodnode = std::dynamic_pointer_cast<gua::node::PLODNode>(node);
  if (plodnode) {
    auto radius_scale = plodnode->get_radius_scale();
    plodnode->set_radius_scale(std::min(2.0f, 1.1f * radius_scale));
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
    plodnode->set_radius_scale(std::max(0.1f, 0.9f * radius_scale));
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
    plodnode->set_error_threshold(std::min(16.0f, 1.1f * radius_scale));
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
    plodnode->set_error_threshold(std::max(1.0f, 0.9f * radius_scale));
    std::cout << "Setting  error threshold to " << plodnode->get_error_threshold() << std::endl;
  }
  for (auto const& c : node->get_children()) {
    decrease_error_threshold(c);
  }
}


void key_press(gua::PipelineDescription& pipe, gua::SceneGraph& graph, LensConfig& lens, int key, int scancode, int action, int mods)
{
  if (action == 0) return;

  float v = 0.0f;

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

    // toggle lens modes and parametrization
  case 't':
    lens.radius = std::min(10.0f, 1.1f * lens.radius);
    lens.square_ss_min = lens.screen_position - gua::math::vec2(lens.radius);
    lens.square_ss_max = lens.screen_position + gua::math::vec2(lens.radius);
    std::cout << "Set lens radius to " << lens.radius << std::endl;
    break;
  case 'g':
    lens.radius = std::max(0.01f, 0.9f * lens.radius);
    lens.square_ss_min = lens.screen_position - gua::math::vec2(lens.radius);
    lens.square_ss_max = lens.screen_position + gua::math::vec2(lens.radius);
    std::cout << "Set lens radius to " << lens.radius << std::endl;
    break;
  case 'o':
    lens.radius = std::max(0.01f, 0.9f * lens.radius);
    std::cout << "Set lens  to " << lens.radius << std::endl;
    break;
  case 'l':
    lens.radius = std::max(0.01f, 0.9f * lens.radius);
    std::cout << "Set lens radius to " << lens.radius << std::endl;
    break;
  case 'y':
      lens.vis_mode = static_cast<LensConfig::LensVisMode>((lens.vis_mode + 1) % 5);
    std::cout << "Set lens vis_mode to " << lens.vis_mode << std::endl;
    // 0 = off
    // 1 = distance to plane
    // 2 = normal visualization
    break;
  case 'h':
      lens.geo_mode = static_cast<LensConfig::LensGeoMode>((lens.geo_mode + 1) % 3);
      std::cout << "Set lens geo_mode to " << lens.geo_mode << std::endl;
      // 0 = sphere_os
      // 1 = sphere_ss
      // 2 = box_ss
      break;

  case 'm': // toggle environment lighting vis_mode

    if (pipe.get_resolve_pass()->environment_lighting_mode() == gua::ResolvePassDescription::EnvironmentLightingMode::AMBIENT_COLOR) {
      std::cout << "Setting to gua::ResolvePassDescription::EnvironmentLightingMode::SPHEREMAP" << std::endl;
      pipe.get_resolve_pass()->environment_lighting_mode(gua::ResolvePassDescription::EnvironmentLightingMode::SPHEREMAP);
    }
    else if (pipe.get_resolve_pass()->environment_lighting_mode() == gua::ResolvePassDescription::EnvironmentLightingMode::SPHEREMAP) {
      std::cout << "Setting to gua::ResolvePassDescription::EnvironmentLightingMode::CUBEMAP" << std::endl;
      pipe.get_resolve_pass()->environment_lighting_mode(gua::ResolvePassDescription::EnvironmentLightingMode::CUBEMAP);
    }
    else {
      std::cout << "Setting to gua::ResolvePassDescription::EnvironmentLightingMode::AMBIENT_COLOR" << std::endl;
      pipe.get_resolve_pass()->environment_lighting_mode(gua::ResolvePassDescription::EnvironmentLightingMode::AMBIENT_COLOR);
    }

    pipe.get_resolve_pass()->touch();
    break;

  case 'b': // toggle background vis_mode

    if (pipe.get_resolve_pass()->background_mode() == gua::ResolvePassDescription::BackgroundMode::COLOR) {
      std::cout << "Setting to gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE" << std::endl;
      pipe.get_resolve_pass()->background_mode(gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE);
    }
    else if (pipe.get_resolve_pass()->background_mode() == gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE) {
      std::cout << "Setting to gua::ResolvePassDescription::BackgroundMode::SKYMAP_TEXTURE" << std::endl;
      pipe.get_resolve_pass()->background_mode(gua::ResolvePassDescription::BackgroundMode::SKYMAP_TEXTURE);
    }
    else {
      std::cout << "Setting to gua::ResolvePassDescription::BackgroundMode::AMBIENT_COLOR" << std::endl;
      pipe.get_resolve_pass()->background_mode(gua::ResolvePassDescription::BackgroundMode::COLOR);
    }

    pipe.get_resolve_pass()->touch();
    break;

  case 'a':  // toggle screen space shadows
    pipe.get_resolve_pass()->screen_space_shadows(!pipe.get_resolve_pass()->screen_space_shadows());
    break;

  case 'q':
    pipe.get_resolve_pass()->screen_space_shadow_radius(std::min(2.0f, 1.2f * pipe.get_resolve_pass()->screen_space_shadow_radius()));
    break;

  case 'z':
    pipe.get_resolve_pass()->screen_space_shadow_radius(std::max(0.002f, 0.9f * pipe.get_resolve_pass()->screen_space_shadow_radius()));
    break;

  case 'w':
    pipe.get_resolve_pass()->screen_space_shadow_intensity(std::min(1.0f, 1.1f * pipe.get_resolve_pass()->screen_space_shadow_intensity()));
    break;

  case 'x':
    pipe.get_resolve_pass()->screen_space_shadow_intensity(std::max(0.1f, 0.9f * pipe.get_resolve_pass()->screen_space_shadow_intensity()));
    break;


  case 's':  // toggle SSAO
    pipe.get_resolve_pass()->ssao_enable(!pipe.get_resolve_pass()->ssao_enable());
    break;

  case '1':
    pipe.get_resolve_pass()->ssao_intensity(std::min(5.0f, 1.1f * pipe.get_resolve_pass()->ssao_intensity()));
    break;
  case '2':
    pipe.get_resolve_pass()->ssao_intensity(std::max(0.02f, 0.9f * pipe.get_resolve_pass()->ssao_intensity()));
    break;

  case '3':
    pipe.get_resolve_pass()->ssao_radius(std::min(64.0f, 1.1f * pipe.get_resolve_pass()->ssao_radius()));
    break;
  case '4':
    pipe.get_resolve_pass()->ssao_radius(std::max(1.0f, 0.9f * pipe.get_resolve_pass()->ssao_radius()));
    break;

  case '5':
    pipe.get_resolve_pass()->ssao_falloff(std::min(256.0f, 1.1f * pipe.get_resolve_pass()->ssao_falloff()));
    break;
  case '6':
    pipe.get_resolve_pass()->ssao_falloff(std::max(0.1f, 0.9f * pipe.get_resolve_pass()->ssao_falloff()));
    break;

  case 'f':
    if (pipe.get_ssaa_pass()->mode() == gua::SSAAPassDescription::SSAAMode::FXAA311) {
      std::cout << "Switching to simple FAST_FXAA\n" << std::endl;
      pipe.get_ssaa_pass()->mode(gua::SSAAPassDescription::SSAAMode::FAST_FXAA);
    }
    else if (pipe.get_ssaa_pass()->mode() == gua::SSAAPassDescription::SSAAMode::FAST_FXAA) {
      std::cout << "Switching to No FXAA\n" << std::endl;
      pipe.get_ssaa_pass()->mode(gua::SSAAPassDescription::SSAAMode::DISABLED);
    }
    else {
      std::cout << "Switching to FXAA 3.11\n" << std::endl;
      pipe.get_ssaa_pass()->mode(gua::SSAAPassDescription::SSAAMode::FXAA311);
    }
    break;

  case '7':
    v = std::min(1.0f, 1.1f * pipe.get_ssaa_pass()->fxaa_quality_subpix());
    std::cout << "Setting quality_subpix to " << v << std::endl;
    pipe.get_ssaa_pass()->fxaa_quality_subpix(v);
    break;
  case '8':
    v = std::max(0.2f, 0.9f * pipe.get_ssaa_pass()->fxaa_quality_subpix());
    std::cout << "Setting quality_subpix to " << v << std::endl;
    pipe.get_ssaa_pass()->fxaa_quality_subpix(v);
    break;

  case '9':
    v = std::min(0.333f, 1.1f * pipe.get_ssaa_pass()->fxaa_edge_threshold());
    std::cout << "Setting edge_threshold to " << v << std::endl;
    pipe.get_ssaa_pass()->fxaa_edge_threshold(v);
    break;
  case '0':
    v = std::max(0.063f, 0.9f * pipe.get_ssaa_pass()->fxaa_edge_threshold());
    std::cout << "Setting edge_threshold to " << v << std::endl;
    pipe.get_ssaa_pass()->fxaa_edge_threshold(v);
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
#define RENDER_PITOTI_HUNTING_SCENE 1
#define RENDER_EXAMPLE_STEREO 0

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

  auto rough_red = pbr_overwrite_color_shader->make_new_material();
  rough_red->set_uniform("color", gua::math::vec3f(0.8f, 0.0f, 0.0f));
  rough_red->set_uniform("metalness", 0.0f);
  rough_red->set_uniform("roughness", 0.8f);
  rough_red->set_uniform("emissivity", 0.0f);

  /////////////////////////////////////////////////////////////////////////////
  // create scene
  /////////////////////////////////////////////////////////////////////////////

  // create scene graph object
  gua::SceneGraph graph("main_scenegraph");

  // configure plod-renderer and create point-based objects
  gua::PLODLoader plodLoader;
  gua::TriMeshLoader trimesh_loader;

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

#if RENDER_PITOTI_HUNTING_SCENE
  #if WIN32
  auto plod_geometry1(plodLoader.load_geometry("hunter1", "data/objects/Area-1_Warrior-scene_P01-2_knn.kdn", plod_rough, gua::PLODLoader::DEFAULTS | gua::PLODLoader::MAKE_PICKABLE ));
  auto plod_geometry2(plodLoader.load_geometry("hunter2", "data/objects/Area-1_Warrior-scene_P01-3_knn.kdn", plod_rough, gua::PLODLoader::DEFAULTS | gua::PLODLoader::MAKE_PICKABLE ));
  auto plod_geometry3(plodLoader.load_geometry("hunter3", "data/objects/Area-1_Warrior-scene_P01-4_knn.kdn", plod_rough, gua::PLODLoader::DEFAULTS | gua::PLODLoader::MAKE_PICKABLE ));
  auto plod_geometry4(plodLoader.load_geometry("hunter4", "data/objects/Area-1_Warrior-scene_P02-1_knn.kdn", plod_rough, gua::PLODLoader::DEFAULTS | gua::PLODLoader::MAKE_PICKABLE ));
  auto plod_geometry5(plodLoader.load_geometry("hunter5", "data/objects/Area-2_Plowing-scene_P02-3_knn.kdn", plod_rough, gua::PLODLoader::DEFAULTS | gua::PLODLoader::MAKE_PICKABLE ));
  auto plod_geometry6(plodLoader.load_geometry("hunter6", "data/objects/Area-2_Plowing-scene_P02-4_knn.kdn", plod_rough, gua::PLODLoader::DEFAULTS | gua::PLODLoader::MAKE_PICKABLE ));
  auto plod_geometry7(plodLoader.load_geometry("hunter7", "data/objects/Area-1_Warrior-scene_P03-1_knn.kdn", plod_rough, gua::PLODLoader::DEFAULTS | gua::PLODLoader::MAKE_PICKABLE ));
  auto plod_geometry8(plodLoader.load_geometry("hunter8", "data/objects/Area-1_Warrior-scene_P03-2_knn.kdn", plod_rough, gua::PLODLoader::DEFAULTS | gua::PLODLoader::MAKE_PICKABLE ));
  auto plod_geometry9(plodLoader.load_geometry("hunter9", "data/objects/Area-1_Warrior-scene_P03-3_knn.kdn", plod_rough, gua::PLODLoader::DEFAULTS | gua::PLODLoader::MAKE_PICKABLE ));
  auto plod_geometry10(plodLoader.load_geometry("hunter10", "data/objects/Area-1_Warrior-scene_P03-4_knn.kdn", plod_rough, gua::PLODLoader::DEFAULTS | gua::PLODLoader::MAKE_PICKABLE));
  auto plod_geometry11(plodLoader.load_geometry("hunter11", "data/objects/TLS_Seradina_Rock-12C_knn.kdn", plod_rough, gua::PLODLoader::DEFAULTS | gua::PLODLoader::MAKE_PICKABLE));
#else
  auto plod_geometry1(plodLoader.load_geometry("hunter1",   "/mnt/pitoti/3d_pitoti/seradina_12c/areas/Area-1_Warrior-scene_P01-2_knn.kdn", plod_rough, gua::PLODLoader::DEFAULTS));
  auto plod_geometry2(plodLoader.load_geometry("hunter2",   "/mnt/pitoti/3d_pitoti/seradina_12c/areas/Area-1_Warrior-scene_P01-3_knn.kdn", plod_rough, gua::PLODLoader::DEFAULTS));
  auto plod_geometry3(plodLoader.load_geometry("hunter3",   "/mnt/pitoti/3d_pitoti/seradina_12c/areas/Area-1_Warrior-scene_P01-4_knn.kdn", plod_rough, gua::PLODLoader::DEFAULTS));
  auto plod_geometry4(plodLoader.load_geometry("hunter4",   "/mnt/pitoti/3d_pitoti/seradina_12c/areas/Area-1_Warrior-scene_P02-1_knn.kdn", plod_rough, gua::PLODLoader::DEFAULTS));
  auto plod_geometry5(plodLoader.load_geometry("hunter5",   "/mnt/pitoti/3d_pitoti/seradina_12c/areas/Area-2_Plowing-scene_P02-3_knn.kdn", plod_rough, gua::PLODLoader::DEFAULTS));
  auto plod_geometry6(plodLoader.load_geometry("hunter6",   "/mnt/pitoti/3d_pitoti/seradina_12c/areas/Area-2_Plowing-scene_P02-4_knn.kdn", plod_rough, gua::PLODLoader::DEFAULTS));
  auto plod_geometry7(plodLoader.load_geometry("hunter7",   "/mnt/pitoti/3d_pitoti/seradina_12c/areas/Area-1_Warrior-scene_P03-1_knn.kdn", plod_rough, gua::PLODLoader::DEFAULTS));
  auto plod_geometry8(plodLoader.load_geometry("hunter8",   "/mnt/pitoti/3d_pitoti/seradina_12c/areas/Area-1_Warrior-scene_P03-2_knn.kdn", plod_rough, gua::PLODLoader::DEFAULTS));
  auto plod_geometry9(plodLoader.load_geometry("hunter9",   "/mnt/pitoti/3d_pitoti/seradina_12c/areas/Area-1_Warrior-scene_P03-3_knn.kdn", plod_rough, gua::PLODLoader::DEFAULTS));
  auto plod_geometry10(plodLoader.load_geometry("hunter10", "/mnt/pitoti/3d_pitoti/seradina_12c/areas/Area-1_Warrior-scene_P03-4_knn.kdn", plod_rough, gua::PLODLoader::DEFAULTS));
  auto plod_geometry11(plodLoader.load_geometry("hunter11", "/mnt/pitoti/3d_pitoti/seradina_12c/rock/TLS_Seradina_Rock-12C_knn.kdn", plod_rough, gua::PLODLoader::DEFAULTS));
#endif
  setup_plod_node(plod_geometry1);
  setup_plod_node(plod_geometry2);
  setup_plod_node(plod_geometry3);
  setup_plod_node(plod_geometry4);
  setup_plod_node(plod_geometry5);
  setup_plod_node(plod_geometry6);
  setup_plod_node(plod_geometry7);
  setup_plod_node(plod_geometry8);
  setup_plod_node(plod_geometry9);
  setup_plod_node(plod_geometry10);
  setup_plod_node(plod_geometry11);

  // connect scene graph
  graph.add_node("/transform/model_xf", plod_geometry1);
  graph.add_node("/transform/model_xf", plod_geometry2);
  graph.add_node("/transform/model_xf", plod_geometry3);
  graph.add_node("/transform/model_xf", plod_geometry4);
  graph.add_node("/transform/model_xf", plod_geometry5);
  graph.add_node("/transform/model_xf", plod_geometry6);
  graph.add_node("/transform/model_xf", plod_geometry7);
  graph.add_node("/transform/model_xf", plod_geometry8);
  graph.add_node("/transform/model_xf", plod_geometry9);
  graph.add_node("/transform/model_xf", plod_geometry10);
  graph.add_node("/transform/model_xf", plod_geometry11);

  model_xf->translate(-plod_geometry1->get_bounding_box().center());

#else
  auto plod_geometry(plodLoader.load_geometry("plod_pig", "data/objects/pig.kdn", plod_rough, gua::PLODLoader::NORMALIZE_POSITION | gua::PLODLoader::NORMALIZE_SCALE | gua::PLODLoader::MAKE_PICKABLE));
  setup_plod_node(plod_geometry);
  graph.add_node("/transform/model_xf", plod_geometry);

  auto teapot(trimesh_loader.create_geometry_from_file("teapot", "data/objects/teapot.obj", gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE | gua::TriMeshLoader::MAKE_PICKABLE));
  graph.add_node("/transform/model_xf", teapot);
  teapot->translate(0.6, 0.0, 0.0);

#endif 

  auto pick_proxy_geometry(trimesh_loader.create_geometry_from_file("pick_proxy", "data/objects/sphere.obj", rough_red, gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));
  pick_proxy_geometry->scale(0.01);
  auto pick_transform = graph.add_node<gua::node::TransformNode>("/transform/model_xf", "pick_transform");
  pick_transform->add_child(pick_proxy_geometry);

  auto light_proxy_geometry(trimesh_loader.create_geometry_from_file("light_proxy", "data/objects/sphere.obj", rough_white, gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));
  auto camera_proxy_geometry(trimesh_loader.create_geometry_from_file("camera_proxy", "data/objects/sphere.obj", rough_white, gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));
  light_proxy_geometry->scale(0.01);

  /////////////////////////////////////////////////////////////////////////////
  // create lighting
  /////////////////////////////////////////////////////////////////////////////
  auto light = graph.add_node<gua::node::LightNode>("/transform/model_xf", "light");
  light->data.set_type(gua::node::LightNode::Type::POINT);
  light->data.set_enable_shadows(true);
  light->data.set_brightness(30.f);
  light->scale(3.f);
  light->translate(0.5, 0.3, 1.3);
  light->add_child(light_proxy_geometry);

  /////////////////////////////////////////////////////////////////////////////
  // create viewing setup
  /////////////////////////////////////////////////////////////////////////////
  auto screen = graph.add_node<gua::node::ScreenNode>("/", "screen");
  //screen->data.set_size(gua::math::vec2(1.92f, 1.08f));
  //screen->data.set_size(gua::math::vec2(1.6f, 0.9f));
  screen->data.set_size(gua::math::vec2(1.2f, 0.8f));
  //screen->data.set_size(gua::math::vec2(12.0f, 8.0f));
  screen->translate(0.0, 0.0, 10.0);

  // add mouse interaction
  gua::utils::Trackball trackball(0.01, 0.002, 0.2);

  // setup rendering pipeline and window
  //auto resolution = gua::math::vec2ui(2560, 1440);
  auto resolution = gua::math::vec2ui(1920, 1080);

  /////////////////////////////////////////////////////////////////////////////
  // create scene camera and pipeline
  /////////////////////////////////////////////////////////////////////////////

  auto camera = graph.add_node<gua::node::CameraNode>("/screen", "cam");
  camera->translate(0, 0, 10.0);
  camera->config.set_resolution(resolution);
  camera->config.set_screen_path("/screen");
  camera->config.set_eye_dist(0.06f);
  camera->config.set_left_screen_path("/screen");
  camera->config.set_right_screen_path("/screen");
  camera->config.set_scene_graph_name("main_scenegraph");
  camera->config.set_output_window_name("main_window");
#if RENDER_EXAMPLE_STEREO
  camera->config.set_enable_stereo(true);
#else
  camera->config.set_enable_stereo(false);
#endif
  camera->config.set_far_clip(100.0f);
  camera->config.set_near_clip(0.01f);
  camera->add_child(camera_proxy_geometry);
 //camera->set_pre_render_cameras({portal_camera});

  auto pipe = std::make_shared<gua::PipelineDescription>();

  pipe->add_pass(std::make_shared<gua::TriMeshPassDescription>());
  pipe->add_pass(std::make_shared<gua::TexturedQuadPassDescription>());
  //pipe->add_pass(std::make_shared<gua::BBoxPassDescription>());
  pipe->add_pass(std::make_shared<gua::PLODPassDescription>());
  pipe->add_pass(std::make_shared<gua::LightVisibilityPassDescription>());
  pipe->add_pass(std::make_shared<gua::ResolvePassDescription>());
  pipe->add_pass(std::make_shared<gua::SSAAPassDescription>());
  //pipe->add_pass(std::make_shared<gua::DebugViewPassDescription>());

  pipe->get_resolve_pass()->background_mode(gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE);
  pipe->get_resolve_pass()->background_texture("data/images/skymap.jpg");

  pipe->get_pass_by_type<gua::ResolvePassDescription>()->ssao_enable(true);
  pipe->get_pass_by_type<gua::ResolvePassDescription>()->ssao_radius(16.0);

  pipe->get_pass_by_type<gua::ResolvePassDescription>()->screen_space_shadows(true);
  pipe->get_pass_by_type<gua::ResolvePassDescription>()->screen_space_shadow_radius(0.2f);
  pipe->get_pass_by_type<gua::ResolvePassDescription>()->screen_space_shadow_max_radius_px(200);
  pipe->get_pass_by_type<gua::ResolvePassDescription>()->screen_space_shadow_intensity(1.0);

  camera->set_pipeline_description(pipe);

  /////////////////////////////////////////////////////////////////////////////
  // create window and callback setup
  /////////////////////////////////////////////////////////////////////////////
  float lense_init_size = 0.1f;

  LensConfig lens_config = { gua::math::vec3{ 0.0, 0.0, 0.0 }, 
      gua::math::vec2{ 0.5, 0.5 },
      gua::math::vec3{ 1.0, 0.0, 0.0 }, 
      LensConfig::LensVisMode::off, 
      LensConfig::LensGeoMode::sphere_os, 
      lense_init_size, 
      gua::math::vec2(-lense_init_size),
      gua::math::vec2(lense_init_size),
      gua::math::vec3(-lense_init_size),
      gua::math::vec3(lense_init_size),
      true };

  auto window = std::make_shared<gua::GlfwWindow>();
  gua::WindowDatabase::instance()->add("main_window", window);
  window->config.set_enable_vsync(false);
  window->config.set_size(resolution);
  window->config.set_resolution(resolution);
#if RENDER_EXAMPLE_STEREO
  window->config.set_stereo_mode(gua::StereoMode::ANAGLYPH_RED_CYAN);
#else
  window->config.set_stereo_mode(gua::StereoMode::MONO);
#endif

  window->on_resize.connect([&](gua::math::vec2ui const& new_size) {
    window->config.set_resolution(new_size);
    camera->config.set_resolution(new_size);
    screen->data.set_size(gua::math::vec2(0.001f * new_size.x, 0.001f * new_size.y));
  });

  window->on_move_cursor.connect([&](gua::math::vec2 const& pos) {
    trackball.motion((int)pos.x, (int)pos.y);
  });

  window->on_button_press.connect(std::bind(mouse_button,
    std::ref(trackball),
    std::ref(graph),
    std::cref(screen),
    std::cref(camera),
    std::cref(window->config.get_left_resolution()), 
    std::ref(lens_config),
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  window->on_key_press.connect(std::bind(key_press, 
                               std::ref(*(camera->get_pipeline_description())), 
                               std::ref(graph), 
                               std::ref(lens_config), 
                               std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

  window->open();

  gua::Renderer renderer;

  // application loop
  gua::events::MainLoop loop;
  gua::events::Ticker ticker(loop, 1.0/500.0);

  std::size_t ctr = 0;

  auto last_frame_time = std::chrono::steady_clock::now();

  ticker.on_tick.connect([&]() {
    gua::math::mat4 modelmatrix = scm::math::make_translation(gua::math::float_t(trackball.shiftx()),
                                                              gua::math::float_t(trackball.shifty()),
                                                              gua::math::float_t(trackball.distance())) * gua::math::mat4(trackball.rotation());

    transform->set_transform(modelmatrix);

    static unsigned framecounter = 0;
    ++framecounter;

    auto current_time = std::chrono::steady_clock::now();
    double milliseconds = std::chrono::duration_cast<std::chrono::microseconds>(current_time - last_frame_time).count() / 1000.0;

    if (lens_config.dirty_flag) {
      pick_transform->set_transform(scm::math::inverse(model_xf->get_world_transform()) * scm::math::make_translation(lens_config.world_position));
      lens_config.dirty_flag = false;
    }

    plod_rough->set_uniform("lens_center", pick_transform->get_world_position());
    plod_rough->set_uniform("lens_center_ss", lens_config.screen_position);
    plod_rough->set_uniform("lens_normal", lens_config.world_normal);
    plod_rough->set_uniform("lens_vis_mode", int(lens_config.vis_mode));
    plod_rough->set_uniform("lens_geo_mode", int(lens_config.geo_mode));
    plod_rough->set_uniform("lens_radius", lens_config.radius);
    plod_rough->set_uniform("lens_square_ss_min", lens_config.square_ss_min);
    plod_rough->set_uniform("lens_square_ss_max", lens_config.square_ss_max);
    plod_rough->set_uniform("lens_square_ws_min", lens_config.square_ws_min);
    plod_rough->set_uniform("lens_square_ws_max", lens_config.square_ws_max);

    if (rotate_light) {
      // modify scene
      
      if (milliseconds > 0.0 ) {
        double time_per_rotation = 15;
        light->translate(-0.3, -1.0, -1.0);
        light->rotate((360 * milliseconds) / (time_per_rotation * 1000.0), 0.0, 0.0, 1.0);
        light->translate(0.3, 1.0, 1.0);
      }  
    }
    last_frame_time = current_time;

    if (ctr++ % 150 == 0) {
      //std::cout << "Frame time: " << 1000.f / window->get_rendering_fps() << " ms, fps: "
      //  << window->get_rendering_fps() << ", app fps: "
      //  << camera->get_application_fps() << std::endl;
      std::cout << lens_config.screen_position << " , " << lens_config.world_position << " , " << lens_config.world_normal << " , " << lens_config.radius << std::endl;
      
    }

    // apply trackball matrix to object
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
