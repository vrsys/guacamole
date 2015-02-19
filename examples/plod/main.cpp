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

#include <gua/renderer/TexturedQuadPass.hpp>

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

int main(int argc, char** argv) {

  // initialize guacamole
  gua::init(argc, argv);

  // setup scene
  gua::SceneGraph graph("main_scenegraph");

  // create simple untextured material shader
  auto desc = std::make_shared<gua::MaterialShaderDescription>();
  desc->load_from_file("./data/materials/PLODGlossy.gmd");
  //desc->load_from_file("./data/materials/PLODPassthrough.gmd");
  
  //use this material for models where shading does not make sense
  //desc->load_from_file("./data/materials/PLODUnshaded.gmd");
  auto material_shader(std::make_shared<gua::MaterialShader>("PLOD_unshaded_material", desc));

  gua::MaterialShaderDatabase::instance()->add(material_shader);

  auto PLOD_unshaded_mat  = material_shader->make_new_material();

  gua::TriMeshLoader loader;
  gua::PLODLoader plodLoader;

  plodLoader.set_upload_budget_in_mb(32);
  plodLoader.set_render_budget_in_mb(4048);
  plodLoader.set_out_of_core_budget_in_mb(4096);


  auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");

  //auto teapot(loader.create_geometry_from_file("teapot", "data/objects/teapot.obj", gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));

  //auto plod_geometry(plodLoader.load_geometry("plod_pig", "data/objects/pig.kdn", PLOD_unshaded_mat, gua::PLODLoader::NORMALIZE_POSITION  ));
  auto plod_geometry(plodLoader.load_geometry("data/objects/pig.kdn", gua::PLODLoader::NORMALIZE_POSITION  ));
  //auto plod_geometry(plodLoader.load_geometry("/mnt/pitoti/XYZ_ALL/new_pitoti_sampling/Area_4_hunter_with_bow.kdn", gua::PLODLoader::NORMALIZE_POSITION  ));
  //auto plod_geometry(plodLoader.load_geometry("plod_pig", "/opt/3d_models/point_based/plod/pig.kdn", PLOD_unshaded_mat, gua::PLODLoader::NORMALIZE_POSITION | gua::PLODLoader::NORMALIZE_SCALE ));
  //auto plod_geometry(plodLoader.load_geometry("plod_pig", "/mnt/pitoti/Seradina_FULL_SCAN/sera_fixed/sera_part_01.kdn", PLOD_unshaded_mat, gua::PLODLoader::NORMALIZE_POSITION | gua::PLODLoader::NORMALIZE_SCALE ));
  //auto plod_geometry2(plodLoader.load_geometry("plod_pig2", "/mnt/pitoti/KDN_LOD/PITOTI_KDN_LOD/_seradina.kdn", PLOD_unshaded_mat, gua::PLODLoader::NORMALIZE_POSITION | gua::PLODLoader::NORMALIZE_SCALE ) );

  plod_geometry->set_draw_bounding_box(true);
  //seradina_rock_geometry->set_draw_bounding_box(true);

  //seradina_rock_geometry->scale(5.0);

  transform->add_child(plod_geometry);  
  
  auto portal = graph.add_node<gua::node::TexturedQuadNode>("/", "portal");
  portal->data.set_size(gua::math::vec2(1.2f, 0.8f));
  portal->data.set_texture("portal");
  portal->translate(0.5f, 0.f, -0.2f);
  portal->rotate(-30, 0.f, 1.f, 0.f);
  portal->translate(0.0f, 0.f, -4.8f);
  //portal->translate(0.0f, 0.0, 2.0f);
/*
  auto light = graph.add_node<gua::node::PointLightNode>("/", "light");
  light->data.set_enable_shadows(true);
  light->scale(10.f);
  light->rotate(-20, 0.f, 1.f, 0.f);
  light->translate(-1.f, 0.f,  3.f);


  auto light2 = graph.add_node<gua::node::PointLightNode>("/", "light2");
  light2->data.color = gua::utils::Color3f(1.0f, 1.0f, 1.0f);
  light2->scale(10.f);
  light2->translate(-2.f, 3.f, 5.f);
*/

  auto screen = graph.add_node<gua::node::ScreenNode>("/", "screen");
  //screen->data.set_size(gua::math::vec2(1.92f, 1.08f));
  //screen->data.set_size(gua::math::vec2(1.6f, 0.9f));
  screen->data.set_size(gua::math::vec2(0.45f, 0.25f));
  screen->translate(0, 0, 1.0);


  auto portal_screen = graph.add_node<gua::node::ScreenNode>("/", "portal_screen");
  portal_screen->data.set_size(gua::math::vec2(1.2f, 0.8f));

  // add mouse interaction
  gua::utils::Trackball trackball(0.01, 0.002, 0.2);

  // setup rendering pipeline and window
  //auto resolution = gua::math::vec2ui(2560, 1440);
  auto resolution = gua::math::vec2ui(1920, 1080);
  
  /*
  auto portal_camera = graph.add_node<gua::node::CameraNode>("/portal_screen", "portal_cam");
  portal_camera->translate(0, 0, 2.0);
  portal_camera->config.set_resolution(gua::math::vec2ui(1200, 800));
  //portal_camera->config.set_resolution(resolution);
  portal_camera->config.set_screen_path("/portal_screen");
  portal_camera->config.set_scene_graph_name("main_scenegraph");
  portal_camera->config.set_output_texture_name("portal");
  portal_camera->config.set_enable_stereo(false);

  auto portal_pipe = std::make_shared<gua::PipelineDescription>();
  portal_pipe->add_pass<gua::TriMeshPassDescription>();
  portal_pipe->add_pass<gua::PLODPassDescription>();
  portal_pipe->add_pass<gua::LightVisibilityPassDescription>();
  portal_pipe->add_pass<gua::ResolvePassDescription>()
    .mode(gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE).mode(gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE);

  //portal_pipe->add_pass<gua::EmissivePassDescription>();
  //portal_pipe->add_pass<gua::PhysicallyBasedShadingPassDescription>();
  portal_pipe->add_pass<gua::BackgroundPassDescription>()
    .mode(gua::BackgroundPassDescription::QUAD_TEXTURE)
    .texture("/opt/guacamole/resources/skymaps/skymap.jpg");
  portal_camera->set_pipeline_description(portal_pipe);
  */
  
  auto camera = graph.add_node<gua::node::CameraNode>("/screen", "cam");
  camera->translate(0, 0, 2.0);
  camera->config.set_resolution(resolution);
  camera->config.set_screen_path("/screen");
  camera->config.set_eye_dist(0.06);
  camera->config.set_left_screen_path("/screen");
  camera->config.set_right_screen_path("/screen");
  camera->config.set_scene_graph_name("main_scenegraph");
  camera->config.set_output_window_name("main_window");
  camera->config.set_enable_stereo(true);
  //camera->config.set_enable_stereo(false);
  camera->config.set_far_clip(500.0);
  camera->config.set_near_clip(0.0001);
  //camera->set_pre_render_cameras({portal_camera});

  auto pipe = std::make_shared<gua::PipelineDescription>();

  pipe->add_pass(std::make_shared<gua::TriMeshPassDescription>());
  pipe->add_pass(std::make_shared<gua::TexturedQuadPassDescription>());
  pipe->add_pass(std::make_shared<gua::BBoxPassDescription>());
  pipe->add_pass(std::make_shared<gua::PLODPassDescription>());
  pipe->add_pass(std::make_shared<gua::LightVisibilityPassDescription>());
  pipe->add_pass(std::make_shared<gua::ResolvePassDescription>());

  pipe->get_resolve_pass()->mode(gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE);
  pipe->get_resolve_pass()->texture("/opt/guacamole/resources/skymaps/skymap.jpg");
  
  
  //camera->get_pipeline_description()->get_pass<gua::BackgroundPassDescription>()
//  camera->get_pipeline_description()->get_pass<gua::ResolvePassDescription>()
//    .mode(gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE)
 //   .texture("/opt/guacamole/resources/skymaps/skymap.jpg");
    //.enable_fog(true)
    //.fog_start(1)
    //.fog_end(10);
  
 
   camera->set_pipeline_description(pipe);
  
  auto window = std::make_shared<gua::GlfwWindow>();
  gua::WindowDatabase::instance()->add("main_window", window);
  window->config.set_enable_vsync(false);
  window->config.set_size(resolution);
  window->config.set_resolution(resolution);
  window->config.set_stereo_mode(gua::StereoMode::ANAGLYPH_RED_CYAN);
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

  window->open();

  gua::Renderer renderer;

  plod_geometry->set_importance( 1.0f ); 
  plod_geometry->set_enable_backface_culling_by_normal( false ); 
      //camera->translate(3.0, 0, 0.0);
  //plod_geometry->translate(0.0, 0.0, 2.0);

  // application loop
  gua::events::MainLoop loop;
  gua::events::Ticker ticker(loop, 1.0/500.0);

  ticker.on_tick.connect([&]() {
    auto modelmatrix = scm::math::make_translation(trackball.shiftx(), trackball.shifty(), trackball.distance()) * trackball.rotation();
    transform->set_transform(modelmatrix);
    static unsigned framecounter = 0;
    ++framecounter;
    PLOD_unshaded_mat->set_uniform("roughness", float(std::abs(std::sin(framecounter)*0.1)));

    //std::cout << "Frame time: " << 1000.0f / camera->get_rendering_fps()<<" ms, fps:" << camera->get_rendering_fps() << ", app fps: " << camera->get_application_fps() << std::endl;

    // apply trackball matrix to object
    window->process_events();
    if (window->should_close()) {
      renderer.stop();
      window->close();
      loop.stop();
    } else { 
      renderer.queue_draw({&graph}, {camera});
      //camera->translate(0.0003, 0, 0.0);
      //plod_geometry->translate(-0.00015, 0, 0.0);

    }
  });

  loop.start();

  return 0;
}
