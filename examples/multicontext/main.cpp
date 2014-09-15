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
#define _USE_MATH_DEFINES

#include <gua/guacamole.hpp>
#include <gua/node/TexturedQuadNode.hpp>
#include <gua/node/TriMeshNode.hpp>

#include <gua/platform.hpp>

#include <gua/renderer/TriMeshLoader.hpp>
#include <gua/renderer/NURBSLoader.hpp>
#include <gua/renderer/Video3DLoader.hpp>

#include <thread>
#include <chrono>
#include <cmath>

void set_pipe_defaults(gua::Pipeline* pipe, unsigned width, unsigned height)
{
  pipe->config.set_resolution(gua::math::vec2ui(width, height));
  // pipe->config.set_enable_fps_display(true);
  // pipe->config.set_enable_frustum_culling(true);
  // pipe->config.set_enable_backface_culling(true);
  // pipe->config.set_enable_preview_display(true);
  // pipe->config.set_enable_bbox_display(true);
  // pipe->config.set_near_clip(0.2f);
  // pipe->config.set_far_clip(1000.0f);
  // pipe->config.set_background_color(gua::utils::Color3f(0.0, 0.0f, 0.0f));
}

void set_window_default(gua::WindowBase* window, unsigned width, unsigned height)
{
  window->config.set_size(gua::math::vec2ui(width, height));
  window->config.set_left_resolution(gua::math::vec2ui(width, height));
  window->config.set_right_resolution(gua::math::vec2ui(width, height));
  window->config.set_enable_vsync(true);
}

int main(int argc, char** argv) {

  unsigned width = 800;
  unsigned height = 600;

  // initialize guacamole
  int argc_d = 0;
  char** argv_d = {};
  gua::init(argc_d, argv_d);

  //gua::ShadingModelDatabase::load_shading_models_from("data/materials/");
  //gua::MaterialDatabase::load_materials_from("data/materials/");
  //gua::TextureDatabase::instance()->load("data/textures/0001MM_diff.jpg");

  // setup scene
  gua::SceneGraph graph("main_scenegraph");

  gua::TriMeshLoader trimeshloader;
  // gua::NURBSLoader nurbsloader;
  // gua::Video3DLoader videoloader;

  // auto video_geode(videoloader.create_geometry_from_file("video_geode", argv[1]));
  auto teapot_geode(trimeshloader.create_geometry_from_file("teapot_geode", "data/objects/teapot.obj", gua::MaterialInstance(), gua::TriMeshLoader::DEFAULTS));
  auto plate_geode(trimeshloader.create_geometry_from_file("plate_geode", "data/objects/plate.obj", gua::MaterialInstance(), gua::TriMeshLoader::DEFAULTS));
  // auto nurbs_geode(nurbsloader.create_geometry_from_file("nurbs_geode", "data/objects/teapot.igs", "data/materials/Orange.gmd", gua::NURBSLoader::DEFAULTS));

  auto video = graph.add_node<gua::node::TransformNode>("/", "video");
  auto teapot = graph.add_node<gua::node::TransformNode>("/", "teapot");
  auto nurbs = graph.add_node<gua::node::TransformNode>("/", "nurbs");
  auto plate = graph.add_node<gua::node::TransformNode>("/", "plate");

  // graph.add_node("/video", video_geode);
  graph.add_node("/teapot", teapot_geode);
  // graph.add_node("/nurbs", nurbs_geode);
  graph.add_node("/plate", plate_geode);


  const float aspect = width * 1.0f/height;
  auto screen = graph.add_node<gua::node::ScreenNode>("/", "screen");
  //screen->data.set_size(gua::math::vec2(aspect*0.05, 0.05));
  screen->data.set_size(gua::math::vec2(aspect*2.0, 2.0));
  screen->translate(0, 0, 6.f);

  auto screen2 = graph.add_node<gua::node::ScreenNode>("/", "screen2");
  //screen2->data.set_size(gua::math::vec2(aspect*0.05, 0.05));
  screen2->data.set_size(gua::math::vec2(aspect*2.0, 2.0));
  screen2->translate(0, 0, 6.f);

  auto eye = graph.add_node<gua::node::TransformNode>("/screen", "eye");
  //eye->translate(0.0, 0.0, 0.025);
  eye->translate(0.0, 0.0, 1.0);

  auto eye2 = graph.add_node<gua::node::TransformNode>("/screen2", "eye2");
  //eye2->translate(0.0, 0.0, 0.025);
  eye2->translate(0.0, 0.0, 1.0);

  auto eye3 = graph.add_node<gua::node::TransformNode>("/", "eye3");
  eye3->translate(-0.05, 0, 7.5);

  auto eye4 = graph.add_node<gua::node::TransformNode>("/", "eye4");
  eye4->translate(0.05, 0, 7.5);

  auto eye5 = graph.add_node<gua::node::TransformNode>("/", "eye5");
  eye5->translate(-0.05, 0, 6.5);

  auto eye6 = graph.add_node<gua::node::TransformNode>("/", "eye6");
  eye6->translate(0.05, 0, 6.5);

  auto eye7 = graph.add_node<gua::node::TransformNode>("/", "eye7");
  eye7->translate(-0.05, 0, 8.5);

  auto eye8 = graph.add_node<gua::node::TransformNode>("/", "eye8");
  eye8->translate(0.05, 0, 8.5);

#if 0

  auto sunlight = graph.add_node<gua::node::SunLightNode>("/", "sunlight");
  sunlight->data.set_shadow_map_size(1024);
  sunlight->data.set_shadow_offset(0.005f);
  sunlight->data.set_enable_shadows(true);
  sunlight->rotate(-90, 1, 0, 0);
  sunlight->translate(0, 0, -5);
#endif

#if 1
  auto spotlight = graph.add_node<gua::node::SpotLightNode>("/", "spotlight");
  spotlight->scale(30.0f);
  spotlight->rotate(-90, 1, 0, 0);
  spotlight->translate(1.0, 18.0, 1.0);

  spotlight->data.set_shadow_map_size(1024);
  spotlight->data.set_falloff(0.1f);
  spotlight->data.set_shadow_offset(0.005f);
  spotlight->data.set_color({ 1.0f, 1.0f, 1.0f });
  //spotlight->data.set_enable_shadows(true);
  spotlight->data.set_enable_shadows(true);
  spotlight->data.set_enable_specular_shading(true);
  spotlight->data.set_enable_diffuse_shading(true);
#endif

  auto pipe  = new gua::Pipeline();
  auto pipe2 = new gua::Pipeline();
  auto pipe3 = new gua::Pipeline();
  auto pipe4 = new gua::Pipeline();


  pipe->config.set_camera(gua::Camera("/screen/eye", "/screen2/eye2",
    "/screen", "/screen2",
    "main_scenegraph"));
  pipe2->config.set_camera(gua::Camera("/eye3", "/eye4",
    "/screen", "/screen",
    "main_scenegraph"));
  pipe3->config.set_camera(gua::Camera("/eye5", "/eye6",
    "/screen", "/screen",
    "main_scenegraph"));
  pipe4->config.set_camera(gua::Camera("/eye7", "/eye8",
    "/screen", "/screen",
    "main_scenegraph"));



  set_pipe_defaults(pipe , width, height);
  set_pipe_defaults(pipe2, width, height);
  set_pipe_defaults(pipe3, width, height);
  set_pipe_defaults(pipe4, width, height);

  auto window (new gua::Window);
  auto window2(new gua::Window);
  auto window3(new gua::Window);
  auto window4(new gua::Window);

#if WIN32
  window->config.set_display_name("\\\\.\\DISPLAY1");
  window2->config.set_display_name("\\\\.\\DISPLAY1");
  window3->config.set_display_name("\\\\.\\DISPLAY1");
  window4->config.set_display_name("\\\\.\\DISPLAY1");
#else
  window->config.set_display_name(":0.0");
  window2->config.set_display_name(":0.0");
  window3->config.set_display_name(":0.0");
  window4->config.set_display_name(":0.0");
#endif


  set_window_default(window, width, height);
  set_window_default(window2, width, height);
  set_window_default(window3, width, height);
  set_window_default(window4, width, height);

#if 0
  //pipe->config.set_enable_stereo(true);
  pipe->config.set_enable_ssao(true);
  pipe->config.set_ssao_intensity(2.f);
  window->config.set_stereo_mode(gua::StereoMode::SIDE_BY_SIDE);
  window->config.set_size(gua::math::vec2ui(2 * width, height));
  window->config.set_left_position(gua::math::vec2ui(0, 0));
  window->config.set_right_position(gua::math::vec2ui(width, 0));
#endif


  // pipe->set_window(window);
  // pipe2->set_window(window2);
  // pipe3->set_window(window3);
  // pipe4->set_window(window4);

#if 1
  gua::Renderer renderer({ pipe,
           pipe2,
           pipe3,
           pipe4
                         });
#else
  gua::Renderer renderer({ pipe });
#endif

  // transform teapot
  auto bbox = teapot_geode->get_bounding_box();
  teapot->translate(-bbox.center());
  teapot->scale(5.0f / std::sqrt(bbox.size(0)*bbox.size(0) +
    bbox.size(1)*bbox.size(1) +
    bbox.size(2)*bbox.size(2)));
  teapot->translate(gua::math::vec3{ -2.0f, -1.5f, -4.0f });

  // tranform nurbs
  // bbox = nurbs_geode->get_bounding_box();
  // nurbs->translate(-bbox.center());
  // nurbs->scale(5.0f / std::sqrt(bbox.size(0)*bbox.size(0) +
  //   bbox.size(1)*bbox.size(1) +
  //   bbox.size(2)*bbox.size(2)));
  // nurbs->rotate(-90, 1, 0, 0);
  // nurbs->translate(gua::math::vec3{ 3.0f, -1.5f, -4.0f });

  // transform plate
  plate->scale(0.07);
  plate->translate(0.0f, -4.0f, -4.0f);

  float time_value = 0;

  //nurbs_geode->translate(0.0f, 0.0f, 0.0f);

  // application loop
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

   scm::math::mat4f ident;
   scm::math::set_identity(ident);
   video->set_transform(ident);

   video->scale(2.0f + std::sin(time_value));
   //video->rotate(10.0f*time_value, 0, 1, 0);
   //video->translate(0, -2.0, -2.0);

   time_value += 0.01f;

   //video_geode->rotate(0.1, 0, 1, 0);

   teapot_geode->rotate(0.3, 0, 1, 0);

   // nurbs_geode->rotate(0.3, 0, 0, 1);

   plate->translate(-plate->get_bounding_box().center());
   plate->rotate(0.04f, 0, 1, 0);
   plate->translate(plate->get_bounding_box().center());

    renderer.queue_draw({ &graph });
  }

  return 0;
}

