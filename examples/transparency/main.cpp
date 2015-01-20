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
#include <scm/gl_util/manipulators/trackball_manipulator.h>
#include <iostream>
#include <gua/guacamole.hpp>
#include <gua/renderer/TriMeshLoader.hpp>

int main(int argc, char** argv) {
  
  auto resolution = gua::math::vec2ui(1920, 1080);

  // navigation
  scm::gl::trackball_manipulator trackball;
  trackball.transform_matrix(scm::math::make_translation(0.f, -0.5f, 0.f));
  trackball.dolly(0.2f);
  float dolly_sens = 1.5f;
  gua::math::vec2 trackball_init_pos(0.f);
  gua::math::vec2 last_mouse_pos(0.f);
  int button_state = -1;

  // initialize guacamole
  gua::init(argc, argv);

  // setup scene
  gua::SceneGraph graph("main_scenegraph");

  auto load_mat = [](std::string const& file){
    auto desc(std::make_shared<gua::MaterialShaderDescription>());
    desc->load_from_file(file);
    auto shader(std::make_shared<gua::MaterialShader>(file, desc));
    gua::MaterialShaderDatabase::instance()->add(shader);
    return shader->make_new_material();
  };

  gua::TriMeshLoader loader;

  auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");
  auto transform2 = graph.add_node<gua::node::TransformNode>("/", "transform2");

  // Load Monkey
  auto mat_glass(load_mat("data/materials/Glass.gmd"));
  mat_glass->set_uniform("color", gua::math::vec3(1.f, 0.f, 0.f)).set_show_back_faces(true);
  auto monkey(loader.create_geometry_from_file("monkey", "data/objects/monkey.obj", mat_glass,
                                               gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));
  monkey->scale(0.3f);
  monkey->translate(0.3f, 0.15f, 0.f);
  monkey->set_draw_bounding_box(true);
  graph.add_node("/transform", monkey);

  // Load bottle
  auto mat_bottle(load_mat("data/materials/Bottle.gmd"));
  mat_bottle->set_uniform("ColorMap",     std::string("data/objects/bottle/albedo.png"))
             .set_uniform("RoughnessMap", std::string("data/objects/bottle/roughness.jpg"))
             .set_show_back_faces(true);

  // Original bottle model is taken from http://www.sweethome3d.com (Licensed under Free Art License)
  auto bottle(loader.create_geometry_from_file("bottle", "data/objects/bottle/bottle.obj", mat_bottle,
                                               gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));
  graph.add_node("/transform2", bottle);

  // Load box (texture originals are taken from http://texturise.blogspot.de/)
  std::string directory("/home/zapa4360/Textures/");
  auto mat_box(gua::MaterialShaderDatabase::instance()->lookup("gua_default_material")->make_new_material());
  mat_box->set_uniform("ColorMap",     directory + "parquet2/albedo.jpg")
          .set_uniform("NormalMap",    directory + "parquet2/normal.jpg")
          .set_uniform("RoughnessMap", directory + "parquet2/roughness.jpg");
  auto box(loader.create_geometry_from_file("box", "data/objects/inverted_box.obj", mat_box, gua::TriMeshLoader::DEFAULTS));
  graph.add_node("/transform", box);

  transform2->scale(0.3f);
  transform2->translate(0.f, 0.15f, 0.f);

  // Portal
  auto portal = graph.add_node<gua::node::TexturedQuadNode>("/", "portal");
  portal->data.set_size(gua::math::vec2(1.2f, 0.8f));
  portal->data.set_texture("portal");
  portal->translate(1.5f, 0.f, -0.2f);
  portal->rotate(-30, 0.f, 1.f, 0.f);

  // Lights
  auto light = graph.add_node<gua::node::SpotLightNode>("/", "light");
  light->data.set_brightness(10.0f);
  light->scale(2.f);
  light->rotate(-10, 0.f, 1.f, 0.f);
  light->rotate(-10, 1.f, 0.f, 0.f);
  light->translate(-0.2f, 0.5f,  1.f);

  auto light2 = graph.add_node<gua::node::PointLightNode>("/", "light2");
  light2->data.set_brightness(10.0f);
  light2->scale(1.2f);
  light2->translate(0.4f, 0.7f, -0.4f);

  auto light3 = graph.add_node<gua::node::PointLightNode>("/", "light3");
  light3->scale(6.f);
  light3->translate(2.f, 1.f, -5.f);

  auto screen = graph.add_node<gua::node::ScreenNode>("/", "screen");
  screen->data.set_size(gua::math::vec2(1.92f, 1.08f));

  auto portal_screen = graph.add_node<gua::node::ScreenNode>("/", "portal_screen");
  portal_screen->data.set_size(gua::math::vec2(1.2f, 0.8f));

  // setup rendering pipeline and window
  auto portal_camera = graph.add_node<gua::node::CameraNode>("/portal_screen", "portal_cam");
  portal_camera->translate(0.f, 0.8f, 2.f);
  portal_camera->config.set_resolution(gua::math::vec2ui(1200, 800));
  portal_camera->config.set_screen_path("/portal_screen");
  portal_camera->config.set_scene_graph_name("main_scenegraph");
  portal_camera->config.set_output_texture_name("portal");
  portal_camera->config.set_enable_stereo(false);

  gua::TextureDatabase::instance()->load("data/checkerboard.png");

  auto portal_pipe = std::make_shared<gua::PipelineDescription>();
  portal_pipe->add_pass<gua::TriMeshPassDescription>();
  portal_pipe->add_pass<gua::LightVisibilityPassDescription>();
  portal_pipe->add_pass<gua::ResolvePassDescription>();
  portal_pipe->set_enable_abuffer(true);
  portal_camera->set_pipeline_description(portal_pipe);

  auto camera = graph.add_node<gua::node::CameraNode>("/screen", "cam");
  camera->translate(0, 0, 2.0);
  camera->config.set_resolution(resolution);
  camera->config.set_screen_path("/screen");
  camera->config.set_scene_graph_name("main_scenegraph");
  camera->config.set_output_window_name("main_window");
  camera->config.set_enable_stereo(false);
  camera->set_pre_render_cameras({portal_camera});
  camera->get_pipeline_description()->get_pass<gua::ResolvePassDescription>()
    .mode(gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE)
    .texture("data/checkerboard.png");
  camera->get_pipeline_description()->set_enable_abuffer(true);

  auto window = std::make_shared<gua::GlfwWindow>();
  gua::WindowDatabase::instance()->add("main_window", window);
  window->config.set_enable_vsync(false);
  window->config.set_size(resolution);
  window->config.set_resolution(resolution);
  window->config.set_stereo_mode(gua::StereoMode::MONO);
  window->on_resize.connect([&](gua::math::vec2ui const& new_size) {
    window->config.set_resolution(new_size);
    camera->config.set_resolution(new_size);
    screen->data.set_size(gua::math::vec2(0.001 * new_size.x, 0.001 * new_size.y));
    resolution = new_size;
  });

  bool drag_mode = false;
  window->on_move_cursor.connect([&](gua::math::vec2 const& pos) {
    float nx = 2.f * float(pos.x - (resolution.x/2))/float(resolution.x);
    float ny = -2.f * float(resolution.y - pos.y - (resolution.y/2))/float(resolution.y);
    if (button_state != -1) {
      if (drag_mode) {
        auto ssize = screen->data.get_size();
        gua::math::vec3 sm_translation = gua::math::vec3(ssize.x *(nx - last_mouse_pos.x), ssize.y * (ny - last_mouse_pos.y), 0.f);
        auto object_transform_s = scm::math::inverse(screen->get_world_transform()) * transform2->get_world_transform();
        transform2->set_world_transform( screen->get_world_transform() * scm::math::make_translation(sm_translation) * object_transform_s);
      } 
      else {
        if (button_state == 0) { // left
          trackball.rotation(trackball_init_pos.x, trackball_init_pos.y, nx, ny);
        }
        if (button_state == 1) { // right
          trackball.dolly(dolly_sens*0.5f * (ny - trackball_init_pos.y));
        }
        if (button_state == 2) { // middle
          float f = dolly_sens < 1.0f ? 0.02f : 0.3f;
          trackball.translation(f*(nx - trackball_init_pos.x), f*(ny - trackball_init_pos.y));
        }
        trackball_init_pos.x = nx;
        trackball_init_pos.y = ny;
      }
    }
    last_mouse_pos.x = nx;
    last_mouse_pos.y = ny;
  });

  window->on_button_press.connect([&](int mousebutton, int action, int mods) {
    if (action == 1) {
      drag_mode = mods == 1;
      trackball_init_pos = last_mouse_pos;
      button_state = mousebutton;
    } else
      button_state = -1;
  });


  float blend_thres = 0.99f;
  window->on_key_press.connect([&](int key, int scancode, int action, int mods) {
      if (action != 0) {
        //std::cout << "key press " << char(key) << " action: " << action <<"\n";
        auto& d = camera->get_pipeline_description()->get_pass<gua::LightVisibilityPassDescription>();
        auto& d_r = camera->get_pipeline_description()->get_pass<gua::ResolvePassDescription>();

        if ('W' == key) { // forward
          bottle->rotate(-5.0, 1.f, 0.f, 0.f);
        }
        if ('S' == key) { // backward
          bottle->rotate(5.0, 1.f, 0.f, 0.f);
        }

        if ('1' == key) {
          d.rasterization_mode(gua::LightVisibilityPassDescription::AUTO);
          std::cout << "Rast mode: AUTO\n"; d.touch();
        }
        if ('2' == key) {
          d.rasterization_mode(gua::LightVisibilityPassDescription::SIMPLE);
          std::cout << "Rast mode: SIMPLE\n"; d.touch();
        }
        if ('3' == key) {
          d.rasterization_mode(gua::LightVisibilityPassDescription::CONSERVATIVE);
          std::cout << "Rast mode: CONSERVATIVE\n"; d.touch();
        }
        if ('4' == key) {
          d.rasterization_mode(gua::LightVisibilityPassDescription::MULTISAMPLED_2);
          std::cout << "Rast mode: MULTISAMPLED_2\n"; d.touch();
        }
        if ('5' == key) {
          d.rasterization_mode(gua::LightVisibilityPassDescription::MULTISAMPLED_4);
          std::cout << "Rast mode: MULTISAMPLED_4\n"; d.touch();
        }
        if ('6' == key) {
          d.rasterization_mode(gua::LightVisibilityPassDescription::MULTISAMPLED_8);
          std::cout << "Rast mode: MULTISAMPLED_8\n"; d.touch();
        }
        if ('7' == key) {
          d.rasterization_mode(gua::LightVisibilityPassDescription::MULTISAMPLED_16);
          std::cout << "Rast mode: MULTISAMPLED_16\n"; d.touch();
        }
        if ('8' == key) {
          d.rasterization_mode(gua::LightVisibilityPassDescription::FULLSCREEN_FALLBACK);
          std::cout << "Rast mode: FULLSCREEN_FALLBACK\n"; d.touch();
        }

        if ('9' == key || '0' == key) {
          d.tile_power(d.tile_power() + ((key=='9')?1:-1));
          std::cout << "Tile size: " << std::pow(2, d.tile_power()) <<"\n";
          d.touch();
        }
        if ('Q' == key) {
          d_r.debug_tiles(!d_r.debug_tiles());
          std::cout << "Debug tiles: " << d_r.debug_tiles() <<"\n";
          d_r.touch();
        }
        if ('T' == key) {
          auto& desc = camera->get_pipeline_description();
          desc->set_enable_abuffer(!desc->get_enable_abuffer());
          portal_pipe->set_enable_abuffer(!portal_pipe->get_enable_abuffer());
          std::cout << "Enable A-Buffer: " << desc->get_enable_abuffer() <<"\n";
        }
        if ('Z' == key || 'X' == key) {
          blend_thres += ('Z' == key) ? 0.005f : -0.005f;
          blend_thres = std::max(std::min(blend_thres, 1.f), 0.5f);
          camera->get_pipeline_description()->set_blending_termination_threshold(blend_thres);
          std::cout << "Early termination threshold: " << blend_thres << std::endl;
        }
      }
    });

  window->open();

  gua::Renderer renderer;

  // application loop
  gua::events::MainLoop loop;
  gua::events::Ticker ticker(loop, 1.0/500.0);

  size_t ctr{};
  float alpha = 0.f;
  float alpha_d = 0.01f;

  ticker.on_tick.connect([&]() {
    
    screen->set_transform(scm::math::inverse(trackball.transform_matrix()));

    if (ctr++ % 500 == 0)
      std::cout << camera->get_rendering_fps() << " "
                << camera->get_application_fps() << std::endl;

    if (ctr % 25 == 0) {
      alpha += alpha_d;
      if (alpha > 1.f || alpha < 0.f) alpha_d = -alpha_d;
      mat_glass->set_uniform("alpha", alpha);
    }

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
