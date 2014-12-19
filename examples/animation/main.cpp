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
#include <memory>

#include <gua/guacamole.hpp>
#include <gua/renderer/SkeletalAnimationLoader.hpp>
#include <gua/utils/Trackball.hpp>

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

  /*auto load_mat = [](std::string const& file){
    gua::MaterialShaderDescription desc;
    desc.load_from_file(file);
    auto shader(std::make_shared<gua::MaterialShader>(file, desc));
    gua::MaterialShaderDatabase::instance()->add(shader);
    return shader->get_default_material();
  };

  auto mat1(load_mat("data/materials/pinky.gmd"));*/



  gua::SkeletalAnimationLoader loader;
  gua::SkeletalAnimationLoader loader2;

  auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");
  auto transform2 = graph.add_node<gua::node::TransformNode>("/", "transform2");

  auto teapot2(loader2.create_geometry_from_file("bobby", "data/objects/pinky/pinky.md5mesh", gua::SkeletalAnimationLoader::LOAD_MATERIALS | gua::SkeletalAnimationLoader::NORMALIZE_POSITION | gua::SkeletalAnimationLoader::NORMALIZE_SCALE));
  loader2.load_animation(teapot2, "data/objects/pinky/idle1.md5anim", 0);
  loader2.load_animation(teapot2, "data/objects/pinky/attack.md5anim", 0);
  loader2.load_animation(teapot2, "data/objects/pinky/run.md5anim", 0);
  
  auto teapot(loader.create_geometry_from_file("bob", "data/objects/marine/mpplayer.md5mesh",  gua::SkeletalAnimationLoader::LOAD_MATERIALS | gua::SkeletalAnimationLoader::NORMALIZE_POSITION | gua::SkeletalAnimationLoader::NORMALIZE_SCALE));
  // loader.load_animation(teapot, "data/objects/marine/jog.md5anim", 0);
  loader.load_animation(teapot, "data/objects/marine/crouch.md5anim", 0);
  loader.load_animation(teapot, "data/objects/marine/fists_idle.md5anim", 0);
  loader.load_animation(teapot, "data/objects/marine/run.md5anim", 0);
  loader.load_animation(teapot, "data/objects/marine/fists_idle.md5anim", 0);

  //std::shared_ptr<gua::node::SkeletalAnimationNode> skel_node = std::dynamic_pointer_cast<gua::node::SkeletalAnimationNode>(teapot2);
  //skel_node->set_animation_mode(0);
  
  // auto teapot(loader.create_geometry_from_file("bob", "data/objects/bob/boblampclean.md5mesh", mat1, gua::SkeletalAnimationLoader::LOAD_MATERIALS | gua::SkeletalAnimationLoader::NORMALIZE_POSITION | gua::SkeletalAnimationLoader::NORMALIZE_SCALE));
  // loader.load_animation(teapot, "data/objects/bob/boblampclean.md5anim", 0);
  

  graph.add_node("/transform", teapot);
<<<<<<< HEAD
  graph.add_node("/transform2", teapot2);
  // graph.add_node("/transform2", teapot2);
  teapot->set_draw_bounding_box(true);
  teapot2->set_draw_bounding_box(true);

  /*auto light = graph.add_node<gua::node::SpotLightNode>("/", "light");
  light->data.set_enable_shadows(true);
  light->scale(10.f);
  light->rotate(-20, 0.f, 1.f, 0.f);
  light->translate(-1.f, 0.f,  3.f);*/
=======
  // graph.add_node("/transform2", teapot2);
  // teapot->set_draw_bounding_box(true);

  // auto light = graph.add_node<gua::node::SpotLightNode>("/", "light");
  // light->data.set_enable_shadows(true);
  // light->scale(10.f);
  // light->rotate(-20, 0.f, 1.f, 0.f);
  // light->translate(-1.f, 0.f,  3.f);
  auto light = graph.add_node<gua::node::PointLightNode>("/", "light1");
  light->data.color = gua::utils::Color3f(1.0f, 1.0f, 0.8f);
  light->scale(40.f);
  light->data.set_brightness(100.f);
  light->translate(2.f, 3.f, -5.f);
>>>>>>> replaced string-transform map with pose class

  auto light2 = graph.add_node<gua::node::PointLightNode>("/", "light2");
  light2->data.color = gua::utils::Color3f(1.0f, 1.0f, 1.0f);
  light2->scale(10.f);
  light2->translate(-2.f, 3.f, 5.f);

  auto screen = graph.add_node<gua::node::ScreenNode>("/", "screen");
  screen->data.set_size(gua::math::vec2(1.92f, 1.08f));
  screen->translate(0, 0, 1.0);

  //gua::VolumeLoader vloader;
  //auto volume(vloader.create_volume_from_file("volume", "/opt/gua_vrgeo_2013/data/objects/head_w256_h256_d225_c1_b8.raw", 0));
  //graph.add_node("/transform", volume);

  // add mouse interaction
  gua::utils::Trackball trackball(0.01, 0.002, 0.2);

  // setup rendering pipeline and window
  auto resolution = gua::math::vec2ui(1920, 1080);
  
  gua::TextureDatabase::instance()->load("/opt/guacamole/resources/skymaps/skymap.jpg");

  /*auto camera = graph.add_node<gua::node::CameraNode>("/screen", "cam");
  camera->translate(0, 0, 2.0);
  camera->config.set_resolution(resolution);
  camera->config.set_screen_path("/screen");
  camera->config.set_scene_graph_name("main_scenegraph");
  camera->config.set_output_window_name("main_window");
  camera->config.set_enable_stereo(false);
  camera->config.pipeline_description().get_pass<gua::BackgroundPassDescription>()
    .mode(gua::BackgroundPassDescription::QUAD_TEXTURE)
    .texture("/opt/guacamole/resources/skymaps/skymap.jpg");
  camera->config.pipeline_description().get_pass<gua::SSAOPassDescription>()
    .radius(3)
    .intensity(0.1);*/

  auto camera = graph.add_node<gua::node::CameraNode>("/screen", "cam");
  camera->translate(0, 0, 2.0);
  camera->config.set_resolution(resolution);
  camera->config.set_screen_path("/screen");
  camera->config.set_scene_graph_name("main_scenegraph");
  camera->config.set_output_window_name("main_window");
  camera->config.set_enable_stereo(false);

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
  });
  window->on_move_cursor.connect([&](gua::math::vec2 const& pos) {
    trackball.motion(pos.x, pos.y);
  });
  window->on_button_press.connect(std::bind(mouse_button, std::ref(trackball), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  window->open();

  gua::Renderer renderer;

  // application loop
  gua::events::MainLoop loop;
  gua::events::Ticker ticker(loop, 1.0/500.0);

  ticker.on_tick.connect([&]() {

    // apply trackball matrix to object
    auto modelmatrix = scm::math::make_translation(trackball.shiftx(), trackball.shifty(), trackball.distance()) * trackball.rotation();
    transform->set_transform(scm::math::make_translation(-0.75f,0.0f,0.0f) * modelmatrix);
    transform2->set_transform(scm::math::make_translation(0.75f,0.0f,0.0f) * modelmatrix);

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
