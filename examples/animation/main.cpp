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
#include <gua/Skelanim.hpp>
#include <gua/renderer/TriMeshLoader.hpp>
#include <gua/utils/Trackball.hpp>

#include <gua/renderer/BBoxPass.hpp>
#include <gua/renderer/TexturedQuadPass.hpp>
#include <gua/renderer/TexturedScreenSpaceQuadPass.hpp>
#include <gua/renderer/SkeletalAnimationPass.hpp>
#include <gua/renderer/DebugViewPass.hpp>

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

  auto load_mat = [](std::string const& file){
    gua::MaterialShaderDescription desc;
    desc.load_from_file(file);
    auto shader(std::make_shared<gua::MaterialShader>(file, desc));
    gua::MaterialShaderDatabase::instance()->add(shader);
    return shader->make_new_material();
  };

  auto mat1(gua::MaterialShaderDatabase::instance()->lookup("gua_default_material")->make_new_material());
  gua::SkeletalAnimationLoader loader;
  gua::TriMeshLoader tri_loader;

  auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");
  auto transform2 = graph.add_node<gua::node::TransformNode>("/", "transform2");

  // auto skelNode2(loader.create_geometry_from_file("bobby", "data/objects/pinky/pinky.md5mesh", gua::SkeletalAnimationLoader::LOAD_MATERIALS | gua::SkeletalAnimationLoader::NORMALIZE_POSITION | gua::SkeletalAnimationLoader::NORMALIZE_SCALE));
  // graph.add_node("/transform", skelNode2);
  // skelNode2->add_animations("data/objects/pinky/idle1.md5anim", "idle");
  // skelNode2->add_animations("data/objects/pinky/attack.md5anim", "attack");
  // skelNode2->add_animations("data/objects/pinky/run.md5anim", "run");
  // skelNode2->set_animation_2("run");
  // skelNode2->set_draw_bounding_box(true);

  // auto bob(loader.create_geometry_from_file("bob", "data/objects/bob/boblampclean.md5mesh", mat1, gua::SkeletalAnimationLoader::LOAD_MATERIALS | gua::SkeletalAnimationLoader::NORMALIZE_POSITION | gua::SkeletalAnimationLoader::NORMALIZE_SCALE));
  // bob->add_animations("data/objects/bob/boblampclean.md5anim");
  // auto bob(loader.create_geometry_from_file("bob", "data/objects/marine/mpplayer.md5mesh",  gua::SkeletalAnimationLoader::LOAD_MATERIALS | gua::SkeletalAnimationLoader::NORMALIZE_POSITION | gua::SkeletalAnimationLoader::NORMALIZE_SCALE));
  // bob->set_draw_bounding_box(true);
  // graph.add_node("/transform", bob);
  // bob->add_animations("data/objects/marine/crouch.md5anim", "unterschiedlich");
  // bob->add_animations("data/objects/marine/fists_idle.md5anim", "unterschiedlich");
  // bob->add_animations("data/objects/marine/run.md5anim", "unterschiedlich");
  // bob->add_animations("data/objects/marine/fists_idle.md5anim", "unterschiedlich");


  // auto triNode(tri_loader.create_geometry_from_file("fbx", "data/objects/fbx/barrel.fbx", mat1, gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));
  // auto triNode(tri_loader.create_geometry_from_file("fbx", "data/objects/fbx/highrise/highrise.fbx", mat1, gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));
  // auto triNode(tri_loader.create_geometry_from_file("fbx", "data/objects/fbx/office/BlueprintOffice.FBX", mat1, gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));
  // auto triNode(tri_loader.create_geometry_from_file("fbx", "data/objects/fbx/SunTemple/SunTemple.FBX", mat1, gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));
  
  // auto skelNode(loader.create_geometry_from_file("fbx", "data/objects/fbx/face.fbx", mat1, gua::SkeletalAnimationLoader::NORMALIZE_POSITION | gua::SkeletalAnimationLoader::NORMALIZE_SCALE));
  auto skelNode(loader.create_geometry_from_file("fbx", "data/objects/fbx/Necris/Necris_LP.FBX", mat1, gua::SkeletalAnimationLoader::LOAD_MATERIALS | gua::SkeletalAnimationLoader::NORMALIZE_POSITION | gua::SkeletalAnimationLoader::NORMALIZE_SCALE));
  // auto skelNode(loader.create_geometry_from_file("fbx", "data/objects/fbx/Necris/Necris_LP.FBX", mat1, gua::SkeletalAnimationLoader::LOAD_MATERIALS | gua::SkeletalAnimationLoader::NORMALIZE_POSITION | gua::SkeletalAnimationLoader::NORMALIZE_SCALE));
  skelNode->add_animations("data/objects/fbx/Necris/Swim_Bwd_Rif.FBX", "swim");
  // skelNode->add_animations("data/objects/fbx/Necris/Idle_Ready_Pis.FBX", "pistol");
  // skelNode->add_animations("data/objects/fbx/Necris/Taunt_NoNo.FBX", "no");

  // skelNode->set_animation_2("no");
  skelNode->set_animation_2("swim");
  // // skelNode->set_blend_factor(0.5);
  // // skelNode->get_director()->fade_to("no", 1.5, false);
  skelNode->set_draw_bounding_box(true);
  graph.add_node("/transform2", skelNode);

  // auto fbx2(loader.create_geometry_from_file("fbx2", "data/objects/fbx/jill/Mortimer.FBX", mat1, gua::SkeletalAnimationLoader::NORMALIZE_POSITION | gua::SkeletalAnimationLoader::NORMALIZE_SCALE));
  // auto fbx2(loader.create_geometry_from_file("fbx2", "data/objects/fbx/malcolm/malcolm_ut4_SKELMESH.FBX", mat1, gua::SkeletalAnimationLoader::LOAD_MATERIALS | gua::SkeletalAnimationLoader::NORMALIZE_POSITION | gua::SkeletalAnimationLoader::NORMALIZE_SCALE));
  // auto fbx2(loader.create_geometry_from_file("fbx2", "data/objects/fbx/maw/maw.FBX", mat1, gua::SkeletalAnimationLoader::LOAD_MATERIALS | gua::SkeletalAnimationLoader::NORMALIZE_POSITION | gua::SkeletalAnimationLoader::NORMALIZE_SCALE));
  // fbx2->add_animations("data/objects/fbx/Necris/Taunt_NoNo.FBX", "no");
  // fbx2->add_animations("data/objects/fbx/Maw/Walk.FBX", "no");
  // std::shared_ptr<gua::node::SkeletalAnimationNode> skelNode2 = std::dynamic_pointer_cast<gua::node::SkeletalAnimationNode, gua::node::Node>(fbx2);
  // skelNode2->set_animation_2("no");
  // graph.add_node("/transform", skelNode2);

  // auto skelNode(loader.create_geometry_from_file("fbx", "data/objects/fbx/HeroTPP.FBX", mat1, gua::SkeletalAnimationLoader::LOAD_MATERIALS | gua::SkeletalAnimationLoader::NORMALIZE_POSITION | gua::SkeletalAnimationLoader::NORMALIZE_SCALE));
  // skelNode->add_animations("data/objects/fbx/Idle.FBX", "unterschiedlich");
  // skelNode->add_animations("data/objects/fbx/face.fbx", "unterschiedlich");
  // skelNode->add_animations("data/objects/fbx/Walk.FBX", "unterschiedlich");
  // skelNode->add_animations("data/objects/fbx/Run.FBX", "unterschiedlich");
  

  // auto light2 = graph.add_node<gua::node::PointLightNode>("/", "light2");
  // light2->data.color = gua::utils::Color3f(1.0f, 1.0f, 1.0f);
  // light2->data.set_brightness(100.f);
  // light2->scale(10.f);
  // light2->translate(-2.f, 3.f, 5.f);

  auto screen = graph.add_node<gua::node::ScreenNode>("/", "screen");
  screen->data.set_size(gua::math::vec2(1.92f, 1.08f));
  screen->translate(0, 0, 1.0);

  // add mouse interaction
  gua::utils::Trackball trackball(0.01, 0.002, 0.2);

  // setup rendering pipeline and window
  auto resolution = gua::math::vec2ui(1920, 1080);
  
  gua::TextureDatabase::instance()->load("/opt/guacamole/resources/skymaps/skymap.jpg");


  auto camera = graph.add_node<gua::node::CameraNode>("/screen", "cam");
  camera->translate(0, 0, 2.0);
  camera->config.set_resolution(resolution);
  camera->config.set_screen_path("/screen");
  camera->config.set_scene_graph_name("main_scenegraph");
  camera->config.set_output_window_name("main_window");
  camera->config.set_enable_stereo(false);
  camera->config.set_near_clip(0.001f);
  camera->config.set_far_clip(10.0f);

  auto pipe = gua::PipelineDescription::make_default();
  pipe->add_pass(std::make_shared<gua::TriMeshPassDescription>());
  pipe->add_pass(std::make_shared<gua::SkeletalAnimationPassDescription>());
  pipe->add_pass(std::make_shared<gua::TexturedQuadPassDescription>());
  pipe->add_pass(std::make_shared<gua::LightVisibilityPassDescription>());
  pipe->add_pass(std::make_shared<gua::BBoxPassDescription>());
  pipe->add_pass(std::make_shared<gua::ResolvePassDescription>());
  pipe->add_pass(std::make_shared<gua::TexturedScreenSpaceQuadPassDescription>());
  // pipe->add_pass(std::make_shared<gua::DebugViewPassDescription>());
  camera->set_pipeline_description(pipe);

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
  float i = -1;
  ticker.on_tick.connect([&]() {

    // apply trackball matrix to object
    auto modelmatrix = scm::math::make_translation(trackball.shiftx(), trackball.shifty(), trackball.distance()) * trackball.rotation();
    transform->set_transform(scm::math::make_translation(-0.75,0.0,0.0) * modelmatrix);
    transform2->set_transform(scm::math::make_translation(0.75,0.0,0.0) * modelmatrix);
    skelNode->set_time_2(i);
    // skelNode2->set_time_2(i);
    i += 0.001;
    if (i >0.5) i = 0;
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
