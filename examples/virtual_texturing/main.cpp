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
#include <gua/renderer/ToneMappingPass.hpp>
#include <gua/renderer/DebugViewPass.hpp>

#include <gua/virtual_texturing/DeferredVirtualTexturingPass.hpp>

#include <gua/utils/Trackball.hpp>

// forward mouse interaction to trackball
void mouse_button(gua::utils::Trackball& trackball,
                  int mousebutton,
                  int action,
                  int mods) {
  gua::utils::Trackball::button_type button;
  gua::utils::Trackball::state_type state;

  switch (mousebutton) {
    case 0:
      button = gua::utils::Trackball::left;
      break;
    case 2:
      button = gua::utils::Trackball::middle;
      break;
    case 1:
      button = gua::utils::Trackball::right;
      break;
  };

  switch (action) {
    case 0:
      state = gua::utils::Trackball::released;
      break;
    case 1:
      state = gua::utils::Trackball::pressed;
      break;
  };

  trackball.mouse(button, state, trackball.posx(), trackball.posy());
}


int main(int argc, char** argv) {
  // initialize guacamole
  gua::init(argc, argv);

  // setup scene
  gua::SceneGraph graph("main_scenegraph");

  gua::TriMeshLoader loader;

  auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");


  // create simple untextured material shader
  auto virtual_texture_mat_input_descriptor = std::make_shared<gua::MaterialShaderDescription>("./data/materials/MinimalVirtualTexturing.gmd");
  auto virtual_texturing_preparation_shader(std::make_shared<gua::MaterialShader>("VirtualTexturing", virtual_texture_mat_input_descriptor));
  gua::MaterialShaderDatabase::instance()->add(virtual_texturing_preparation_shader);

  //create material for virtual_texturing
  auto vt = virtual_texturing_preparation_shader->make_new_material();
  vt->set_uniform("metalness", 0.0f);
  vt->set_uniform("roughness", 1.0f);
  vt->set_uniform("emissivity", 1.0f);




  auto plane(loader.create_geometry_from_file(
      "plane", "/mnt/terabytes_of_textures/montblanc/montblanc_1202116x304384.obj",
      vt,
      gua::TriMeshLoader::NORMALIZE_POSITION |
      //gua::TriMeshLoader::NORMALIZE_SCALE |  
      gua::TriMeshLoader::MAKE_PICKABLE)  );
  graph.add_node("/transform", plane);

  scm::math::vec3d plane_translation(0.0, 0.0, -3.0);

  plane->set_draw_bounding_box(true);

  //std::string const& texture_atlas_path = "data/objects/onepointfive_texture_2048_w2048_h2048.atlas";
  std::string const& texture_atlas_path = "/mnt/terabytes_of_textures/montblanc/montblanc_w1202116_h304384.atlas";


  // LOAD VIRTUAL TEXTURE
  gua::TextureDatabase::instance()->load(texture_atlas_path);





  auto light2 = graph.add_node<gua::node::LightNode>("/", "light2");
  light2->data.set_type(gua::node::LightNode::Type::POINT);
  light2->data.brightness = 150.0f;
  light2->scale(12.f);
  light2->translate(-3.f, 5.f, 5.f);

  auto screen = graph.add_node<gua::node::ScreenNode>("/", "screen");
  screen->data.set_size(gua::math::vec2(1.92f, 1.08f));
  screen->translate(0, 0, 1.0);


  // add mouse interaction
  gua::utils::Trackball trackball(0.01, 0.002, 0.2);

  // setup rendering pipeline and window
  auto resolution = gua::math::vec2ui(1920, 1080);

  auto camera = graph.add_node<gua::node::CameraNode>("/screen", "cam");
  camera->translate(0, 0, 2.0);
  camera->config.set_resolution(resolution);
  camera->config.set_screen_path("/screen");
  camera->config.set_scene_graph_name("main_scenegraph");
  camera->config.set_output_window_name("Virtual_Texturing_Example");
  camera->config.set_enable_stereo(false);

  camera->get_pipeline_description()->add_pass(
    std::make_shared<gua::DebugViewPassDescription>());


  auto pipe = std::make_shared<gua::PipelineDescription>();
  pipe->add_pass(std::make_shared<gua::TriMeshPassDescription>());

  pipe->add_pass(std::make_shared<gua::DeferredVirtualTexturingPassDescription>());

  pipe->add_pass(std::make_shared<gua::LightVisibilityPassDescription>());
  auto resolve_pass = std::make_shared<gua::ResolvePassDescription>();
  resolve_pass->background_mode(
      gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE);
  resolve_pass->tone_mapping_exposure(1.0f);
  pipe->add_pass(resolve_pass);
  //pipe->add_pass(std::make_shared<gua::DebugViewPassDescription>());
  camera->set_pipeline_description(pipe);







  auto window = std::make_shared<gua::GlfwWindow>();
  gua::WindowDatabase::instance()->add("Virtual_Texturing_Example", window);

  window->config.set_enable_vsync(false);
  window->config.set_size(resolution);
  window->config.set_resolution(resolution);
  window->config.set_stereo_mode(gua::StereoMode::MONO);

  window->on_resize.connect([&](gua::math::vec2ui const& new_size) {
    window->config.set_resolution(new_size);
    camera->config.set_resolution(new_size);
    screen->data.set_size(
        gua::math::vec2(0.001 * new_size.x, 0.001 * new_size.y));
  });
  window->on_move_cursor.connect(
      [&](gua::math::vec2 const& pos) { trackball.motion(pos.x, pos.y); });
  window->on_button_press.connect(
      std::bind(mouse_button, std::ref(trackball), std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));

  gua::Renderer renderer;

  // application loop
  gua::events::MainLoop loop;
  gua::events::Ticker ticker(loop, 1.0 / 500.0);


  ticker.on_tick.connect([&]() {

    // apply trackball matrix to object
    gua::math::mat4 modelmatrix =
        scm::math::make_translation(plane_translation[0], plane_translation[1], plane_translation[2]) * 
        scm::math::make_translation(trackball.shiftx(), trackball.shifty(),
                                    trackball.distance()) *
        gua::math::mat4(trackball.rotation()) * scm::math::make_rotation(90.0, 0.0, 0.0, 1.0) * scm::math::make_rotation(90.0, 1.0, 0.0, 0.0) ;

    transform->set_transform(modelmatrix);

    if (window->should_close()) {
      renderer.stop();
      window->close();
      loop.stop();
    } else {

      renderer.queue_draw({&graph});
    }

        std::cout << "Frame time: " << 1000.f / window->get_rendering_fps() 
                  << " ms, fps: "
                  << window->get_rendering_fps() << ", app fps: "
                  << renderer.get_application_fps() << std::endl;

  });

  loop.start();

  return 0;
}
