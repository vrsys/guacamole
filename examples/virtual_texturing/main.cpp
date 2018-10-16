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


void set_window_default(std::shared_ptr<gua::WindowBase> const& window, gua::math::vec2ui const& res) {
  window->config.set_size(res);
  window->config.set_resolution(res);
  window->config.set_enable_vsync(false);
  window->config.set_stereo_mode(gua::StereoMode::MONO);
}

int main(int argc, char** argv) {
  // initialize guacamole
  gua::init(argc, argv);

  // setup scene
  gua::SceneGraph graph("main_scenegraph");

  gua::TriMeshLoader loader;

  auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");

  // VT STEP 1/5: - create a material
  auto earth_vt_mat = gua::MaterialShaderDatabase::instance()->lookup("gua_default_material")->make_new_material();
  // VT STEP 2/5: - load *.atlas-File as uniform
  earth_vt_mat->set_uniform("earth_vt_mat", std::string("/opt/3d_models/virtual_texturing/earth_colour_86400x43200_256x256_1_rgb.atlas"));
  // VT STEP 3/5: - enable virtual texturing for this material
  earth_vt_mat->set_enable_virtual_texturing(true);

  //prepare geometry
  auto earth_1_transform = graph.add_node<gua::node::TransformNode>("/transform", "earth_1_transform");

  // VT STEP 4/5: - load earth with vt material
  auto earth_geode_1(loader.create_geometry_from_file(
      "earth_geode", "/opt/3d_models/virtual_texturing/earth_86400x43200_smooth_normals.obj",
      earth_vt_mat,
      gua::TriMeshLoader::NORMALIZE_POSITION |
      gua::TriMeshLoader::MAKE_PICKABLE)  );

  earth_1_transform->translate(1.5, 0.0, 0.0);
  graph.add_node("/transform/earth_1_transform", earth_geode_1);

  auto earth_2_transform = graph.add_node<gua::node::TransformNode>("/transform", "earth_2_transform");

  // VT STEP 4*/5: - load second earth with vt material
  auto earth_geode_2(loader.create_geometry_from_file(
      "earth_geode_2", "/opt/3d_models/virtual_texturing/earth_86400x43200_smooth_normals.obj",
      earth_vt_mat,
      gua::TriMeshLoader::NORMALIZE_POSITION |
      gua::TriMeshLoader::MAKE_PICKABLE)  );

  earth_2_transform->rotate(180.0,0.0, 1.0, 0.0);
  earth_2_transform->translate(-1.5, 0.0, 0.0);
  graph.add_node("/transform/earth_2_transform", earth_geode_2);

  auto money_transform = graph.add_node<gua::node::TransformNode>("/transform", "money_transform");
  auto money_geode(loader.create_geometry_from_file("money", "/opt/3d_models/50cent/50Cent.obj",
                      gua::TriMeshLoader::LOAD_MATERIALS | 
                      gua::TriMeshLoader::NORMALIZE_POSITION |
                      gua::TriMeshLoader::NORMALIZE_SCALE  ) );

  graph.add_node("/transform/money_transform", money_geode);

  auto light = graph.add_node<gua::node::LightNode>("/", "light2");
  light->data.set_type(gua::node::LightNode::Type::POINT);
  light->data.brightness = 150.0f;
  light->scale(12.f);
  light->translate(-3.f, 5.f, 5.f);

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


  auto pipe = std::make_shared<gua::PipelineDescription>();
  pipe->add_pass(std::make_shared<gua::TriMeshPassDescription>());
  // VT STEP 5/5: - add DeferredVirtualTexturingPassDescription
  pipe->add_pass(std::make_shared<gua::DeferredVirtualTexturingPassDescription>()); // <- ONLY USE THIS PASS IF YOU LOAD VT MODELS
  pipe->add_pass(std::make_shared<gua::LightVisibilityPassDescription>());
  auto resolve_pass = std::make_shared<gua::ResolvePassDescription>();
  resolve_pass->background_mode(
      gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE);
  resolve_pass->tone_mapping_exposure(1.0f);
  pipe->add_pass(resolve_pass);
  camera->set_pipeline_description(pipe);


  std::shared_ptr<gua::GlfwWindow> main_window = nullptr;

  uint32_t window_count = 0;

  auto add_window = [&](std::string const& window_name, 
                       std::shared_ptr<gua::node::CameraNode> const& cam_node)
  {
    auto window = std::make_shared<gua::GlfwWindow>();
    gua::WindowDatabase::instance()->add(window_name, window);
    set_window_default(window, cam_node->config.get_resolution());
    cam_node->config.set_output_window_name(window_name);

    if(++window_count == 1) {
      main_window = window;
      window->on_resize.connect([&](gua::math::vec2ui const& new_size) {
        window->config.set_resolution(new_size);
        cam_node->config.set_resolution(new_size);
        screen->data.set_size(
            gua::math::vec2(0.001 * new_size.x, 0.001 * new_size.y));
      });


      window->on_move_cursor.connect(
          [&](gua::math::vec2 const& pos) { trackball.motion(pos.x, pos.y); });
      window->on_button_press.connect(
          std::bind(mouse_button, std::ref(trackball), std::placeholders::_1,
                    std::placeholders::_2, std::placeholders::_3));      
    }
  };

  add_window("Virtual_Texturing_Example_Window", camera);

  gua::Renderer renderer;

  // application loop
  gua::events::MainLoop loop;
  gua::events::Ticker ticker(loop, 1.0 / 500.0);


  double extra_rotation = 0.0;

  ticker.on_tick.connect([&]() {

    extra_rotation += 0.01;
    // apply trackball matrix to object
    gua::math::mat4 manipulation_matrix = scm::math::make_translation(0.0, 0.0, -3.0) *
        scm::math::make_translation(trackball.shiftx(), trackball.shifty(),
                                    trackball.distance() * 0.15f) *
        gua::math::mat4(trackball.rotation());


    earth_geode_1->set_transform(scm::math::make_rotation(extra_rotation, 0.0, 1.0, 0.0) );
    earth_geode_2->set_transform(scm::math::make_rotation(extra_rotation, 1.0, 1.0, 0.0) );
    money_transform->set_transform(scm::math::make_rotation(45*std::sin(extra_rotation*3.0), 1.0, 0.0, 0.0) );


    transform->set_transform(manipulation_matrix);

    if (main_window->should_close()) {
      renderer.stop();
      main_window->close();
      loop.stop();
    } else {

      renderer.queue_draw({&graph});
    }

        std::cout << "Frame time: " << 1000.f / main_window->get_rendering_fps() 
                  << " ms, fps: "
                  << 
                  main_window->get_rendering_fps() << ", app fps: "
                  << renderer.get_application_fps() << std::endl;

  });

  loop.start();

  return 0;
}
