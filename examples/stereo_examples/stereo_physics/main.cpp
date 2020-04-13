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
#include <gua/utils/Trackball.hpp>
#include <gua/physics.hpp>
#include <GLFW/glfw3.h>

// forward mouse interaction to trackball
void mouse_button(gua::utils::Trackball& trackball, int mousebutton, int action, int mods)
{
    gua::utils::Trackball::button_type button;
    gua::utils::Trackball::state_type state;

    switch(mousebutton)
    {
    case 0:
        button = gua::utils::Trackball::left;
        break;
    case 2:
        button = gua::utils::Trackball::middle;
        break;
    case 1:
        button = gua::utils::Trackball::right;
        break;
    }

    switch(action)
    {
    case 0:
        state = gua::utils::Trackball::released;
        break;
    case 1:
        state = gua::utils::Trackball::pressed;
        break;
    }

    trackball.mouse(button, state, trackball.posx(), trackball.posy());
}

void add_sphere(std::vector<std::shared_ptr<gua::physics::RigidBodyNode>>& balls,
                std::shared_ptr<gua::node::TransformNode>& transform,
                std::shared_ptr<gua::physics::Physics>& physics,
                gua::TriMeshLoader& loader)
{
    auto sphere_body =
        std::make_shared<gua::physics::RigidBodyNode>("sphere_body", 5, 0.5, 0.7, scm::math::make_translation(1.0 - 2.0 * std::rand() / RAND_MAX, 5.0, 1.0 - 2.0 * std::rand() / RAND_MAX));
    auto sphere_shape = std::make_shared<gua::physics::CollisionShapeNode>("sphere_shape");
    sphere_shape->data.set_shape("sphere");
    // graph.get_root()->add_child(sphere_body);
    transform->add_child(sphere_body);
    sphere_body->add_child(sphere_shape);

    auto sphere_geometry(loader.create_geometry_from_file("sphere_geometry",
                                                          "data/objects/sphere.obj",
                                                          gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::LOAD_MATERIALS |
                                                              gua::TriMeshLoader::OPTIMIZE_MATERIALS | gua::TriMeshLoader::NORMALIZE_SCALE));
    sphere_shape->add_child(sphere_geometry);
    sphere_geometry->scale(0.5);

    physics->add_rigid_body(sphere_body);

    balls.push_back(sphere_body);
}

void adjust_arguments(int& argc, char**& argv)
{
    char* argv_tmp[] = {argv[0], NULL};
    int argc_tmp = sizeof(argv_tmp) / sizeof(char*) - 1;
    argc = argc_tmp;
    argv = argv_tmp;
}

enum class Side_By_Side_Mode {
  DEFAULT_SIDE_BY_SIDE = 0,
  SOFTWARE_MULTI_VIEW_RENDERING = 1,
  HARDWARE_MULTI_VIEW_RENDERING = 2,
  NUM_SIDE_BY_SIDE_MODES = 3
};


Side_By_Side_Mode sbs_mode = Side_By_Side_Mode::DEFAULT_SIDE_BY_SIDE;

std::string parse_model_from_cmd_line(int argc, char** argv)
{
    {
        if(argc > 1) {

            sbs_mode = Side_By_Side_Mode(std::atoi(argv[1]));
            gua::Logger::LOG_MESSAGE << "Setting side by side mode to " << (int(sbs_mode) == 0 ? " SIDE_BY_SIDE " : "SIDE_BY_SIDE_SOFTWARE_MULTI_VIEW_RENDERING") << std::endl; 
        }
    }

    return "";
}


int main(int argc, char** argv)
{
    // initialize guacamole

    // initialize guacamole
    std::string model_path = parse_model_from_cmd_line(argc, argv);

    adjust_arguments(argc, argv);
    gua::init(argc, argv);

    auto physics = std::make_shared<gua::physics::Physics>();

    // setup scene
    gua::SceneGraph graph("main_scenegraph");

    gua::TriMeshLoader loader;

    auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");

    auto light = graph.add_node<gua::node::LightNode>("/transform", "light");
    light->data.set_type(gua::node::LightNode::Type::POINT);
    light->data.brightness = 150.0f;
    light->scale(12.f);
    light->translate(-3.f, 5.f, 5.f);

    auto screen = graph.add_node<gua::node::ScreenNode>("/", "screen");
    screen->data.set_size(gua::math::vec2(1.92f, 1.08f));
    screen->translate(0, 2, 1.0);

    // PLANE
    auto plane(
        loader.create_geometry_from_file("plane", "data/objects/plane.obj", gua::TriMeshLoader::OPTIMIZE_GEOMETRY | gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));
    plane->scale(10);
    plane->translate(0, 1, 0);
    auto casted(std::dynamic_pointer_cast<gua::node::TriMeshNode>(plane));
    if(casted)
    {
        casted->get_material()->set_show_back_faces(true);
        casted->get_material()->set_uniform("Metalness", 0.0f);
        casted->get_material()->set_uniform("Roughness", 0.5f);
        casted->get_material()->set_uniform("RoughnessMap", std::string("data/textures/tiles_specular.jpg"));
        casted->get_material()->set_uniform("ColorMap", std::string("data/textures/tiles_diffuse.jpg"));
        casted->get_material()->set_uniform("NormalMap", std::string("data/textures/tiles_normal.jpg"));
    }

    graph.add_node("/transform", plane);

    // PHYSICS

    gua::physics::CollisionShapeDatabase::add_shape("sphere", new gua::physics::SphereShape(0.25));
    gua::physics::CollisionShapeDatabase::add_shape("box", new gua::physics::BoxShape(gua::math::vec3(5, 1, 5)));

    auto floor_body = std::make_shared<gua::physics::RigidBodyNode>("floor_body", 0, 0.5, 0.7);
    auto floor_shape = std::make_shared<gua::physics::CollisionShapeNode>("floor_shape");
    floor_shape->data.set_shape("box");
    // graph.get_root()->add_child(floor_body);
    transform->add_child(floor_body);

    floor_body->add_child(floor_shape);
    physics->add_rigid_body(floor_body);

    std::vector<std::shared_ptr<gua::physics::RigidBodyNode>> balls;

    // add mouse interaction
    gua::utils::Trackball trackball(0.01, 0.002, 0.2);

    // setup rendering pipeline and window
    auto resolution = gua::math::vec2ui(960, 540);

    auto resolve_pass = std::make_shared<gua::ResolvePassDescription>();
    resolve_pass->background_mode(gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE);
    resolve_pass->tone_mapping_exposure(1.0f);

    auto camera = graph.add_node<gua::node::CameraNode>("/screen", "cam");
    camera->translate(0, 0, 2.0);
    camera->config.set_resolution(resolution);
    camera->config.set_left_screen_path("/screen");
    camera->config.set_right_screen_path("/screen");
    camera->config.set_scene_graph_name("main_scenegraph");
    camera->config.set_output_window_name("main_window");
    camera->config.set_enable_stereo(true);

    camera->get_pipeline_description()->get_resolve_pass()->tone_mapping_exposure(1.0f);

    auto window = std::make_shared<gua::GlfwWindow>();
    gua::WindowDatabase::instance()->add("main_window", window);

    //gua::WindowDatabase::instance()->add("main_window", window);
    window->config.set_enable_vsync(false);
    window->config.set_size( scm::math::vec2ui(2 * resolution.x, resolution.y) );
    window->config.set_resolution(scm::math::vec2ui(resolution.x * 2, resolution.y ) );
    window->config.set_right_position(scm::math::vec2ui(resolution.x , 0));
    window->config.set_left_resolution(scm::math::vec2ui(resolution.x , resolution.y));
    window->config.set_right_resolution(scm::math::vec2ui(resolution.x , resolution.y));

    if( 0 == int(sbs_mode) ) {
        window->config.set_stereo_mode(gua::StereoMode::SIDE_BY_SIDE);
    } else if( 1 == int(sbs_mode) ) {
        window->config.set_stereo_mode(gua::StereoMode::SIDE_BY_SIDE_SOFTWARE_MULTI_VIEW_RENDERING);
    } else if( 2 == int(sbs_mode) ) {
        window->config.set_stereo_mode(gua::StereoMode::SIDE_BY_SIDE_HARDWARE_MULTI_VIEW_RENDERING);        
    }

    window->on_resize.connect([&](gua::math::vec2ui const& new_size) {
        window->config.set_resolution(new_size);
        camera->config.set_resolution(new_size);
        screen->data.set_size(gua::math::vec2(0.001 * new_size.x, 0.001 * new_size.y));
    });
    window->on_move_cursor.connect([&](gua::math::vec2 const& pos) { trackball.motion(pos.x, pos.y); });
    window->on_button_press.connect(std::bind(mouse_button, std::ref(trackball), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    bool done = false;
    window->on_key_press.connect([&done](int key, int scancode, int action, int mods) {
        if(key == GLFW_KEY_ESCAPE)
            done = true;
    });

    gua::Renderer renderer;

    // application loop
    gua::events::MainLoop loop;

    gua::events::Ticker ticker(loop, 1.0 / 5000.0);


    double frame_time_avg = 0.0;
    double last_frame_time = -1.0;
    
    uint32_t valid_frames_recorded = 0;
    uint32_t waiting_frames_recorded = 0;
    uint32_t const num_frames_to_average = 1000;
    uint32_t const waiting_frames = num_frames_to_average/10;

    gua::Timer frame_timer;
    frame_timer.start();

    auto too_old = [&physics, &transform](std::shared_ptr<gua::physics::RigidBodyNode> const& b) {
        if(gua::math::get_translation(b->get_transform()).y < -5.0)
        {
            physics->remove_rigid_body(b);
            transform->remove_child(b);
            return true;
        }
        else
        {
            return false;
        }
    };

    ticker.on_tick.connect([&]() {
        // apply trackball matrix to object
        gua::math::mat4 modelmatrix = scm::math::make_translation(trackball.shiftx(), trackball.shifty(), trackball.distance()) * gua::math::mat4(trackball.rotation());

        transform->set_transform(modelmatrix);

        if(frame_timer.get_elapsed() > 0.02f)
        {
            frame_timer.reset();
            add_sphere(balls, transform, physics, loader);
        }

        balls.erase(std::remove_if(balls.begin(), balls.end(), too_old), balls.end());

        physics->synchronize(true);



        if(window->get_rendering_fps() > 0) {
          if(valid_frames_recorded < num_frames_to_average) {
            double current_frame_time = 1.0 / window->get_rendering_fps();
            if(last_frame_time != current_frame_time) {
              //std::cout << "draw time: " << 1.0 / window->get_rendering_fps() << std::endl;

              last_frame_time = current_frame_time;

              if(waiting_frames_recorded < waiting_frames) {
                ++waiting_frames_recorded;
              } else {
                frame_time_avg += current_frame_time;
                ++valid_frames_recorded;
              }
            }
          } else if(num_frames_to_average == valid_frames_recorded) {
            std::cout << "avg frame time: " << frame_time_avg / valid_frames_recorded << std::endl;
            ++valid_frames_recorded;
          }
        }


        if(done || window->should_close())
        {
            loop.stop();
        }
        else
        {
            renderer.queue_draw({&graph});
        }
    });

    loop.start();

    renderer.stop();
    window->close();

    return 0;
}
