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

#include <gua/guacamole.hpp>
#include <gua/utils/Trackball.hpp>

#define COUNT 5

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
    };

    switch(action)
    {
    case 0:
        state = gua::utils::Trackball::released;
        break;
    case 1:
        state = gua::utils::Trackball::pressed;
        break;
    };

    trackball.mouse(button, state, trackball.posx(), trackball.posy());
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

  NUM_SIDE_BY_SIDE_MODES = 2
};


Side_By_Side_Mode sbs_mode = Side_By_Side_Mode::DEFAULT_SIDE_BY_SIDE;

std::string parse_model_from_cmd_line(int argc, char** argv)
{
    std::string model_path = "data/objects/teapot.obj";

    std::string log_message_model_string = "";
    if(argc < 2)
    {
        gua::Logger::LOG_MESSAGE << argv[0] << ": Did not provide any model file." << std::endl;
        log_message_model_string = "Using default model path: " + model_path;
    }
    else
    {
        model_path = argv[1];
        log_message_model_string = std::string(argv[0]) + ": Using provided model path:";

        if(argc > 2) {

            sbs_mode = Side_By_Side_Mode(std::atoi(argv[2]));
            gua::Logger::LOG_MESSAGE << "Setting side by side mode to " << (int(sbs_mode) == 0 ? " SIDE_BY_SIDE " : "SIDE_BY_SIDE_SOFTWARE_MULTI_VIEW_RENDERING") << std::endl; 
        }

    }
    gua::Logger::LOG_MESSAGE << log_message_model_string + model_path << std::endl;

    return model_path;
}

int main(int argc, char** argv)
{
    std::string const model_path = parse_model_from_cmd_line(argc, argv);
    adjust_arguments(argc, argv);
    // initialize guacamole
    gua::init(argc, argv);

    // setup scene
    gua::SceneGraph graph("main_scenegraph");

    gua::TriMeshLoader loader;
    auto add_mesh = [&](int x, int y, int z) {
        auto t = graph.add_node<gua::node::TransformNode>("/", "mesh_" + std::to_string(x) + "_" + std::to_string(y));
        t->scale(0.5);
        t->translate((x - COUNT * 0.5 + 0.5) / 2, (y - COUNT * 0.5 + 0.5) / 2, (z - COUNT * 0.5 + 0.5) / 2);

        auto mesh(loader.create_geometry_from_file("example_model", model_path, gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));
        t->add_child(mesh);
    };

    for(int x(0); x < COUNT; ++x)
    {
        for(int y(0); y < COUNT; ++y)
        {
            for(int z(0); z < COUNT; ++z)
            {
                add_mesh(x, y, z);
            }
        }
    }

    auto resolution = gua::math::vec2ui(1920, 1080);

    auto light = graph.add_node<gua::node::LightNode>("/", "light");
    light->data.set_type(gua::node::LightNode::Type::POINT);
    light->scale(4.4f);
    light->translate(1.f, 0.f, 2.f);

    auto screen = graph.add_node<gua::node::ScreenNode>("/", "screen");
    screen->data.set_size(gua::math::vec2(0.001 * resolution.x, 0.001 * resolution.y));
    screen->translate(0, 0, 1.0);

    auto camera = graph.add_node<gua::node::CameraNode>("/screen", "cam");
    camera->translate(0, 0, 2.0);
    camera->config.set_resolution(resolution);
    camera->config.set_left_screen_path("/screen");
    camera->config.set_right_screen_path("/screen");
    camera->config.set_scene_graph_name("main_scenegraph");
    camera->config.set_output_window_name("main_window");
    camera->config.set_enable_stereo(true);
    camera->get_pipeline_description()->get_resolve_pass()->tone_mapping_exposure(0.8f);

    auto plane_transform = graph.add_node<gua::node::TransformNode>("/", "plane_transform");

    auto add_clipping_plane = [&](gua::math::vec3 const& pos, gua::math::vec4 const& rot) {
        auto clipping_plane = graph.add_node<gua::node::ClippingPlaneNode>("/plane_transform", "clipping_plane");
        clipping_plane->rotate(rot.x, rot.y, rot.z, rot.w);
        clipping_plane->translate(pos);
        auto plane(loader.create_geometry_from_file("plane", "data/objects/plane.obj", gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));
        plane->rotate(-90, 1, 0, 0);
        // clipping_plane->add_child(plane);
    };

    add_clipping_plane(gua::math::vec3(0, 0, -0.5), gua::math::vec4(180, 1, 0, 0));
    add_clipping_plane(gua::math::vec3(0, 0, 0.5), gua::math::vec4(0, 0, 0, 0));
    add_clipping_plane(gua::math::vec3(0, 0.5, 0), gua::math::vec4(-90, 1, 0, 0));
    add_clipping_plane(gua::math::vec3(0, -0.5, 0), gua::math::vec4(90, 1, 0, 0));
    add_clipping_plane(gua::math::vec3(-0.5, 0, 0), gua::math::vec4(-90, 0, 1, 0));
    add_clipping_plane(gua::math::vec3(0.5, 0, 0), gua::math::vec4(90, 0, 1, 0));

    // auto clipping_plane_2 = graph.add_node<gua::node::ClippingPlaneNode>("/plane_transform", "clipping_plane_2");
    // clipping_plane_2->rotate(90, 1, 0, 0);
    // clipping_plane_2->translate(0, 0.5, 0);

    // add mouse interaction
    gua::utils::Trackball trackball(0.01, 0.002, 0.2);



    auto window = std::make_shared<gua::GlfwWindow>();
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
    }

    gua::WindowDatabase::instance()->add("main_window", window);
    //window->config.set_enable_vsync(true);
    //window->config.set_size(resolution);
    //window->config.set_resolution(resolution);

    //window->on_resize.connect([&](gua::math::vec2ui const& new_size) {
    //    window->config.set_resolution(new_size);
    //    camera->config.set_resolution(new_size);
    //    screen->data.set_size(gua::math::vec2(0.001 * new_size.x, 0.001 * new_size.y));
    //});

    window->on_move_cursor.connect([&](gua::math::vec2 const& pos) { trackball.motion(pos.x, pos.y); });
    window->on_button_press.connect(std::bind(mouse_button, std::ref(trackball), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    window->open();

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
    ticker.on_tick.connect([&]() {
        // apply trackball matrix to object
        gua::math::mat4 modelmatrix = scm::math::make_translation(gua::math::float_t(trackball.shiftx()), gua::math::float_t(trackball.shifty()), gua::math::float_t(trackball.distance())) *
                                      gua::math::mat4(trackball.rotation());

        plane_transform->set_transform(modelmatrix);

        window->process_events();


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

        if(window->should_close())
        {
            renderer.stop();
            window->close();
            loop.stop();
        }
        else
        {
            renderer.queue_draw({&graph});
        }
    });

    loop.start();

    return 0;
}
