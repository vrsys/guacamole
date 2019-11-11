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

#include "glfw_callbacks.hpp"
#include "scene_utils.hpp"
#include "navigation.hpp"

#include <functional>

#include <gua/guacamole.hpp>
#include <gua/renderer/TriMeshLoader.hpp>
#include <gua/renderer/ToneMappingPass.hpp>
#include <gua/renderer/DebugViewPass.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/utils/Trackball.hpp>


// global state variables
extern WASD_state cam_navigation_state;  //only declared in main - definition is in navigation.cpp


void adjust_arguments(int& argc, char**& argv)
{
    char* argv_tmp[] = {argv[0], NULL};
    int argc_tmp = sizeof(argv_tmp) / sizeof(char*) - 1;
    argc = argc_tmp;
    argv = argv_tmp;
}


// helper for command line parsing
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
    }
    gua::Logger::LOG_MESSAGE << log_message_model_string + model_path << std::endl;

    return model_path;
}

int main(int argc, char** argv)
{
    // contains the model's obj path - path to the teapot if no argument is supplied, otherwise it is the path that the user provides
    std::string const model_path = parse_model_from_cmd_line(argc, argv);

    // guacamole is rewriting the arguments, so we need this additiona call
    adjust_arguments(argc, argv);
    // init the guacamole backend
    gua::init(argc, argv);

    // initialize an empty scene graph, only containing a root node. We will attach nodes to other nodes to build the entire graph
    gua::SceneGraph graph("main_scenegraph");

    // for every type of geometry, there is a loader that knows how to deal with it
    gua::TriMeshLoader loader;

    // we provide our model with the gua-default material
    auto model_material(gua::MaterialShaderDatabase::instance()->lookup("gua_default_material")->make_new_material());


    model_material->set_render_wireframe(false);
    model_material->set_show_back_faces(false);


    auto transform_node = graph.add_node<gua::node::TransformNode>("/", "transform");
    transform_node->set_draw_bounding_box(false);
    auto teapot_model(
        loader.create_geometry_from_file("teapot", model_path, model_material,  gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE | gua::TriMeshLoader::MAKE_PICKABLE));

    auto scene = graph.add_node<gua::node::TransformNode>("/transform", "scene");
    scene->set_draw_bounding_box(true);

    graph.add_node("/transform/scene", teapot_model);
    teapot_model->set_draw_bounding_box(true);

    auto teapot_model2(loader.create_geometry_from_file("teapot_2", model_path, model_material,  gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE | gua::TriMeshLoader::MAKE_PICKABLE));
    teapot_model2->translate(1.0, 0.0, 0.0);
    teapot_model2->set_draw_bounding_box(true);
    graph.add_node("/transform/scene", teapot_model2);


    // add a cluster of pseudorandomly placed objects in the scene. See: scene_utils.cpp 
    place_objects_randomly(model_path, scene);

    // add a point light source to the scene and attach it to the tranform node
    auto light = graph.add_node<gua::node::LightNode>("/transform", "light");
    light->data.set_type(gua::node::LightNode::Type::POINT);
    light->data.brightness = 150.0f;
    light->scale(12.f);
    light->translate(-3.f, 5.f, 5.f);


    // we put a transform node above camera and screen, because we want to keep the relative orientation and position between constant
    // when we navigate, we change the transformation of the navigation node instead the transformation of the camera!
    auto navigation_node = graph.add_node<gua::node::TransformNode>("/", "navigation_node");


    // Screen nodes are special nodes which allow us to model a "window into the virtual world"
    auto screen = graph.add_node<gua::node::ScreenNode>("/navigation_node", "screen");
    // setting the size of the screen metrically correct allows us to perceive virtual objects 1:1. Here: 1.92m by 1.08 meters (powerwall)
    screen->data.set_size(gua::math::vec2(1.92f, 1.08f));
    screen->translate(0, 0, -3.0);


    // add mouse interaction
    gua::utils::Trackball trackball(0.01, 0.002, 0.2);

    // setup rendering pipeline and window
    auto resolution = gua::math::vec2ui(1920, 1080);


    auto resolve_pass = std::make_shared<gua::ResolvePassDescription>();
    resolve_pass->background_mode(gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE);
    resolve_pass->tone_mapping_exposure(1.0f);

    auto camera = graph.add_node<gua::node::CameraNode>("/navigation_node", "cam");
    camera->translate(0, 0, 0);
    camera->config.set_resolution(resolution);
    camera->config.set_screen_path("/navigation_node/screen");
    camera->config.set_scene_graph_name("main_scenegraph");
    camera->config.set_output_window_name("main_window");
    camera->config.set_enable_stereo(false);


    camera->get_pipeline_description()->get_resolve_pass()->tone_mapping_exposure(1.0f);
    camera->get_pipeline_description()->add_pass(std::make_shared<gua::DebugViewPassDescription>());

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
    window->on_move_cursor.connect([&](gua::math::vec2 const& pos) { trackball.motion(pos.x, pos.y); });
    window->on_button_press.connect(std::bind(mouse_button, std::ref(trackball), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    window->on_key_press.connect(
        std::bind(key_press, std::ref(*(camera->get_pipeline_description())), std::ref(graph), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));


    gua::Renderer renderer;

    // application loop
    gua::events::MainLoop loop;
    // set up the ticker that tries to executes the main loop every of 1.0 / 500.0 s. T
    // he application thread triggers the rendering, so our rendering framerate can never be higher than our application framerate
    gua::events::Ticker ticker(loop, 1.0 / 500.0);


    // define the callback for the ticker - this is basically the content of the mainloop

    ticker.on_tick.connect([&]() {

        std::cout << std::endl;

        // apply trackball matrix to object
        gua::math::mat4 transform_node_model_matrix = scm::math::make_translation(trackball.shiftx(), trackball.shifty(), trackball.distance()) * gua::math::mat4(trackball.rotation());


        transform_node->set_transform(transform_node_model_matrix);
        transform_node->translate(0.0f, 0.0f, -5.0f);




        float application_fps = renderer.get_application_fps();
        float rendering_fps   = window->get_rendering_fps();


        float elapsed_application_time_milliseconds = 0.0;

        if(application_fps > 0.0f) {
            elapsed_application_time_milliseconds = 1.0 / application_fps;
        }

        std::cout << "elapsed application time ms: " << elapsed_application_time_milliseconds << " ms" << std::endl;

        float elapsed_rendering_time_milliseconds = 0.0;

        if(rendering_fps > 0.0f) {
            elapsed_rendering_time_milliseconds = 1.0 / rendering_fps;
        }

        std::cout << "elapsed rendering time ms: " << elapsed_rendering_time_milliseconds << " ms" << std::endl;


        // apply changes to the current navigation node, such that the scene graph will see the change
        update_cam_matrix(camera, navigation_node, elapsed_application_time_milliseconds);

        if(window->should_close())
        {
            // clean up
            renderer.stop();
            window->close();
            loop.stop();
        }
        else
        {
            // enqueue the scene graph in the gua rendering queue
            renderer.queue_draw({&graph});
        }
    });


    // start the rendering loop
    loop.start();

    return 0;
}