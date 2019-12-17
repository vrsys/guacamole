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
#include <gua/renderer/BBoxPass.hpp> //to add a pass that visualizes bounding boxes in gua
#include <gua/renderer/DebugViewPass.hpp>

#include <gua/renderer/FullscreenColorBufferViewPass.hpp>
#include <gua/renderer/OcclusionCullingTriMeshPass.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/utils/Trackball.hpp>


// global variables
extern WASD_state cam_navigation_state;  //only declared in main - definition is in navigation.cpp

bool print_times = false;

bool show_bounding_boxes = false;
bool was_set_to_show_bounding_boxes = false;


bool print_scenegraph_once = false;

gua::OcclusionCullingStrategy current_culling_mode;


std::shared_ptr<gua::PipelineDescription> occlusion_culling_pipeline_description = std::make_shared<gua::PipelineDescription>();     
std::shared_ptr<gua::PipelineDescription> default_trimesh_pipeline_description = std::make_shared<gua::PipelineDescription>();     

std::string model_path = "data/objects/teapot.obj"; //place this object
std::string model_path_bus = "/opt/3d_models/vehicle/cars/autobus/auobus.obj"; //place this object
// std::string model_path_hairball_room = "data/objects/hairball_room.obj";
std::string model_path_town = "/opt/3d_models/architecture/medieval_harbour/town.obj"; //town obj path
// std::string model_path_plane = "data/objects/plane.obj"; //place this object
int32_t num_models_to_place = 1000; //place 1000 objects
float one_d_cube_size = 8.0; //8m*8m*8m cube for random object placement

int current_bb_level_to_visualize = -1;

void configure_pipeline_descriptions() {
    /* guacamole supports different rendering primitives - Triangle Meshes, LOD-PointClouds, Volumes, RGBD Streams, etc.
       It is our responsibility to keep the rendering stages minimal by describing the geometry that we actually plan to render
       
       In addition to the Geometry passes, we usually want to have shading in the scene. 
       For this, we add the LightVisibilityPass and ResolvePassDescription after the geometry passes.
       After the resolve pass, we may add post processing passes (or a DebugView)

    */

    // first pipe

#if 1 // USE OCCLUSION
    occlusion_culling_pipeline_description->add_pass(std::make_shared<gua::OcclusionCullingTriMeshPassDescription>());         // geometry pass for rendering trimesh files (obj, ply, ...)
    auto oc_tri_mesh_pass = occlusion_culling_pipeline_description->get_occlusion_culling_tri_mesh_pass();
    oc_tri_mesh_pass->set_occlusion_query_type(gua::OcclusionQueryType::Number_Of_Samples_Passed);
#else
    occlusion_culling_pipeline_description->add_pass(std::make_shared<gua::TriMeshPassDescription>());  
#endif
    occlusion_culling_pipeline_description->add_pass(std::make_shared<gua::BBoxPassDescription>());            // geometry pass for rendering bounding boxes of nodes
    //----------------------------------------------------------------------------------------
    occlusion_culling_pipeline_description->add_pass(std::make_shared<gua::LightVisibilityPassDescription>()); // treats the light as geometry and rasterizes it into a light buffer
    occlusion_culling_pipeline_description->add_pass(std::make_shared<gua::ResolvePassDescription>());         // resolves the shading in screen space
       // visualizes the GBuffer-content
    occlusion_culling_pipeline_description->add_pass(std::make_shared<gua::FullscreenColorBufferViewPassDescription>());       // visualizes the GBuffer-content
    //occlusion_culling_pipeline_description->add_pass(std::make_shared<gua::DebugViewPassDescription>());

    occlusion_culling_pipeline_description->get_full_screen_color_buffer_view_pass()->enable(false);
    // configure the resolve pass
    occlusion_culling_pipeline_description->get_resolve_pass()->tone_mapping_exposure(3.f);
    occlusion_culling_pipeline_description->get_resolve_pass()->tone_mapping_method(gua::ResolvePassDescription::ToneMappingMethod::UNCHARTED);

    occlusion_culling_pipeline_description->get_resolve_pass()->ssao_intensity(4.0);
    occlusion_culling_pipeline_description->get_resolve_pass()->ssao_enable(true);
    occlusion_culling_pipeline_description->get_resolve_pass()->ssao_falloff(1.0);
    occlusion_culling_pipeline_description->get_resolve_pass()->ssao_radius(4.0);
    occlusion_culling_pipeline_description->get_resolve_pass()->environment_lighting_mode(gua::ResolvePassDescription::EnvironmentLightingMode::AMBIENT_COLOR);
    occlusion_culling_pipeline_description->get_resolve_pass()->environment_lighting_texture("data/textures/skymap_5k.jpg");

    occlusion_culling_pipeline_description->get_resolve_pass()->background_mode(gua::ResolvePassDescription::BackgroundMode::SKYMAP_TEXTURE);
    occlusion_culling_pipeline_description->get_resolve_pass()->background_texture("data/textures/skymap_5k.jpg");

}


void adjust_arguments(int& argc, char**& argv)
{
    char* argv_tmp[] = {argv[0], NULL};
    int argc_tmp = sizeof(argv_tmp) / sizeof(char*) - 1;
    argc = argc_tmp;
    argv = argv_tmp;
}


// helper for command line parsing
void parse_model_from_cmd_line(int argc, char** argv)
{

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
            num_models_to_place = std::atoi(argv[2]);

            if(argc > 3) {
                one_d_cube_size = std::atof(argv[3]);
            }
        }
    }
    gua::Logger::LOG_MESSAGE << log_message_model_string + model_path << std::endl;

    
}


void print_keyboard_controls() {
    std::cout << "Keyboard Controls:" << std::endl;
    std::cout << "==================" << std::endl;
    std::cout << "W/S: move camera forward/backward" << std::endl;
    std::cout << "A/D: move camera left/right" << std::endl;

    std::cout << "I/K: rotate camera around right-axis" << std::endl;
    std::cout << "J/L: rotate camera around up-axis" << std::endl;

    std::cout << "V: toggle depth complexity visualization" << std::endl;
    std::cout << "B: toggle bounding boxes" << std::endl;
    std::cout << "P: print scenegraph (once)" << std::endl;
    std::cout << "T: toggle printing of elapsed time (application & render time)" << std::endl;

    std::cout << std::endl;
}



int main(int argc, char** argv)
{
    // contains the model's obj path - path to the teapot if no argument is supplied, otherwise it is the path that the user provides
    parse_model_from_cmd_line(argc, argv);

    // guacamole is rewriting the arguments, so we need this additiona call
    adjust_arguments(argc, argv);
    // init the guacamole backend
    gua::init(argc, argv);

    print_keyboard_controls();

    // initialize an empty scene graph, only containing a root node. We will attach nodes to other nodes to build the entire graph
    gua::SceneGraph graph("main_scenegraph");

    // for every type of geometry, there is a loader that knows how to deal with it
    gua::TriMeshLoader loader;

    auto transform_node = graph.add_node<gua::node::TransformNode>("/", "transform_node");

    auto occlusion_group_node = graph.add_node<gua::node::OcclusionCullingGroupNode>("/transform_node", "occlusion_group_node");

    // add a cluster of pseudorandomly placed objects in the scene. See: scene_utils.cpp 
    //place_objects_randomly(model_path, num_models_to_place, one_d_cube_size, occlusion_group_node);


    //create_simple_debug_scene(occlusion_group_node);
    // create_city_scene(occlusion_group_node);
    create_simple_demo_scene(occlusion_group_node);
    //create_occlusion_scene(model_path_plane, model_path_town, occlusion_group_node);
    
    occlusion_group_node->regroup_children();
    //occlusion_group_node->median_regroup_children();


    // add a point light source to the scene and attach it to the tranform node
    auto light_node = graph.add_node<gua::node::LightNode>("/transform_node", "light_node");
    light_node->data.set_type(gua::node::LightNode::Type::POINT);
    light_node->data.brightness = 35000.0f;
    light_node->scale(1200.f);
    light_node->translate(0.f, 50.f, 0.f);


    // we put a transform node above camera and screen, because we want to keep the relative orientation and position between constant
    // when we navigate, we change the transformation of the navigation node instead the transformation of the camera!
    auto navigation_node = graph.add_node<gua::node::TransformNode>("/", "navigation_node");


    // Screen nodes are special nodes which allow us to model a "window into the virtual world"
    auto screen = graph.add_node<gua::node::ScreenNode>("/navigation_node", "screen_node");
    // setting the size of the screen metrically correct allows us to perceive virtual objects 1:1. Here: 1.92m by 1.08 meters (powerwall)
    screen->data.set_size(gua::math::vec2(1.92f, 1.08f));
    screen->translate(0, 0, -3.0);


    // add mouse interaction
    gua::utils::Trackball trackball(0.01, 0.002, 0.2);

    // setup rendering pipeline and window
    auto resolution = gua::math::vec2ui(2*1280, 2*720);


    configure_pipeline_descriptions();

    //add a camera node. Without a camera, we can not render
    auto camera_node = graph.add_node<gua::node::CameraNode>("/navigation_node", "cam_node");
    //we just leave the camera in 0, 0, 0 in its local coordinates
    camera_node->translate(0, 0, 0);
    camera_node->config.set_resolution(resolution);
    //we tell the camera to which screen it belongs (camera position and screen boundaries define a frustum)
    camera_node->config.set_screen_path("/navigation_node/screen_node");
    //we associate the camera with our scenegraph
    camera_node->config.set_scene_graph_name("main_scenegraph");
    //and finally tell it in which window to render
    camera_node->config.set_output_window_name("main_window");
    //for now, we do not render in stereo
    camera_node->config.set_enable_stereo(false);
    //we associate our current pipeline description with the camera
    camera_node->set_pipeline_description(occlusion_culling_pipeline_description);


    //create the window and configure it with correct resolution, stereo mode and window events
    auto window = std::make_shared<gua::GlfwWindow>();
    gua::WindowDatabase::instance()->add("main_window", window);

    window->config.set_enable_vsync(false);
    window->config.set_size(resolution);
    window->config.set_resolution(resolution);
    window->config.set_stereo_mode(gua::StereoMode::MONO);

    window->on_resize.connect([&](gua::math::vec2ui const& new_size) {
        window->config.set_resolution(new_size);
        camera_node->config.set_resolution(new_size);
        screen->data.set_size(gua::math::vec2(0.001 * new_size.x, 0.001 * new_size.y));
    });
    window->on_move_cursor.connect([&](gua::math::vec2 const& pos) { trackball.motion(pos.x, pos.y); });
    window->on_button_press.connect(std::bind(mouse_button, std::ref(trackball), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    window->on_key_press.connect(
        std::bind(key_press, std::ref(*(camera_node->get_pipeline_description())), std::ref(graph), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));


    // the renderer triggers the execution of the pipeline according t our description
    gua::Renderer renderer;

    // application loop
    gua::events::MainLoop loop;
    // set up the ticker that tries to executes the main loop every of 1.0 / 500.0 s. T
    // he application thread triggers the rendering, so our rendering framerate can never be higher than our application framerate
    gua::events::Ticker ticker(loop, 1.0 / 500.0);


    // define the callback for the ticker - this is basically the content of the mainloop
    ticker.on_tick.connect([&]() {
        // apply trackball matrix to object
        // we update the model transformatin according to the internal state of the trackball.
        gua::math::mat4 transform_node_model_matrix = scm::math::make_translation(trackball.shiftx(), trackball.shifty(), trackball.distance()) * gua::math::mat4(trackball.rotation());

        // set transform completely overrides the old transformation of this node
        transform_node->set_transform(transform_node_model_matrix);
        //translate, rotate, and scale apply a corresponding matrix to the current matrix stored in the node
        transform_node->translate(0.0f, 0.0f, -5.0f);


        //we ask the renderer for the currently averaged applicatin fps and the window for the rendering fps
        float application_fps = renderer.get_application_fps();

        if(print_times) {
            print_draw_times(renderer, window);
        }

        float elapsed_application_time_milliseconds = 0.0;

        if(application_fps > 0.0f) {
            elapsed_application_time_milliseconds = 1.0 / application_fps;
        }


        // apply changes to the current navigation node, such that the scene graph will see the change
        update_cam_matrix(camera_node, navigation_node, elapsed_application_time_milliseconds);

        if(window->should_close())
        {
            // clean up
            renderer.stop();
            window->close();
            loop.stop();
        }
        else
        {
            if(show_bounding_boxes != was_set_to_show_bounding_boxes) {

                show_scene_bounding_boxes(occlusion_group_node, show_bounding_boxes, current_bb_level_to_visualize);

                was_set_to_show_bounding_boxes = show_bounding_boxes;
            }

            if(print_scenegraph_once) {
                print_graph(graph.get_root());

                print_scenegraph_once = false;
            }


            // enqueue the scene graph in the gua rendering queue
            renderer.queue_draw({&graph});
        }
    });


    // start the rendering loop
    loop.start();

    return 0;
}