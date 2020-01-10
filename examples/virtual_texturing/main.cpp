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
#include "navigation.hpp"

#include <functional>
#include <memory>
#include <algorithm>

#include <gua/guacamole.hpp>
#include <gua/renderer/TriMeshLoader.hpp>
#include <gua/renderer/DebugViewPass.hpp>
#include <gua/renderer/ToneMappingPass.hpp>
#include <gua/renderer/SSAAPass.hpp>
#include <gua/renderer/BBoxPass.hpp>
#include <gua/renderer/ScreenGrabPass.hpp>
#include <gua/renderer/DebugViewPass.hpp>
#include <scm/gl_util/manipulators/trackball_manipulator.h>

#include <gua/virtual_texturing/VTBackend.hpp>
#include <lamure/vt/VTConfig.h>
#include <lamure/ren/model_database.h>
#include <gua/renderer/PBSMaterialFactory.hpp>

extern WASD_state cam_navigation_state;

bool print_frame_times = true;

int main(int argc, char** argv)
{
    std::string model_file = "";
    std::string atlas_file = "";
    std::vector<std::string> model_files;
    if (argc > 2) {
        for(int i = 1; i < (argc - 1); ++i){
            model_files.push_back(argv[i]);
        }
      
      atlas_file = argv[argc - 1];
    }
    else if(2 == argc){ // only a single obj is provided
      model_file = argv[1];
    }
    else {
      std::cout << "usage: " << argv[0] << " filename_vt.obj ... filename.atlas" << std::endl;
      std::cout << "or" << std::endl;
      std::cout << "usage: " << argv[0] << " filename.obj" << std::endl;
      return 0;  
    }
    
    char* argv_tmp[] = {"./example-virtual_texturing", nullptr};
    int argc_tmp = sizeof(argv_tmp) / sizeof(char*) - 1;
    
    // initialize guacamole
    gua::init(argc_tmp, argv_tmp);

    // setup scenegraph
    gua::SceneGraph graph("main_scenegraph");

    auto scene_transform = graph.add_node<gua::node::TransformNode>("/", "transform");
    auto model_transform = graph.add_node<gua::node::TransformNode>("/transform", "model_transform");
    auto view_transform = graph.add_node<gua::node::TransformNode>("/", "view_transform");


    auto vt_mat_wo_early_depth_test = gua::PBSMaterialFactory::create_material((gua::PBSMaterialFactory::Capabilities)(
        gua::PBSMaterialFactory::Capabilities::ROUGHNESS_VALUE |
        gua::PBSMaterialFactory::Capabilities::METALNESS_VALUE |
        gua::PBSMaterialFactory::Capabilities::EMISSIVITY_VALUE) );

    auto vt_mat_with_early_depth_test = gua::PBSMaterialFactory::create_material((gua::PBSMaterialFactory::Capabilities)(
        gua::PBSMaterialFactory::Capabilities::ROUGHNESS_VALUE |
        gua::PBSMaterialFactory::Capabilities::METALNESS_VALUE |
        gua::PBSMaterialFactory::Capabilities::EMISSIVITY_VALUE) );

    std::vector<std::shared_ptr<gua::node::TriMeshNode>> model_nodes; 



    if(atlas_file != ""){
        gua::VTBackend::set_physical_texture_size(2048);
        gua::VTBackend::set_update_throughput_size(4);
        gua::VTBackend::set_ram_cache_size(32768);


        
        vt_mat_wo_early_depth_test->set_uniform("Metalness", 0.25f);
        vt_mat_wo_early_depth_test->set_uniform("Roughness", 0.75f);
        vt_mat_wo_early_depth_test->set_uniform("Emissivity", 0.5f);
        vt_mat_wo_early_depth_test->set_uniform("Color", gua::math::vec4f(.5f, .5f, .7f, 1.0f));
        vt_mat_wo_early_depth_test->set_uniform("vt_test", atlas_file);
        vt_mat_wo_early_depth_test->set_enable_virtual_texturing(true);
        vt_mat_wo_early_depth_test->set_enable_early_fragment_test(true);
        vt_mat_wo_early_depth_test->set_show_back_faces(false);

        vt_mat_with_early_depth_test->set_uniform("Metalness", 0.25f);
        vt_mat_with_early_depth_test->set_uniform("Roughness", 0.75f);
        vt_mat_with_early_depth_test->set_uniform("Emissivity", 0.5f);
        vt_mat_with_early_depth_test->set_uniform("Color", gua::math::vec4f(.5f, .5f, .7f, 1.0f));
        vt_mat_with_early_depth_test->set_uniform("vt_test", atlas_file);
        vt_mat_with_early_depth_test->set_enable_virtual_texturing(true);
        vt_mat_with_early_depth_test->set_enable_early_fragment_test(true);
        vt_mat_with_early_depth_test->set_show_back_faces(false);


        gua::TriMeshLoader loader;

        for(unsigned i = 0; i <  model_files.size(); ++i){

            std::cout << "start loading " << model_files[i] << std::endl;
            auto model_node(loader.create_geometry_from_file("model_node" /*should be unique*/ , model_files[i].c_str(), vt_mat_wo_early_depth_test, /*gua::TriMeshLoader::OPTIMIZE_GEOMETRY*/0));
            std::cout << model_files[i] << "...ready with " << model_node->get_children().size() << " children" << std::endl;
            graph.add_node("/transform/model_transform", model_node);

            model_nodes.push_back(std::dynamic_pointer_cast<gua::node::TriMeshNode>(model_node));


            if(0 == i){
                //center camera on model (only works without NORMALIZE_POS and NORMALIZE_SCALE)
                auto bb = model_node->get_bounding_box();
                auto model_dim = scm::math::length(bb.max - bb.min);
                auto center = (bb.max + bb.min) / 2.f; 
                view_transform->translate(center.x, center.y, center.z + model_dim); 

                cam_navigation_state.accumulated_translation_world_space = scm::math::make_translation(center.x, center.y, center.z + model_dim);
            }

        }


       

#if 0
        model_file = "/mnt/confidential/profiling/shared_atlas_test/04_Ottoaltar_LQ/04_Ottoaltar_LQ_vt.obj";
        std::cout << "start loading " << model_file << std::endl;
        
        auto model_node2(loader.create_geometry_from_file("model_node2", model_file.c_str(), vt_mat, 0));
        std::cout << model_file << " ready" << std::endl;
        graph.add_node("/transform/model_transform", model_node2);
#endif

    }
    else{
#if 1
        // this has to be profiled
        auto load_mat = [](std::string const& file) {
            auto desc(std::make_shared<gua::MaterialShaderDescription>());
            desc->load_from_file(file);
            auto shader(std::make_shared<gua::MaterialShader>(file, desc));
            gua::MaterialShaderDatabase::instance()->add(shader);
            return shader->make_new_material();
        };

        auto mat_textured(load_mat("data/materials/textured.gmd"));
#endif
        std::cout << "start loading " << model_file << std::endl;
        gua::TriMeshLoader loader;
        auto model_node(loader.create_geometry_from_file("model_node", model_file.c_str(), mat_textured, gua::TriMeshLoader::LOAD_MATERIALS));
        std::cout << model_file << " ready" << std::endl;
        graph.add_node("/transform/model_transform", model_node);

        //center camera on model (only works without NORMALIZE_POS and NORMALIZE_SCALE)
        auto bb = model_node->get_bounding_box();
        auto model_dim = scm::math::length(bb.max - bb.min);
        auto center = (bb.max + bb.min) / 2.f; 
        view_transform->translate(center.x, center.y, center.z + model_dim);
    }
    


    // create a lightsource
    auto light_transform = graph.add_node<gua::node::TransformNode>("/transform", "light_transform");

    auto light = graph.add_node<gua::node::LightNode>("/transform/light_transform", "light");
    light->data.set_type(gua::node::LightNode::Type::POINT);
    light->data.set_enable_shadows(false);

    light->data.set_shadow_map_size(512);
    light->data.set_shadow_offset(0.01f);
    light->data.set_softness(3.f);
    light->data.set_shadow_far_clipping_in_sun_direction(2.0f);
    light->data.set_shadow_near_clipping_in_sun_direction(0.1f);

    light->data.brightness = 40.0f;
    light->scale(8.0f);
    light->translate(0.f, 1.5f, 2.0f);

    auto light2 = graph.add_node<gua::node::LightNode>("/transform/light_transform", "light2");
    light2->data.set_type(gua::node::LightNode::Type::POINT);
    light2->data.set_enable_shadows(false);

    light2->data.set_shadow_map_size(512);
    light2->data.set_shadow_offset(0.01f);
    light2->data.set_softness(3.f);
    light2->data.set_shadow_far_clipping_in_sun_direction(2.0f);
    light2->data.set_shadow_near_clipping_in_sun_direction(0.1f);

    light2->data.brightness = 40.0f;
    light2->scale(8.0f);
    light2->translate(0.f, 1.5f, -2.0f);


    






    auto screen = graph.add_node<gua::node::ScreenNode>("/view_transform", "screen");
    screen->data.set_size(gua::math::vec2(1.92f, 1.08f));
    screen->translate(0, 0, 1.0);

    // add mouse interaction
    scm::gl::trackball_manipulator trackball;
    trackball.transform_matrix() * scm::math::make_translation(0.01f, 0.002f, 0.2f);
    trackball.dolly(0.5f);
    float dolly_sens = 2.5f;
    gua::math::vec2 trackball_init_pos(0.f);
    gua::math::vec2 last_mouse_pos(0.f);
    int button_state = -1;

    // setup rendering pipeline and window
    auto resolution = gua::math::vec2ui(1920*2, 1080*2);


    auto camera = graph.add_node<gua::node::CameraNode>("/view_transform/screen", "cam");
    camera->translate(0.f, 0.f, 2.5f);
    

    camera->config.set_resolution(resolution);

    // use close near plane to allow inspection of details
    camera->config.set_near_clip(0.01f);
    camera->config.set_far_clip(100.0f);
    camera->config.set_screen_path("/view_transform/screen");
    camera->config.set_scene_graph_name("main_scenegraph");
    camera->config.set_output_window_name("main_window");
    camera->config.set_enable_stereo(false);
    camera->config.set_enable_frustum_culling(true);

    auto pipe = std::make_shared<gua::PipelineDescription>();
    pipe->add_pass(std::make_shared<gua::TriMeshPassDescription>());
    pipe->add_pass(std::make_shared<gua::LightVisibilityPassDescription>());
    pipe->add_pass(std::make_shared<gua::ResolvePassDescription>());

    pipe->get_resolve_pass()->tone_mapping_exposure(1.f);
    pipe->get_resolve_pass()->tone_mapping_method(gua::ResolvePassDescription::ToneMappingMethod::UNCHARTED);

    pipe->get_resolve_pass()->background_mode(gua::ResolvePassDescription::BackgroundMode::SKYMAP_TEXTURE);
    pipe->get_resolve_pass()->background_texture("data/textures/envlightmap.jpg");

    // pipe->set_abuffer_size(1920);
    pipe->set_enable_abuffer(false);

    // pipe->add_pass(std::make_shared<gua::DebugViewPassDescription>());

    camera->set_pipeline_description(pipe);

    // init window and window behaviour
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

    // trackball controls
    bool drag_mode = false;
    window->on_move_cursor.connect([&](gua::math::vec2 const& pos) {
        float nx = 2.f * float(pos.x - (resolution.x / 2)) / float(resolution.x);
        float ny = -2.f * float(resolution.y - pos.y - (resolution.y / 2)) / float(resolution.y);
        if(button_state != -1)
        {
            if(drag_mode)
            {
                auto ssize = screen->data.get_size();
                gua::math::vec3 trans = gua::math::vec3(ssize.x * (nx - last_mouse_pos.x), ssize.y * (ny - last_mouse_pos.y), 0.f);
                auto object_trans = scm::math::inverse(screen->get_world_transform()) * model_transform->get_world_transform();
                model_transform->set_world_transform(screen->get_world_transform() * scm::math::make_translation(trans) * object_trans);
            }
            else
            {
                if(button_state == 0)
                { // left
                    trackball.rotation(trackball_init_pos.x, trackball_init_pos.y, nx, ny);
                }
                if(button_state == 1)
                { // right
                    trackball.dolly(dolly_sens * 0.5f * (ny - trackball_init_pos.y));
                }
                if(button_state == 2)
                { // middle
                    float f = dolly_sens < 1.0f ? 0.02f : 0.3f;
                    trackball.translation(f * (nx - trackball_init_pos.x), f * (ny - trackball_init_pos.y));
                }
                trackball_init_pos.x = nx;
                trackball_init_pos.y = ny;
            }
        }
        last_mouse_pos.x = nx;
        last_mouse_pos.y = ny;
    });

    window->on_button_press.connect([&](int mousebutton, int action, int mods) {
        if(action == 1)
        {
            drag_mode = mods == 1;
            trackball_init_pos = last_mouse_pos;
            button_state = mousebutton;
        }
        else
            button_state = -1;
    });

    window->on_key_press.connect(
        std::bind(key_press, std::ref(*(camera->get_pipeline_description())), std::ref(graph), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));


    window->open();

    if (atlas_file != "") {
      auto vt_backend = &gua::VTBackend::get_instance();
      vt_backend->add_camera(camera);
      vt_backend->start_backend();
    }

    gua::Renderer renderer;

    // application loop
    gua::events::MainLoop loop;
    gua::events::Ticker ticker(loop, 1.0 / 1000.0);
    std::size_t framecount = 0;

    ticker.on_tick.connect([&]() {
        screen->set_transform(scm::math::inverse(gua::math::mat4(trackball.transform_matrix())));

        //we ask the renderer for the currently averaged applicatin fps and the window for the rendering fps
        float application_fps = renderer.get_application_fps();


        float elapsed_application_time_milliseconds = 0.0;

        if(application_fps > 0.0f) {
            elapsed_application_time_milliseconds = 1.0 / application_fps;
        }

        update_cam_matrix(camera, view_transform, elapsed_application_time_milliseconds);

        light_transform->rotate(0.05, 0.f, 1.f, 0.f);
        window->process_events();
        if(window->should_close())
        {
            if (atlas_file != "") {
              auto vt_backend = &gua::VTBackend::get_instance();
              vt_backend->stop_backend();
            }
            renderer.stop();
            window->close();
            loop.stop();
        }
        else
        {
            renderer.queue_draw({&graph});
            if(framecount++ % 200 == 0)
            {
                if(print_frame_times) {
                    std::cout << "FPS: " << window->get_rendering_fps() << "  Frametime: " << 1000.f / window->get_rendering_fps() << std::endl;
                }
            }
        }
    });

    loop.start();

    return 0;
}

