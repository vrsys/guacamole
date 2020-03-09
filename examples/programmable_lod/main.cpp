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

#include <algorithm>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>


#include <gua/guacamole.hpp>
#include <gua/renderer/TriMeshLoader.hpp>
#include <gua/renderer/DebugViewPass.hpp>
#include <gua/renderer/ToneMappingPass.hpp>
#include <gua/renderer/SSAAPass.hpp>
#include <gua/renderer/BBoxPass.hpp>
#include <gua/renderer/DebugViewPass.hpp>
#include <gua/renderer/LodLoader.hpp>
#include <gua/renderer/PLodPass.hpp>
#include <gua/renderer/MLodPass.hpp>
#include <gua/node/PLodNode.hpp>
#include <gua/node/MLodNode.hpp>
#include <scm/gl_util/manipulators/trackball_manipulator.h>

#define USE_MESH_LOD_MODEL 0
#define USE_POINTCLOUD_LOD_MODEL 1
#define USE_REGULAR_TRIMESH_MODEL 1


void print_graph(std::shared_ptr<gua::node::Node> const& scene_root_node, int depth = 0) {
    
    //see https://en.wikipedia.org/wiki/Box-drawing_character#Unicode for ascii table characters
    for(int dash_index = 0; dash_index < depth; ++dash_index) {
        std::cout <<" ";
    }

    //2 unicode characters for the table elements
    std::cout << "\u2517";
    std::cout << "\u2501";
    // name, tabs, typestring
    std::cout << " " << scene_root_node->get_name() << "\t\t" << scene_root_node->get_type_string() << std::endl;

    //print

    //std::cout << std::endl;

    for(auto const& child : scene_root_node->get_children()) {
        print_graph(child, depth+1);
    }

}

void adjust_arguments(int& argc, char**& argv)
{
    char* argv_tmp[] = {argv[0], NULL};
    int argc_tmp = sizeof(argv_tmp) / sizeof(char*) - 1;
    argc = argc_tmp;
    argv = argv_tmp;
}

std::string parse_model_from_cmd_line(int argc, char** argv)
{
    std::string model_path = "./fe_vis_mat.vis";

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

extern WASD_state cam_navigation_state;

bool print_frame_times = true;


int main(int argc, char** argv)
{
   
    std::string vis_file_path = parse_model_from_cmd_line(argc, argv);

    adjust_arguments(argc, argv);

    // init guacamole
    gua::init(argc, argv);

    // create simple untextured material shader
    auto lod_keep_input_desc = std::make_shared<gua::MaterialShaderDescription>("./data/materials/PLOD_use_input_color.gmd");
    auto lod_keep_color_shader(std::make_shared<gua::MaterialShader>("PLOD_pass_input_color", lod_keep_input_desc));
    gua::MaterialShaderDatabase::instance()->add(lod_keep_color_shader);

    // create material for pointcloud
    auto lod_rough = lod_keep_color_shader->make_new_material();
    lod_rough->set_uniform("metalness", 0.0f);
    lod_rough->set_uniform("roughness", 1.0f);
    lod_rough->set_uniform("emissivity", 1.0f);

    // configure lod backend
    gua::LodLoader lod_loader;
    lod_loader.set_out_of_core_budget_in_mb(4096);
    lod_loader.set_render_budget_in_mb(1024);
    lod_loader.set_upload_budget_in_mb(30);

    // setup scenegraph
    gua::SceneGraph graph("main_scenegraph");
    auto scene_transform = graph.add_node<gua::node::TransformNode>("/", "transform");

    auto mlod_transform = graph.add_node<gua::node::TransformNode>("/transform", "mlod_transform");
    auto plod_transform = graph.add_node<gua::node::TransformNode>("/transform", "plod_transform");
    auto tri_transform = graph.add_node<gua::node::TransformNode>("/transform", "tri_transform");

    bool show_arrows = true;

    gua::TriMeshLoader loader;

    //auto model_mat_fem_model(gua::MaterialShaderDatabase::instance()->lookup("gua_default_material")->make_new_material());

    //model_mat_fem_model->set_render_wireframe(false);
    //model_mat_fem_model->set_show_back_faces(true);

    auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");
    auto fem_model(
        loader.create_geometry_from_file("fem_model", "/mnt/pitoti/AISTec/FEM_simulation/Scherkondetal_Time_Series_20190822/FEM_OBJS/2020_01_23_Verbundtreffen_3_Demostate/Joined_Scherkonde_Geom_2020_01_23_registered.obj", 
                                                       gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE | gua::TriMeshLoader::LOAD_MATERIALS));

    //auto fem_model(
    //    loader.create_geometry_from_file("fem_model", "./data/objects/arrow.obj",
    //                                                   gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE | gua::TriMeshLoader::LOAD_MATERIALS));



    //auto fem_trimesh_model = (std::dynamic_pointer_cast<gua::node::TriMeshNode>(fem_model));//->get_material();

    //(fem_trimesh_model)->get_material()->set_show_back_faces(true);

    graph.add_node("/transform/plod_transform", fem_model);



    auto vector_of_lod_nodes = lod_loader.load_lod_pointclouds_from_vis_file(vis_file_path,
                                                                             //lod_rough,
                                                                              gua::LodLoader::MAKE_PICKABLE);





    for(auto const& node : vector_of_lod_nodes) {
        graph.add_node("/transform/plod_transform/fem_model", node);
    }



    for(auto child : fem_model->get_children()) {

        auto trimesh_node = (std::dynamic_pointer_cast<gua::node::TriMeshNode>(fem_model));
        trimesh_node->set_render_to_gbuffer(false);
        //trimesh_node->translate(additional_fem_model_translation, additional_fem_model_translation, additional_fem_model_translation);
    }




    auto light_transform = graph.add_node<gua::node::TransformNode>("/transform", "light_transform");
    auto light = graph.add_node<gua::node::LightNode>("/transform/light_transform", "light");
    light->data.set_type(gua::node::LightNode::Type::SPOT);
    light->data.set_enable_shadows(true);

    light->data.set_shadow_map_size(1920);
    light->data.set_shadow_offset(0.001f);
    light->data.set_softness(0.6f);
    light->data.set_shadow_far_clipping_in_sun_direction(2.0f);
    light->data.set_shadow_near_clipping_in_sun_direction(0.1f);

    light->data.brightness = 10.0f;
    light->scale(6.0f);
    light_transform->rotate(-90, 1.0, 0.0, 0.0);
    light_transform->translate(0.f, 3.f, 1.0f);


    auto nav = graph.add_node<gua::node::TransformNode>("/", "navigation");
    auto screen = graph.add_node<gua::node::ScreenNode>("/navigation", "screen");
    screen->data.set_size(gua::math::vec2(1.92f, 1.08f));
    screen->translate(0, 0, 1.0);

    // add mouse interaction
    scm::gl::trackball_manipulator trackball;
    trackball.transform_matrix() * scm::math::make_translation(0.01f, 0.002f, 0.2f);
    trackball.dolly(0.2f);
    float dolly_sens = 1.5f;
    gua::math::vec2 trackball_init_pos(0.f);
    gua::math::vec2 last_mouse_pos(0.f);
    int button_state = -1;

    // setup rendering pipeline and window
     auto resolution = gua::math::vec2ui(3840, 2160);
    //auto resolution = gua::math::vec2ui(1920, 1080);

    auto camera = graph.add_node<gua::node::CameraNode>("/navigation/screen", "cam");
    camera->translate(0.0f, 0, 2.5f);
    camera->config.set_resolution(resolution);

    // use close near plane to allow inspection of details
    camera->config.set_near_clip(0.01f);
    camera->config.set_far_clip(20.0f);
    camera->config.set_screen_path("/navigation/screen");
    camera->config.set_scene_graph_name("main_scenegraph");
    camera->config.set_output_window_name("main_window");
    // camera->config.set_enable_stereo(true);

    auto PLod_Pass = std::make_shared<gua::PLodPassDescription>();

    auto pipe = std::make_shared<gua::PipelineDescription>();
    pipe->add_pass(std::make_shared<gua::TriMeshPassDescription>());
    pipe->add_pass(std::make_shared<gua::MLodPassDescription>());
    pipe->add_pass(PLod_Pass);
    pipe->add_pass(std::make_shared<gua::BBoxPassDescription>());
    pipe->add_pass(std::make_shared<gua::LightVisibilityPassDescription>());
    pipe->add_pass(std::make_shared<gua::ResolvePassDescription>());
    pipe->add_pass(std::make_shared<gua::DebugViewPassDescription>());
    camera->set_pipeline_description(pipe);

    pipe->get_resolve_pass()->tone_mapping_exposure(1.f);
    pipe->get_resolve_pass()->tone_mapping_method(gua::ResolvePassDescription::ToneMappingMethod::UNCHARTED);

    pipe->get_resolve_pass()->background_mode(gua::ResolvePassDescription::BackgroundMode::SKYMAP_TEXTURE);
    pipe->get_resolve_pass()->background_texture("data/textures/envlightmap.jpg");

    auto& p_desc = camera->get_pipeline_description();
    p_desc->set_enable_abuffer(true);
    p_desc->set_abuffer_size(1500);

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

    for(auto& plod_node : vector_of_lod_nodes) {
        auto casted_plod_node = std::dynamic_pointer_cast<gua::node::PLodNode>(plod_node);
        casted_plod_node->set_time_series_playback_speed(1.0f);
        casted_plod_node->set_time_series_deform_factor(3000.0f);
    }

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
                auto object_trans = scm::math::inverse(screen->get_world_transform()) * mlod_transform->get_world_transform();
                mlod_transform->set_world_transform(screen->get_world_transform() * scm::math::make_translation(trans) * object_trans);
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


    float elapsed_frame_time = 0.0f;


    auto first_casted_plod_node = std::dynamic_pointer_cast<gua::node::PLodNode>(vector_of_lod_nodes[0]);

    std::vector<std::shared_ptr<gua::node::Node>> train_axis_geodes( first_casted_plod_node->get_number_of_simulation_positions() );
    std::vector<std::shared_ptr<gua::node::TriMeshNode>> arrow_nodes;

    auto model_mat_arrows(gua::MaterialShaderDatabase::instance()->lookup("gua_default_material")->make_new_material());


    for(uint32_t train_axis_index = 0; train_axis_index < train_axis_geodes.size(); ++train_axis_index) {
        train_axis_geodes[train_axis_index] = loader.create_geometry_from_file( std::string("train_axis") + std::to_string(train_axis_index) , "./data/objects/arrow_270deg_x_rotated.obj", model_mat_arrows,  gua::TriMeshLoader::LOAD_MATERIALS);
    
        arrow_nodes.push_back((std::dynamic_pointer_cast<gua::node::TriMeshNode>(train_axis_geodes[train_axis_index]) ) );
        graph.add_node("/transform/plod_transform/fem_model", train_axis_geodes[train_axis_index]);
    }


    for(auto const& node : arrow_nodes) {
        node->set_render_to_gbuffer(show_arrows);
    }




    window->on_key_press.connect(std::bind(
        [&](gua::PipelineDescription& pipe, gua::SceneGraph& graph, int key, int scancode, int action, int mods) {
        
            if(action == 0)
                return;

            switch(scancode) {
                //ARROW UP
                case 111: {
                    for(auto& plod_node : vector_of_lod_nodes) {
                        auto casted_plod_node = std::dynamic_pointer_cast<gua::node::PLodNode>(plod_node);
                        casted_plod_node->set_attribute_to_visualize_index( casted_plod_node->get_attribute_to_visualize_index() + 1);
                    }
                    break;
                }

                //ARROW DOWN
                case 116: {
                    for(auto& plod_node : vector_of_lod_nodes) {
                        auto casted_plod_node = std::dynamic_pointer_cast<gua::node::PLodNode>(plod_node);
                        casted_plod_node->set_attribute_to_visualize_index( casted_plod_node->get_attribute_to_visualize_index() - 1);
                    }
                    break;
                }

            }

            switch(std::tolower(key))
            {
            case '1':
                PLod_Pass->mode(gua::PLodPassDescription::SurfelRenderMode::HQ_TWO_PASS);
                PLod_Pass->touch();
                std::cout << "PLOD rendering set to high-quality two pass splatting." << std::endl;
                break;
            case '2':
                PLod_Pass->mode(gua::PLodPassDescription::SurfelRenderMode::LQ_ONE_PASS);
                PLod_Pass->touch();
                std::cout << "PLOD rendering set to low-quality one-pass splatting with ellipsoid surfels." << std::endl;
                break;
            case 'b':
                for(auto& plod_node : vector_of_lod_nodes) {
                    auto casted_plod_node = std::dynamic_pointer_cast<gua::node::PLodNode>(plod_node);
                    casted_plod_node->set_enable_backface_culling_by_normal(!casted_plod_node->get_enable_backface_culling_by_normal());
                }
                break;
            case 'd':
                for(auto& plod_node : vector_of_lod_nodes) {
                    auto casted_plod_node = std::dynamic_pointer_cast<gua::node::PLodNode>(plod_node);
                    casted_plod_node->set_enable_time_series_deformation(!casted_plod_node->get_enable_time_series_deformation());
                }
                break;
            case 'c':
                for(auto& plod_node : vector_of_lod_nodes) {
                    auto casted_plod_node = std::dynamic_pointer_cast<gua::node::PLodNode>(plod_node);
                    casted_plod_node->set_enable_time_series_coloring(!casted_plod_node->get_enable_time_series_coloring());
                }
                break;
            case '4':
                for(auto& plod_node : vector_of_lod_nodes) {
                    auto casted_plod_node = std::dynamic_pointer_cast<gua::node::PLodNode>(plod_node);
                    casted_plod_node->set_max_surfel_radius(std::max(0.0001f, 0.9f * casted_plod_node->get_max_surfel_radius()));
                    std::cout << "Max. surfel size set to : " << casted_plod_node->get_max_surfel_radius() << std::endl;
                }
                break;
            case '5':
                for(auto& plod_node : vector_of_lod_nodes) {
                    auto casted_plod_node = std::dynamic_pointer_cast<gua::node::PLodNode>(plod_node);
                    casted_plod_node->set_max_surfel_radius(1.1 * casted_plod_node->get_max_surfel_radius());

                    std::cout << "Max. surfel size set to : " << casted_plod_node->get_max_surfel_radius() << std::endl;
                }
                break;

            case 'a':
                show_arrows = !show_arrows;
                for(auto arrow_node : arrow_nodes) {
                    arrow_node->set_render_to_gbuffer(show_arrows);
                }
                break;


            case '9':
                for(auto const& plod_node : vector_of_lod_nodes) {
                    auto casted_plod_node = std::dynamic_pointer_cast<gua::node::PLodNode>(plod_node);
                    uint32_t new_active_index = (casted_plod_node->get_active_time_series_index() + 1) %  casted_plod_node->get_time_series_data_descriptions().size();

                    casted_plod_node->set_active_time_series_index(new_active_index);
                }
                break;

            case 's':
                for(auto const& plod_node : vector_of_lod_nodes) {
                    auto casted_plod_node = std::dynamic_pointer_cast<gua::node::PLodNode>(plod_node);
                    casted_plod_node->set_time_cursor_position(casted_plod_node->get_time_cursor_position() + 1.0f);

                    //std::cout << "Max. surfel size set to : " << plod_node->get_max_surfel_radius() << std::endl;
                }
                break;
            case 'j': {
                float current_attribute_color_mix_in_factor = 0.0f;
                    for(auto const& plod_node : vector_of_lod_nodes) {
                        auto casted_plod_node = std::dynamic_pointer_cast<gua::node::PLodNode>(plod_node);
                        casted_plod_node->set_attribute_color_mix_in_factor( casted_plod_node->get_attribute_color_mix_in_factor() + 0.1f );
                        current_attribute_color_mix_in_factor = casted_plod_node->get_attribute_color_mix_in_factor();
                    }
                    std::cout << "Set color mix in factor to: " << 1.0 - current_attribute_color_mix_in_factor << std::endl;
                }
                break;
            case 'u': {
                float current_attribute_color_mix_in_factor = 0.0f;
                    for(auto const& plod_node : vector_of_lod_nodes) {
                        auto casted_plod_node = std::dynamic_pointer_cast<gua::node::PLodNode>(plod_node);
                        casted_plod_node->set_attribute_color_mix_in_factor( casted_plod_node->get_attribute_color_mix_in_factor() - 0.1f );
                        current_attribute_color_mix_in_factor = casted_plod_node->get_attribute_color_mix_in_factor();
                    }
                    std::cout << "Set color mix in factor to: " << 1.0 - current_attribute_color_mix_in_factor << std::endl;
                }
                break;
            case 'p':
                for(auto& plod_node : vector_of_lod_nodes) {
                    auto casted_plod_node = std::dynamic_pointer_cast<gua::node::PLodNode>(plod_node);
                    casted_plod_node->set_enable_automatic_playback(!casted_plod_node->get_enable_automatic_playback() );
                }
                    //
                    //print_graph( graph.get_root() );
                break;

            case 'o': {
                    float current_playback_speed = 0.0f;
                    for(auto const& plod_node : vector_of_lod_nodes) {
                        auto casted_plod_node = std::dynamic_pointer_cast<gua::node::PLodNode>(plod_node);
                        casted_plod_node->set_time_series_playback_speed(std::min(1.0f, casted_plod_node->get_time_series_playback_speed() * 2.0f) );
                        current_playback_speed = casted_plod_node->get_time_series_playback_speed();
                    }
                    std::cout << "Set playback speed to: " << current_playback_speed << std::endl;
                }
                break;
            case 'l': {
                    float current_playback_speed = 0.0f;
                    for(auto const& plod_node : vector_of_lod_nodes) {
                        auto casted_plod_node = std::dynamic_pointer_cast<gua::node::PLodNode>(plod_node);
                        casted_plod_node->set_time_series_playback_speed(std::max(0.01f, casted_plod_node->get_time_series_playback_speed()/2.0f) );
                        current_playback_speed = casted_plod_node->get_time_series_playback_speed();
                    }
                    std::cout << "Set playback speed to: " << current_playback_speed << std::endl;
                }
                break;
            case 'i': {
                    float current_deform_factor = 0.0f;
                    for(auto const& plod_node : vector_of_lod_nodes) {
                        auto casted_plod_node = std::dynamic_pointer_cast<gua::node::PLodNode>(plod_node);
                        casted_plod_node->set_time_series_deform_factor(std::min(3000.0f, casted_plod_node->get_time_series_deform_factor() * 2.0f) );
                        current_deform_factor = casted_plod_node->get_time_series_deform_factor();
                    }
                    std::cout << "Set deform factor to: " << current_deform_factor << std::endl;
                }
                break;
            case 'k': {
                    float current_deform_factor = 0.0f;
                    for(auto const& plod_node : vector_of_lod_nodes) {
                        auto casted_plod_node = std::dynamic_pointer_cast<gua::node::PLodNode>(plod_node);
                        casted_plod_node->set_time_series_deform_factor(std::max(1.0f, casted_plod_node->get_time_series_deform_factor()/2.0f) );

                        current_deform_factor = casted_plod_node->get_time_series_deform_factor();
                    }
                    std::cout << "Set deform factor to: " << current_deform_factor << std::endl;
                }
            break;
            case 't':
                for(auto const& plod_node : vector_of_lod_nodes) {
                    auto casted_plod_node = std::dynamic_pointer_cast<gua::node::PLodNode>(plod_node);
                    casted_plod_node->set_enable_temporal_interpolation( !casted_plod_node->get_enable_temporal_interpolation() );
                }
                break;

            default:
                break;
            }
            
        },
        std::ref(*(camera->get_pipeline_description())),
        std::ref(graph),
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3,
        std::placeholders::_4));

    window->open();

    gua::Renderer renderer;

    // application loop
    gua::events::MainLoop loop;
    gua::events::Ticker ticker(loop, 1.0 / 500.0);
    std::size_t framecount = 0;
    
    std::chrono::time_point<std::chrono::system_clock> start, end;


                for(auto& plod_node : vector_of_lod_nodes) {
                    auto casted_plod_node = std::dynamic_pointer_cast<gua::node::PLodNode>(plod_node);
                    casted_plod_node->set_enable_time_series_deformation(true);
                    casted_plod_node->set_enable_time_series_coloring(true);
                }




    model_mat_arrows->set_render_wireframe(false);
    model_mat_arrows->set_show_back_faces(false);



    light_transform->rotate(90.0f, 0.f, 1.f, 0.f);

    ticker.on_tick.connect([&]() {
        end = std::chrono::system_clock::now();

        elapsed_frame_time = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
        start = std::chrono::system_clock::now();
        screen->set_transform(scm::math::inverse(gua::math::mat4(trackball.transform_matrix())));


        float application_fps = renderer.get_application_fps();
        float elapsed_application_time_milliseconds = 0.0;

        if(application_fps > 0.0f) {
            elapsed_application_time_milliseconds = 1.0 / application_fps;
        }

        update_cam_matrix(camera, nav, elapsed_application_time_milliseconds);

        //light_transform->rotate(0.1, 0.f, 1.f, 0.f);
        window->process_events();
        if(window->should_close())
        {
            renderer.stop();
            window->close();
            loop.stop();
        }
        else
        {
            for(auto const& plod_node : vector_of_lod_nodes) {
                auto casted_plod_node = std::dynamic_pointer_cast<gua::node::PLodNode>(plod_node);
                casted_plod_node->update_time_cursor(elapsed_frame_time / 1e3f);
            }
            


            auto first_casted_plod_node = std::dynamic_pointer_cast<gua::node::PLodNode>(vector_of_lod_nodes[0]);

            auto node_pretransform = gua::math::get_rotation(scm::math::mat4d(first_casted_plod_node->get_active_time_series_transform()) );


            auto current_simulation_positions = first_casted_plod_node->get_current_simulation_positions();

            for(uint32_t train_axis_index = 0; train_axis_index < current_simulation_positions.size(); ++train_axis_index) {

                auto const& current_sim_position = current_simulation_positions[train_axis_index];
                auto current_train_translation = scm::math::make_translation(current_sim_position[0], 
                                                                             current_sim_position[1], 
                                                                             current_sim_position[2] );

                auto train_axis_transformation =  current_train_translation * scm::math::mat4f(node_pretransform);// * scm::math::make_rotation(-90.0f, 1.0f, 0.0f, 0.0f);

                train_axis_geodes[train_axis_index]->set_transform( gua::math::mat4(train_axis_transformation) );
            }

            renderer.queue_draw({&graph});
            if(framecount++ % 200 == 0)
            {
                std::cout << "FPS: " << window->get_rendering_fps() << "  Frametime: " << 1000.f / window->get_rendering_fps() << std::endl;
            }
        }


    });

    loop.start();

    return 0;
}
