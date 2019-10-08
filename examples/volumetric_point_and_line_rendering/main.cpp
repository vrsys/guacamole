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
#include <gua/renderer/ToneMappingPass.hpp>
#include <gua/renderer/DebugViewPass.hpp>
#include <gua/utils/Trackball.hpp>

#include <boost/program_options.hpp>

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

int main(int argc, char** argv)
{
    // initialize guacamole
    gua::init(1, argv);

    namespace po = boost::program_options;
    po::options_description desc("options: ");
    desc.add_options()("input,f", po::value<std::string>(), "specify input file with *.lob or *.pob model to use in the example");

    po::variables_map vm;

    std::string example_model_name = "data/objects/pig_ALPHA_SHAPES.lob";

    try
    {
        auto parsed_options = po::command_line_parser(argc, argv).options(desc).run();
        po::store(parsed_options, vm);
        po::notify(vm);

        if(vm.count("help"))
        {
            std::cout << desc;
            return 0;
        }

        // single scene decription file is provided
        if(vm.count("input"))
        {
            example_model_name = vm["input"].as<std::string>();
        }
    }
    catch(std::exception& e)
    {
        std::cout << "Warning: No input file specified. \n" << desc;
        return 0;
    }

    // setup scene
    gua::SceneGraph graph("main_scenegraph");

    gua::LineStripLoader line_strip_loader;

    auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");

    auto line_strip_example_real_geometry_node(
        line_strip_loader.create_geometry_from_file("ls_example_node", example_model_name, gua::LineStripLoader::NORMALIZE_POSITION | gua::LineStripLoader::NORMALIZE_SCALE));

    graph.add_node("/transform", line_strip_example_real_geometry_node);

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
    camera->config.set_output_window_name("main_window");
    camera->config.set_enable_stereo(false);

    camera->get_pipeline_description()->get_resolve_pass()->tone_mapping_exposure(1.0f);
    // camera->get_pipeline_description()->add_pass(
    //  std::make_shared<gua::DebugViewPassDescription>());

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

    uint32_t pushed_spiral_vertices = 0;

    float line_width = 1.0f;
    //////////////////////////////////////////////////////////////////////////////////////
    // key press events
    //////////////////////////////////////////////////////////////////////////////////////
    window->on_key_press.connect([&](int key, int scancode, int action, int mods) {
        if(action == 0)
            return; // only press events

        switch(key)
        {
        case ' ':
        {
            float b = 0.1;
            float a = 6.28;
            float spiral_float_parameter = pushed_spiral_vertices * 0.1;

            float norm_pos_x = b * spiral_float_parameter * std::cos(spiral_float_parameter + a);
            float norm_pos_y = spiral_float_parameter * 0.3;
            float norm_pos_z = b * spiral_float_parameter * std::sin(spiral_float_parameter + a);

            float rand_r = std::rand() / (float)RAND_MAX;
            float rand_g = std::rand() / (float)RAND_MAX;
            float rand_b = std::rand() / (float)RAND_MAX;

            ++pushed_spiral_vertices;

            /*          if(line_strip_example_node->has_children()) { //work on grouped line strips
                        for( auto& child : line_strip_example_node->get_children() ) {
                          auto line_strip_child = std::dynamic_pointer_cast<gua::node::LineStripNode>(child);

                          line_strip_child->push_vertex(norm_pos_x, norm_pos_y, norm_pos_z, rand_r, rand_g, rand_b, 1.0f, 0.2f, 0.0f, 1.0f, 0.0f);
                        }
                      } else { //work on parent node
                        auto line_strip_parent = std::dynamic_pointer_cast<gua::node::LineStripNode>(line_strip_example_node);
                        line_strip_parent->push_vertex(norm_pos_x, norm_pos_y, norm_pos_z, rand_r, rand_g, rand_b, 1.0f, 0.2f, 0.0f, 1.0f, 0.0f);
                      }*/
        }
        break;

        case 'B':
        {
            /*          if(line_strip_example_node->has_children()) { //work on grouped line strips
                        for( auto& child : line_strip_example_node->get_children() ) {
                          auto line_strip_child = std::dynamic_pointer_cast<gua::node::LineStripNode>(child);

                          line_strip_child->clear_vertices();
                        }
                      } else { //work on parent node
                        auto line_strip_parent = std::dynamic_pointer_cast<gua::node::LineStripNode>(line_strip_example_node);
                        bool render_as_points = line_strip_parent->get_render_vertices_as_points();

                        line_strip_parent->clear_vertices();
                      }*/
        }
        break;

        case 'M':
        {
            /*          if(line_strip_example_node->has_children()) { //work on grouped line strips
                        for( auto& child : line_strip_example_node->get_children() ) {
                          auto line_strip_child = std::dynamic_pointer_cast<gua::node::LineStripNode>(child);

                          line_strip_child->pop_front_vertex();
                        }
                      } else { //work on parent node
                        auto line_strip_parent = std::dynamic_pointer_cast<gua::node::LineStripNode>(line_strip_example_node);
                        bool render_as_points = line_strip_parent->get_render_vertices_as_points();

                        line_strip_parent->pop_front_vertex();
                      }*/
        }
        break;

        case 'N':
        {
            /*          if(line_strip_example_node->has_children()) { //work on grouped line strips
                        for( auto& child : line_strip_example_node->get_children() ) {
                          auto line_strip_child = std::dynamic_pointer_cast<gua::node::LineStripNode>(child);

                          line_strip_child->pop_back_vertex();
                        }
                      } else { //work on parent node
                        auto line_strip_parent = std::dynamic_pointer_cast<gua::node::LineStripNode>(line_strip_example_node);
                        bool render_as_points = line_strip_parent->get_render_vertices_as_points();

                        line_strip_parent->pop_back_vertex();
                      }*/
        }
        break;

        case 'A':
        {
            /*          if(line_strip_example_node->has_children()) { //work on grouped line strips
                        for( auto& child : line_strip_example_node->get_children() ) {
                          auto line_strip_child = std::dynamic_pointer_cast<gua::node::LineStripNode>(child);
                          bool render_as_points = line_strip_child->get_render_vertices_as_points();
                          line_strip_child->set_render_vertices_as_points(!render_as_points);
                        }
                      } else { //work on parent node
                        auto line_strip_parent = std::dynamic_pointer_cast<gua::node::LineStripNode>(line_strip_example_node);
                        bool render_as_points = line_strip_parent->get_render_vertices_as_points();
                        line_strip_parent->set_render_vertices_as_points(!render_as_points);
                      }*/
        }
        break;

        case 'S':
            /*          if(line_strip_example_node->has_children()) { //work on grouped line strips
                        for( auto& child : line_strip_example_node->get_children() ) {
                          auto line_strip_child = std::dynamic_pointer_cast<gua::node::LineStripNode>(child);
                          bool render_volumetric = line_strip_child->get_render_volumetric();
                          line_strip_child->set_render_volumetric(!render_volumetric);
                        }
                      } else { //work on parent node
                        auto line_strip_parent = std::dynamic_pointer_cast<gua::node::LineStripNode>(line_strip_example_node);
                        bool render_volumetric = line_strip_parent->get_render_volumetric();
                        line_strip_parent->set_render_volumetric(!render_volumetric);
                      }*/
            break;

        case 'R':
            /*          if(line_strip_example_node->has_children()) { //work on grouped line strips
                        for( auto& child : line_strip_example_node->get_children() ) {
                          auto line_strip_child = std::dynamic_pointer_cast<gua::node::LineStripNode>(child);
                          bool render_volumetric = line_strip_child->get_render_volumetric();

                          line_width += 1.0f;
                          line_width = std::min(10.0f, line_width);
                          line_strip_child->set_screen_space_line_width(line_width);
                        }
                      } else { //work on parent node
                        auto line_strip_parent = std::dynamic_pointer_cast<gua::node::LineStripNode>(line_strip_example_node);
                        bool render_volumetric = line_strip_parent->get_render_volumetric();

                        line_width += 1.0f;
                        line_width = std::min(10.0f, line_width);
                        line_strip_parent->set_screen_space_line_width(line_width);
                      }*/
            break;

        case 'F':
            /*          if(line_strip_example_node->has_children()) { //work on grouped line strips
                        for( auto& child : line_strip_example_node->get_children() ) {
                          auto line_strip_child = std::dynamic_pointer_cast<gua::node::LineStripNode>(child);
                          bool render_volumetric = line_strip_child->get_render_volumetric();

                          line_width -= 1.0f;
                          line_width = std::max(1.0f, line_width);
                          line_strip_child->set_screen_space_line_width(line_width);
                        }
                      } else { //work on parent node
                        auto line_strip_parent = std::dynamic_pointer_cast<gua::node::LineStripNode>(line_strip_example_node);
                        bool render_volumetric = line_strip_parent->get_render_volumetric();

                        line_width -= 1.0f;
                        line_width = std::max(1.0f, line_width);
                        line_strip_parent->set_screen_space_line_width(line_width);
                      }*/
            break;

            break;
        default:
            break;
        };
    });

    gua::Renderer renderer;

    // application loop
    gua::events::MainLoop loop;
    gua::events::Ticker ticker(loop, 1.0 / 500.0);

    ticker.on_tick.connect([&]() {
        // apply trackball matrix to object
        gua::math::mat4 modelmatrix = scm::math::make_translation(trackball.shiftx(), trackball.shifty(), trackball.distance()) * gua::math::mat4(trackball.rotation());

        transform->set_transform(modelmatrix);

        float norm_x = 50.0 * std::rand() / (float)RAND_MAX - 25.0;
        float norm_y = 50.0 * std::rand() / (float)RAND_MAX - 25.0;
        float norm_z = 50.0 * std::rand() / (float)RAND_MAX - 25.0;

        float rand_r = std::rand() / (float)RAND_MAX;
        float rand_g = std::rand() / (float)RAND_MAX;
        float rand_b = std::rand() / (float)RAND_MAX;

        /*
                  if(line_strip_example_node->has_children()) { //work on grouped line strips
                    for( auto& child : line_strip_example_node->get_children() ) {
                      auto line_strip_child = std::dynamic_pointer_cast<gua::node::LineStripNode>(child);

                      line_strip_child->push_vertex(norm_x, norm_y, norm_z, rand_r, rand_g, rand_b, 0.002f);
                    }
                  } else { //work on parent node
                    auto line_strip_parent = std::dynamic_pointer_cast<gua::node::LineStripNode>(line_strip_example_node);
                    bool render_as_points = line_strip_parent->get_render_vertices_as_points();

                    line_strip_parent->push_vertex(norm_x, norm_y, norm_z, rand_r, rand_g, rand_b, 0.002f);
                  }
        */

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
