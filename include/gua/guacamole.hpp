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

#ifndef GUA_GUACAMOLE_HPP
#define GUA_GUACAMOLE_HPP

// guacamole headers
#include <gua/config.hpp>
#include <gua/databases.hpp>
#include <gua/events.hpp>
#include <gua/math.hpp>
#ifdef GUACAMOLE_ENABLE_PHYSICS
#include <gua/physics.hpp>
#endif
#include <gua/renderer.hpp>
#include <gua/scenegraph.hpp>
#include <gua/utils.hpp>

namespace gua
{
/**
 * Initialize guacamole.
 *
 * This should be called once at the beginning of every application
 * using guacamole.
 */

void GUA_DLL init(int argc, char** argv);

void GUA_DLL create_resource_material(std::string const& material_name, std::vector<unsigned char> const& shading_model_resource, std::vector<unsigned char> const& material_resource);

} // namespace gua

#endif // GUA_GUACAMOLE_HPP

/**
 * \mainpage
 * \section welcome Welcome to guacamole!
 * Welcome to guacamole, an incredibly exciting virtual reality framework.
 * It features an awesome scene graph and a modern OpenGL rendering engine
 * that is capable of handling multiple contexts.
 * \section example A short example
 * Below you can discover a little example of guacamole displaying some
 * stuff with guacamole.
 * \code
 * #include <gua/guacamole.hpp>
 *
 * gua::Pipeline* create_pipe() {
 *     // setup rendering pipeline
 *     auto pass = new gua::GeometryPass("main", "camera", "screen");
 *     pass->add_buffer(gua::ColorBufferDescription("color", 0));
 *     pass->add_buffer(gua::DepthStencilBufferDescription("depth_stencil"));
 *
 *     auto pipe = new gua::Pipeline(gua::Window::Description(1600, 900,
 * "simple_example", ":0.0", gua::StereoMode::MONO));
 *     pipe->add_render_pass(pass);
 *     pipe->set_final_buffer("main", "color");
 *
 *     return pipe;
 * }

 * std::vector<gua::SceneGraph::Iterator> add_lights(gua::SceneGraph& graph,
 * int count) {
 *
 *     std::vector<gua::SceneGraph::Iterator> lights(count);
 *
 *     auto sphere_core = new gua::GeometryNode("light_sphere", "bright");
 *
 *     for (int i(0); i<count; ++i) {
 *         auto light_core = new gua::LightCore(gua::utils::Color3f::random());
 *
 *         lights[i] = graph.add_node("/",
 * "sphere"+gua::string_utils::to_string(i), sphere_core);
 *         lights[i].scale(0.02, 0.02, 0.02);
 *         lights[i].translate(gua::randomizer::random(-0.8f, 0.8f),
 * gua::randomizer::random(0.05f, 0.1f), gua::randomizer::random(-0.8f, 0.8f));
 *
 *         auto light = lights[i].add_child("light", light_core);
 *         light.scale(20, 20, 20);
 *     }
 *
 *     return lights;
 * }
 *
 * int main(int argc, char** argv) {
 *
 *     // initialize guacamole
 *     gua::init(argc, argv);
 *
 *     gua::GeometryDatabase::load_objects_from("data/objects/");
 *     gua::TextureDatabase::load_textures_from("data/textures/");
 *     gua::MaterialShaderDatabase::load_materials_from("data/materials/");
 *
 *     // setup scene
 *     gua::SceneGraph graph;
 *
 *     auto plane_core = new gua::GeometryNode("plane", "tiles");
 *     auto floor = graph.add_node("/", "floor", plane_core);
 *     floor.scale(1.6, 1, 2);
 *
 *     auto cube_core = new gua::GeometryCore("cube", "tiles_small");
 *     auto box = graph.add_node("/", "box", cube_core);
 *     box.scale(0.2, 0.2, 0.2);
 *     box.translate(0, 0.1, 0);
 *
 *     auto monkey_core = new gua::GeometryCore("monkey", "wood");
 *     auto ape = graph.add_node("/box", "monkey", monkey_core);
 *     ape.scale(0.5, 0.5, 0.5);
 *     ape.translate(0, 1, 0);
 *
 *     auto lights = add_lights(graph, 36);
 *
 *     auto screen_core(new gua::ScreenNode(1.6, 0.9));
 *     auto screen = graph.add_node("/", "screen", screen_core);
 *     screen.translate(0, 0.45, 0.5);
 *
 *     auto camera_core = new gua::ViewNode(0.1f);
 *     auto camera = graph.add_node("/screen", "camera", camera_core);
 *     camera.translate(0, 0, 1.5);
 *
 *     gua::Renderer renderer({create_pipe()});
 *
 *     gua::Timer timer;
 *     timer.start();
 *
 *     double time(0);
 *
 *     // application loop
 *     while (true) {
 *         renderer.queue_draw(&graph);
 *
 *         std::this_thread::sleep_for(std::chrono::milliseconds(5));
 *
 *         double frame_time(timer.get_elapsed());
 *         time += frame_time;
 *         timer.reset();
 *
 *         for (int i=0; i<lights.size(); ++i) {
 *             lights[i].translate(0, std::sin(time*(i*0.1 +
 * 0.5))*frame_time*0.5, 0);
 *         }
 *
 *         graph["/box/monkey"].rotate(50*frame_time, 0, 1, 0);
 *         graph["/screen"].rotate(20*frame_time, 0, 1, 0);
 *     }
 *
 *     return 0;
 * }
 * \endcode
 */
