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
#include <gua/utils/Trackball.hpp>

#include <gua/renderer/DynamicLinePass.hpp>
#include <gua/renderer/DynamicTrianglePass.hpp>
#include <gua/virtual_texturing/VTBackend.hpp>
// #include <gua/virtual_texturing/DeferredVirtualTexturingPass.hpp> OBSOLETE

// following lines can be used to get the type of a auto object
#include <typeinfo>
#include <boost/core/demangle.hpp>
template <typename T>
std::string type_str()
{
    return boost::core::demangle(typeid(T).name());
}
// std::cout << "typeof(i) = " << type_str<decltype(AUTO_INSTANCE)>() << '\n';

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

//// Helper functions ////

// function to create quad
std::shared_ptr<gua::node::Node> create_dynamic_triangle_quad(std::shared_ptr<gua::Material> vt_texture)
{
    gua::DynamicTriangleLoader dynamic_triangle_loader;

    auto dynamic_triangle_node(dynamic_triangle_loader.create_empty_geometry("dynamic_triangle_node",
                                                                             "empty_node.lob",
                                                                             vt_texture,
                                                                             // my_texture_mat, // use iF NOT VT
                                                                             gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE | gua::TriMeshLoader::MAKE_PICKABLE));

    auto node = std::dynamic_pointer_cast<gua::node::DynamicTriangleNode>(dynamic_triangle_node);

    node->set_draw_bounding_box(true);
    node->set_render_volumetric(false);
    // node->set_screen_space_line_width(2.5f);

    node->push_vertex(-1.0f, 1.0f, -4.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.1f, 0.1f, 0.2f);
    node->push_vertex(1.0f, -1.0f, -4.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.1f, 0.2f, 0.1f);
    node->push_vertex(1.0f, 1.0f, -4.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.2f, 0.20f);

    node->push_vertex(-1.0f, 1.0f, -4.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.1f, 0.1f, 0.2f);
    node->push_vertex(-1.0f, -1.0f, -4.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.1f, 0.1f, 0.1f);
    node->push_vertex(1.0f, -1.0f, -4.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.2f, 0.1f);

    return dynamic_triangle_node;
}

std::shared_ptr<gua::node::Node> create_dynamic_lines()
{
    gua::DynamicLineLoader dynamic_line_loader;
    std::shared_ptr<gua::node::Node> dynamic_line_node(
        dynamic_line_loader.create_empty_geometry("line_node",
                                                  "empty_node_2.lob",
                                                  gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE | gua::TriMeshLoader::MAKE_PICKABLE));
    auto node = std::dynamic_pointer_cast<gua::node::DynamicLineNode>(dynamic_line_node);
    node->set_draw_bounding_box(true);

    node->set_render_volumetric(false);
    node->set_screen_space_line_width(2.5f);

    node->push_vertex(-1.0f, 1.0f, -2.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.1f);
    node->push_vertex(1.0f, -1.0f, -2.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.1f);

    node->push_vertex(1.0f, 1.0f, -2.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.1f);
    node->push_vertex(-1.0f, 1.0f, -2.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.1f);

    node->push_vertex(-1.0f, -1.0f, -2.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.1f);
    node->push_vertex(1.0f, -1.0f, -2.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.1f);

    node->push_vertex(-1.0f, -1.0f, -2.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.1f);
    node->push_vertex(1.0f, 1.0f, -2.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.1f);

    return dynamic_line_node;
}

int main(int argc, char** argv)
{
    // initialize guacamole
    gua::init(argc, argv);

    // setup scene
    gua::SceneGraph graph("main_scenegraph");

    // VT STEP 1/5: - create a material
    auto atlas_material = gua::MaterialShaderDatabase::instance()->lookup("gua_default_material")->make_new_material();

    // VT STEP 2/5: - load *.atlas-File as uniform
    atlas_material->set_uniform("atlas_material", std::string("/opt/3d_models/lamure/provenance/salem/salem.atlas"));

    // VT STEP 3/5: - enable virtual texturing for this material
    atlas_material->set_enable_virtual_texturing(true);
    std::shared_ptr<gua::node::DynamicTriangleNode> dynamic_tri_node = std::dynamic_pointer_cast<gua::node::DynamicTriangleNode>(create_dynamic_triangle_quad(atlas_material));
    auto dl_node = create_dynamic_lines();

    auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");
    graph.add_node("/transform", dynamic_tri_node);
    graph.add_node("/transform", dl_node);

    gua::TriMeshLoader loader;

    scm::math::vec3d cube_translation(0.0, 0.0, -5.0);

    // create and add pick ray geometry
    auto ray_geometry(loader.create_geometry_from_file("ray_geometry", "data/objects/cylinder.obj", gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));
    graph.add_node("/", ray_geometry);

    ray_geometry->scale(0.02, 0.02, 0.1);

    // create and add light node
    auto light2 = graph.add_node<gua::node::LightNode>("/", "light2");
    light2->data.set_type(gua::node::LightNode::Type::POINT);
    light2->data.brightness = 150.0f;
    light2->scale(12.f);
    light2->translate(-3.f, 5.f, 5.f);

    // create and add screen node
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
    camera->config.set_output_window_name("Example Dynamic Triangle and Line");
    camera->config.set_enable_stereo(false);

    camera->get_pipeline_description()->get_resolve_pass()->tone_mapping_exposure(1.0f);

    auto pipe = std::make_shared<gua::PipelineDescription>();
    pipe->add_pass(std::make_shared<gua::DynamicLinePassDescription>());
    pipe->add_pass(std::make_shared<gua::DynamicTrianglePassDescription>());
    pipe->add_pass(std::make_shared<gua::TriMeshPassDescription>());

    // VT STEP 5/5: - add DeferredVirtualTexturingPassDescription
    pipe->add_pass(std::make_shared<gua::LightVisibilityPassDescription>());

    auto resolve_pass = std::make_shared<gua::ResolvePassDescription>();
    resolve_pass->background_mode(gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE);
    resolve_pass->tone_mapping_exposure(1.0f);
    pipe->add_pass(resolve_pass);
    camera->set_pipeline_description(pipe);

    auto window = std::make_shared<gua::GlfwWindow>();
    gua::WindowDatabase::instance()->add("Example Dynamic Triangle and Line", window);

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
    
    auto vt_backend = &gua::VTBackend::get_instance();
    vt_backend->add_camera(camera);
    vt_backend->start_backend();

    gua::Renderer renderer;

    // application loop
    gua::events::MainLoop loop;
    gua::events::Ticker ticker(loop, 1.0 / 500.0);

    scm::math::vec3d camera_positon(0.0, 0.0, 2.0);
    scm::math::vec3d negative_z_viewing_direction(0.0, 0.0, -100.0);
    scm::math::vec3d::value_type t_max(200.0);

    // create ray that is located in the origin of the current screen and looks along the negative z-axis (pick ray is seen as circle)
    gua::Ray ray_from_camera_position(camera_positon, negative_z_viewing_direction, t_max);

    size_t frame_count = 0;

    ticker.on_tick.connect([&]() {
        // apply trackball matrix to object
        gua::math::mat4 modelmatrix = scm::math::make_translation(cube_translation[0], cube_translation[1], cube_translation[2]) *
                                      scm::math::make_translation(trackball.shiftx(), trackball.shifty(), trackball.distance()) * gua::math::mat4(trackball.rotation());

        transform->set_transform(modelmatrix);

        if(window->should_close())
        {
            renderer.stop();
            window->close();
            loop.stop();
            vt_backend->stop_backend();
        }
        else
        {
            // Update vertices (either position or tex coords)
            // dynamic_tri_node->update_vertex(0, -1.0f,  1.0f, -4.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.1f, (frame_count % 300) / 300.0f, 0.0f);
            auto pick_results = graph.ray_test(ray_from_camera_position, gua::PickResult::Options::PICK_ONLY_FIRST_FACE | gua::PickResult::Options::GET_WORLD_POSITIONS);

            if(0 == (++frame_count) % 100)
            {   
                // Print pick results of ray
                // std::cout << "World space intersection position: " << pick_results.begin()->world_position << "\n";
                // std::cout << "World NAME : " << pick_results.begin()->object << "\n";
            }

            renderer.queue_draw({&graph});
        }
    });

    loop.start();

    return 0;
}
