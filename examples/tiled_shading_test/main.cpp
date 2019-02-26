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
#include <gua/renderer/EmissivePass.hpp>
#include <gua/renderer/LightingPass.hpp>
#include <gua/renderer/PhysicallyBasedShadingPass.hpp>
#include <gua/renderer/ToneMappingPass.hpp>
#include <random>

int main(int argc, char** argv)
{
    // initialize guacamole
    gua::init(0, 0);

    // setup scene
    gua::SceneGraph graph("main_scenegraph");

    auto desc(std::make_shared<gua::MaterialShaderDescription>());
    desc->load_from_file("data/materials/SimpleMaterial.gmd");

    auto shader(std::make_shared<gua::MaterialShader>("simple_mat", desc));
    gua::MaterialShaderDatabase::instance()->add(shader);

    gua::TriMeshLoader loader;

    int mode{1};
    if(argc > 1)
    {
        if(argv[1][0] == '1')
            mode = 1;
        else if(argv[1][0] == '2')
            mode = 2;
        else if(argv[1][0] == '3')
            mode = 3;
    }

    auto add_oilrig = [&](float x, float y) {
        auto oilrig(loader.create_geometry_from_file("oilrig_" + std::to_string(x) + std::to_string(y),
                                                     "/opt/3d_models/OIL_RIG_GUACAMOLE/oilrig.obj",
                                                     shader->make_new_material(),
                                                     gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE | gua::TriMeshLoader::LOAD_MATERIALS |
                                                         gua::TriMeshLoader::OPTIMIZE_GEOMETRY));
        oilrig->translate(x, y, 0.f);
        return oilrig;
    };

    auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");
    graph.add_node("/transform", add_oilrig(0, 0));
    graph.add_node("/transform", add_oilrig(0, 1));
    graph.add_node("/transform", add_oilrig(-1, 0));
    graph.add_node("/transform", add_oilrig(-1, 1));
    graph.add_node("/transform", add_oilrig(-2, 2));
    graph.add_node("/transform", add_oilrig(-2, 1));
    graph.add_node("/transform", add_oilrig(-1, 2));
    graph.add_node("/transform", add_oilrig(-1, 2));
    transform->scale(2.1f);
    transform->rotate(-90, 1, 0, 0);
    transform->rotate(-40, 0, 1, 0);
    transform->rotate(10, 1, 0, 0);
    transform->translate(0.4f, 0.f, 0.f);

    auto resolution = gua::math::vec2ui(1920, 1080);
    std::mt19937 gen(std::minstd_rand0{}());

    std::uniform_real_distribution<> dstx(-2.f, 2.f);
    std::uniform_real_distribution<> dsty(0.f, 2.f);
    std::uniform_real_distribution<> dstz(-6.f, 1.f);
    std::uniform_real_distribution<> dstc(0.2, 0.7);

    int max_lights = 32;

    if(mode == 1 || mode == 2)
    {
        for(int i = 0; i < 256; ++i)
        {
            auto light = graph.add_node<gua::node::LightNode>("/", "light_" + std::to_string(i));
            light->data.set_type(gua::node::LightNode::Type::POINT);
            light->data.set_color(gua::utils::Color3f(dstc(gen), dstc(gen), dstc(gen)));
            if(mode == 1)
                light->scale(1.2);
            else
                light->scale(0.8);
            light->translate(dstx(gen), dsty(gen), dstz(gen));
        }
        max_lights = 256;
    }

    if(mode == 3)
    {
        auto light1 = graph.add_node<gua::node::LightNode>("/", "big_light1");
        light1->data.set_type(gua::node::LightNode::Type::POINT);
        light1->scale(16.0);
        light1->translate(0.f, 3.f, 2.f);
        auto light2 = graph.add_node<gua::node::LightNode>("/", "big_light2");
        light2->data.set_type(gua::node::LightNode::Type::POINT);
        light2->scale(18.0);
        light2->translate(3.f, -3.f, 1.f);
    }

    auto screen = graph.add_node<gua::node::ScreenNode>("/", "screen");
    screen->data.set_size(gua::math::vec2(0.001 * resolution.x, 0.001 * resolution.y));
    screen->translate(0, 0, 1.0);

    // standard deferred shading pipeline
    auto pipe_deferred(std::make_shared<gua::PipelineDescription>());
    pipe_deferred->add_pass(std::make_shared<gua::TriMeshPassDescription>());
    pipe_deferred->add_pass(std::make_shared<gua::EmissivePassDescription>());
    pipe_deferred->add_pass(std::make_shared<gua::PhysicallyBasedShadingPassDescription>());
    auto tone_mapping_pass = std::make_shared<gua::ToneMappingPassDescription>();
    tone_mapping_pass->exposure(0.1f);
    pipe_deferred->add_pass(tone_mapping_pass);
    pipe_deferred->add_pass(std::make_shared<gua::BackgroundPassDescription>());

    // tiled shading pipeline with A-Buffer support
    auto pipe_tiled(std::make_shared<gua::PipelineDescription>());
    pipe_tiled->add_pass(std::make_shared<gua::TriMeshPassDescription>());

    auto light_visibility_pass = std::make_shared<gua::LightVisibilityPassDescription>();
    light_visibility_pass->tile_power(3);
    pipe_tiled->add_pass(light_visibility_pass);

    auto resolve_pass = std::make_shared<gua::ResolvePassDescription>();
    resolve_pass->tone_mapping_exposure(0.1f);
    resolve_pass->debug_tiles(false);
    pipe_tiled->add_pass(resolve_pass);

    pipe_tiled->set_max_lights_count(max_lights);

    auto camera = graph.add_node<gua::node::CameraNode>("/screen", "cam");
    camera->translate(0, 0, 2.0);
    camera->config.set_resolution(resolution);
    camera->config.set_screen_path("/screen");
    camera->config.set_scene_graph_name("main_scenegraph");
    camera->config.set_output_window_name("main_window");
    camera->config.set_enable_frustum_culling(false);
    camera->config.set_enable_frustum_culling(false);
    camera->set_pipeline_description(pipe_deferred);

    auto window = std::make_shared<gua::GlfwWindow>();
    gua::WindowDatabase::instance()->add("main_window", window);
    window->config.set_enable_vsync(false);
    window->config.set_size(resolution);
    window->config.set_resolution(resolution);

    window->on_resize.connect([&](gua::math::vec2ui const& new_size) {
        window->config.set_resolution(new_size);
        camera->config.set_resolution(new_size);
        screen->data.set_size(gua::math::vec2(0.001 * new_size.x, 0.001 * new_size.y));
    });
    window->on_key_press.connect([&](int key, int scancode, int action, int mods) {
        if(action != 0)
            return;
        if('Q' == key)
        {
            camera->set_pipeline_description(pipe_deferred);
            std::cout << "Pipeline: deferred" << std::endl;
        }
        else if('W' == key)
        {
            camera->set_pipeline_description(pipe_tiled);
            std::cout << "Pipeline: tiled" << std::endl;
        }

        if('T' == key)
        {
            pipe_tiled->set_enable_abuffer(!pipe_tiled->get_enable_abuffer());
            std::cout << "Enable A-Buffer: " << pipe_tiled->get_enable_abuffer() << std::endl;
        }

        auto& d = *(pipe_tiled->get_pass_by_type<gua::LightVisibilityPassDescription>());
        if('1' == key)
        {
            d.rasterization_mode(gua::LightVisibilityPassDescription::AUTO);
            std::cout << "Rast mode: AUTO\n";
            d.touch();
        }
        if('2' == key)
        {
            d.rasterization_mode(gua::LightVisibilityPassDescription::SIMPLE);
            std::cout << "Rast mode: SIMPLE\n";
            d.touch();
        }
        if('3' == key)
        {
            d.rasterization_mode(gua::LightVisibilityPassDescription::CONSERVATIVE);
            std::cout << "Rast mode: CONSERVATIVE\n";
            d.touch();
        }
        if('4' == key)
        {
            d.rasterization_mode(gua::LightVisibilityPassDescription::MULTISAMPLED_2);
            std::cout << "Rast mode: MULTISAMPLED_2\n";
            d.touch();
        }
        if('5' == key)
        {
            d.rasterization_mode(gua::LightVisibilityPassDescription::MULTISAMPLED_4);
            std::cout << "Rast mode: MULTISAMPLED_4\n";
            d.touch();
        }
        if('6' == key)
        {
            d.rasterization_mode(gua::LightVisibilityPassDescription::MULTISAMPLED_8);
            std::cout << "Rast mode: MULTISAMPLED_8\n";
            d.touch();
        }
        if('7' == key)
        {
            d.rasterization_mode(gua::LightVisibilityPassDescription::MULTISAMPLED_16);
            std::cout << "Rast mode: MULTISAMPLED_16\n";
            d.touch();
        }
        if('8' == key)
        {
            d.rasterization_mode(gua::LightVisibilityPassDescription::FULLSCREEN_FALLBACK);
            std::cout << "Rast mode: FULLSCREEN_FALLBACK\n";
            d.touch();
        }

        if('9' == key || '0' == key)
        {
            d.tile_power(d.tile_power() + ((key == '9') ? 1 : -1));
            std::cout << "Tile size: " << std::pow(2, d.tile_power()) << "\n";
            d.touch();
        }
    });
    window->open();

    gua::Renderer renderer;

    // application loop
    gua::events::MainLoop loop;
    gua::events::Ticker ticker(loop, 1.0 / 500.0);
    size_t ctr{};

    ticker.on_tick.connect([&]() {
        if(ctr++ % 150 == 0)
            std::cout << "Frame time: " << 1000.f / window->get_rendering_fps() << " ms, fps: " << window->get_rendering_fps() << ", app fps: " << renderer.get_application_fps() << std::endl;

        window->process_events();
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
