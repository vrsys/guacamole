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

#include <scm/gl_util/manipulators/trackball_manipulator.h>

#include <gua/guacamole.hpp>
#include <gua/spoints.hpp>
#include <gua/renderer/TriMeshLoader.hpp>
#include <gua/renderer/SSAAPass.hpp>
#include <gua/renderer/TexturedQuadPass.hpp>
#include <gua/renderer/ToneMappingPass.hpp>
#include <gua/utils/Trackball.hpp>
#include <gua/renderer/DebugViewPass.hpp>
#include <functional>
#include <iostream>
#include <fstream>

const bool SHOW_FRAME_RATE = true;

struct AnimEvent
{
    float start;
    float duration;
    gua::math::vec3 translation;
};

struct PartDesc
{
    std::string name;
    int style;
    AnimEvent ev;
};

enum class Mode
{
    SeeThrough,
    Highlight,
    ModelOpacity
};

float saturate(float x) { return std::max(std::min(x, 1.f), 0.f); }

float smootherstep(float edge0, float edge1, float x)
{
    x = saturate((x - edge0) / (edge1 - edge0));
    return x * x * x * (x * (x * 6.f - 15.f) + 10.f);
}

int main(int argc, char** argv)
{
    using namespace gua::math;

    auto resolution = gua::math::vec2ui(1920, 1080);
#if WIN32
    const std::string engine_directory = "data/engine/";
#else
    const std::string engine_directory = "data/engine/";
#endif
    Mode mode{Mode::SeeThrough};

    std::vector<PartDesc> part_names{
        {"part_inner_04", 2, AnimEvent({0.f, 0.0f, vec3(0.f, 0.f, 0.f)})},      // 1
        {"part_inner_03", 2, AnimEvent({0.01f, 0.09f, vec3(0.f, 13.f, 0.f)})},  // 2
        {"part_inner_02", 2, AnimEvent({0.09f, 0.09f, vec3(0.f, -13.f, 0.f)})}, // 3
        {"part_08", 2, AnimEvent({0.16f, 0.09f, vec3(0.f, -13.f, 0.f)})},       // 4
        {"part_10", 1, AnimEvent({0.22f, 0.09f, vec3(0.f, 13.f, 0.f)})},        // 5
        {"part_09", 1, AnimEvent({0.30f, 0.09f, vec3(0.f, 0.f, -13.f)})},       // 6
        {"part_04", 2, AnimEvent({0.37f, 0.09f, vec3(0.f, 13.f, 0.f)})},        // 7
        {"part_05", 1, AnimEvent({0.44f, 0.08f, vec3(0.f, 10.f, 0.f)})},        // 8
        {"part_06", 2, AnimEvent({0.52f, 0.09f, vec3(0.f, 10.f, 0.f)})},        // 9
        {"part_07", 2, AnimEvent({0.60f, 0.09f, vec3(0.f, 0.f, -13.f)})},       // 10
        {"part_02", 2, AnimEvent({0.68f, 0.09f, vec3(0.f, 0.f, 13.f)})},        // 11 -
        {"part_03", 2, AnimEvent({0.75f, 0.09f, vec3(0.f, 0.f, -13.f)})},       // 12 -
        {"part_inner_01", 1, AnimEvent({0.83f, 0.08f, vec3(0.f, 7.f, 0.f)})},   // 13
        {"part_01", 0, AnimEvent({0.92f, 0.08f, vec3(0.f, 9.f, 0.f)})}          // 14
    };

    float anim_time = 0.f;

    // navigation
    scm::gl::trackball_manipulator trackball;
    trackball.transform_matrix(scm::math::make_translation(0.f, -0.5f, 0.f));
    trackball.dolly(0.2f);
    float dolly_sens = 1.5;
    gua::math::vec2 trackball_init_pos(0.0);
    gua::math::vec2 last_mouse_pos(0.0);
    int button_state = -1;

    if(argc < 3)
    {
        std::cout << "ERROR: please provide two *.sr file containing at least a 'serverport' attribute!" << std::endl;
        return -1;
    }
    std::string spoints_avatar_1_resource_string(argv[1]);
    std::string spoints_avatar_2_resource_string(argv[2]);

    gua::SPointsLoader sloader;
    auto avatar_1_geode(sloader.create_geometry_from_file("avatar_1", spoints_avatar_1_resource_string.c_str()));
    auto avatar_2_geode(sloader.create_geometry_from_file("avatar_2", spoints_avatar_2_resource_string.c_str()));

    char* argv_tmp[] = {argv[0], NULL};
    int argc_tmp = sizeof(argv_tmp) / sizeof(char*) - 2;
    // initialize guacamole
    gua::init(argc_tmp, argv_tmp);

    // setup scene
    gua::SceneGraph graph("main_scenegraph");

    auto main_trans = graph.add_node<gua::node::TransformNode>("/", "main_trans");
    main_trans->scale(0.75f * 0.06f);
    main_trans->rotate(45.0f, 0.0f, 1.0f, 0.0f);
    main_trans->translate(0.0f, 0.25f, 0.0f);

    auto avatar_trans = graph.add_node<gua::node::TransformNode>("/", "avatar_trans");
    avatar_trans->translate(0.0f, -1.0f, 0.8f);
    auto avatar_trans2 = graph.add_node<gua::node::TransformNode>("/", "avatar_trans2");
    avatar_trans2->rotate(180.0f, 0.0f, 1.0f, 0.0f);
    avatar_trans2->translate(0.0f, -1.0f, -0.8f);
    graph.add_node("/avatar_trans", avatar_1_geode);
    graph.add_node("/avatar_trans2", avatar_2_geode);

    avatar_trans->rotate(-225.0f, 0.0f, 1.0f, 0.0f);
    avatar_trans2->rotate(-225.0f, 0.0f, 1.0f, 0.0f);
    main_trans->rotate(-225.0f, 0.0f, 1.0f, 0.0f);

    avatar_trans->translate(0.0f, 0.0f, -1.0f);
    avatar_trans2->translate(0.0f, 0.0f, -1.0f);
    main_trans->translate(0.0f, 0.0f, -1.0f);

    auto vr_room_trans = graph.add_node<gua::node::TransformNode>("/", "vr_room_trans");

    gua::TriMeshLoader loader;
    std::unordered_map<std::string, std::shared_ptr<gua::node::TriMeshNode>> parts;
    std::unordered_map<std::string, AnimEvent> anim_events;
    std::unordered_map<std::string, std::function<bool()>> tasks;

    // material
    auto desc(std::make_shared<gua::MaterialShaderDescription>());
    desc->load_from_file("data/main.gmd");
    auto shader(std::make_shared<gua::MaterialShader>("data/main.gmd", desc));
    gua::MaterialShaderDatabase::instance()->add(shader);

    // prepare engine parts
    for(const auto& p : part_names)
    {
        auto mat = shader->make_new_material();
        mat->set_uniform("style", p.style).set_uniform("opacity", 1.f).set_uniform("roughness_map", std::string("data/roughness.jpg")).set_show_back_faces(true);

        // derived from http://www.turbosquid.com/3d-models/speculate-3ds-free/302188
        // Royalty Free License
        auto part(loader.create_geometry_from_file(p.name, engine_directory + p.name + ".obj", mat, gua::TriMeshLoader::MAKE_PICKABLE));
        parts[p.name] = std::dynamic_pointer_cast<gua::node::TriMeshNode>(part);
        anim_events[p.name] = p.ev;
        graph.add_node("/main_trans", part);
    }

    auto vr_room_mat(gua::MaterialShaderDatabase::instance()->lookup("gua_default_material")->make_new_material());
    vr_room_mat->set_uniform("Emissivity", 1.0f);
    vr_room_mat->set_uniform("ColorMap", std::string("data/textures/grid.png"));
    auto vr_room(loader.create_geometry_from_file("cyberspace", "data/objects/cyberspace.obj", vr_room_mat, gua::TriMeshLoader::DEFAULTS));

    graph.add_node("/vr_room_trans", vr_room);
    vr_room->rotate(45.0f, 0.0f, 1.0f, 0.0f);
    vr_room->translate(0.0f, -1.0f, 0.0f);
    vr_room->scale(10.0f, 10.0f, 10.0f);

    // Lights
    auto light = graph.add_node<gua::node::LightNode>("/", "light");
    light->data.set_type(gua::node::LightNode::Type::SPOT);
    light->data.set_brightness(4.0f);
    light->scale(8.f);
    light->rotate(-30, 0.f, 1.f, 0.f);
    light->rotate(-90, 1.f, 0.f, 0.f);
    light->translate(-2.f, 2.f, 0.f);

    auto light2 = graph.add_node<gua::node::LightNode>("/", "light2");
    light2->data.set_type(gua::node::LightNode::Type::POINT);
    light2->data.set_brightness(14.0f);
    light2->scale(8.f);
    light2->translate(1.4f, 2.0f, -2.5f);

    auto light3 = graph.add_node<gua::node::LightNode>("/", "light3");
    light3->data.set_type(gua::node::LightNode::Type::POINT);
    light3->data.set_brightness(14.0f);
    light3->scale(8.f);
    light3->translate(-1.f, -2.f, 2.f);

    // screen, camera
    auto screen = graph.add_node<gua::node::ScreenNode>("/", "screen");
    screen->data.set_size(gua::math::vec2(1.92f, 1.08f));

    auto camera = graph.add_node<gua::node::CameraNode>("/screen", "cam");
    camera->translate(0, 0, 2.0);
    camera->config.set_resolution(resolution);
    camera->config.set_screen_path("/screen");
    camera->config.set_scene_graph_name("main_scenegraph");
    camera->config.set_output_window_name("main_window");
    camera->config.set_enable_stereo(false);

    auto pipe = std::make_shared<gua::PipelineDescription>();
    pipe->add_pass(std::make_shared<gua::TriMeshPassDescription>());
    pipe->add_pass(std::make_shared<gua::SPointsPassDescription>());
    pipe->add_pass(std::make_shared<gua::TexturedQuadPassDescription>());
    pipe->add_pass(std::make_shared<gua::LightVisibilityPassDescription>());
    pipe->add_pass(std::make_shared<gua::ResolvePassDescription>());
    pipe->set_enable_abuffer(true);
    pipe->set_abuffer_size(1500);
    pipe->set_blending_termination_threshold(0.99f);
    pipe->get_resolve_pass()->background_color(gua::utils::Color3f(0.7f, 0.7f, 0.7f)).environment_lighting(gua::utils::Color3f(0.005f, 0.005f, 0.005f));

    std::string frag_shader_source = "";
    std::string line_buffer = "";
    std::ifstream in_shader_file("data/shaders/effect3.frag", std::ios::in);

    while(std::getline(in_shader_file, line_buffer))
    {
        frag_shader_source += std::string(line_buffer + "\n");
    }

    in_shader_file.close();

    std::ifstream in_shader_file2("data/shaders/effect4.frag", std::ios::in);
    std::string frag_shader_source2 = "";
    while(std::getline(in_shader_file2, line_buffer))
    {
        frag_shader_source2 += std::string(line_buffer + "\n");
    }
    in_shader_file2.close();

    // auto fullscreen_effect_pass = std::make_shared<gua::FullscreenPassDescription>();
    // fullscreen_effect_pass->source(frag_shader_source);
    // pipe->add_pass(fullscreen_effect_pass);
    // auto fullscreen_effect_pass2 = std::make_shared<gua::FullscreenPassDescription>();
    // fullscreen_effect_pass2->source(frag_shader_source2);
    // fullscreen_effect_pass2->writes_only_color_buffer(true);
    // pipe->add_pass(fullscreen_effect_pass2);

    pipe->add_pass(std::make_shared<gua::SSAAPassDescription>());
    pipe->get_ssaa_pass()->mode(gua::SSAAPassDescription::SSAAMode::FXAA311);
    // pipe->add_pass(std::make_shared<gua::DebugViewPassDescription>());
    camera->set_pipeline_description(pipe);

    /*
      // pipeline setup
      auto& p_desc = camera->get_pipeline_description();
      p_desc->set_enable_abuffer(true);
      p_desc->set_abuffer_size(1500);
      p_desc->set_blending_termination_threshold(0.99f);
      p_desc->get_resolve_pass()->background_color(gua::utils::Color3f(0.7f,0.7f,0.7f))
                                 .environment_lighting(gua::utils::Color3f(0.005f,0.005f,0.005f));
                                 //.ssao_enable(true).ssao_radius(3.0).ssao_intensity(4.f);
      p_desc->add_pass(std::make_shared<gua::SSAAPassDescription>());
      p_desc->get_ssaa_pass()->mode(gua::SSAAPassDescription::SSAAMode::FXAA311);
    */

    // window
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
        resolution = new_size;
    });

    gua::node::TriMeshNode* selected_part = nullptr;
    bool action_mode = false;
    auto cut_parts = {"part_08", "part_10", "part_02", "part_01", "part_07", "part_03", "part_06"};

    // special
    auto uniform_change = [](const std::shared_ptr<gua::Material>& m, const std::string& name, float delta) {
        float h = boost::get<float>(m->get_uniforms().at(name).get().data);
        if((delta > 0 && h >= 1.f) || (delta < 0 && h <= 0.f))
            return false;
        m->set_uniform(name, h + delta);
        return true;
    };

    // functions
    float global_opacity = 1.f;
    auto apply_global_opacity = [&]() {
        for(const auto& p : parts)
        {
            p.second->get_material()->set_uniform("opacity", global_opacity);
            p.second->get_material()->set_uniform("highlight", 0.f);
        }
    };

    auto apply_cut = [&](const gua::math::vec3& pos, const gua::math::vec3& n) {
        for(const auto& name : cut_parts)
        {
            parts[name]->get_material()->set_uniform("cut_pos", pos);
            parts[name]->get_material()->set_uniform("cut_n", n);
        }
    };

    auto apply_cut_radius = [&](float delta) {
        for(const auto& name : cut_parts)
        {
            uniform_change(parts[name]->get_material(), "cut_rad", delta);
        }
    };

    auto assemble = [&anim_time](float delta) {
        anim_time += delta;
        if(delta > 0 && anim_time >= 1.2)
        {
            anim_time = 1.2;
            return false;
        }
        if(delta < 0 && anim_time <= 0.0)
        {
            anim_time = 0.0;
            return false;
        }
        return true;
    };

    auto reset_all = [&]() {
        for(const auto& p : parts)
        {
            tasks[p.first] = [p, uniform_change]() {
                return uniform_change(p.second->get_material(), "cut_rad", -0.01f) || uniform_change(p.second->get_material(), "opacity", 0.01f) ||
                       uniform_change(p.second->get_material(), "highlight", -0.01f);
            };
        }
        global_opacity = 1.f;
        selected_part = nullptr;
        tasks["a"] = [&]() { return assemble(0.005); };
    };

    auto compute_ray = [&](double x, double y) -> vec3 {
        auto sz = screen->data.get_size();
        vec4 t = screen->get_world_transform() * vec4(x * sz.x / 2, y * sz.y / 2, 0.0, 1.0);
        t /= t.w;
        return normalize(vec3(t) - get_translation(camera->get_world_transform()));
    };

    // mouse move
    window->on_move_cursor.connect([&](gua::math::vec2 const& pos) {
        // current mouse pos
        double nx = 2.0 * double(pos.x - (resolution.x / 2)) / resolution.x;
        double ny = -2.0 * double(resolution.y - pos.y - (resolution.y / 2)) / resolution.y;

        // show-through
        if(mode == Mode::SeeThrough && action_mode)
        {
            apply_cut(get_translation(camera->get_world_transform()), compute_ray(nx, ny));
        }

        // highlight
        if(mode == Mode::Highlight)
        {
            const double ray_dist = 20.0;
            std::set<gua::PickResult> picks =
                graph.ray_test(gua::Ray(get_translation(camera->get_world_transform()), compute_ray(nx, ny) * ray_dist, 1.0),
                               gua::PickResult::PICK_ONLY_FIRST_OBJECT | gua::PickResult::PICK_ONLY_FIRST_FACE | gua::PickResult::GET_WORLD_POSITIONS | gua::PickResult::GET_POSITIONS);

            if(!picks.empty() && button_state == -1)
            {
                auto picked = dynamic_cast<gua::node::TriMeshNode*>(picks.begin()->object);
                if(picked != selected_part)
                {
                    if(selected_part)
                    {
                        tasks[selected_part->get_name()] = [selected_part, uniform_change]() { return uniform_change(selected_part->get_material(), "highlight", -0.01f); };
                    }
                    tasks[picked->get_name()] = [picked, uniform_change]() { return uniform_change(picked->get_material(), "highlight", 0.01f); };
                    selected_part = picked;
                }
            }
            else
            {
                if(selected_part)
                {
                    tasks[selected_part->get_name()] = [selected_part, uniform_change]() { return uniform_change(selected_part->get_material(), "highlight", -0.01f); };
                }
                selected_part = nullptr;
            }
        }

        // navigation
        if(!action_mode && button_state != -1)
        {
            if(button_state == 0)
            { // left
                trackball.rotation(trackball_init_pos.x, trackball_init_pos.y, (nx), (ny));
            }
            if(button_state == 1)
            { // right
                trackball.dolly(dolly_sens * 0.5 * ((ny)-trackball_init_pos.y));
            }
            if(button_state == 2)
            { // middle
                float f = dolly_sens < 1.0f ? 0.02f : 0.3f;
                trackball.translation(f * ((nx)-trackball_init_pos.x), f * (float(ny) - trackball_init_pos.y));
            }
            trackball_init_pos.x = nx;
            trackball_init_pos.y = ny;
        }
        last_mouse_pos.x = nx;
        last_mouse_pos.y = ny;
    });

    // mouse scroll
    window->on_scroll.connect([&](gua::math::vec2 const& pos) {
        if(mode == Mode::ModelOpacity)
        {
            global_opacity += pos.y * 0.1;
            global_opacity = saturate(global_opacity);
            apply_global_opacity();
        }
        if(mode == Mode::SeeThrough)
        {
            apply_cut_radius(pos.y * 0.02);
        }
        if(mode == Mode::Highlight)
        {
            if(selected_part)
            {
                uniform_change(selected_part->get_material(), "opacity", pos.y * 0.1);
            }
        }
    });

    // mouse press
    window->on_button_press.connect([&](int mousebutton, int action, int mods) {
        if(action == 1)
        {
            trackball_init_pos = last_mouse_pos;
            button_state = mousebutton;
        }
        else
            button_state = -1;
    });

    // key press
    window->on_key_press.connect([&](int key, int scancode, int action, int mods) {
        if(key == 340)
        { // action mode (SHIFT)
            action_mode = (action != 0);

            if(mode == Mode::SeeThrough && action_mode)
            {
                apply_cut(get_translation(camera->get_world_transform()), compute_ray(last_mouse_pos.x, last_mouse_pos.y));
            }
        }

        if(action != 0)
        {
            // assembly
            if('A' == key)
            {
                if(anim_time < 0.00001)
                {
                    tasks["a"] = [&]() { return assemble(0.0005); };
                }
                else if(anim_time > 1.0)
                {
                    tasks["a"] = [&]() { return assemble(-0.0005); };
                }
            }

            // modes
            if('1' == key)
            {
                mode = Mode::SeeThrough;
                reset_all();
                std::cout << "See-Through mode" << std::endl;
            }
            if('2' == key)
            {
                mode = Mode::Highlight;
                reset_all();
                std::cout << "Highlight mode" << std::endl;
            }
            if('3' == key)
            {
                mode = Mode::ModelOpacity;
                reset_all();
                std::cout << "Model opacity mode" << std::endl;
            }
            if('4' == key)
            {
                avatar_2_geode->translate(0.0f, -10.0f, 0.0);
                avatar_1_geode->translate(0.0f, -10.0f, 0.0);

                for(const auto& p : part_names)
                {
                    parts[p.name]->translate(0.0f, -10.0f, 0.0);
                }
            }
            if('5' == key)
            {
                avatar_2_geode->translate(0.0f, 10.0f, 0.0);
                avatar_1_geode->translate(0.0f, 10.0f, 0.0);

                for(const auto& p : part_names)
                {
                    parts[p.name]->translate(0.0f, 10.0f, 0.0);
                }
            }
            // A-buffer
            if('T' == key)
            {
                auto& desc = camera->get_pipeline_description();
                desc->set_enable_abuffer(!desc->get_enable_abuffer());
                std::cout << "Enable A-Buffer: " << desc->get_enable_abuffer() << std::endl;
            }
        }
    });

    window->open();

    gua::Renderer renderer;

    // application loop
    gua::events::MainLoop loop;
    gua::events::Ticker ticker(loop, 1.0 / 500.0);

    size_t ctr{};

    ticker.on_tick.connect([&]() {
        // navigation
        screen->set_transform(scm::math::inverse(gua::math::mat4(trackball.transform_matrix())));

        // task execution
        std::vector<std::string> erases;
        for(auto& t : tasks)
        {
            bool result = t.second();
            if(!result)
                erases.push_back(t.first);
        }
        for(auto& e : erases)
            tasks.erase(e);

        // animation
        for(const auto& p : parts)
        {
            const auto& ev = anim_events[p.first];
            if(ev.duration < 0.001)
                continue;
            float anim_ts = (anim_time - ev.start) / ev.duration;

            float t = smootherstep(0.f, 1.f, saturate(anim_ts / 1.8));
            p.second->get_material()->set_uniform("opacity_total", t);

            float t_sm = smootherstep(0.f, 1.f, saturate(anim_ts));
            p.second->set_transform(scm::math::make_translation(ev.translation * (1.f - t_sm)));
        }

        if(SHOW_FRAME_RATE && ctr++ % 150 == 0)
        {
            std::cout << "Frame time: " << 1000.f / window->get_rendering_fps() << " ms, fps: " << window->get_rendering_fps() << ", app fps: " << renderer.get_application_fps() << std::endl;
        }

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

    // info
    std::cout << "Keys:" << std::endl
              << "  Assemble/disassemble:         <A>" << std::endl
              << "  Activate/deactivate A-buffer: <T>" << std::endl
              << "  Modes: " << std::endl
              << "    See-through:   <1> (<SHIFT> - move cutting cylinder, Mouse Scroll - radius size)" << std::endl
              << "    Highlight:     <2> (Mouse Scroll - opacity)" << std::endl
              << "    Model opacity: <3> (Mouse Scroll - opacity)" << std::endl;

    loop.start();

    return 0;
}
