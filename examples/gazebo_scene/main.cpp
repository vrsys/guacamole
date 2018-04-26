#include <GLFW/glfw3.h>

#include <gua/guacamole.hpp>

#include <gua/renderer/SSAAPass.hpp>
#include <gua/renderer/MaterialLoader.hpp>
#include <gua/utils/Trackball.hpp>

#include <gua/nrp/pagoda_binder.hpp>
#include <gua/nrp/nrp_node.hpp>
#include <gua/nrp.hpp>

void mouse_button(gua::utils::Trackball &trackball, int mousebutton, int action, int mods)
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

void key_press(std::shared_ptr<gua::GlfwWindow> window, bool &should_close, int key, int scancode, int action, int mods)
{
    switch(key)
    {
    case GLFW_KEY_ESCAPE:
        should_close = true;
        break;
    }
}

int main(int argc, char **argv)
{
    bool should_close = false;

    gua::init(argc, argv);
    gua::SceneGraph graph("main_scenegraph");
    gua::TriMeshLoader loader;

    auto nrp_root = graph.add_node<gua::nrp::NRPNode>("/", "transform");

    auto plain_red_material(gua::MaterialShaderDatabase::instance()->lookup("gua_default_material")->make_new_material());
    plain_red_material->set_uniform("Color", gua::math::vec4(1.0f, 0.0f, 0.0f, 1.0f));
    auto plain_green_material(gua::MaterialShaderDatabase::instance()->lookup("gua_default_material")->make_new_material());
    plain_green_material->set_uniform("Color", gua::math::vec4(0.0f, 1.0f, 0.0f, 1.0f));
    auto plain_blue_material(gua::MaterialShaderDatabase::instance()->lookup("gua_default_material")->make_new_material());
    plain_blue_material->set_uniform("Color", gua::math::vec4(0.0f, 0.0f, 1.0f, 1.0f));

    auto x_axis = loader.create_geometry_from_file("x_axis", std::string(GUACAMOLE_INSTALL_DIR) + "/resources/geometry/primitive_sphere.obj", plain_red_material,
                                                   gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE);

    x_axis->scale(1.0, 0.01, 0.01);
    x_axis->translate(0.5, 0., 0.);

    graph.add_node("/transform", x_axis);

    auto y_axis = loader.create_geometry_from_file("y_axis", std::string(GUACAMOLE_INSTALL_DIR) + "/resources/geometry/primitive_sphere.obj", plain_green_material,
                                                   gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE);

    y_axis->scale(0.01, 1.0, 0.01);
    y_axis->translate(0., 0.5, 0.);

    graph.add_node("/transform", y_axis);

    auto z_axis = loader.create_geometry_from_file("z_axis", std::string(GUACAMOLE_INSTALL_DIR) + "/resources/geometry/primitive_sphere.obj", plain_blue_material,
                                                   gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE);

    z_axis->scale(0.01, 0.01, 1.0);
    z_axis->translate(0., 0., 0.5);

    graph.add_node("/transform", z_axis);

    auto screen = graph.add_node<gua::node::ScreenNode>("/", "screen");
    screen->data.set_size(gua::math::vec2(1.92f, 1.08f));
    screen->translate(0, 0, 1.0);

    gua::utils::Trackball trackball(0.01, 0.002, 0.2);

    auto resolution = gua::math::vec2ui(1920, 1080);

    auto camera = graph.add_node<gua::node::CameraNode>("/screen", "cam");
    camera->translate(0, 0, 2.0);
    camera->config.set_resolution(resolution);
    camera->config.set_screen_path("/screen");
    camera->config.set_scene_graph_name("main_scenegraph");
    camera->config.set_output_window_name("main_window");
    camera->config.set_enable_stereo(false);
    camera->config.set_near_clip(0.1f);
    camera->config.set_far_clip(1000.0f);

    auto pipe = std::make_shared<gua::PipelineDescription>();
    pipe->add_pass(std::make_shared<gua::nrp::NRPPassDescription>());
    pipe->add_pass(std::make_shared<gua::TriMeshPassDescription>());
    pipe->add_pass(std::make_shared<gua::LightVisibilityPassDescription>());
    pipe->add_pass(std::make_shared<gua::ResolvePassDescription>());
    pipe->add_pass(std::make_shared<gua::SSAAPassDescription>());

    camera->get_pipeline_description()->get_resolve_pass()->tone_mapping_exposure(1.0f);

    camera->get_pipeline_description()->get_resolve_pass()->ssao_intensity(1.0);
    camera->get_pipeline_description()->get_resolve_pass()->ssao_enable(true);
    camera->get_pipeline_description()->get_resolve_pass()->ssao_falloff(1.0);
    camera->get_pipeline_description()->get_resolve_pass()->ssao_radius(4.0);

    camera->get_pipeline_description()->get_resolve_pass()->screen_space_shadows(true);
    camera->get_pipeline_description()->get_resolve_pass()->screen_space_shadow_radius(1.0);
    camera->get_pipeline_description()->get_resolve_pass()->screen_space_shadow_intensity(0.9);

    camera->get_pipeline_description()->get_resolve_pass()->environment_lighting_mode(gua::ResolvePassDescription::EnvironmentLightingMode::AMBIENT_COLOR);
    camera->get_pipeline_description()->get_resolve_pass()->background_mode(gua::ResolvePassDescription::BackgroundMode::SKYMAP_TEXTURE);

    camera->set_pipeline_description(pipe);

    auto window = std::make_shared<gua::GlfwWindow>();
    gua::WindowDatabase::instance()->add("main_window", window);

    window->config.set_enable_vsync(true);
    window->config.set_size(resolution);
    window->config.set_resolution(resolution);
    window->config.set_stereo_mode(gua::StereoMode::MONO);

    window->on_resize.connect([&](gua::math::vec2ui const &new_size) {
        window->config.set_resolution(new_size);
        camera->config.set_resolution(new_size);
        screen->data.set_size(gua::math::vec2(0.001 * new_size.x, 0.001 * new_size.y));
    });

    window->on_move_cursor.connect([&](gua::math::vec2 const &pos) { trackball.motion(static_cast<int>(pos.x), static_cast<int>(pos.y)); });
    window->on_button_press.connect(std::bind(mouse_button, std::ref(trackball), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    window->on_key_press.connect(std::bind(key_press, window, std::ref(should_close), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

    gua::Renderer renderer;

    gua::events::MainLoop loop;
    gua::events::Ticker ticker(loop, 1.0 / 500.0);

    ticker.on_tick.connect([&]() {
        if(window->should_close() || should_close)
        {
            renderer.stop();
            window->close();
            loop.stop();
        }
        else
        {
            gua::math::mat4 modelmatrix = scm::math::make_translation(trackball.shiftx(), trackball.shifty(), trackball.distance()) * gua::math::mat4(trackball.rotation());
            nrp_root->set_transform(modelmatrix);

            renderer.queue_draw({&graph});
        }
    });

    loop.start();

    return 0;
}