#include "common.h"
#include "pagoda.cpp"

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
    }
}

int main(int argc, char **argv)
{
    bool should_close = false;

    gua::init(argc, argv);
    gua::SceneGraph graph("main_scenegraph");
    gua::TriMeshLoader loader;

    auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");

    auto light2 = graph.add_node<gua::node::LightNode>("/", "light2");
    light2->data.set_type(gua::node::LightNode::Type::SUN);
    light2->data.set_brightness(2.f);
    light2->data.set_shadow_cascaded_splits({0.1f, 1.f, 2.f, 5.f});
    light2->data.set_shadow_near_clipping_in_sun_direction(1.0f);
    light2->data.set_shadow_far_clipping_in_sun_direction(100.0f);
    light2->data.set_enable_shadows(false);
    light2->data.set_shadow_map_size(1024);
    light2->rotate(-45, 1, 1, 0);

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

    Pagoda pagoda;
    pagoda.bind_scene_graph(&graph);
    pagoda.bind_transport_layer(argc, argv);

    gua::Renderer renderer;

    gua::events::MainLoop loop;
    gua::events::Ticker ticker(loop, 1.0 / 500.0);

    ticker.on_tick.connect([&]() {
        if(window->should_close() || should_close)
        {
            renderer.stop();
            window->close();
            loop.stop();
            pagoda.halt_transport_layer();
        }
        else
        {
            gua::math::mat4 modelmatrix = scm::math::make_translation(trackball.shiftx(), trackball.shifty(), trackball.distance()) * gua::math::mat4(trackball.rotation());
            transform->set_transform(modelmatrix);

            pagoda.lock_scenegraph();

            // TODO: PreRender fails
            // pagoda.get_tproc()->PreRender();

            renderer.queue_draw({&graph});
            pagoda.unlock_scenegraph();
        }
    });

    loop.start();

    return 0;
}