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

// class header
#include <gua/renderer/Renderer.hpp>

// guacamole headers
#include <memory>
#include <tuple>

#include <gua/platform.hpp>
#include <gua/scenegraph.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/WindowDatabase.hpp>
#include <gua/node/CameraNode.hpp>
#include <gua/utils.hpp>
#include <gua/concurrent/Doublebuffer.hpp>
#include <gua/concurrent/pull_items_iterator.hpp>
#include <gua/memory.hpp>
#include <gua/config.hpp>

namespace
{
void display_loading_screen(gua::WindowBase& window)
{
    auto loading_texture(gua::TextureDatabase::instance()->lookup("gua_loading_texture"));
    gua::math::vec2ui loading_texture_size(loading_texture->width(), loading_texture->height());

    auto tmp_left_resolution(window.config.left_resolution());
    auto tmp_right_resolution(window.config.right_resolution());

    auto tmp_left_position(window.config.left_position());
    auto tmp_right_position(window.config.right_position());

    window.config.set_left_resolution(loading_texture_size);
    window.config.set_left_position(tmp_left_position + (tmp_left_resolution - loading_texture_size) / 2);

    window.config.set_right_resolution(loading_texture_size);
    window.config.set_right_position(tmp_right_position + (tmp_right_resolution - loading_texture_size) / 2);

    // window.display(loading_texture);
    window.finish_frame();
    ++(window.get_context()->framecount);

    window.config.set_left_position(tmp_left_position);
    window.config.set_left_resolution(tmp_left_resolution);

    window.config.set_right_position(tmp_right_position);
    window.config.set_right_resolution(tmp_right_resolution);
}

template <class T>
using DB = std::shared_ptr<gua::concurrent::Doublebuffer<T>>;

template <class T>
std::pair<DB<T>, DB<T>> spawnDoublebufferred()
{
    auto db = std::make_shared<gua::concurrent::Doublebuffer<T>>();
    return {db, db};
}

} // namespace

namespace gua
{
std::shared_ptr<const Renderer::SceneGraphs> garbage_collected_copy(std::vector<SceneGraph const*> const& scene_graphs)
{
    auto sgs = std::make_shared<Renderer::SceneGraphs>();
    for(auto graph : scene_graphs)
    {
        sgs->push_back(gua::make_unique<SceneGraph>(*graph));
    }
    return sgs;
}

Renderer::~Renderer() { stop(); }

void Renderer::renderclient(Mailbox in, std::string window_name)
{
    FpsCounter fpsc(20);
    fpsc.start();

    for(auto& cmd : gua::concurrent::pull_items_range<Item, Mailbox>(in))
    {
        // auto window_name(cmd.serialized_cam->config.get_output_window_name());

        if(window_name != "")
        {
            auto window = WindowDatabase::instance()->lookup(window_name);

            if(window && !window->get_is_open())
            {
                window->open();
            }

            // update window if one is assigned
            if(window && window->get_is_open())
            {
                window->set_active(true);
                window->start_frame();

                if(window->get_context()->framecount == 0)
                {
                    display_loading_screen(*window);
                }

                // make sure pipeline was created
                std::shared_ptr<Pipeline> pipe = nullptr;
                auto pipe_iter = window->get_context()->render_pipelines.find(cmd.serialized_cam->uuid);

                if(pipe_iter == window->get_context()->render_pipelines.end())
                {
                    pipe = std::make_shared<Pipeline>(*window->get_context(), cmd.serialized_cam->config.get_resolution());

                    window->get_context()->render_pipelines.insert(std::make_pair(cmd.serialized_cam->uuid, pipe));
                }
                else
                {
                    pipe = pipe_iter->second;
                }

                window->rendering_fps = fpsc.fps;

                pipe->fulfil_pre_render_responsibilities(*window->get_context());

                if(cmd.serialized_cam->config.get_enable_stereo())
                {
                    if(window->config.get_stereo_mode() == StereoMode::NVIDIA_3D_VISION)
                    {
#ifdef GUACAMOLE_ENABLE_NVIDIA_3D_VISION
                        if((window->get_context()->framecount % 2) == 0)
                        {
                            auto img(pipe->render_scene(CameraMode::LEFT, *cmd.serialized_cam, *cmd.scene_graphs));
                            if(img)
                                window->display(img, true);
                        }
                        else
                        {
                            auto img(pipe->render_scene(CameraMode::RIGHT, *cmd.serialized_cam, *cmd.scene_graphs));
                            if(img)
                                window->display(img, false);
                        }
#else
                        Logger::LOG_WARNING << "guacamole has not been compiled with NVIDIA 3D Vision support!" << std::endl;
#endif
                    }
                    else if(window->config.get_stereo_mode() == StereoMode::SEPARATE_WINDOWS)
                    {
                        bool is_left = cmd.serialized_cam->config.get_left_output_window() == window_name;
                        // auto mode = window->config.get_is_left() ? CameraMode::LEFT : CameraMode::RIGHT;
                        auto mode = is_left ? CameraMode::LEFT : CameraMode::RIGHT;
                        auto img = pipe->render_scene(mode, *cmd.serialized_cam, *cmd.scene_graphs);
                        if(img)
                        {
                            window->display(img, false);
                        }
                    }
                    else
                    {
                        // TODO: add alternate frame rendering here? -> take clear and render methods
                        auto img(pipe->render_scene(CameraMode::LEFT, *cmd.serialized_cam, *cmd.scene_graphs));
                        if(img)
                            window->display(img, true);
                        img = pipe->render_scene(CameraMode::RIGHT, *cmd.serialized_cam, *cmd.scene_graphs);
                        if(img)
                            window->display(img, false);
                    }
                }
                else
                {
                    auto img(pipe->render_scene(cmd.serialized_cam->config.get_mono_mode(), *cmd.serialized_cam, *cmd.scene_graphs));

                    if(img)
                        window->display(img, cmd.serialized_cam->config.get_mono_mode() != CameraMode::RIGHT);
                }

                pipe->clear_frame_cache();
                pipe->fulfil_post_render_responsibilities(*window->get_context());

                // swap buffers
                window->finish_frame();

                ++(window->get_context()->framecount);

                fpsc.step();
            }
        }
    }
}

Renderer::Renderer() : render_clients_(), application_fps_(20) { application_fps_.start(); }

void Renderer::send_renderclient(std::string const& window_name, std::shared_ptr<const Renderer::SceneGraphs> sgs, node::CameraNode* cam, bool alternate_frame_rendering)
{
    auto rclient = render_clients_.find(window_name);
    if(rclient != render_clients_.end())
    {
        rclient->second.first->push_back(Item(std::make_shared<node::SerializedCameraNode>(cam->serialize()), sgs, alternate_frame_rendering));
    }
    else
    {
        if(auto win = WindowDatabase::instance()->lookup(window_name))
        {
            auto p = spawnDoublebufferred<Item>();
            p.first->push_back(Item(std::make_shared<node::SerializedCameraNode>(cam->serialize()), sgs));
            render_clients_[window_name] = std::make_pair(p.first, std::thread(Renderer::renderclient, p.second, window_name));
        }
    }
}

void Renderer::queue_draw(std::vector<SceneGraph const*> const& scene_graphs, bool alternate_frame_rendering)
{
    for(auto graph : scene_graphs)
    {
        graph->update_cache();
    }

    auto sgs = garbage_collected_copy(scene_graphs);

    for(auto graph : scene_graphs)
    {
        for(auto& cam : graph->get_camera_nodes())
        {
            if(cam->config.separate_windows())
            {
                send_renderclient(cam->config.get_left_output_window(), sgs, cam, alternate_frame_rendering);
                send_renderclient(cam->config.get_right_output_window(), sgs, cam, alternate_frame_rendering);
            }
            else
            {
                send_renderclient(cam->config.get_output_window_name(), sgs, cam, alternate_frame_rendering);
            }
        }
    }

    application_fps_.step();
}

void Renderer::draw_single_threaded(std::vector<SceneGraph const*> const& scene_graphs)
{
    for(auto graph : scene_graphs)
    {
        graph->update_cache();
    }

    auto sgs = garbage_collected_copy(scene_graphs);

    for(auto graph : scene_graphs)
    {
        for(auto& cam : graph->get_camera_nodes())
        {
            auto window_name(cam->config.get_output_window_name());
            auto serialized_cam(cam->serialize());

            if(window_name != "")
            {
                auto window = WindowDatabase::instance()->lookup(window_name);

                if(window && !window->get_is_open())
                {
                    window->open();
                }
                // update window if one is assigned
                if(window && window->get_is_open())
                {
                    window->set_active(true);
                    window->start_frame();

                    if(window->get_context()->framecount == 0)
                    {
                        display_loading_screen(*window);
                    }

                    // make sure pipeline was created
                    std::shared_ptr<Pipeline> pipe = nullptr;
                    auto pipe_iter = window->get_context()->render_pipelines.find(serialized_cam.uuid);

                    if(pipe_iter == window->get_context()->render_pipelines.end())
                    {
                        pipe = std::make_shared<Pipeline>(*window->get_context(), serialized_cam.config.get_resolution());
                        window->get_context()->render_pipelines.insert(std::make_pair(serialized_cam.uuid, pipe));
                    }
                    else
                    {
                        pipe = pipe_iter->second;
                    }

                    window->rendering_fps = application_fps_.fps;

                    pipe->fulfil_pre_render_responsibilities(*window->get_context());

                    if(serialized_cam.config.get_enable_stereo())
                    {
                        if(window->config.get_stereo_mode() == StereoMode::NVIDIA_3D_VISION)
                        {
#ifdef GUACAMOLE_ENABLE_NVIDIA_3D_VISION
                            if((window->get_context()->framecount % 2) == 0)
                            {
                                auto img(pipe->render_scene(CameraMode::LEFT, serialized_cam, *sgs));
                                if(img)
                                    window->display(img, true);
                            }
                            else
                            {
                                auto img(pipe->render_scene(CameraMode::RIGHT, serialized_cam, *sgs));
                                if(img)
                                    window->display(img, false);
                            }
#else
                            Logger::LOG_WARNING << "guacamole has not been compiled with NVIDIA 3D Vision support!" << std::endl;
#endif
                        }
                        else
                        {
                            auto img(pipe->render_scene(CameraMode::LEFT, serialized_cam, *sgs));
                            if(img)
                                window->display(img, true);
                            img = pipe->render_scene(CameraMode::RIGHT, serialized_cam, *sgs);
                            if(img)
                                window->display(img, false);
                        }
                    }
                    else
                    {
                        auto img(pipe->render_scene(serialized_cam.config.get_mono_mode(), serialized_cam, *sgs));
                        if(img)
                            window->display(img, serialized_cam.config.get_mono_mode() != CameraMode::RIGHT);
                    }

                    pipe->clear_frame_cache();
                    pipe->fulfil_post_render_responsibilities(*window->get_context());

                    // swap buffers
                    window->finish_frame();
                    ++(window->get_context()->framecount);
                }
            }
        }
    }
    application_fps_.step();
}

void Renderer::stop()
{
    for(auto& rc : render_clients_)
    {
        rc.second.first->close();
    }
    for(auto& rc : render_clients_)
    {
        rc.second.second.join();
    }
    render_clients_.clear();
}
} // namespace gua
