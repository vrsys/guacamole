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
#include <gua/platform.hpp>
#include <gua/scenegraph.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/WindowDatabase.hpp>
#include <gua/node/CameraNode.hpp>
#include <gua/utils.hpp>
#include <gua/concurrent/Doublebuffer.hpp>
#include <gua/concurrent/pull_items_iterator.hpp>
#include <gua/memory.hpp>

namespace gua {

template <class T>
std::pair<std::shared_ptr<gua::concurrent::Doublebuffer<T> >, std::shared_ptr<gua::concurrent::Doublebuffer<T> > > spawnDoublebufferred() {
  auto db = std::make_shared<gua::concurrent::Doublebuffer<T> >();
  return {db, db};
}

std::shared_ptr<Renderer::ConstRenderVector> garbage_collected_copy(
    std::vector<SceneGraph const*> const& scene_graphs) {
  auto sgs = std::make_shared<Renderer::RenderVector>();
  for (auto graph : scene_graphs) {
    sgs->push_back(gua::make_unique<SceneGraph>(*graph));
  }
  return sgs;
}


Renderer::~Renderer() {
  for (auto& rc : render_clients_) { rc.second.first->close(); }
  for (auto& rc : render_clients_) { rc.second.second.join(); }
}

void Renderer::renderclient(Mailbox in) {
  FpsCounter fpsc(20);
  fpsc.start();

  for (auto& x : gua::concurrent::pull_items_range<Item, Mailbox>(in)) {

    auto window_name(std::get<0>(x)->config.get_output_window_name());

    if (window_name != "") {
      auto window = WindowDatabase::instance()->lookup(window_name);

      // update window if one is assigned
      if (window) {
        if (!window->get_is_open()) {
          window->open();
          window->create_shader();
        }
        window->set_active(true);
      
        std::get<0>(x)->rendering_pipeline->process(
          window->get_context(), *std::get<0>(x), *std::get<1>(x), 
          std::get<2>(x), fpsc.fps
        );

        // swap buffers
        window->finish_frame();
        ++(window->get_context()->framecount);

        fpsc.step();
      }
    }
  }
}

Renderer::Renderer()
  : render_clients_(),
    application_fps_(20) {

  application_fps_.start();
}

void Renderer::queue_draw(std::vector<SceneGraph const*> const& scene_graphs,
                          std::vector<std::shared_ptr<node::CameraNode>> const& cameras) {
  for (auto graph : scene_graphs) {
    graph->update_cache();
  }
  auto sgs = garbage_collected_copy(scene_graphs);
  for (auto& cam : cameras) {
    auto window_name(cam->config.get_output_window_name());
    auto rclient(render_clients_.find(window_name));
    if (rclient != render_clients_.end()) {
      rclient->second.first->push_back(std::make_tuple(std::make_shared<node::SerializedCameraNode>(cam->serialize()), sgs, application_fps_.fps));
    } else {
      auto window(WindowDatabase::instance()->lookup(window_name));

      if (window) {
        // FpsCounter fpsc(20);
        // fpsc.start();
        // while(true) {
        //   auto c = cam->serialize();
        //   c.rendering_pipeline->process(c, *sgs, 10, fpsc.fps);
        //   fpsc.step();
        // }
        auto p = spawnDoublebufferred<Item>();
        p.first->push_back(std::make_tuple(std::make_shared<node::SerializedCameraNode>(cam->serialize()), sgs, application_fps_.fps));
        render_clients_[window_name] = std::make_pair(p.first, std::thread(Renderer::renderclient, p.second));
      } else {
        Logger::LOG_WARNING << "Cannot render camera: window \"" 
                            << window_name 
                            << "\" not registered in WindowDatabase!" 
                            << std::endl;
      }
    }
  }
  application_fps_.step();
}

void Renderer::stop() {
  for (auto& rclient : render_clients_) {
    rclient.second.first->close();
  }
}

}
