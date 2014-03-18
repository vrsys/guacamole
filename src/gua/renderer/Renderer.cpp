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
#include <gua/utils.hpp>
#include <gua/utils/Doublebuffer.hpp>
#include <gua/utils/pull_items_iterator.hpp>
#include <gua/memory.hpp>

namespace gua {

template <class T>
std::pair<std::shared_ptr<gua::utils::Doublebuffer<T> >, std::shared_ptr<gua::utils::Doublebuffer<T> > > spawnDoublebufferred() {
  auto db = std::make_shared<gua::utils::Doublebuffer<T> >();
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
  for (auto& rc : render_clients_) { rc.first->close(); }
  for (auto& rc : render_clients_) { rc.second.join(); }
}

void Renderer::renderclient(Mailbox& in, Pipeline* pipe) {
  FpsCounter fpsc(20);
  fpsc.start();

  for (auto& x : gua::utils::pull_items_range<Item, Mailbox>(in)) {
    pipe->process(*(x.first), x.second, fpsc.fps);
    fpsc.step();
  }

}

Renderer::Renderclient Renderer::make_renderclient(Pipeline* pipe) {
  auto p = spawnDoublebufferred<Item>();
  return {p.first, std::thread{&Renderer::renderclient, this, p.second, pipe}};
}

Renderer::Renderer(std::vector<Pipeline*> const& pipelines)
    : render_clients_(),
      application_fps_(20) {
  application_fps_.start();
  for (auto& pipe : pipelines) {
    render_clients_.push_back(make_renderclient(pipe));
  }
}

void Renderer::queue_draw(std::vector<SceneGraph const*> const& scene_graphs) {
  for (auto graph : scene_graphs) {
    graph->update_cache();
  }
  auto sgs = garbage_collected_copy(scene_graphs);
  for (auto& rclient : render_clients_) {
    rclient.first->push_back({sgs, application_fps_.fps});
  }
  application_fps_.step();
}

}
