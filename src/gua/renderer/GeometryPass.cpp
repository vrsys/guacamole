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
#include <gua/renderer/GeometryPass.hpp>

#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/renderer/RessourceRenderer.hpp>
#include <gua/databases/GeometryDatabase.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

PipelinePassDescription* GeometryPassDescription::make_copy() const {
  return new GeometryPassDescription(*this);
}


////////////////////////////////////////////////////////////////////////////////

PipelinePass GeometryPassDescription::make_pass(RenderContext const& ctx) const {
  PipelinePass pass{};

  pass.needs_color_buffer_as_input_ = false;
  pass.writes_only_color_buffer_ = false;

  auto renderers_ = std::make_shared<
    std::unordered_map<std::type_index, std::shared_ptr<RessourceRenderer>>>();

  pass.process_ = [renderers_](PipelinePass& pass, PipelinePassDescription*, Pipeline& pipe) {

    auto get_renderer = [&](std::type_index const& id)
        -> std::shared_ptr<RessourceRenderer> {
      auto renderer = renderers_->find(id);

      if (renderer != renderers_->end()) {
        return renderer->second;
      }

      auto new_renderer = RessourceRenderer::get_renderer(id);
      (*renderers_)[id] = new_renderer;

      return new_renderer;
    };

    auto const& ctx(pipe.get_context());

    pipe.get_gbuffer().bind(ctx, &pass);
    pipe.get_gbuffer().set_viewport(ctx);

    for (auto const& type_ressource_pair : pipe.get_scene().geometrynodes_) {
      auto const& ressources = type_ressource_pair.second;
      if (ressources.size() > 0 && ressources.begin()->second.size() > 0) {
        auto const& renderer = get_renderer(type_ressource_pair.first);
        if (renderer)
          renderer->draw(ressources, &pipe);
        else
          Logger::LOG_WARNING << "Unable to render geometry of type "
                              << type_ressource_pair.first.name()
                              << ": No renderer registered!"
                              << std::endl;
      }
    }

    pipe.get_gbuffer().unbind(ctx);
  };

  return pass;
}

////////////////////////////////////////////////////////////////////////////////

}
