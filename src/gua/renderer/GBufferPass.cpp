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
#include <gua/renderer/GBufferPass.hpp>

#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/databases/GeometryDatabase.hpp>

namespace gua {

void GBufferPass::process(Pipeline* pipe) {
  RenderContext const& ctx(pipe->get_context());

  pipe->get_gbuffer().bind(ctx, this);
  pipe->get_gbuffer().set_viewport(ctx);

  for (auto const& type_ressource_pair : pipe->get_scene().geometrynodes_) {
    auto const& ressources = type_ressource_pair.second;

    if (ressources.size() > 0 && ressources.begin()->second.size() > 0) {
      auto const& ressource = GeometryDatabase::instance()->lookup(ressources.begin()->second[0]->get_filename());
      auto const& renderer = pipe->get_renderer(*ressource);
      renderer->draw(ressources, pipe);
    }
  }

  pipe->get_gbuffer().unbind(ctx);
}

}
