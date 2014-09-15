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
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/MaterialDatabase.hpp>
#include <gua/utils/Logger.hpp>

namespace gua {

void GBufferPass::process(Pipeline* pipe) {
  RenderContext const& ctx(pipe->get_context());
  
  pipe->get_gbuffer().bind(ctx, this);
  pipe->get_gbuffer().set_viewport(ctx);

  for (auto const& type_ressource_pair : pipe->get_scene().geometrynodes_) {
    auto const& ressources = type_ressource_pair.second;
    std::shared_ptr<RessourceRenderer> renderer;

    for (auto const& object : ressources) {

      auto const& ressource = GeometryDatabase::instance()->lookup(object->get_filename());
      if (ressource) {

        auto const& material = MaterialDatabase::instance()->lookup(object->get_material().get_material_name());
        if (material) {

          if (!renderer) {
            renderer = pipe->get_renderer(*ressource);
          }

          renderer->draw(ressource, material, object->get_material(), object->get_cached_world_transform(), pipe);

        } else {
          Logger::LOG_WARNING << "GBufferPass::process(): Cannot find material: " << object->get_material().get_material_name() << std::endl;
        }

      } else {
        Logger::LOG_WARNING << "GBufferPass::process(): Cannot find geometry ressource: " << object->get_filename() << std::endl;
      }
    }
  }

  pipe->get_gbuffer().unbind(ctx);
}

}
