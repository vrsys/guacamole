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
#include <gua/renderer/TriMeshRenderer.hpp>

#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/Resources.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

TriMeshRenderer::TriMeshRenderer() {}

////////////////////////////////////////////////////////////////////////////////

void TriMeshRenderer::draw(std::shared_ptr<GeometryResource> const& object,
                           std::shared_ptr<Material> const& material,
                           MaterialInstance const& material_overwrite,
                           math::mat4 const& transformation,
                           Pipeline* pipe) const {

  auto shader(material->get_shader(*object,
                                   Resources::lookup_shader(
                                    Resources::shaders_tri_mesh_shader_vert
                                   ),
                                   Resources::lookup_shader(
                                    Resources::shaders_tri_mesh_shader_frag
                                   )
                                  )
              );

  auto const& ctx(pipe->get_context());

  shader->use(ctx);

  shader->set_uniform(ctx, transformation, "gua_model_matrix");
  shader->set_uniform(ctx, scm::math::transpose(scm::math::inverse(transformation)), "gua_normal_matrix");

  material->apply_uniforms(ctx, shader, material_overwrite);

  object->draw(ctx);
}

////////////////////////////////////////////////////////////////////////////////

}

