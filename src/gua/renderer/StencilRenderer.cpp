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
#include <gua/renderer/StencilRenderer.hpp>

#include <gua/config.hpp>
#include <gua/node/TriMeshNode.hpp>

#include <gua/renderer/ResourceFactory.hpp>
#include <gua/renderer/TriMeshRessource.hpp>
#include <gua/renderer/Pipeline.hpp>

#include <gua/databases/Resources.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>

#include <scm/core/math/math.h>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////

StencilRenderer::StencilRenderer() {}

////////////////////////////////////////////////////////////////////////////////

void StencilRenderer::create_state_objects(RenderContext const& ctx)
{
    rs_cull_back_ = ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_BACK);
    rs_cull_none_ = ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE);
}

////////////////////////////////////////////////////////////////////////////////

void StencilRenderer::render(Pipeline& pipe, std::shared_ptr<ShaderProgram> const& shader)
{
    auto const& scene = *pipe.current_viewstate().scene;

    auto objects(scene.nodes.find(std::type_index(typeid(node::TriMeshNode))));

    if(objects != scene.nodes.end() && objects->second.size() > 0)
    {
        RenderContext const& ctx(pipe.get_context());

#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
        std::string const gpu_query_name = "GPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / StencilPass";
        std::string const cpu_query_name = "CPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / StencilPass";

        pipe.begin_gpu_query(ctx, gpu_query_name);
        pipe.begin_cpu_query(cpu_query_name);
#endif

        auto current_rasterizer_state = rs_cull_back_;
        ctx.render_context->apply();

        // loop through all objects, sorted by material ----------------------------
        for(auto const& object : objects->second)
        {
            auto tri_mesh_node(reinterpret_cast<node::TriMeshNode*>(object));

            if(!tri_mesh_node->get_render_to_stencil_buffer())
            {
                continue;
            }

            if(tri_mesh_node->get_geometry())
            {
                auto model_view_mat = scene.rendering_frustum.get_view() * tri_mesh_node->get_cached_world_transform();
                UniformValue normal_mat(math::mat4f(scm::math::transpose(scm::math::inverse(tri_mesh_node->get_cached_world_transform()))));

                shader->apply_uniform(ctx, "gua_model_matrix", math::mat4f(tri_mesh_node->get_cached_world_transform()));
                shader->apply_uniform(ctx, "gua_model_view_matrix", math::mat4f(model_view_mat));
                shader->apply_uniform(ctx, "gua_normal_matrix", normal_mat);

                current_rasterizer_state = tri_mesh_node->get_material()->get_show_back_faces() ? rs_cull_none_ : rs_cull_back_;

                if(ctx.render_context->current_rasterizer_state() != current_rasterizer_state)
                {
                    ctx.render_context->set_rasterizer_state(current_rasterizer_state);
                    ctx.render_context->apply_state_objects();
                }

                ctx.render_context->apply_program();

                tri_mesh_node->get_geometry()->draw(pipe.get_context());
            }
        }

#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
        pipe.end_gpu_query(ctx, gpu_query_name);
        pipe.end_cpu_query(cpu_query_name);
#endif

        ctx.render_context->reset_state_objects();
    }
}

////////////////////////////////////////////////////////////////////////////////

} // namespace gua
