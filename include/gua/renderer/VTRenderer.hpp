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

#ifndef GUA_VT_RENDERER_HPP
#define GUA_VT_RENDERER_HPP

#include <gua/platform.hpp>
#include <gua/config.hpp>

#ifdef GUACAMOLE_ENABLE_VIRTUAL_TEXTURING

#include <scm/gl_core/shader_objects.h>

#include <gua/node/TriMeshNode.hpp>

#include <gua/renderer/ResourceFactory.hpp>
#include <gua/renderer/TriMeshRessource.hpp>
#include <gua/renderer/Pipeline.hpp>

#include <gua/databases/Resources.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>

#include <scm/core/math/math.h>
#include <scm/core/io/tools.h>

#include <gua/virtual_texturing/VTBackend.hpp>

// lamure headers
#include <lamure/vt/common.h>
#include <lamure/vt/VTConfig.h>
#include <lamure/vt/ren/CutDatabase.h>
#include <lamure/vt/ren/CutUpdate.h>
#include <boost/assign.hpp>

namespace gua
{
class RenderContext;
class Pipeline;
class PipelinePassDescription;

class GUA_DLL VTRenderer
{
  public:
    VTRenderer(RenderContext const& ctx, SubstitutionMap const& smap);

    void pre_render(Pipeline& pipe) const;
    void post_render(Pipeline& pipe) const;

  protected:
    scm::gl::program_ptr shader_vt_feedback_;

    void _lazy_create_physical_texture(const RenderContext& ctx) const;
    void _apply_cut_update(const RenderContext& ctx) const;
    void _update_feedback_layout(const RenderContext& ctx) const;
    void _collect_feedback(const RenderContext& ctx) const;
};

} // namespace gua

#endif

#endif // GUA_TRIMESH_RENDERER_HPP
