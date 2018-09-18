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

#ifndef GUA_DEFERREDVIRTUALTEXTURINGRENDERER_HPP
#define GUA_DEFERREDVIRTUALTEXTURINGRENDERER_HPP

#include <string>
#include <map>
#include <unordered_map>

// guacamole vt headers
#include <gua/virtual_texturing/DeferredVirtualTexturingPass.hpp>

// guacamole core headers
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/View.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/ResourceFactory.hpp>



#include <scm/core.h>


namespace gua {
namespace vt {
  class MaterialShader;
  class ShaderProgram;
  //class plod_shared_resources;

  class DeferredVirtualTexturingRenderer {
 
  public:

    DeferredVirtualTexturingRenderer();
    void render(Pipeline& pipe, PipelinePassDescription const& desc);
    void set_global_substitution_map(SubstitutionMap const& smap);
    void apply_cut_update(gua::RenderContext const& ctx, uint64_t cut_id, uint16_t ctx_id);
    void update_index_texture(gua::RenderContext const& ctx, uint64_t cut_id, uint32_t dataset_id, uint16_t context_id, const uint8_t *buf_cpu);
    void update_physical_texture_blockwise(gua::RenderContext const& ctx, uint16_t context_id, const uint8_t *buf_texel, size_t slot_position);
    void collect_feedback(gua::RenderContext const& ctx);
    

  private:  //shader related auxiliary methods

    void  _create_gpu_resources(gua::RenderContext const& ctx/*, uint64_t cut_id,
                                        scm::math::vec2ui const& render_target_dims*/); 
    
    void  _check_for_resource_updates(gua::Pipeline const& pipe, RenderContext const& ctx);

  private:  
  
  private:
    unsigned                                                                          previous_frame_count_;
    SubstitutionMap                                                                   global_substitution_map_;
    ResourceFactory                                                                   factory_;
  };

}
}

#endif // GUA_DEFERREDVIRTUALTEXTURINGRENDERER_HPP