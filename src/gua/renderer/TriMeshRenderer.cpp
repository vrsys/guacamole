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

#include <gua/config.hpp>
#include <gua/node/TriMeshNode.hpp>

#include <gua/renderer/ResourceFactory.hpp>
#include <gua/renderer/TriMeshRessource.hpp>
#include <gua/renderer/Pipeline.hpp>

#include <gua/databases/Resources.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>

#include <scm/core/math/math.h>

// lamure headers
#include <lamure/vt/common.h>
#include <lamure/vt/VTConfig.h>
#include <lamure/vt/ren/CutDatabase.h>
#include <lamure/vt/ren/CutUpdate.h>
#include <gua/virtual_texturing/VTBackend.hpp>

namespace {

gua::math::vec2ui get_handle(scm::gl::texture_image_ptr const& tex) {
  uint64_t handle = 0;
  if (tex) {
    handle = tex->native_handle();
  }
  return gua::math::vec2ui(handle & 0x00000000ffffffff, handle & 0xffffffff00000000);
}

}

namespace gua {

////////////////////////////////////////////////////////////////////////////////

TriMeshRenderer::TriMeshRenderer(RenderContext const& ctx, SubstitutionMap const& smap)
  : rs_cull_back_(ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_BACK))
  , rs_cull_none_(ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE))
  , rs_wireframe_cull_back_(ctx.render_device->create_rasterizer_state(scm::gl::FILL_WIREFRAME, scm::gl::CULL_BACK))
  , rs_wireframe_cull_none_(ctx.render_device->create_rasterizer_state(scm::gl::FILL_WIREFRAME, scm::gl::CULL_NONE))
  , program_stages_()
  , programs_()
  , global_substitution_map_(smap)
{
#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
  ResourceFactory factory;
  std::string v_shader = factory.read_shader_file("resources/shaders/tri_mesh_shader.vert");
  std::string f_shader = factory.read_shader_file("resources/shaders/tri_mesh_shader.frag");
#else
  std::string v_shader = Resources::lookup_shader("shaders/tri_mesh_shader.vert");
  std::string f_shader = Resources::lookup_shader("shaders/tri_mesh_shader.frag");
#endif

  program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER,   v_shader));
  program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, f_shader));
}

////////////////////////////////////////////////////////////////////////////////

void TriMeshRenderer::render(Pipeline& pipe, PipelinePassDescription const& desc)
{
  auto& scene = *pipe.current_viewstate().scene;
  auto sorted_objects(scene.nodes.find(std::type_index(typeid(node::TriMeshNode))));

  if (sorted_objects != scene.nodes.end() && sorted_objects->second.size() > 0) {

    auto& target = *pipe.current_viewstate().target;
    auto const& camera = pipe.current_viewstate().camera;

    std::sort(sorted_objects->second.begin(), sorted_objects->second.end(),
              [](node::Node* a, node::Node* b) {
                return reinterpret_cast<node::TriMeshNode*>(a)->get_material()->get_shader()
                     < reinterpret_cast<node::TriMeshNode*>(b)->get_material()->get_shader();
              });

    RenderContext const& ctx(pipe.get_context());

#ifdef GUACAMOLE_ENABLE_VIRTUAL_TEXTURING
    if(gua::VTBackend::get_instance().has_camera(pipe.current_viewstate().camera.uuid)){
      _create_physical_texture(ctx);
      ctx.render_context->sync();

      _apply_cut_update(ctx);
    }
#endif

    std::string const gpu_query_name = "GPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / TrimeshPass";
    std::string const cpu_query_name = "CPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / TrimeshPass";

    pipe.begin_gpu_query(ctx, gpu_query_name);
    pipe.begin_cpu_query(cpu_query_name);

    bool write_depth = true;
    target.bind(ctx, write_depth);
    target.set_viewport(ctx);

    int view_id(camera.config.get_view_id());

    MaterialShader*                current_material(nullptr);
    std::shared_ptr<ShaderProgram> current_shader;
    auto current_rasterizer_state = rs_cull_back_;
    ctx.render_context->apply();

    // loop through all objects, sorted by material ----------------------------
    for (auto const& object : sorted_objects->second) {

      auto tri_mesh_node(reinterpret_cast<node::TriMeshNode*>(object));
      if (pipe.current_viewstate().shadow_mode && tri_mesh_node->get_shadow_mode() == ShadowMode::OFF) {
        continue;
      }

      if (!tri_mesh_node->get_render_to_gbuffer()) {
        continue;
      }

      if (current_material != tri_mesh_node->get_material()->get_shader()) {
        current_material = tri_mesh_node->get_material()->get_shader();
        if (current_material) {

          auto shader_iterator = programs_.find(current_material);
          if (shader_iterator != programs_.end())
          {
            current_shader = shader_iterator->second;
          }
          else {
            auto smap = global_substitution_map_;
            for (const auto& i: current_material->generate_substitution_map())
              smap[i.first] = i.second;

            current_shader = std::make_shared<ShaderProgram>();

#ifndef GUACAMOLE_ENABLE_VIRTUAL_TEXTURING
            current_shader->set_shaders(program_stages_, std::list<std::string>(), false, smap);
#else
            bool virtual_texturing_enabled = tri_mesh_node->get_material()->get_enable_virtual_texturing();
            current_shader->set_shaders(program_stages_, std::list<std::string>(), false, smap, virtual_texturing_enabled);            
#endif
            programs_[current_material] = current_shader;
          }
        }
        else {
          Logger::LOG_WARNING << "TriMeshPass::process(): Cannot find material: "
                              << tri_mesh_node->get_material()->get_shader_name() << std::endl;
        }
        if (current_shader) {
          current_shader->use(ctx);
          current_shader->set_uniform(ctx, math::vec2ui(target.get_width(),
                                                       target.get_height()),
                                      "gua_resolution"); //TODO: pass gua_resolution. Probably should be somehow else implemented
          current_shader->set_uniform(ctx, 1.0f / target.get_width(),  "gua_texel_width");
          current_shader->set_uniform(ctx, 1.0f / target.get_height(), "gua_texel_height");
          // hack
          current_shader->set_uniform(ctx, ::get_handle(target.get_depth_buffer()),
                                        "gua_gbuffer_depth");
        }
      }

      if (current_shader && tri_mesh_node->get_geometry())
      {
        auto model_view_mat = scene.rendering_frustum.get_view() * tri_mesh_node->get_cached_world_transform();
        UniformValue normal_mat (math::mat4f(scm::math::transpose(scm::math::inverse(tri_mesh_node->get_cached_world_transform()))));

        int rendering_mode = pipe.current_viewstate().shadow_mode ? (tri_mesh_node->get_shadow_mode() == ShadowMode::HIGH_QUALITY ? 2 : 1) : 0;

        current_shader->apply_uniform(ctx, "gua_model_matrix", math::mat4f(tri_mesh_node->get_cached_world_transform()));
        current_shader->apply_uniform(ctx, "gua_model_view_matrix", math::mat4f(model_view_mat));
        current_shader->apply_uniform(ctx, "gua_normal_matrix", normal_mat);
        current_shader->apply_uniform(ctx, "gua_rendering_mode", rendering_mode);

        // lowfi shadows dont need material input
        if (rendering_mode != 1) {
          tri_mesh_node->get_material()->apply_uniforms(ctx, current_shader.get(), view_id);
        }

        bool show_backfaces   = tri_mesh_node->get_material()->get_show_back_faces();
        bool render_wireframe = tri_mesh_node->get_material()->get_render_wireframe();

        if (show_backfaces) {
          if (render_wireframe) {
            current_rasterizer_state = rs_wireframe_cull_none_;
          } else {
            current_rasterizer_state = rs_cull_none_;
          }
        } else {
          if (render_wireframe) {
            current_rasterizer_state = rs_wireframe_cull_back_;
          } else {
            current_rasterizer_state = rs_cull_back_;
          }
        }

        if (ctx.render_context->current_rasterizer_state() != current_rasterizer_state) {
          ctx.render_context->set_rasterizer_state(current_rasterizer_state);
          ctx.render_context->apply_state_objects();
        }

        ctx.render_context->apply_program();

        tri_mesh_node->get_geometry()->draw(pipe.get_context());
      }
    }

    target.unbind(ctx);

    pipe.end_gpu_query(ctx, gpu_query_name);
    pipe.end_cpu_query(cpu_query_name);

    ctx.render_context->reset_state_objects();

    ctx.render_context->sync();

#ifdef GUACAMOLE_ENABLE_VIRTUAL_TEXTURING
    if(gua::VTBackend::get_instance().has_camera(pipe.current_viewstate().camera.uuid)){
        _collect_feedback(ctx);
    }
#endif
  }
}

void TriMeshRenderer::_create_physical_texture(gua::RenderContext const& ctx) {
  if(VirtualTexture2D::physical_texture_ptr_per_context_[ctx.id] == nullptr) {
    VirtualTexture2D::physical_texture_ptr_per_context_[ctx.id] = std::make_shared<LayeredPhysicalTexture2D>();

    uint32_t phys_tex_creation_px_width  = ::vt::VTConfig::get_instance().get_phys_tex_px_width();
    uint32_t phys_tex_creation_px_height = ::vt::VTConfig::get_instance().get_phys_tex_px_width();
    uint32_t phys_tex_creation_num_layers = ::vt::VTConfig::get_instance().get_phys_tex_layers();;
    uint32_t phys_tex_creation_tile_size = ::vt::VTConfig::get_instance().get_size_tile();

    VirtualTexture2D::physical_texture_ptr_per_context_[ctx.id]->upload_to(ctx,
                                            phys_tex_creation_px_width, phys_tex_creation_px_height,
                                            phys_tex_creation_num_layers, phys_tex_creation_tile_size);
  }
}

void TriMeshRenderer::_apply_cut_update(gua::RenderContext const& ctx) {
  auto *cut_db = &::vt::CutDatabase::get_instance();

  auto vector_of_vt_ptr = TextureDatabase::instance()->get_virtual_textures();

  for (::vt::cut_map_entry_type cut_entry : (*cut_db->get_cut_map())) {

    if(::vt::Cut::get_context_id(cut_entry.first) != VirtualTexture2D::vt_info_per_context_[ctx.id].context_id_)
    {
      continue;
    }

    ::vt::Cut *cut = cut_db->start_reading_cut(cut_entry.first);

    if (!cut->is_drawn()) {
      cut_db->stop_reading_cut(cut_entry.first);
      continue;
    }

    std::set<uint16_t> updated_levels;

    for (auto position_slot_updated : cut->get_front()->get_mem_slots_updated()) {
      const ::vt::mem_slot_type *mem_slot_updated = cut_db->read_mem_slot_at(position_slot_updated.second, VirtualTexture2D::vt_info_per_context_[ctx.id].context_id_);

      if (mem_slot_updated == nullptr || !mem_slot_updated->updated
          || !mem_slot_updated->locked || mem_slot_updated->pointer == nullptr) {
        if (mem_slot_updated == nullptr) {
          std::cerr << "Mem slot at " << position_slot_updated.second << " is null" << std::endl;
        } else {
          std::cerr << "Mem slot at " << position_slot_updated.second << std::endl;
          std::cerr << "Mem slot #" << mem_slot_updated->position << std::endl;
          std::cerr << "Tile id: " << mem_slot_updated->tile_id << std::endl;
          std::cerr << "Locked: " << mem_slot_updated->locked << std::endl;
          std::cerr << "Updated: " << mem_slot_updated->updated << std::endl;
          std::cerr << "Pointer valid: " << (mem_slot_updated->pointer != nullptr) << std::endl;
        }

        throw std::runtime_error("updated mem slot inconsistency");
      }

      updated_levels.insert(vt::QuadTree::get_depth_of_node(mem_slot_updated->tile_id));

      // update_physical_texture_blockwise
      size_t slots_per_texture = ::vt::VTConfig::get_instance().get_phys_tex_tile_width() *
          ::vt::VTConfig::get_instance().get_phys_tex_tile_width();
      size_t layer = mem_slot_updated->position / slots_per_texture;
      size_t rel_slot_position = mem_slot_updated->position - layer * slots_per_texture;

      size_t x_tile = rel_slot_position % ::vt::VTConfig::get_instance().get_phys_tex_tile_width();
      size_t y_tile = rel_slot_position / ::vt::VTConfig::get_instance().get_phys_tex_tile_width();

      scm::math::vec3ui origin = scm::math::vec3ui(
          (uint32_t) x_tile * ::vt::VTConfig::get_instance().get_size_tile(),
          (uint32_t) y_tile * ::vt::VTConfig::get_instance().get_size_tile(), (uint32_t) layer);
      scm::math::vec3ui dimensions = scm::math::vec3ui(::vt::VTConfig::get_instance().get_size_tile(),
                                                       ::vt::VTConfig::get_instance().get_size_tile(), 1);


      auto physical_tex = VirtualTexture2D::physical_texture_ptr_per_context_[ctx.id]->get_physical_texture_ptr();



      auto phys_tex_format = scm::gl::FORMAT_RGBA_8;
      switch (::vt::VTConfig::get_instance().get_format_texture()) {
        case ::vt::VTConfig::R8:
          phys_tex_format = scm::gl::FORMAT_R_8;
              break;
        case ::vt::VTConfig::RGB8:
          phys_tex_format = scm::gl::FORMAT_RGB_8;
              break;
        case ::vt::VTConfig::RGBA8:
        default:
          phys_tex_format = scm::gl::FORMAT_RGBA_8;
              break;
      }


      ctx.render_context->update_sub_texture(physical_tex, scm::gl::texture_region(origin, dimensions), 0,
                                             phys_tex_format, mem_slot_updated->pointer);
    }


    for (auto position_slot_cleared : cut->get_front()->get_mem_slots_cleared()) {
      const ::vt::mem_slot_type *mem_slot_cleared = cut_db->read_mem_slot_at(position_slot_cleared.second, VirtualTexture2D::vt_info_per_context_[ctx.id].context_id_);

      if (mem_slot_cleared == nullptr) {
        std::cerr << "Mem slot at " << position_slot_cleared.second << " is null" << std::endl;
      }

      updated_levels.insert(::vt::QuadTree::get_depth_of_node(position_slot_cleared.first));
    }

    for( auto const& vt_ptr : vector_of_vt_ptr ) {
      if( ::vt::Cut::get_dataset_id(cut_entry.first) == vt_ptr->get_lamure_texture_id()) {
        // update_index_texture

        std::vector<std::pair<uint16_t, uint8_t*>> level_pairs_to_update;
        for (uint16_t updated_level : updated_levels) {
          uint8_t* level_address = cut->get_front()->get_index(updated_level);
          level_pairs_to_update.emplace_back(updated_level, level_address);

          vt_ptr->update_index_texture_hierarchy(ctx, level_pairs_to_update);
        }

      }
    }

    cut_db->stop_reading_cut(cut_entry.first);
  }
  ctx.render_context->sync();
}

void TriMeshRenderer::_collect_feedback(gua::RenderContext const &ctx) {
  auto& gua_layered_physical_texture_for_context = VirtualTexture2D::physical_texture_ptr_per_context_[ctx.id];

  auto feedback_lod_storage_ptr = gua_layered_physical_texture_for_context->get_feedback_lod_storage_ptr();
  auto feedback_lod_cpu_buffer_ptr = gua_layered_physical_texture_for_context->get_feedback_lod_cpu_buffer();

  std::size_t num_feedback_slots = gua_layered_physical_texture_for_context->get_num_feedback_slots();

  int32_t *feedback_lod = (int32_t *) ctx.render_context->map_buffer(feedback_lod_storage_ptr, scm::gl::ACCESS_READ_ONLY);

  memcpy(feedback_lod_cpu_buffer_ptr, feedback_lod, num_feedback_slots * size_of_format(scm::gl::FORMAT_R_32I));
  ctx.render_context->unmap_buffer(feedback_lod_storage_ptr);
  ctx.render_context->clear_buffer_data(feedback_lod_storage_ptr, scm::gl::FORMAT_R_32I, nullptr);


  auto feedback_count_storage_ptr = gua_layered_physical_texture_for_context->get_feedback_count_storage_ptr();
  auto feedback_count_cpu_buffer_ptr = gua_layered_physical_texture_for_context->get_feedback_count_cpu_buffer();

  uint32_t *feedback_count = (uint32_t *) ctx.render_context->map_buffer(feedback_count_storage_ptr,
                                                                         scm::gl::ACCESS_READ_ONLY);
  memcpy(feedback_count_cpu_buffer_ptr, feedback_count, num_feedback_slots * size_of_format(scm::gl::FORMAT_R_32UI));
  ctx.render_context->unmap_buffer(feedback_count_storage_ptr);
  ctx.render_context->clear_buffer_data(feedback_count_storage_ptr, scm::gl::FORMAT_R_32UI, nullptr);

  auto& vt_info_per_context = VirtualTexture2D::vt_info_per_context_;
  auto& current_vt_info = vt_info_per_context[ctx.id];

  if (current_vt_info.cut_update_){

    /*std::cout << "Context " << current_vt_info.context_id_ << std::endl;

    for(int i = 0; i < num_feedback_slots; i ++){
      std::cout << feedback_lod_cpu_buffer_ptr[i];
    }

    std::cout << std::endl;*/

    current_vt_info.cut_update_->feedback(current_vt_info.context_id_, feedback_lod_cpu_buffer_ptr, feedback_count_cpu_buffer_ptr);
  }

  ctx.render_context->sync();
}

////////////////////////////////////////////////////////////////////////////////

} // namespace gua
