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

#include <gua/renderer/VTRenderer.hpp>

#ifdef GUACAMOLE_ENABLE_VIRTUAL_TEXTURING

namespace
{
gua::math::vec2ui get_handle(scm::gl::texture_image_ptr const &tex)
{
    uint64_t handle = 0;
    if(tex)
    {
        handle = tex->native_handle();
    }
    return gua::math::vec2ui(handle & 0x00000000ffffffff, handle & 0xffffffff00000000);
}

} // namespace

namespace gua
{
VTRenderer::VTRenderer(RenderContext const &ctx, SubstitutionMap const &smap)
{
    ResourceFactory factory;

    {
        using namespace boost::assign;
        using namespace scm::gl;

        std::string vs_vt_feedback = factory.read_shader_file("resources/shaders/virtual_texturing_inv_index_generation.vert");

        shader_vt_feedback_ = ctx.render_device->create_program(list_of(ctx.render_device->create_shader(STAGE_VERTEX_SHADER, vs_vt_feedback)));
    }
}
VTContextState VTRenderer::pre_render(Pipeline &pipe, PipelinePassDescription const &desc)
{
    VTContextState state;

    if(!VirtualTexture2D::initialized_vt_system)
    {
        state.has_camera = false;
        state.feedback_enabled = false;
        return state;
    }

    state.has_camera = gua::VTBackend::get_instance().has_camera(pipe.current_viewstate().camera.uuid);
    state.feedback_enabled = false;

    if(state.has_camera)
    {
        RenderContext const &ctx(pipe.get_context());

        ctx.render_context->apply();
        _lazy_create_physical_texture(ctx);
        ctx.render_context->sync();

        auto &vt_info_per_context = VTBackend::get_instance().vt_info_per_context_;
        auto &current_vt_info = vt_info_per_context[ctx.id];

        if(current_vt_info.cut_update_ == nullptr)
        {
            state.has_camera = false;
            state.feedback_enabled = false;
            return state;
        }

        bool should_update = VTBackend::get_instance().should_update_on_context(pipe.current_viewstate().camera.uuid);
        state.feedback_enabled = current_vt_info.cut_update_->can_accept_feedback(current_vt_info.context_id_);

        if(state.feedback_enabled && should_update)
        {
            _update_feedback_layout(ctx);

            ctx.render_context->sync();

            _apply_cut_update(ctx);

            ctx.render_context->sync();
        }
    }

    return state;
}
void VTRenderer::post_render(Pipeline &pipe, PipelinePassDescription const &desc, VTContextState &state)
{
    if(state.has_camera)
    {
        RenderContext const &ctx(pipe.get_context());

        bool should_collect = VTBackend::get_instance().should_collect_feedback_on_context(pipe.current_viewstate().camera.uuid);
        if(state.feedback_enabled && should_collect)
        {
            _collect_feedback(ctx);
        }
    }
}

void VTRenderer::_lazy_create_physical_texture(gua::RenderContext const &ctx)
{
    if(VTBackend::get_instance().physical_texture_ptr_per_context_[ctx.id] == nullptr)
    {
        VTBackend::get_instance().physical_texture_ptr_per_context_[ctx.id] = std::make_shared<LayeredPhysicalTexture2D>();

        uint32_t phys_tex_creation_px_width = ::vt::VTConfig::get_instance().get_phys_tex_px_width();
        uint32_t phys_tex_creation_px_height = ::vt::VTConfig::get_instance().get_phys_tex_px_width();
        uint32_t phys_tex_creation_num_layers = ::vt::VTConfig::get_instance().get_phys_tex_layers();
        uint32_t phys_tex_creation_tile_size = ::vt::VTConfig::get_instance().get_size_tile();

        VTBackend::get_instance().physical_texture_ptr_per_context_[ctx.id]->upload_to(ctx, phys_tex_creation_px_width, phys_tex_creation_px_height, phys_tex_creation_num_layers,
                                                                                       phys_tex_creation_tile_size);
    }
}

void VTRenderer::_apply_cut_update(gua::RenderContext const &ctx)
{
    auto *cut_db = &::vt::CutDatabase::get_instance();

    auto vector_of_vt_ptr = TextureDatabase::instance()->get_virtual_textures();

    for(::vt::cut_map_entry_type cut_entry : (*cut_db->get_cut_map()))
    {
        if(::vt::Cut::get_context_id(cut_entry.first) != VTBackend::get_instance().vt_info_per_context_[ctx.id].context_id_)
        {
            continue;
        }

        ::vt::Cut *cut = cut_db->start_reading_cut(cut_entry.first);

        if(!cut->is_drawn())
        {
            cut_db->stop_reading_cut(cut_entry.first);
            continue;
        }

        std::set<uint16_t> updated_levels;

        for(auto position_slot_updated : cut->get_front()->get_mem_slots_updated())
        {
            const ::vt::mem_slot_type *mem_slot_updated = cut_db->read_mem_slot_at(position_slot_updated.second, VTBackend::get_instance().vt_info_per_context_[ctx.id].context_id_);

            if(mem_slot_updated == nullptr || !mem_slot_updated->updated || !mem_slot_updated->locked || mem_slot_updated->pointer == nullptr)
            {
                if(mem_slot_updated == nullptr)
                {
                    std::cerr << "Mem slot at " << position_slot_updated.second << " is null" << std::endl;
                }
                else
                {
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
            size_t slots_per_texture = ::vt::VTConfig::get_instance().get_phys_tex_tile_width() * ::vt::VTConfig::get_instance().get_phys_tex_tile_width();
            size_t layer = mem_slot_updated->position / slots_per_texture;
            size_t rel_slot_position = mem_slot_updated->position - layer * slots_per_texture;

            size_t x_tile = rel_slot_position % ::vt::VTConfig::get_instance().get_phys_tex_tile_width();
            size_t y_tile = rel_slot_position / ::vt::VTConfig::get_instance().get_phys_tex_tile_width();

            scm::math::vec3ui origin =
                scm::math::vec3ui((uint32_t)x_tile * ::vt::VTConfig::get_instance().get_size_tile(), (uint32_t)y_tile * ::vt::VTConfig::get_instance().get_size_tile(), (uint32_t)layer);
            scm::math::vec3ui dimensions = scm::math::vec3ui(::vt::VTConfig::get_instance().get_size_tile(), ::vt::VTConfig::get_instance().get_size_tile(), 1);

            auto physical_tex = VTBackend::get_instance().physical_texture_ptr_per_context_[ctx.id]->get_physical_texture_ptr();

            auto phys_tex_format = scm::gl::FORMAT_RGBA_8;
            switch(::vt::VTConfig::get_instance().get_format_texture())
            {
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

            ctx.render_context->update_sub_texture(physical_tex, scm::gl::texture_region(origin, dimensions), 0, phys_tex_format, mem_slot_updated->pointer);
        }

        for(auto position_slot_cleared : cut->get_front()->get_mem_slots_cleared())
        {
            const ::vt::mem_slot_type *mem_slot_cleared = cut_db->read_mem_slot_at(position_slot_cleared.second, VTBackend::get_instance().vt_info_per_context_[ctx.id].context_id_);

            if(mem_slot_cleared == nullptr)
            {
                std::cerr << "Mem slot at " << position_slot_cleared.second << " is null" << std::endl;
            }

            updated_levels.insert(::vt::QuadTree::get_depth_of_node(position_slot_cleared.first));
        }

        for(auto const &vt_ptr : vector_of_vt_ptr)
        {
            if(::vt::Cut::get_dataset_id(cut_entry.first) == vt_ptr->get_lamure_texture_id())
            {
                // update_index_texture

                std::vector<std::pair<uint16_t, uint8_t *>> level_pairs_to_update;
                for(uint16_t updated_level : updated_levels)
                {
                    uint8_t *level_address = cut->get_front()->get_index(updated_level);
                    level_pairs_to_update.emplace_back(updated_level, level_address);

                    vt_ptr->update_index_texture_hierarchy(ctx, level_pairs_to_update);
                }
            }
        }

        cut_db->stop_reading_cut(cut_entry.first);
    }
    ctx.render_context->sync();
}
void VTRenderer::_update_feedback_layout(const RenderContext &ctx)
{
    auto &vt_info_per_context = VTBackend::get_instance().vt_info_per_context_;
    auto &current_vt_info = vt_info_per_context[ctx.id];

    if(current_vt_info.cut_update_ == nullptr)
    {
        return;
    }

    auto allocated_slots = &current_vt_info.cut_update_->get_context_feedback(current_vt_info.context_id_)->get_allocated_slot_index();

    if(allocated_slots->empty())
    {
        return;
    }

    std::vector<uint32_t> output(allocated_slots->size());
    std::copy(allocated_slots->begin(), allocated_slots->end(), output.begin());

    ctx.render_context->apply();

    auto &physical_texture = VTBackend::get_instance().physical_texture_ptr_per_context_[ctx.id];

    auto mapped_index = (uint32_t *)ctx.render_context->map_buffer_range(physical_texture->get_feedback_index_vb(), 0, output.size() * sizeof(uint32_t), scm::gl::ACCESS_WRITE_ONLY);
    memcpy(&mapped_index[0], &output[0], output.size() * sizeof(uint32_t));
    ctx.render_context->unmap_buffer(physical_texture->get_feedback_index_vb());

    ctx.render_context->clear_buffer_data(physical_texture->get_feedback_inv_index(), scm::gl::FORMAT_R_32UI, nullptr);
    ctx.render_context->sync();

    ctx.render_context->bind_index_buffer(physical_texture->get_feedback_index_ib(), scm::gl::PRIMITIVE_POINT_LIST, scm::gl::TYPE_UINT);
    ctx.render_context->bind_vertex_array(physical_texture->get_feedback_vao());
    ctx.render_context->apply();

    shader_vt_feedback_->uniform("feedback_index_size", (const unsigned int)allocated_slots->size());
    ctx.render_context->bind_program(shader_vt_feedback_);

    ctx.render_context->apply();

    ctx.render_context->draw_elements((const unsigned int)allocated_slots->size());
}
void VTRenderer::_collect_feedback(gua::RenderContext const &ctx)
{
    ctx.render_context->sync();

    auto &vt_info_per_context = VTBackend::get_instance().vt_info_per_context_;
    auto &current_vt_info = vt_info_per_context[ctx.id];

    if(current_vt_info.cut_update_ == nullptr)
    {
        return;
    }

    auto &gua_layered_physical_texture_for_context = VTBackend::get_instance().physical_texture_ptr_per_context_[ctx.id];

    auto feedback_lod_storage_ptr = gua_layered_physical_texture_for_context->get_feedback_lod_storage_ptr();
    auto feedback_lod_cpu_buffer_ptr = gua_layered_physical_texture_for_context->get_feedback_lod_cpu_buffer();

    std::size_t num_feedback_slots = gua_layered_physical_texture_for_context->get_num_feedback_slots();

    size_t compact_size = current_vt_info.cut_update_->get_context_feedback(current_vt_info.context_id_)->get_allocated_slot_index().size();
    memset(feedback_lod_cpu_buffer_ptr, 0, num_feedback_slots * scm::gl::size_of_format(scm::gl::FORMAT_R_32I));
    int32_t *feedback_lod = (int32_t *)ctx.render_context->map_buffer(feedback_lod_storage_ptr, scm::gl::ACCESS_READ_ONLY);
    memcpy(feedback_lod_cpu_buffer_ptr, feedback_lod, compact_size * size_of_format(scm::gl::FORMAT_R_32I));
    ctx.render_context->unmap_buffer(feedback_lod_storage_ptr);
    ctx.render_context->clear_buffer_data(feedback_lod_storage_ptr, scm::gl::FORMAT_R_32I, nullptr);

#ifdef RASTERIZATION_COUNT
    auto feedback_count_storage_ptr = gua_layered_physical_texture_for_context->get_feedback_count_storage_ptr();
    auto feedback_count_cpu_buffer_ptr = gua_layered_physical_texture_for_context->get_feedback_count_cpu_buffer();

    uint32_t *feedback_count = (uint32_t *)ctx.render_context->map_buffer(feedback_count_storage_ptr, scm::gl::ACCESS_READ_ONLY);
    memcpy(feedback_count_cpu_buffer_ptr, feedback_count, num_feedback_slots * size_of_format(scm::gl::FORMAT_R_32UI));
    ctx.render_context->unmap_buffer(feedback_count_storage_ptr);
    ctx.render_context->clear_buffer_data(feedback_count_storage_ptr, scm::gl::FORMAT_R_32UI, nullptr);
#endif

    /*std::cout << "Context " << current_vt_info.context_id_ << std::endl;

    for(int i = 0; i < num_feedback_slots; i ++){
      std::cout << feedback_lod_cpu_buffer_ptr[i];
    }

    std::cout << std::endl;*/

#ifdef RASTERIZATION_COUNT
    current_vt_info.cut_update_->feedback(current_vt_info.context_id_, feedback_lod_cpu_buffer_ptr, feedback_count_cpu_buffer_ptr);
#else
    current_vt_info.cut_update_->feedback(current_vt_info.context_id_, feedback_lod_cpu_buffer_ptr, nullptr);
#endif

    ctx.render_context->sync();
}
} // namespace gua
#endif