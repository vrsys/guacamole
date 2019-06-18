#include <gua/platform.hpp>

#include <gua/renderer/Pipeline.hpp>
#include <gua/guacamole.hpp>

#ifdef GUACAMOLE_ENABLE_VIRTUAL_TEXTURING

#include <fstream>
#include <regex>

#include <lamure/vt/ren/CutDatabase.h>
#include <lamure/vt/ren/CutUpdate.h>

#include <gua/virtual_texturing/VTBackend.hpp>

void gua::VTBackend::add_context(uint16_t ctx_id, gua::node::CameraNode& camera)
{
    init_vt(ctx_id, camera);
    register_cuts(ctx_id);

    camera_contexts_.insert({&camera, ctx_id});
    context_updated_[ctx_id] = false;
    camera_drawn_[camera.uuid()] = false;
    context_states_.insert({camera.uuid(), VTContextState{false, false}});
}
void gua::VTBackend::start_backend()
{
    if(::vt::VTConfig::CONFIG_PATH.empty())
    {
        std::cerr << "VTBackend will not be started, due to undefined configuration file. Check if any VT model is provided";
        return;
    }

    for(auto cam_ctx : camera_contexts_)
    {
        auto& current_vt_info = vt_info_per_context_[cam_ctx.second];

        current_vt_info.cut_update_ = &::vt::CutUpdate::get_instance();
        current_vt_info.cut_updated_running_ = true;
    }

    ::vt::CutUpdate::get_instance().start();
}
void gua::VTBackend::stop_backend()
{
    if(::vt::VTConfig::CONFIG_PATH.empty())
    {
        return;
    }
    ::vt::CutUpdate::get_instance().stop();
}
void gua::VTBackend::init_vt(uint16_t ctx_id, gua::node::CameraNode const& cam)
{
    auto current_vt_info_per_context_iterator = vt_info_per_context_.find(ctx_id);

    // if vt_info for this render context doesnt exist, register context
    if(vt_info_per_context_.end() == current_vt_info_per_context_iterator)
    {
        auto& current_vt_info = vt_info_per_context_[ctx_id];

        current_vt_info.context_id_ = ::vt::CutDatabase::get_instance().register_context();
    }

    auto& current_vt_info = vt_info_per_context_[ctx_id];
    auto camera_iter = current_vt_info.gua_camera_id_to_lamure_view_id_.find(cam.uuid());

    // if camera uuid is not in the list, register camera and create lamure view
    // for it
    if(current_vt_info.gua_camera_id_to_lamure_view_id_.end() == camera_iter)
    {
        uint16_t lamure_view_id = ::vt::CutDatabase::get_instance().register_view();
        current_vt_info.gua_camera_id_to_lamure_view_id_[cam.uuid()] = lamure_view_id;
    }
}
void gua::VTBackend::register_cuts(uint16_t ctx_id)
{
    auto& current_vt_info = vt_info_per_context_[ctx_id];
    uint16_t lamure_ctx_id = current_vt_info.context_id_;
    for(auto const& view_entry : current_vt_info.gua_camera_id_to_lamure_view_id_)
    {
        uint16_t view_id = view_entry.second;
        auto vector_of_vt_ptr = TextureDatabase::instance()->get_virtual_textures();
        for(auto const& vt_ptr : vector_of_vt_ptr)
        {
            uint32_t tex_id = vt_ptr->get_lamure_texture_id();
            // render
            uint64_t cut_id = (((uint64_t)tex_id) << 32) | ((uint64_t)view_id << 16) | ((uint64_t)lamure_ctx_id);
            auto cut_iter = current_vt_info.cut_id_to_lamure_triple_.find(cut_id);

            // check if cut got registered already
            if(current_vt_info.cut_id_to_lamure_triple_.end() == cut_iter)
            {
                current_vt_info.cut_id_to_lamure_triple_[cut_id] = ::vt::CutDatabase::get_instance().register_cut(tex_id, view_id, lamure_ctx_id);
            }
        }
    }
}
void gua::VTBackend::add_camera(const std::shared_ptr<gua::node::CameraNode>& camera)
{
    if(::vt::VTConfig::CONFIG_PATH.empty())
    {
        std::cerr << "VTBackend will not be started, due to undefined configuration file. Check if any VT model is provided";
        return;
    }

    std::lock_guard<std::mutex> lock(vt_backend_mutex_);
    add_context((uint16_t)(camera_contexts_.size()), *camera);
}
const bool gua::VTBackend::has_camera(size_t uuid) const
{
    if(::vt::VTConfig::CONFIG_PATH.empty())
    {
        return false;
    }

    for(auto camera : camera_contexts_)
    {
        if(camera.first->uuid() == uuid)
        {
            return true;
        }
    }

    return false;
}
bool gua::VTBackend::should_collect_feedback_on_context(size_t uuid)
{
    std::lock_guard<std::mutex> lock(vt_backend_mutex_);

    uint16_t context_id = UINT16_MAX;

    for(auto camera : camera_contexts_)
    {
        if(camera.first->uuid() == uuid)
        {
            context_id = camera.second;
            break;
        }
    }

    bool should_collect = true;

    for(auto camera : camera_contexts_)
    {
        if(camera.second == context_id)
        {
            should_collect = should_collect && camera_drawn_[camera.first->uuid()];
        }
    }

    if(should_collect)
    {
        context_updated_[context_id] = false;
        for(auto camera : camera_contexts_)
        {
            if(camera.second == context_id)
            {
                camera_drawn_[camera.first->uuid()] = false;
            }
        }
    }

    return should_collect;
}
bool gua::VTBackend::should_update_on_context(size_t uuid)
{
    std::lock_guard<std::mutex> lock(vt_backend_mutex_);

    camera_drawn_[uuid] = true;

    for(auto camera : camera_contexts_)
    {
        if(camera.first->uuid() == uuid)
        {
            if(!context_updated_[camera.second])
            {
                context_updated_[camera.second] = true;
                return true;
            }
        }
    }

    return false;
}
gua::VTBackend::~VTBackend()
{
    stop_backend();

    physical_texture_ptr_per_context_.clear();
    vt_info_per_context_.clear();

    camera_drawn_.clear();
    context_updated_.clear();
    camera_contexts_.clear();
    context_states_.clear();
}
gua::VTContextState& gua::VTBackend::get_state(size_t uuid)
{
    std::lock_guard<std::mutex> lock(vt_backend_mutex_);

    if(::vt::VTConfig::CONFIG_PATH.empty())
    {
        VTContextState null_state{false, false};
        return null_state;
    }

    auto state = context_states_.find(uuid);

    if(state != context_states_.end())
    {
        return context_states_.at(uuid);
    }
    else
    {
        VTContextState null_state{false, false};
        return null_state;
    }
}

#endif // GUACAMOLE_ENABLE_VIRTUAL_TEXTURING
