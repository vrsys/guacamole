
#include <gua/platform.hpp>

#include <gua/virtual_texturing/VirtualTexture2D.hpp>

#include <gua/renderer/Pipeline.hpp>
#include <gua/guacamole.hpp>

#include <fstream>
#include <regex>

#include <lamure/vt/ren/CutDatabase.h>
#include <lamure/vt/ren/CutUpdate.h>

#include <gua/virtual_texturing/VTBackend.hpp>

void gua::VTBackend::add_context(uint16_t ctx_id,
                                       gua::node::CameraNode& camera){
  init_vt(ctx_id, camera);
  register_cuts(ctx_id);

  _camera_contexts.insert({&camera, ctx_id});
}
void gua::VTBackend::start_backend(){
  auto& vt_info_per_context = VirtualTexture2D::vt_info_per_context_;
  for (auto cam_ctx : _camera_contexts) {
    auto& current_vt_info = vt_info_per_context[cam_ctx.second];

    current_vt_info.cut_update_ = &::vt::CutUpdate::get_instance();
    current_vt_info.cut_updated_running_ = true;
  }

  ::vt::CutUpdate::get_instance().start();
}
void gua::VTBackend::stop_backend(){
  ::vt::CutUpdate::get_instance().stop();
}
void gua::VTBackend::init_vt(uint16_t ctx_id,
                                   gua::node::CameraNode const& cam){
  auto& vt_info_per_context = VirtualTexture2D::vt_info_per_context_;
  auto current_vt_info_per_context_iterator = vt_info_per_context.find(ctx_id);

  // if vt_info for this render context doesnt exist, register context
  if (vt_info_per_context.end() == current_vt_info_per_context_iterator) {
    auto& current_vt_info = vt_info_per_context[ctx_id];

    current_vt_info.context_id_ =
        ::vt::CutDatabase::get_instance().register_context();
  }

  auto& current_vt_info = vt_info_per_context[ctx_id];
  auto camera_iter =
      current_vt_info.gua_camera_id_to_lamure_view_id_.find(cam.uuid());

  // if camera uuid is not in the list, register camera and create lamure view
  // for it
  if (current_vt_info.gua_camera_id_to_lamure_view_id_.end() == camera_iter) {
    uint16_t lamure_view_id = ::vt::CutDatabase::get_instance().register_view();
    current_vt_info.gua_camera_id_to_lamure_view_id_[cam.uuid()] =
        lamure_view_id;
  }
}
void gua::VTBackend::register_cuts(uint16_t ctx_id) {
  auto& vt_info_per_context = VirtualTexture2D::vt_info_per_context_;
  auto& current_vt_info = vt_info_per_context[ctx_id];
  uint16_t lamure_ctx_id = current_vt_info.context_id_;
  for (auto const& view_entry :
       current_vt_info.gua_camera_id_to_lamure_view_id_) {
    uint16_t view_id = view_entry.second;
    auto vector_of_vt_ptr = TextureDatabase::instance()->get_virtual_textures();
    for (auto const& vt_ptr : vector_of_vt_ptr) {
      uint32_t tex_id = vt_ptr->get_lamure_texture_id();
      // render
      uint64_t cut_id = (((uint64_t)tex_id) << 32) | ((uint64_t)view_id << 16) |
                        ((uint64_t)lamure_ctx_id);
      auto cut_iter = current_vt_info.cut_id_to_lamure_triple_.find(cut_id);

      // check if cut got registered already
      if (current_vt_info.cut_id_to_lamure_triple_.end() == cut_iter) {
        current_vt_info.cut_id_to_lamure_triple_[cut_id] =
            ::vt::CutDatabase::get_instance().register_cut(tex_id, view_id,
                                                           lamure_ctx_id);
      }
    }
  }
}
void gua::VTBackend::add_camera(
    const std::shared_ptr<gua::node::CameraNode>& camera) {
  add_context((uint16_t)(_camera_contexts.size()), *camera);
}
const bool gua::VTBackend::has_camera(size_t uuid) const {
  for (auto camera : _camera_contexts) {
    if (camera.first->uuid() == uuid) {
      return true;
    }
  }

  return false;
}
