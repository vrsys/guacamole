#ifndef GUACAMOLE_VTBACKEND_H
#define GUACAMOLE_VTBACKEND_H

namespace gua
{
class GUA_DLL VTBackend
{
  public:
    static VTBackend& get_instance()
    {
        static VTBackend instance;
        return instance;
    }

    VTBackend(VTBackend const&) = delete;
    void operator=(VTBackend const&) = delete;

    ~VTBackend();

    void add_camera(const std::shared_ptr<gua::node::CameraNode>& camera);
    void start_backend();
    void stop_backend();

    const bool has_camera(size_t uuid) const;
    bool should_update_on_context(size_t uuid);
    bool should_collect_feedback_on_context(size_t uuid);

    // per render (gua) contexts
    std::map<std::size_t, std::shared_ptr<LayeredPhysicalTexture2D>> physical_texture_ptr_per_context_;
    std::map<std::size_t, VTInfo> vt_info_per_context_;

  private:
    VTBackend() : physical_texture_ptr_per_context_(), vt_info_per_context_(), _camera_contexts(), _context_updated(), _camera_drawn() {}

    std::mutex _camera_contexts_mutex;

    std::map<gua::node::CameraNode*, uint16_t> _camera_contexts;
    std::map<uint16_t, bool> _context_updated;
    std::map<size_t, bool> _camera_drawn;

    void add_context(uint16_t ctx_id, gua::node::CameraNode& camera);
    void init_vt(uint16_t ctx_id, gua::node::CameraNode const& cam);
    void register_cuts(uint16_t ctx_id);
};
} // namespace gua

#endif // GUACAMOLE_VTBACKEND_H
