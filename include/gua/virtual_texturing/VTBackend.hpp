#ifndef GUACAMOLE_VTBACKEND_H
#define GUACAMOLE_VTBACKEND_H

namespace gua
{
struct VTContextState
{
    VTContextState(bool has_camera, bool feedback_enabled) : has_camera_(has_camera), feedback_enabled_(feedback_enabled) {}

    bool has_camera_{false};
    bool feedback_enabled_{false};
};

class GUA_DLL VTBackend
{
  public:
    static VTBackend& get_instance()
    {
        static VTBackend instance;
        return instance;
    }

    static void set_physical_texture_size(uint32_t);
    static void set_update_throughput_size(uint32_t);
    static void set_ram_cache_size(uint32_t);

    VTBackend(VTBackend const&) = delete;
    void operator=(VTBackend const&) = delete;

    ~VTBackend();

    void add_camera(const std::shared_ptr<gua::node::CameraNode>& camera);
    void start_backend();
    void stop_backend();

    VTContextState& get_state(size_t uuid);

    const bool has_camera(size_t uuid) const;
    bool should_update_on_context(size_t uuid);
    bool should_collect_feedback_on_context(size_t uuid);

    // per render (gua) contexts
    std::map<std::size_t, std::shared_ptr<LayeredPhysicalTexture2D>> physical_texture_ptr_per_context_;
    std::map<std::size_t, VTInfo> vt_info_per_context_;

  private:
    VTBackend() : physical_texture_ptr_per_context_(), vt_info_per_context_(), camera_contexts_(), context_updated_(), camera_drawn_(), context_states_() {}

    std::mutex vt_backend_mutex_;

    std::map<gua::node::CameraNode*, uint16_t> camera_contexts_;
    std::map<uint16_t, bool> context_updated_;
    std::map<size_t, bool> camera_drawn_;

    std::map<uint16_t, VTContextState> context_states_;

    void add_context(uint16_t ctx_id, gua::node::CameraNode& camera);
    void init_vt(uint16_t ctx_id, gua::node::CameraNode const& cam);
    void register_cuts(uint16_t ctx_id);
};
} // namespace gua

#endif // GUACAMOLE_VTBACKEND_H
