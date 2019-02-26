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

    ~VTBackend() { stop_backend(); }

    void add_camera(const std::shared_ptr<gua::node::CameraNode>& camera);
    void start_backend();
    void stop_backend();
    const bool has_camera(size_t uuid) const;

  private:
    VTBackend() : _camera_contexts() {}

    std::map<gua::node::CameraNode*, uint16_t> _camera_contexts;
    void add_context(uint16_t ctx_id, gua::node::CameraNode& camera);
    void init_vt(uint16_t ctx_id, gua::node::CameraNode const& cam);
    void register_cuts(uint16_t ctx_id);
};
} // namespace gua

#endif // GUACAMOLE_VTBACKEND_H
