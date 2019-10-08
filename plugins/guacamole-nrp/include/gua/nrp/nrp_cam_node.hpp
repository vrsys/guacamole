#ifndef GUACAMOLE_NRP_CAMERA_NODE_HPP
#define GUACAMOLE_NRP_CAMERA_NODE_HPP

#include <gua/node/CameraNode.hpp>
#include <gua/nrp/platform.hpp>

namespace gua
{
namespace nrp
{
class GUA_NRP_DLL NRPCameraNode : public gua::node::CameraNode
{
  public:
    NRPCameraNode();
    explicit NRPCameraNode(std::string const &name, std::shared_ptr<PipelineDescription> const &description = PipelineDescription::make_default(), Configuration const &configuration = Configuration(),
                           math::mat4 const &transform = math::mat4::identity());

    std::shared_ptr<node::Node> deep_copy() const override;
    void update_cache() override;
    void set_transform(math::mat4 const &transform) override;
    void scale(math::float_t x, math::float_t y, math::float_t z) override;
    void scale(math::vec3 const &s) override;
    void scale(math::float_t s) override;
    void rotate(math::float_t angle, math::float_t x, math::float_t y, math::float_t z) override;
    void rotate(math::float_t angle, math::vec3 const &axis) override;
    void translate(math::float_t x, math::float_t y, math::float_t z) override;
    void translate(math::vec3 const &offset) override;
};
} // namespace nrp
} // namespace gua

#endif // GUACAMOLE_NRP_CAMERA_NODE_HPP
