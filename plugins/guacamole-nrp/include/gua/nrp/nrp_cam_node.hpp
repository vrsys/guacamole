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
};
}
}

#endif // GUACAMOLE_NRP_CAMERA_NODE_HPP
