#ifndef GUACAMOLE_NRP_NODE_HPP
#define GUACAMOLE_NRP_NODE_HPP

#include <gua/node/TransformNode.hpp>
#include <gua/nrp/platform.hpp>

namespace gua
{
namespace nrp
{
class GUA_NRP_DLL NRPNode : public gua::node::TransformNode
{
  public:
    NRPNode();
    explicit NRPNode(const std::string &name, math::mat4 const &transform = math::mat4::identity());

    std::shared_ptr<node::Node> deep_copy() const override;
    void update_cache() override;
};
}
}

#endif // GUACAMOLE_NRP_NODE_HPP