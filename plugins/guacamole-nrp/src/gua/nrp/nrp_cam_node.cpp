#include <gua/nrp/nrp_cam_node.hpp>

#include <gua/nrp/nrp_pass.hpp>
#include <gua/nrp/nrp_binder.hpp>

namespace gua
{
namespace nrp
{
NRPCameraNode::NRPCameraNode() : gua::node::CameraNode()
{
    auto *binder = &NRPBinder::get_instance();
    binder->bind_cam_node(this);
}
NRPCameraNode::NRPCameraNode(std::string const &name, std::shared_ptr<PipelineDescription> const &description, Configuration const &configuration, math::mat4 const &transform)
    : gua::node::CameraNode(name, description, configuration, transform)
{
    auto *binder = &NRPBinder::get_instance();
    binder->bind_cam_node(this);
}
std::shared_ptr<node::Node> NRPCameraNode::deep_copy() const
{
    NRPBinder::get_instance().lock_scene();
    auto copied_node = node::Node::deep_copy();
    NRPBinder::get_instance().unlock_scene();

    return copied_node;
}
void NRPCameraNode::update_cache()
{
    NRPBinder::get_instance().lock_scene();
    Node::update_cache();
    NRPBinder::get_instance().unlock_scene();
}
}
}