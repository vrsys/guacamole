#include <gua/nrp/nrp_cam_node.hpp>

#include <gua/nrp/nrp_binder.hpp>
#include <gua/nrp/nrp_pass.hpp>

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
std::shared_ptr<node::Node> NRPCameraNode::deep_copy(bool copy_unique_node_ids) const
{
    std::unique_lock<std::mutex> lock(NRPBinder::get_instance().get_scene_mutex());
    auto copied_node = node::Node::deep_copy(copy_unique_node_ids);
    return copied_node;
}
void NRPCameraNode::update_cache()
{
    std::unique_lock<std::mutex> lock(NRPBinder::get_instance().get_scene_mutex());
    Node::update_cache();
}
void NRPCameraNode::set_transform(math::mat4 const &transform)
{
    std::unique_lock<std::mutex> lock(NRPBinder::get_instance().get_scene_mutex());
    Node::set_transform(transform);
}
void NRPCameraNode::scale(math::float_t x, math::float_t y, math::float_t z)
{
    std::unique_lock<std::mutex> lock(NRPBinder::get_instance().get_scene_mutex());
    Node::scale(x, y, z);
}
void NRPCameraNode::scale(math::vec3 const &s)
{
    std::unique_lock<std::mutex> lock(NRPBinder::get_instance().get_scene_mutex());
    Node::scale(s);
}
void NRPCameraNode::scale(math::float_t s)
{
    std::unique_lock<std::mutex> lock(NRPBinder::get_instance().get_scene_mutex());
    Node::scale(s);
}
void NRPCameraNode::rotate(math::float_t angle, math::float_t x, math::float_t y, math::float_t z)
{
    std::unique_lock<std::mutex> lock(NRPBinder::get_instance().get_scene_mutex());
    Node::rotate(angle, x, y, z);
}
void NRPCameraNode::rotate(math::float_t angle, math::vec3 const &axis)
{
    std::unique_lock<std::mutex> lock(NRPBinder::get_instance().get_scene_mutex());
    Node::rotate(angle, axis);
}
void NRPCameraNode::translate(math::float_t x, math::float_t y, math::float_t z)
{
    std::unique_lock<std::mutex> lock(NRPBinder::get_instance().get_scene_mutex());
    Node::translate(x, y, z);
}
void NRPCameraNode::translate(math::vec3 const &offset)
{
    std::unique_lock<std::mutex> lock(NRPBinder::get_instance().get_scene_mutex());
    Node::translate(offset);
}
} // namespace nrp
} // namespace gua