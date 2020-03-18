#include <gua/nrp/nrp_interactive_node.hpp>

#include <gua/nrp/nrp_binder.hpp>

namespace gua
{
namespace nrp
{
NRPInteractiveNode::NRPInteractiveNode() : TransformNode()
{
    auto *binder = &NRPBinder::get_instance();
    binder->bind_interactive_node(this);
}
NRPInteractiveNode::NRPInteractiveNode(const std::string &name, const math::mat4 &transform) : TransformNode(name, transform)
{
    auto *binder = &NRPBinder::get_instance();
    binder->bind_interactive_node(this);
}
std::shared_ptr<node::Node> NRPInteractiveNode::deep_copy(bool copy_unique_node_ids) const
{
    std::unique_lock<std::mutex> lock(NRPBinder::get_instance().get_scene_mutex());
    auto copied_node = node::Node::deep_copy(copy_unique_node_ids);
    return copied_node;
}
void NRPInteractiveNode::update_cache()
{
    std::unique_lock<std::mutex> lock(NRPBinder::get_instance().get_scene_mutex());
    Node::update_cache();
}
void NRPInteractiveNode::set_transform(math::mat4 const &transform)
{
    std::unique_lock<std::mutex> lock(NRPBinder::get_instance().get_scene_mutex());
    Node::set_transform(transform);
}
void NRPInteractiveNode::scale(math::float_t x, math::float_t y, math::float_t z)
{
    std::unique_lock<std::mutex> lock(NRPBinder::get_instance().get_scene_mutex());
    Node::scale(x, y, z);
}
void NRPInteractiveNode::scale(math::vec3 const &s)
{
    std::unique_lock<std::mutex> lock(NRPBinder::get_instance().get_scene_mutex());
    Node::scale(s);
}
void NRPInteractiveNode::scale(math::float_t s)
{
    std::unique_lock<std::mutex> lock(NRPBinder::get_instance().get_scene_mutex());
    Node::scale(s);
}
void NRPInteractiveNode::rotate(math::float_t angle, math::float_t x, math::float_t y, math::float_t z)
{
    std::unique_lock<std::mutex> lock(NRPBinder::get_instance().get_scene_mutex());
    Node::rotate(angle, x, y, z);
}
void NRPInteractiveNode::rotate(math::float_t angle, math::vec3 const &axis)
{
    std::unique_lock<std::mutex> lock(NRPBinder::get_instance().get_scene_mutex());
    Node::rotate(angle, axis);
}
void NRPInteractiveNode::translate(math::float_t x, math::float_t y, math::float_t z)
{
    std::unique_lock<std::mutex> lock(NRPBinder::get_instance().get_scene_mutex());
    Node::translate(x, y, z);
}
void NRPInteractiveNode::translate(math::vec3 const &offset)
{
    std::unique_lock<std::mutex> lock(NRPBinder::get_instance().get_scene_mutex());
    Node::translate(offset);
}
} // namespace nrp
} // namespace gua