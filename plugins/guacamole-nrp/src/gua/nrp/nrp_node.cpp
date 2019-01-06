#include <gua/nrp/nrp_node.hpp>

#include <gua/nrp/nrp_binder.hpp>

namespace gua
{
namespace nrp
{
NRPNode::NRPNode() : TransformNode()
{
    auto *binder = &NRPBinder::get_instance();
    binder->bind_root_node(this);
    binder->bind_transport_layer();
}
NRPNode::NRPNode(const std::string &name, const math::mat4 &transform) : TransformNode(name, transform)
{
    auto *binder = &NRPBinder::get_instance();
    binder->bind_root_node(this);
    binder->bind_transport_layer();
}
std::shared_ptr<node::Node> NRPNode::deep_copy() const
{
    std::unique_lock<std::mutex> lock(NRPBinder::get_instance().get_scene_mutex());
    auto copied_node = node::TransformNode::deep_copy();
    return copied_node;
}
void NRPNode::accept(NodeVisitor &visitor)
{
    std::unique_lock<std::mutex> lock(NRPBinder::get_instance().get_scene_mutex());
    visitor.visit(this);
}
std::shared_ptr<node::Node> NRPNode::copy() const
{
    // std::unique_lock<std::mutex> lock(NRPBinder::get_instance().get_scene_mutex());
    return std::make_shared<NRPNode>(*this);
}
void NRPNode::update_cache()
{
    std::unique_lock<std::mutex> lock(NRPBinder::get_instance().get_scene_mutex());
    Node::update_cache();
}
void NRPNode::set_transform(math::mat4 const &transform)
{
    std::unique_lock<std::mutex> lock(NRPBinder::get_instance().get_scene_mutex());
    Node::set_transform(transform);
}
void NRPNode::scale(math::float_t x, math::float_t y, math::float_t z)
{
    std::unique_lock<std::mutex> lock(NRPBinder::get_instance().get_scene_mutex());
    Node::scale(x, y, z);
}
void NRPNode::scale(math::vec3 const &s)
{
    std::unique_lock<std::mutex> lock(NRPBinder::get_instance().get_scene_mutex());
    Node::scale(s);
}
void NRPNode::scale(math::float_t s)
{
    std::unique_lock<std::mutex> lock(NRPBinder::get_instance().get_scene_mutex());
    Node::scale(s);
}
void NRPNode::rotate(math::float_t angle, math::float_t x, math::float_t y, math::float_t z)
{
    std::unique_lock<std::mutex> lock(NRPBinder::get_instance().get_scene_mutex());
    Node::rotate(angle, x, y, z);
}
void NRPNode::rotate(math::float_t angle, math::vec3 const &axis)
{
    std::unique_lock<std::mutex> lock(NRPBinder::get_instance().get_scene_mutex());
    Node::rotate(angle, axis);
}
void NRPNode::translate(math::float_t x, math::float_t y, math::float_t z)
{
    std::unique_lock<std::mutex> lock(NRPBinder::get_instance().get_scene_mutex());
    Node::translate(x, y, z);
}
void NRPNode::translate(math::vec3 const &offset)
{
    std::unique_lock<std::mutex> lock(NRPBinder::get_instance().get_scene_mutex());
    Node::translate(offset);
}
void NRPNode::ray_test_impl(Ray const &ray, int options, Mask const &mask, std::set<PickResult> &hits)
{
    std::unique_lock<std::mutex> lock(NRPBinder::get_instance().get_scene_mutex());
    Node::ray_test_impl(ray, options, mask, hits);
}
}
}