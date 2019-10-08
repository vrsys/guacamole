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
NRPNode::NRPNode(const std::string &name, const math::mat4 &transform, std::function<void(void)> pre_pass, std::function<void(void)> post_pass)
{
    auto *binder = &NRPBinder::get_instance();
    binder->bind_root_node(this);
    binder->bind_transport_layer();
    this->_pre_pass = std::move(pre_pass);
    this->_post_pass = std::move(post_pass);
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
void NRPNode::callback_pre_pass() { _pre_pass(); }
void NRPNode::callback_post_pass() { _post_pass(); }
void NRPNode::set_pre_pass(const std::function<void()> pre_pass) { this->_pre_pass = std::move(pre_pass); }
void NRPNode::set_post_pass(const std::function<void()> post_pass) { this->_post_pass = std::move(post_pass); }
bool NRPNode::get_should_update_avango() { return _should_update_avango; }
void NRPNode::set_should_update_avango(bool should_update_avango) { this->_should_update_avango = should_update_avango; }
} // namespace nrp
} // namespace gua