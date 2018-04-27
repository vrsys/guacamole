#include <gua/nrp/nrp_node.hpp>

#include <gua/nrp/pagoda_binder.hpp>

namespace gua
{
namespace nrp
{
NRPNode::NRPNode() : TransformNode()
{
    auto *binder = &PagodaBinder::get_instance();
    binder->bind_root_node(this);
    binder->bind_transport_layer();
}
NRPNode::NRPNode(const std::string &name, const math::mat4 &transform) : TransformNode(name, transform)
{
    auto *binder = &PagodaBinder::get_instance();
    binder->bind_root_node(this);
    binder->bind_transport_layer();
}
std::shared_ptr<node::Node> NRPNode::deep_copy() const
{
    PagodaBinder::get_instance().lock_scene();
    auto copied_node = node::Node::deep_copy();
    PagodaBinder::get_instance().unlock_scene();

    return copied_node;
}
void NRPNode::update_cache() {
    PagodaBinder::get_instance().lock_scene();
    Node::update_cache();
    PagodaBinder::get_instance().unlock_scene();
}
}
}