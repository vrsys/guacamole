#include <gua/nrp/nrp_node.hpp>

namespace gua
{
namespace nrp
{
NRPNode::NRPNode() : TransformNode()
{
    _binder = new PagodaBinder();
    _binder->bind_root_node(this);
    _binder->bind_transport_layer();
}
NRPNode::NRPNode(const std::string &name, const math::mat4 &transform) : TransformNode(name, transform)
{
    _binder = new PagodaBinder();
    _binder->bind_root_node(this);
    _binder->bind_transport_layer();
}
void NRPNode::pre_draw()
{
    if(_binder != nullptr)
    {
        _binder->pre_render();
    }
}
}
}