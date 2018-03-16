#ifndef GUACAMOLE_NRP_NODE_HPP
#define GUACAMOLE_NRP_NODE_HPP

#include <gua/nrp/pagoda_binder.hpp>
#include <gua/nrp/pagoda_log.hpp>
#include <gua/nrp/pagoda_scene.hpp>
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

    void pre_draw();

  private:
    PagodaBinder *_binder;
};
}
}

#endif // GUACAMOLE_NRP_NODE_HPP
