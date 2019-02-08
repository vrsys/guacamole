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
    explicit NRPNode(const std::string &name, math::mat4 const &transform = math::mat4::identity(), std::function<void(void)> pre_pass = [&]{}, std::function<void(void)> post_pass = [&]{});

    std::shared_ptr<node::Node> deep_copy() const override;
    void update_cache() override;
    void set_transform(math::mat4 const &transform) override;
    void scale(math::float_t x, math::float_t y, math::float_t z) override;
    void scale(math::vec3 const &s) override;
    void scale(math::float_t s) override;
    void rotate(math::float_t angle, math::float_t x, math::float_t y, math::float_t z) override;
    void rotate(math::float_t angle, math::vec3 const &axis) override;
    void translate(math::float_t x, math::float_t y, math::float_t z) override;
    void translate(math::vec3 const &offset) override;

    void accept(NodeVisitor &visitor) override;
    void ray_test_impl(Ray const &ray, int options, Mask const &mask, std::set<PickResult> &hits) override;

    void callback_pre_pass();
    void callback_post_pass();
    void set_pre_pass(std::function<void()> pre_pass);
    void set_post_pass(std::function<void()> post_pass);

    void set_should_update_avango(bool should_update_avango);
    bool get_should_update_avango();

private:
    std::shared_ptr<node::Node> copy() const override;

    bool _should_update_avango = false;
    std::function<void(void)> _pre_pass, _post_pass;
};
}
}

#endif // GUACAMOLE_NRP_NODE_HPP
