#ifndef GUACAMOLE_NRP_LIGHT_H
#define GUACAMOLE_NRP_LIGHT_H

#include <gazebo/common/Console.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/rendering/Visual.hh>
#include <gua/node/Node.hpp>
#include <gua/node/TransformNode.hpp>
#include <gua/nrp/platform.hpp>
#include <gua/renderer/TriMeshLoader.hpp>
#include <memory>
namespace gua
{
namespace nrp
{
class NRPLight;

typedef std::shared_ptr<NRPLight> ptr_light;

class GUA_NRP_DLL NRPLight
{
  public:
    NRPLight(const std::string &name, node::Node *root_node);
    ~NRPLight();

    void load_from_msg(const boost::shared_ptr<const gazebo::msgs::Light> &msg);

  private:
    void set_pose(const gazebo::math::Pose &pose);
    void set_direction(const gazebo::msgs::Vector3d &direction);

    std::shared_ptr<gua::node::LightNode> _node;
    double _scale;
    scm::math::mat4d _direction;
};
} // namespace nrp
} // namespace gua
#endif // GUACAMOLE_NRP_LIGHT_H
