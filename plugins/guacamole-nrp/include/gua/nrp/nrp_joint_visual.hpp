#ifndef GUACAMOLE_PAGODA_JOINT_VISUAL_H
#define GUACAMOLE_PAGODA_JOINT_VISUAL_H

#include <memory>
#include <gua/nrp/platform.hpp>
#include <gua/nrp/nrp_visual.hpp>
namespace gua
{
namespace nrp
{
class NRPJointVisual;

typedef std::shared_ptr<NRPJointVisual> ptr_joint_visual;

class GUA_NRP_DLL NRPJointVisual : public NRPVisual
{
  public:
    NRPJointVisual(const std::string &name, ptr_visual parent);
    ~NRPJointVisual();

    void update_from_joint_msg(boost::shared_ptr<const gazebo::msgs::Joint> &msg);
};
}
}
#endif // GUACAMOLE_PAGODA_JOINT_VISUAL_H
