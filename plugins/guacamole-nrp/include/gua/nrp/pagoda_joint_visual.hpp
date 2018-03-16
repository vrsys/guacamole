#ifndef GUACAMOLE_PAGODA_JOINT_VISUAL_H
#define GUACAMOLE_PAGODA_JOINT_VISUAL_H

#include <memory>
#include <gua/nrp/platform.hpp>
#include <gua/nrp/pagoda_visual.hpp>
namespace gua
{
namespace nrp
{
class PagodaJointVisual;

typedef std::shared_ptr<PagodaJointVisual> ptr_joint_visual;

class PagodaJointVisual : public PagodaVisual
{
  public:
    PagodaJointVisual(const std::string &name, ptr_visual parent);
    ~PagodaJointVisual();

    void update_from_joint_msg(boost::shared_ptr<const gazebo::msgs::Joint> &msg);
};
}
}
#endif // GUACAMOLE_PAGODA_JOINT_VISUAL_H
