#include <utility>

#include <gua/nrp/nrp_joint_visual.hpp>
namespace gua
{
namespace nrp
{
NRPJointVisual::NRPJointVisual(const std::string &name, ptr_visual parent) : NRPVisual(name, std::move(parent)) {}
NRPJointVisual::~NRPJointVisual() {}
void NRPJointVisual::update_from_joint_msg(boost::shared_ptr<const gazebo::msgs::Joint> &msg)
{
    ignition::math::Pose3d pose;
    if(msg->has_pose())
        pose = gazebo::msgs::ConvertIgn(msg->pose());

    this->set_pose(pose);
}
}
}