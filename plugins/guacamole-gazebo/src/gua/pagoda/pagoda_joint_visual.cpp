#include <utility>

#include "../../../include/gua/pagoda/pagoda_joint_visual.hpp"

PagodaJointVisual::PagodaJointVisual(const std::string &name, ptr_visual parent) : PagodaVisual(name, std::move(parent)) {}
PagodaJointVisual::~PagodaJointVisual() {

}
void PagodaJointVisual::update_from_joint_msg(boost::shared_ptr<const gazebo::msgs::Joint> &msg) {
    ignition::math::Pose3d pose;
    if (msg->has_pose())
        pose = gazebo::msgs::ConvertIgn(msg->pose());

    this->set_pose(pose);
}
