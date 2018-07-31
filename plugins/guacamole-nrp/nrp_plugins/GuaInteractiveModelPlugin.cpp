#include "GuaInteractiveModelPlugin.hpp"

gazebo::GuaInteractiveModelPlugin::GuaInteractiveModelPlugin() {}
gazebo::GuaInteractiveModelPlugin::~GuaInteractiveModelPlugin()
{
    _sub.reset();
    _node.reset();
}
void gazebo::GuaInteractiveModelPlugin::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf)
{
    _sdf = sdf;

    _node.reset(new transport::Node());
    _sub = _node->Subscribe("/nrp-gua/interactive_pos", &GuaInteractiveModelPlugin::callback_pos, this);

    _model = parent;
}
void gazebo::GuaInteractiveModelPlugin::callback_pos(ConstPosesStampedPtr &msg)
{
    _model->SetWorldPose(msgs::ConvertIgn(msg->pose(0)), true, true);
}
