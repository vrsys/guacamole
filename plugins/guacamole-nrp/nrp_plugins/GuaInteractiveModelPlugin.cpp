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

    this->_update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GuaInteractiveModelPlugin::on_update, this));
}
void gazebo::GuaInteractiveModelPlugin::callback_pos(ConstPosesStampedPtr &msg)
{
    _pose = msgs::ConvertIgn(msg->pose(0));
}
void gazebo::GuaInteractiveModelPlugin::on_update()
{
    _model->SetWorldPose(_pose, true, true);
}
