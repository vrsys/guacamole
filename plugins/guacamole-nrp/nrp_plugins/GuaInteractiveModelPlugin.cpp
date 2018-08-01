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
    _pose = _model->GetWorldPose().Ign();

    this->_update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GuaInteractiveModelPlugin::on_update, this));
}
void gazebo::GuaInteractiveModelPlugin::callback_pos(ConstPosesStampedPtr &msg)
{
#if GUA_DEBUG == 1
    gzerr << "callback_pos: begin" << std::endl;
    std::cerr << "callback_pos: begin" << std::endl;
#endif
    _pose = msgs::ConvertIgn(msg->pose(0));
#if GUA_DEBUG == 1
    gzerr << "callback_pos: begin" << std::endl;
    std::cerr << "callback_pos: begin" << std::endl;
#endif
}
void gazebo::GuaInteractiveModelPlugin::on_update()
{
#if GUA_DEBUG == 1
    gzerr << "on_update: begin" << std::endl;
    std::cerr << "on_update: begin" << std::endl;
#endif
    _model->SetWorldPose(_pose, true, true);
#if GUA_DEBUG == 1
    gzerr << "on_update: end" << std::endl;
    std::cerr << "on_update: end" << std::endl;
#endif
}