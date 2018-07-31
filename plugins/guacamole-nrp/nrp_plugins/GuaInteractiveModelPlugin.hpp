#ifndef GUACAMOLE_GUAINTERACTIVEMODELPLUGIN_H
#define GUACAMOLE_GUAINTERACTIVEMODELPLUGIN_H

#include <boost/bind.hpp>
#include <cstdio>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
class GuaInteractiveModelPlugin : public ModelPlugin
{
  public:
    GuaInteractiveModelPlugin();
    ~GuaInteractiveModelPlugin() override;
    void Load(physics::ModelPtr _parent, sdf::ElementPtr) override;

  private:
    void callback_pos(ConstPosesStampedPtr &msg);

    physics::ModelPtr _model;
    sdf::ElementPtr _sdf;

    transport::NodePtr _node;
    transport::SubscriberPtr _sub;
    event::ConnectionPtr _update_connection;
};

GZ_REGISTER_MODEL_PLUGIN(GuaInteractiveModelPlugin)
}

#endif // GUACAMOLE_GUAINTERACTIVEMODELPLUGIN_H
