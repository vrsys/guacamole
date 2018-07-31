#ifndef GUACAMOLE_GUAINTERACTIVEMODELPLUGIN_H
#define GUACAMOLE_GUAINTERACTIVEMODELPLUGIN_H

#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
class GuaInteractiveModelPlugin : public ModelPlugin
{
  public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr)
    {
        this->model = _parent;
        this->update_connection = event::Events::ConnectWorldUpdateBegin(std::bind(&GuaInteractiveModelPlugin::on_update, this));
    }

  public:
    void on_update();

  private:
    physics::ModelPtr model;

  private:
    event::ConnectionPtr update_connection;
};

GZ_REGISTER_MODEL_PLUGIN(GuaInteractiveModelPlugin)
}

#endif // GUACAMOLE_GUAINTERACTIVEMODELPLUGIN_H
