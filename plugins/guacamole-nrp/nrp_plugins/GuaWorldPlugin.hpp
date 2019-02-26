#ifndef GUACAMOLE_GUAWORLDPLUGIN_H
#define GUACAMOLE_GUAWORLDPLUGIN_H

#include <condition_variable>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/physics.hh>
#include <thread>

namespace gazebo
{
class GuaWorldPlugin : public WorldPlugin
{
  public:
    GuaWorldPlugin();
    ~GuaWorldPlugin() override;
    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override;

  private:
    void on_update();

    gazebo::physics::WorldPtr _world;
    sdf::ElementPtr _sdf;

    transport::NodePtr _node;
    transport::PublisherPtr _pub;
    event::ConnectionPtr _update_connection;
};

GZ_REGISTER_WORLD_PLUGIN(GuaWorldPlugin)
} // namespace gazebo

#endif // GUACAMOLE_GUAWORLDPLUGIN_H
