#ifndef GUACAMOLE_GUAWORLDPLUGIN_H
#define GUACAMOLE_GUAWORLDPLUGIN_H

#include <condition_variable>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsTypes.hh>
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
    gazebo::physics::WorldPtr _world;
    sdf::ElementPtr _sdf;

    std::thread _worker;
    std::atomic_bool _worker_should_stop;
    std::mutex _worker_mutex;
    std::condition_variable _worker_cv;

    void _connect_to_transport_layer();
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(GuaWorldPlugin)
}

#endif // GUACAMOLE_GUAWORLDPLUGIN_H
