#ifndef GUACAMOLE_GUAWORLDPLUGIN_H
#define GUACAMOLE_GUAWORLDPLUGIN_H

#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>

namespace gazebo {
class GuaWorldPlugin : public WorldPlugin {
public:
  void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/) {
  }
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(GuaWorldPlugin)
}

#endif //GUACAMOLE_GUAWORLDPLUGIN_H
