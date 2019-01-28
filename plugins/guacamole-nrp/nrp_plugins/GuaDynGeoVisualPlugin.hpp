#ifndef GUACAMOLE_GUADYNGEOVISUALPLUGIN_H
#define GUACAMOLE_GUADYNGEOVISUALPLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <memory>

#include <mutex>

#include <Ogre.h>
#include <gazebo/common/Color.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/transport/Node.hh>

typedef const boost::shared_ptr<gazebo::msgs::PosesStamped const> ConstPosesStampedPtr;

namespace gazebo
{
class GuaDynGeoVisualPluginPrivate;

class GAZEBO_VISIBLE GuaDynGeoVisualPlugin : public VisualPlugin
{
  public:
    GuaDynGeoVisualPlugin();
    ~GuaDynGeoVisualPlugin();

  public:
    virtual void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf);

  private:
    rendering::VisualPtr _visual;
    event::ConnectionPtr _update_connection;
    Ogre::SceneNode *_scene_node;
    Ogre::SceneManager *_scene_manager;
    void Update();
    void AddTriangle();
};
}

#endif // GUACAMOLE_GUADYNGEOVISUALPLUGIN_H
