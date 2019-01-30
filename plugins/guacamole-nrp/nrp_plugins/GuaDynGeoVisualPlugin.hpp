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

#include <zmq.hpp>

#include <atomic>
#include <iostream>
#include <thread>

#include <sgtp/SGTP.h>

typedef const boost::shared_ptr<gazebo::msgs::PosesStamped const> ConstPosesStampedPtr;

namespace gazebo
{
class GuaDynGeoVisualPluginPrivate;

class GAZEBO_VISIBLE GuaDynGeoVisualPlugin : public VisualPlugin
{
  public:
    GuaDynGeoVisualPlugin();
    ~GuaDynGeoVisualPlugin() override;

  public:
    void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf) override;

  private:
    rendering::VisualPtr _visual;
    event::ConnectionPtr _update_connection;

    std::atomic<bool> _is_recv_running;
    std::atomic<bool> _is_need_swap;
    std::mutex _mutex_swap;
    std::thread _thread_recv;

    Ogre::SceneNode *_scene_node, *_avatar_node;
    Ogre::SceneManager *_scene_manager;
    Ogre::Entity * _entity;
    std::string _entity_name;

    std::vector<unsigned char> _buffer_rcv;
    std::vector<int32_t> _buffer_index;
    size_t _num_geometry_bytes = 0;
    float _bb_min[3];
    float _bb_max[3];

    void Update();
    void AddTriangleSoup();
    void RemoveTriangleSoup();

    void _ReadLoop();
};
}

#endif // GUACAMOLE_GUADYNGEOVISUALPLUGIN_H
