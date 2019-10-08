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
#include <condition_variable>

#include <lz4.h>
#include <turbojpeg.h>
#include <unordered_map>

typedef const boost::shared_ptr<gazebo::msgs::PosesStamped const> ConstPosesStampedPtr;

namespace gazebo
{
class GAZEBO_VISIBLE GuaDynGeoVisualPlugin : public VisualPlugin
{
  public:
    GuaDynGeoVisualPlugin();
    ~GuaDynGeoVisualPlugin() override;

  public:
    void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf) override;
    void Init() override;

  private:
    rendering::VisualPtr _visual;
    event::ConnectionPtr _update_connection;

    std::atomic<bool> _is_initialized;
    std::atomic<bool> _is_recv_running;
    std::atomic<bool> _is_need_swap;
    std::mutex _mutex_swap;
    std::mutex _mutex_recv;
    std::mutex _mutex_recv_swap;
    std::condition_variable _cv_recv;
    std::condition_variable _cv_recv_swap;
    std::thread _thread_recv;

    Ogre::SceneNode *_scene_node, *_avatar_node;
    Ogre::SceneManager *_scene_manager;
    Ogre::Entity *_entity;

    unsigned int _texture_width;
    std::string _texture_name;
    std::string _material_name;
    std::string _mesh_name;
    std::string _submesh_name;
    std::string _entity_name;

    SGTP::texture_bounding_box_t _texture_bounding_boxes[16];
    std::vector<unsigned char> _buffer_rcv;
    std::vector<unsigned char> _buffer_rcv_inflated;
    std::vector<unsigned char> _buffer_rcv_texture;
    std::vector<unsigned char> _buffer_rcv_texture_decompressed;
    std::vector<int32_t> _buffer_index;
    size_t _num_geometry_bytes = 0;
    float _bb_min[3];
    float _bb_max[3];

    unsigned char *_tj_compressed_image_buffer;
    std::unordered_map<uint32_t, tjhandle> _jpeg_decompressor_per_layer;

    void Update();
    void UpdateTriangleSoup();

    void _ReadLoop();
};
} // namespace gazebo

#endif // GUACAMOLE_GUADYNGEOVISUALPLUGIN_H
