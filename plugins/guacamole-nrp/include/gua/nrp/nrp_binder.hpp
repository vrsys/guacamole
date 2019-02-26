#ifndef GUACAMOLE_NRP_H
#define GUACAMOLE_NRP_H

#include <functional>
#include <list>
#include <memory>
#include <string>
#include <unordered_set>

#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <boost/enable_shared_from_this.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/unordered/unordered_map.hpp>

#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>

#include <gazebo/msgs/msgs.hh>

#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Exception.hh>

#include <gazebo/transport/Node.hh>
#include <gazebo/transport/TransportIface.hh>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/TransportTypes.hh>

#include <gua/node/TransformNode.hpp>
#include <gua/renderer/MaterialLoader.hpp>
#include <gua/renderer/TriMeshLoader.hpp>
#include <gua/scenegraph/SceneGraph.hpp>

#include <gua/nrp/nrp_log.hpp>
#include <gua/nrp/nrp_scene.hpp>
#include <gua/nrp/platform.hpp>

namespace gua
{
namespace nrp
{
class NRPNode;
class NRPCameraNode;

class GUA_NRP_DLL NRPBinder
{
  public:
    static NRPBinder &get_instance()
    {
        static NRPBinder nrp_binder;
        return nrp_binder;
    }
    NRPBinder(NRPBinder const &) = delete;
    void operator=(NRPBinder const &) = delete;

    ~NRPBinder();

    void bind_root_node(NRPNode *root_node);
    void bind_interactive_node(NRPInteractiveNode *interactive_node);
    void bind_cam_node(NRPCameraNode *cam_node);
    void bind_transport_layer();

    void pre_render();

    std::mutex &get_scene_mutex();

  private:
    NRPBinder();

    std::thread _worker;
    std::atomic_bool _worker_should_stop;
    std::mutex _worker_mutex;
    std::condition_variable _worker_cv;

    // NRPLog _log;
    NRPScene _scene;

    std::atomic_bool _scene_initialized;
    std::atomic_bool _publish_interactive;
    std::atomic_int_fast32_t _scene_frame;

    void _connect_to_transport_layer();
    void _halt_transport_layer();

    void callback_world(ConstWorldStatisticsPtr &_msg);
    void callback_material(ConstMaterialPtr &_msg);
    void callback_skeleton_pose_info(ConstPoseAnimationPtr &ptr);
    void callback_model_info(ConstModelPtr &ptr);
    void callback_pose_info(ConstPosesStampedPtr &ptr);
    void callback_factory_light(ConstLightPtr &ptr);
    void callback_modify_light(ConstLightPtr &ptr);
    void callback_scene(ConstScenePtr &ptr);
    void callback_response(ConstResponsePtr &ptr);
};
} // namespace nrp
} // namespace gua

#endif // GUACAMOLE_NRP_H
