#ifndef GUACAMOLE_PAGODA_H
#define GUACAMOLE_PAGODA_H

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
    void bind_cam_node(NRPCameraNode *cam_node);
    void bind_transport_layer();

    void pre_render();

    void lock_scene();
    void unlock_scene();

  private:
    NRPBinder();

    std::thread _worker;
    std::mutex _worker_mutex;
    std::condition_variable _worker_cv;

    NRPLog _log;
    NRPScene _scene;

    void _connect_to_transport_layer(int argc, char **argv);
    void _halt_transport_layer();

    void callback_world(ConstWorldStatisticsPtr &_msg);
    void callback_material(ConstMaterialPtr &_msg);
    void callback_skeleton_pose_info(ConstPoseAnimationPtr &ptr);
    void callback_model_info(ConstModelPtr &ptr);
    void callback_pose_info(ConstPosesStampedPtr &ptr);
    void callback_factory_light(ConstLightPtr &ptr);
    void callback_modify_light(ConstLightPtr &ptr);
    void callback_request(ConstRequestPtr &ptr);
    void callback_response(ConstResponsePtr &ptr);
};
}
}

#endif // GUACAMOLE_PAGODA_H