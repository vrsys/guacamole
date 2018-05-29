#include <gua/nrp/nrp_node.hpp>
#include <gua/nrp/nrp_cam_node.hpp>
#include <gua/nrp/nrp_binder.hpp>

namespace gua
{
namespace nrp
{
NRPBinder::NRPBinder() : _log("transport"), _scene() {}
NRPBinder::~NRPBinder()
{
    _halt_transport_layer();
    _worker.join();
}
void NRPBinder::bind_root_node(gua::nrp::NRPNode *root_node) { _scene.set_root_node(root_node); }
void NRPBinder::bind_cam_node(gua::nrp::NRPCameraNode *cam_node) { _scene.set_cam_node(cam_node); }
void NRPBinder::bind_transport_layer()
{
    _worker = std::thread([&] { _connect_to_transport_layer(0, nullptr); });
}
void NRPBinder::pre_render()
{
    _scene.pre_render();
    // _log.print_scenegraph(_scene.get_scene_graph()->get_root());
}

void NRPBinder::_connect_to_transport_layer(int argc, char **argv)
{
    NRPLog log("worker");

    log.d("gazebo client: setup");

    gazebo::client::setup(argc, argv);

    log.d("gazebo client: init transport node");

    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    log.d("gazebo client: subscription");

    gazebo::transport::SubscriberPtr sub_world = node->Subscribe("~/world_stats", &NRPBinder::callback_world, this);
    gazebo::transport::SubscriberPtr sub_model = node->Subscribe("~/model/info", &NRPBinder::callback_model_info, this);
    gazebo::transport::SubscriberPtr sub_pose_info = node->Subscribe("~/pose/info", &NRPBinder::callback_pose_info, this);
    gazebo::transport::SubscriberPtr sub_material = node->Subscribe("~/material", &NRPBinder::callback_material, this);

    gazebo::transport::SubscriberPtr sub_factory_light = node->Subscribe("~/factory/light", &NRPBinder::callback_factory_light, this);
    gazebo::transport::SubscriberPtr sub_modify_light = node->Subscribe("~/light/modify", &NRPBinder::callback_modify_light, this);

    gazebo::transport::PublisherPtr pub_request = node->Advertise<gazebo::msgs::Request>("~/request");

    gazebo::transport::SubscriberPtr sub_request = node->Subscribe("~/request", &NRPBinder::callback_request, this);
    gazebo::transport::SubscriberPtr sub_response = node->Subscribe("~/response", &NRPBinder::callback_response, this);
    gazebo::transport::SubscriberPtr sub_skeleton_pose_info = node->Subscribe("~/skeleton_pose/info", &NRPBinder::callback_skeleton_pose_info, this);

    if(!pub_request->WaitForConnection(gazebo::common::Time(5, 0)))
    {
        log.e("gazebo client: connection not established");
    }
    else
    {
        pub_request->Publish(*(gazebo::msgs::CreateRequest("scene_info")), false);
        log.d("gazebo client: connection established");

        std::unique_lock<std::mutex> lk(_worker_mutex);
        _worker_cv.wait(lk);
    }

    pub_request->Fini();
    node->Fini();

    gazebo::client::shutdown();

    log.d("gazebo client: over");
}
void NRPBinder::_halt_transport_layer() { _worker_cv.notify_all(); }
void NRPBinder::callback_world(ConstWorldStatisticsPtr &_msg)
{
    _log.d("callback_world");
    _log.d(_msg->DebugString().c_str());
}
void NRPBinder::callback_material(ConstMaterialPtr &_msg)
{
    _log.d("callback_material");
    _log.d(_msg->DebugString().c_str());
}
void NRPBinder::callback_skeleton_pose_info(ConstPoseAnimationPtr &ptr)
{
    _log.d("callback_skeleton_pose_info");
    _log.d(ptr->DebugString().c_str());

    _scene.on_skeleton_pose_msg(ptr);
}
void NRPBinder::callback_model_info(ConstModelPtr &ptr)
{
    _log.d("callback_model_info");
    _log.d(ptr->DebugString().c_str());

    _scene.on_model_msg(ptr);
}
void NRPBinder::callback_pose_info(ConstPosesStampedPtr &ptr)
{
    _log.d("callback_pose_info");
    _log.d(ptr->DebugString().c_str());

    _scene.on_pose_msg(ptr);
}
void NRPBinder::callback_request(ConstRequestPtr &ptr)
{
    _log.d("callback_request");
    _log.d(ptr->DebugString().c_str());
}
void NRPBinder::callback_response(ConstResponsePtr &ptr)
{
    _log.d("callback_response");
    _log.d(ptr->DebugString().c_str());

    gazebo::msgs::Scene sceneMsg;
    sceneMsg.ParseFromString(ptr->serialized_data());

    _log.d(sceneMsg.DebugString().c_str());

    _scene.on_response_msg(ptr);
}
void NRPBinder::callback_factory_light(ConstLightPtr &ptr)
{
    _log.d("callback_factory_light");
    _log.d(ptr->DebugString().c_str());

    _scene.on_light_factory_msg(ptr);
}
void NRPBinder::callback_modify_light(ConstLightPtr &ptr)
{
    _log.d("callback_modify_light");
    _log.d(ptr->DebugString().c_str());

    _scene.on_light_modify_msg(ptr);
}
void NRPBinder::lock_scene() { _scene.get_mutex_scenegraph().lock(); }
void NRPBinder::unlock_scene() { _scene.get_mutex_scenegraph().unlock(); }
}
}