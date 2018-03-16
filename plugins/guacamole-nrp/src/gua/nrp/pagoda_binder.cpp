#include <gua/nrp/pagoda_binder.hpp>

namespace gua
{
namespace nrp
{
PagodaBinder::PagodaBinder() : _log("transport"), _scene() {}
PagodaBinder::~PagodaBinder()
{
    _halt_transport_layer();
    _worker.join();
}
void PagodaBinder::bind_root_node(gua::node::Node *root_node) { _scene.set_root_node(root_node); }
void PagodaBinder::bind_transport_layer(int argc, char **argv)
{
    _worker = std::thread([&] { _connect_to_transport_layer(argc, argv); });
}
void PagodaBinder::pre_render()
{
    _scene.pre_render();
    // _log.print_scenegraph(_scene.get_scene_graph()->get_root());
}

void PagodaBinder::_connect_to_transport_layer(int argc, char **argv)
{
    PagodaLog log("worker");

    log.d("gazebo client: setup");

    gazebo::client::setup(argc, argv);

    log.d("gazebo client: init transport node");

    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    log.d("gazebo client: subscription");

    gazebo::transport::SubscriberPtr sub_world = node->Subscribe("~/world_stats", &PagodaBinder::callback_world, this);
    gazebo::transport::SubscriberPtr sub_model = node->Subscribe("~/model/info", &PagodaBinder::callback_model_info, this);
    gazebo::transport::SubscriberPtr sub_pose_info = node->Subscribe("~/pose/info", &PagodaBinder::callback_pose_info, this);
    gazebo::transport::SubscriberPtr sub_material = node->Subscribe("~/material", &PagodaBinder::callback_material, this);

    gazebo::transport::PublisherPtr pub_request = node->Advertise<gazebo::msgs::Request>("~/request");

    gazebo::transport::SubscriberPtr sub_request = node->Subscribe("~/request", &PagodaBinder::callback_request, this);
    gazebo::transport::SubscriberPtr sub_response = node->Subscribe("~/response", &PagodaBinder::callback_response, this);
    gazebo::transport::SubscriberPtr sub_skeleton_pose_info = node->Subscribe("~/skeleton_pose/info", &PagodaBinder::callback_skeleton_pose_info, this);

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
void PagodaBinder::_halt_transport_layer() { _worker_cv.notify_all(); }
void PagodaBinder::callback_world(ConstWorldStatisticsPtr &_msg)
{
    _log.d("callback_world");
    _log.d(_msg->DebugString().c_str());
}
void PagodaBinder::callback_material(ConstMaterialPtr &_msg)
{
    _log.d("callback_material");
    _log.d(_msg->DebugString().c_str());
}
void PagodaBinder::callback_skeleton_pose_info(ConstPoseAnimationPtr &ptr)
{
    _log.d("callback_skeleton_pose_info");
    _log.d(ptr->DebugString().c_str());

    _scene.on_skeleton_pose_msg(ptr);
}
void PagodaBinder::callback_model_info(ConstModelPtr &ptr)
{
    _log.d("callback_model_info");
    _log.d(ptr->DebugString().c_str());

    _scene.on_model_msg(ptr);
}
void PagodaBinder::callback_pose_info(ConstPosesStampedPtr &ptr)
{
    _log.d("callback_pose_info");
    _log.d(ptr->DebugString().c_str());

    _scene.on_pose_msg(ptr);
}
void PagodaBinder::callback_request(ConstRequestPtr &ptr)
{
    _log.d("callback_request");
    _log.d(ptr->DebugString().c_str());
}
void PagodaBinder::callback_response(ConstResponsePtr &ptr)
{
    _log.d("callback_response");
    _log.d(ptr->DebugString().c_str());

    gazebo::msgs::Scene sceneMsg;
    sceneMsg.ParseFromString(ptr->serialized_data());

    _log.d(sceneMsg.DebugString().c_str());

    _scene.on_response_msg(ptr);
}
}
}