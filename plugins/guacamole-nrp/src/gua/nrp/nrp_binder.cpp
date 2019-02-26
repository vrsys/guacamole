#include <gua/nrp/nrp_binder.hpp>
#include <gua/nrp/nrp_cam_node.hpp>
#include <gua/nrp/nrp_config.hpp>
#include <gua/nrp/nrp_interactive_node.hpp>
#include <gua/nrp/nrp_node.hpp>

#include <scm/core/math/math.h>

#include <array>

std::array<double, 3> euler_angles(double q0, double q1, double q2, double q3)
{
    return {atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)), asin(2 * (q0 * q2 - q3 * q1)), atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))};
}

namespace gua
{
namespace nrp
{
NRPBinder::NRPBinder()
    : _worker_mutex(), _scene() /*,
 #if GUA_DEBUG == 1
       _log("transport", NRPLog::LOG_LEVEL::DEBUG)
 #else
       _log("transport", NRPLog::LOG_LEVEL::ERROR)
 #endif
 */

{
    _worker_should_stop.store(false);
    _scene_initialized.store(false);
    _publish_interactive.store(false);
    _scene_frame.store(0);
}
NRPBinder::~NRPBinder()
{
    _halt_transport_layer();
    _worker.join();
}
void NRPBinder::bind_root_node(gua::nrp::NRPNode *root_node) { _scene.set_root_node(root_node); }
void NRPBinder::bind_interactive_node(NRPInteractiveNode *interactive_node)
{
    _scene.set_interactive_node(interactive_node);
    _publish_interactive.store(true);
}
void NRPBinder::bind_cam_node(gua::nrp::NRPCameraNode *cam_node) { _scene.set_cam_node(cam_node); }
void NRPBinder::bind_transport_layer()
{
    _worker = std::thread([&] { _connect_to_transport_layer(); });
}
void NRPBinder::pre_render() { _scene.pre_render(); }

void NRPBinder::_connect_to_transport_layer()
{
    /*
    #if GUA_DEBUG == 1
        NRPLog log("worker", NRPLog::LOG_LEVEL::DEBUG);
    #else
        NRPLog log("worker", NRPLog::LOG_LEVEL::ERROR);
    #endif

        log.d("setup");
    */

    auto nrp_config = &NRPConfig::get_instance();

    gazebo::common::load();

    sdf::setFindCallback(boost::bind(&gazebo::common::find_file, _1));

    if(!gazebo::transport::init(nrp_config->get_master_host(), nrp_config->get_master_port(), 1))
    {
        std::string const error_message = "Unable to initialize transport";
        // log.e(error_message);
        throw std::runtime_error(error_message);
    }

    // log.d("starting the model database, fetching models immediately");

    gazebo::common::ModelDatabase::Instance()->Start(true);

    gazebo::transport::run();

    // log.d("init transport node");

    gazebo::transport::NodePtr node = boost::make_shared<gazebo::transport::Node>();
    node->Init();

    // log.d("begin subscription");

    //    gazebo::transport::SubscriberPtr sub_scene = node->Subscribe("/gazebo/default/scene", &NRPBinder::callback_scene, this);
    //
    //    gazebo::transport::SubscriberPtr sub_world = node->Subscribe("/gazebo/default/world_stats", &NRPBinder::callback_world, this);
    gazebo::transport::SubscriberPtr sub_model = node->Subscribe("/gazebo/default/model/info", &NRPBinder::callback_model_info, this);
    gazebo::transport::SubscriberPtr sub_pose_info = node->Subscribe("/gazebo/default/pose/info", &NRPBinder::callback_pose_info, this);
    gazebo::transport::SubscriberPtr sub_material = node->Subscribe("/gazebo/default/material", &NRPBinder::callback_material, this);

    gazebo::transport::SubscriberPtr sub_factory_light = node->Subscribe("/gazebo/default/factory/light", &NRPBinder::callback_factory_light, this);
    gazebo::transport::SubscriberPtr sub_modify_light = node->Subscribe("/gazebo/default/light/modify", &NRPBinder::callback_modify_light, this);

    //    gazebo::transport::SubscriberPtr sub_skeleton_pose_info = node->Subscribe("/gazebo/default/skeleton_pose/info", &NRPBinder::callback_skeleton_pose_info, this);

    gazebo::transport::PublisherPtr pub_request =
        node->Advertise<gazebo::msgs::Request>("/gazebo/default/request", nrp_config->get_request_queue_limit(), nrp_config->get_full_scene_update_frequency());
    gazebo::transport::SubscriberPtr sub_response = node->Subscribe("/gazebo/default/response", &NRPBinder::callback_response, this);

    // log.d("subscription done");

    gazebo::transport::PublisherPtr pub_interactive =
        node->Advertise<gazebo::msgs::PosesStamped>("/nrp-gua/interactive_pos", nrp_config->get_interactive_node_queue_limit(), nrp_config->get_interactive_node_update_frequency());

    if(!pub_interactive->WaitForConnection(gazebo::common::Time(nrp_config->get_network_max_timeout(), 0)))
    {
        // log.e("no interactive node subscribers available");

        _publish_interactive.store(false);
        pub_interactive.reset();
    }

    if(pub_request->WaitForConnection(gazebo::common::Time(nrp_config->get_network_max_timeout(), 0)))
    {
        // log.d("connection established");

        std::unique_lock<std::mutex> lk(_worker_mutex);
        while(!_worker_cv.wait_for(lk, std::chrono::milliseconds(nrp_config->get_worker_wait_milliseconds()), [&] { return _worker_should_stop.load(); }))
        {
            int_fast32_t scene_frame = _scene_frame.load();

            if(!_scene_initialized.load() || scene_frame > nrp_config->get_max_scene_frames_till_update())
            {
                auto request_scene = gazebo::msgs::CreateRequest("scene_info");
                pub_request->Publish(*(request_scene), true);

                _scene_frame.store(0);
            }
            else
            {
                // auto request_entity = gazebo::msgs::CreateRequest("entity_list");
                // pub_request->Publish(*(request_entity), true);

                _scene_frame.store(scene_frame + 1);
            }

            if(_scene_initialized.load() && _publish_interactive.load() && pub_interactive && pub_interactive->HasConnections())
            {
                try
                {
#if GUA_DEBUG == 1
                    auto start = std::chrono::high_resolution_clock::now();
#endif

                    auto interactive_node = _scene.get_interactive_node();
                    auto nrp_node = _scene.get_root_node();

                    if(interactive_node == nullptr)
                    {
                        continue;
                    }

                    gazebo::msgs::PosesStamped msg;
                    gazebo::msgs::Set(msg.mutable_time(), gazebo::common::Time::GetWallTime());

                    scm::math::mat4d transform = scm::math::inverse(nrp_node->get_world_transform()) * interactive_node->get_world_transform();
                    scm::math::vec3d translation = gua::math::get_translation(transform);
                    scm::math::quatd rot_quat = scm::math::quatd::from_matrix(gua::math::get_rotation(transform));
                    std::array<double, 3> rotation = euler_angles(rot_quat.x, rot_quat.y, rot_quat.z, rot_quat.w);

                    ignition::math::Pose3d pose;
                    pose.Set(translation.x, translation.y, translation.z, rotation[0], rotation[1], rotation[2]);

                    gazebo::msgs::Pose *pose_msg = msg.add_pose();
                    pose_msg->set_name(nrp_config->get_interactive_transform_name());
                    pose_msg->set_id(0);
                    gazebo::msgs::Set(pose_msg, pose);

                    for(auto const &child : interactive_node->get_children())
                    {
                        std::list<std::shared_ptr<gua::node::Node>> nodes;
                        nodes.emplace_back(child);
                        while(!nodes.empty())
                        {
                            auto child_node = nodes.front();
                            nodes.pop_front();

                            if(child_node->get_name().empty())
                            {
                                continue;
                            }

                            transform = scm::math::inverse(nrp_node->get_world_transform()) * child_node->get_world_transform();
                            translation = gua::math::get_translation(transform);
                            rot_quat = scm::math::quatd::from_matrix(gua::math::get_rotation(transform));
                            rotation = euler_angles(rot_quat.x, rot_quat.y, rot_quat.z, rot_quat.w);

                            pose.Set(translation.x, translation.y, translation.z, rotation[0], rotation[1], rotation[2]);

                            pose_msg = msg.add_pose();
                            pose_msg->set_name(child_node->get_name());
                            pose_msg->set_id(0);
                            gazebo::msgs::Set(pose_msg, pose);

                            for(auto const &grand_child_node : child_node->get_children())
                            {
                                nodes.emplace_back(grand_child_node);
                            }
                        }
                    }

                    if(pub_interactive && pub_interactive->HasConnections())
                    {
                        pub_interactive->Publish(msg, true);
                    }

#if GUA_DEBUG == 1
                    // std::cout << transform << std::endl;
                    std::cout << pose << std::endl;

                    auto end = std::chrono::high_resolution_clock::now();
                    float interactive_upload = std::chrono::duration<float, std::milli>(end - start).count();

                    std::cout << "interactive_upload: " << interactive_upload << std::endl;
#endif
                }
                catch(const std::exception &e)
                {
                    gzerr << "Exception caught: " << e.what() << std::endl;
                    std::cerr << "Exception caught: " << e.what() << std::endl;
                }
                catch(const gazebo::common::Exception &e)
                {
                    gzerr << "Exception caught: " << e.GetErrorStr() << std::endl;
                    std::cerr << "Exception caught: " << e.GetErrorStr() << std::endl;
                }
                catch(...)
                {
                    gzerr << "Caught unrecognized error" << std::endl;
                    std::cerr << "Caught unrecognized error" << std::endl;
                }
            }
        }

        pub_request.reset();
        sub_response.reset();
    }
    else
    {
        // log.e("connection not established");

        throw std::runtime_error("connection not established");
    }

    pub_interactive.reset();

    // sub_scene.reset();

    // sub_world.reset();
    sub_model.reset();
    sub_pose_info.reset();
    sub_material.reset();

    sub_factory_light.reset();
    sub_modify_light.reset();

    // sub_skeleton_pose_info.reset();

    node->Fini();
    node.reset();

    gazebo::transport::stop();
    gazebo::transport::fini();

    gazebo::common::ModelDatabase::Instance()->Fini();

    // log.d("over");
}
void NRPBinder::_halt_transport_layer()
{
    _worker_should_stop.store(true);
    _worker_cv.notify_all();
}
void NRPBinder::callback_response(ConstResponsePtr &ptr)
{
#if GUA_DEBUG == 1
    auto start = std::chrono::high_resolution_clock::now();
#endif

    //_log.d("callback_response");
    //_log.d(ptr->request().c_str());

    if(ptr->request() == "scene_info")
    {
        gazebo::msgs::Scene scene_msg;

        if(scene_msg.ParseFromString(ptr->serialized_data()))
        {
            //_log.d(scene_msg.DebugString().c_str());
            _scene.on_scene_msg(boost::make_shared<const gazebo::msgs::Scene>(scene_msg));

            _scene_initialized.store(true);
        }
    }
    else if(ptr->request() == "entity_list")
    {
        gazebo::msgs::Model_V models_msg;

        if(models_msg.ParseFromString(ptr->serialized_data()))
        {
            //_log.d(models_msg.DebugString().c_str());
            for(uint32_t i = 0; i < models_msg.models().size(); i++)
            {
                _scene.on_model_msg(boost::make_shared<const gazebo::msgs::Model>(models_msg.models(i)));
            }
        }
    }

#if GUA_DEBUG == 1
    auto end = std::chrono::high_resolution_clock::now();
    float callback_response = std::chrono::duration<float, std::milli>(end - start).count();

    std::cout << "callback_response: " << callback_response << std::endl;
#endif
}
void NRPBinder::callback_world(ConstWorldStatisticsPtr &_msg)
{
    //_log.d("callback_world");
    //_log.d(_msg->DebugString().c_str());
}
void NRPBinder::callback_material(ConstMaterialPtr &_msg)
{
    //_log.d("callback_material");
    //_log.d(_msg->DebugString().c_str());
}
void NRPBinder::callback_skeleton_pose_info(ConstPoseAnimationPtr &ptr)
{
    //_log.d("callback_skeleton_pose_info");
    //_log.d(ptr->DebugString().c_str());

    _scene.on_skeleton_pose_msg(ptr);
}
void NRPBinder::callback_model_info(ConstModelPtr &ptr)
{
    //_log.d("callback_model_info");
    //_log.d(ptr->DebugString().c_str());

    _scene.on_model_msg(ptr);
}
void NRPBinder::callback_pose_info(ConstPosesStampedPtr &ptr)
{
    //_log.d("callback_pose_info");
    //_log.d(ptr->DebugString().c_str());

    _scene.on_pose_msg(ptr);
}
void NRPBinder::callback_scene(ConstScenePtr &ptr)
{
    //_log.d("callback_scene");
    //_log.d(ptr->DebugString().c_str());

    _scene.on_scene_msg(ptr);
}
void NRPBinder::callback_factory_light(ConstLightPtr &ptr)
{
    //_log.d("callback_factory_light");
    //_log.d(ptr->DebugString().c_str());

    _scene.on_light_factory_msg(ptr);
}
void NRPBinder::callback_modify_light(ConstLightPtr &ptr)
{
    //_log.d("callback_modify_light");
    //_log.d(ptr->DebugString().c_str());

    _scene.on_light_modify_msg(ptr);
}
std::mutex &NRPBinder::get_scene_mutex() { return _scene.get_mutex_scenegraph(); }
} // namespace nrp
} // namespace gua