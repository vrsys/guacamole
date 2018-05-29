#include "GuaWorldPlugin.h"
gazebo::GuaWorldPlugin::GuaWorldPlugin() { _worker_should_stop.store(false); }
gazebo::GuaWorldPlugin::~GuaWorldPlugin()
{
    _worker_should_stop.store(true);
    _worker_cv.notify_all();
    _worker.join();
}
void gazebo::GuaWorldPlugin::_connect_to_transport_layer()
{
    gazebo::common::load();

    sdf::setFindCallback(boost::bind(&gazebo::common::find_file, _1));

    if(!gazebo::transport::init("invincible", 11345, 1))
    {
        throw std::runtime_error("Unable to initialize transport");
    }

    gazebo::common::ModelDatabase::Instance()->Start(true);

    gazebo::transport::run();

    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    gazebo::transport::PublisherPtr pub_models = node->Advertise<gazebo::msgs::Pose>("/gazebo/nrp_plugins/models", 60, 60);

    if(pub_models->WaitForConnection(gazebo::common::Time(5, 0)))
    {
        std::unique_lock<std::mutex> lk(_worker_mutex);
        while(!_worker_cv.wait_for(lk, std::chrono::milliseconds(4), [&] { return _worker_should_stop.load(); }))
        {
            if(pub_models && pub_models->HasConnections())
            {
                msgs::PosesStamped msg;
                msgs::Set(msg.mutable_time(), this->_world->GetSimTime());

                if(!this->_world->GetModels().empty())
                {
                    for(auto const &model : this->_world->GetModels())
                    {
                        std::list<physics::ModelPtr> modelList;
                        modelList.push_back(model);
                        while(!modelList.empty())
                        {
                            physics::ModelPtr m = modelList.front();
                            modelList.pop_front();
                            msgs::Pose *poseMsg = msg.add_pose();

                            poseMsg->set_name(m->GetScopedName());
                            poseMsg->set_id(m->GetId());
                            msgs::Set(poseMsg, m->GetRelativePose().Ign());

                            physics::Link_V links = m->GetLinks();
                            for(auto const &link : links)
                            {
                                poseMsg = msg.add_pose();
                                poseMsg->set_name(link->GetScopedName());
                                poseMsg->set_id(link->GetId());
                                msgs::Set(poseMsg, link->GetRelativePose().Ign());
                            }

                            physics::Model_V models = m->NestedModels();
                            for(auto const &n : models)
                                modelList.push_back(n);
                        }
                    }

                    if(pub_models && pub_models->HasConnections())
                        pub_models->Publish(msg);
                }
            }
        }

        pub_models.reset();
    }
    else
    {
        throw std::runtime_error("connection not established");
    }

    node->Fini();
    node.reset();

    gazebo::transport::stop();
    gazebo::transport::fini();

    gazebo::common::ModelDatabase::Instance()->Fini();
}
void gazebo::GuaWorldPlugin::Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf)
{
    _world = world;
    _sdf = sdf;

    _worker = std::thread([&] { _connect_to_transport_layer(); });
}
