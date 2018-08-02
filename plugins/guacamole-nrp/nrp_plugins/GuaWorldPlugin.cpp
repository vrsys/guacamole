#include "GuaWorldPlugin.hpp"

gazebo::GuaWorldPlugin::GuaWorldPlugin() {}
gazebo::GuaWorldPlugin::~GuaWorldPlugin()
{
    _pub.reset();
    _node.reset();
}
void gazebo::GuaWorldPlugin::Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf)
{
    _world = world;
    _sdf = sdf;

    _node.reset(new transport::Node());
    _node->Init();
    _pub = _node->Advertise<gazebo::msgs::PosesStamped>("/gazebo/nrp_plugins/poses", 60, 60);

    _update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GuaWorldPlugin::on_update, this));
}
void gazebo::GuaWorldPlugin::on_update()
{
    try
    {
        if(_pub && _pub->HasConnections())
        {
#if GUA_DEBUG == 1
            gzerr << "Publisher has connections" << std::endl;
            std::cerr << "Publisher has connections" << std::endl;
#endif

            msgs::PosesStamped msg;
            msgs::Set(msg.mutable_time(), this->_world->GetSimTime());

            if(!this->_world->GetModels().empty())
            {
#if GUA_DEBUG == 1
                gzerr << "Collecting models" << std::endl;
                std::cerr << "Collecting models" << std::endl;
#endif

                for(auto const &model : this->_world->GetModels())
                {
#if GUA_DEBUG == 1
                    gzerr << "Collecting a model" << std::endl;
                    std::cerr << "Collecting a model" << std::endl;
#endif

                    std::list<physics::ModelPtr> modelList;
                    modelList.emplace_back(model);
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
                            modelList.emplace_back(n);
                    }
                }

#if GUA_DEBUG == 1
                gzerr << "Preparing to publish" << std::endl;
                std::cerr << "Preparing to publish" << std::endl;
#endif

                if(_pub && _pub->HasConnections())
                {
                    _pub->Publish(msg, true);
                }

#if GUA_DEBUG == 1
                else
                {
                    gzerr << "Publisher has no connections" << std::endl;
                    std::cerr << "Publisher has no connections" << std::endl;
                }
#endif
            }
        }
    }
    catch(const std::exception &e)
    {
        gzerr << "Exception caught: " << e.what() << std::endl;
        std::cerr << "Exception caught: " << e.what() << std::endl;
    }
    catch(...)
    {
        gzerr << "Caught unrecognized error" << std::endl;
        std::cerr << "Caught unrecognized error" << std::endl;
    }
}
