#include "GuaWorldPlugin.h"
gazebo::GuaWorldPlugin::GuaWorldPlugin() {}
gazebo::GuaWorldPlugin::~GuaWorldPlugin()
{
    pub.reset();
    node.reset();
}
void gazebo::GuaWorldPlugin::Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf)
{
    _world = world;
    _sdf = sdf;

    this->node.reset(new transport::Node());
    this->pub = node->Advertise<gazebo::msgs::PosesStamped>("/gazebo/nrp_plugins/poses", 60, 60);

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GuaWorldPlugin::on_update, this));
}
void gazebo::GuaWorldPlugin::on_update()
{
    if(pub && pub->HasConnections())
    {
#if GUA_DEBUG == 1
        gzerr << "8" << std::endl;
        std::cerr << "8" << std::endl;
#endif

        msgs::PosesStamped msg;
        msgs::Set(msg.mutable_time(), this->_world->GetSimTime());

        if(!this->_world->GetModels().empty())
        {
#if GUA_DEBUG == 1
            gzerr << "9" << std::endl;
            std::cerr << "9" << std::endl;
#endif

            for(auto const &model : this->_world->GetModels())
            {
#if GUA_DEBUG == 1
                gzerr << "10" << std::endl;
                std::cerr << "10" << std::endl;
#endif

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

#if GUA_DEBUG == 1
            gzerr << "11" << std::endl;
            std::cerr << "11" << std::endl;
#endif

            if(pub && pub->HasConnections())
                pub->Publish(msg, true);

#if GUA_DEBUG == 1
            gzerr << "12" << std::endl;
            std::cerr << "12" << std::endl;
#endif
        }
    }
}
