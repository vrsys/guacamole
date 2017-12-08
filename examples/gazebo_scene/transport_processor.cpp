#ifndef GUACAMOLE_TRANSPORT_PROCESSOR_H
#define GUACAMOLE_TRANSPORT_PROCESSOR_H

#include "common.h"
namespace gazebo
{
namespace rendering
{
typedef std::map<uint32_t, VisualPtr> Visual_M;
typedef std::list<boost::shared_ptr<msgs::Visual const>> VisualMsgs_L;
typedef std::list<boost::shared_ptr<msgs::Light const>> LightMsgs_L;
typedef std::map<uint32_t, msgs::Pose> PoseMsgs_M;
typedef std::list<boost::shared_ptr<msgs::Scene const>> SceneMsgs_L;
typedef std::list<boost::shared_ptr<msgs::Joint const>> JointMsgs_L;
typedef std::list<boost::shared_ptr<msgs::Link const>> LinkMsgs_L;
typedef std::list<boost::shared_ptr<msgs::Model const>> ModelMsgs_L;
typedef std::list<boost::shared_ptr<msgs::Sensor const>> SensorMsgs_L;
typedef std::list<boost::shared_ptr<msgs::Request const>> RequestMsgs_L;
typedef std::map<std::string, LightPtr> Light_M;
typedef std::list<boost::shared_ptr<msgs::PoseAnimation const>> SkeletonPoseMsgs_L;

class TransportProcessor : public boost::enable_shared_from_this<TransportProcessor>
{
  public:
    struct VisualMessageLess
    {
        bool operator()(const boost::shared_ptr<msgs::Visual const> &_i, const boost::shared_ptr<msgs::Visual const> &_j) { return _i->name().size() < _j->name().size(); }
    } VisualMessageLessOp;

    class GazeboState
    {
      public:
        std::string name;
        VisualMsgs_L modelVisualMsgs;
        VisualMsgs_L linkVisualMsgs;
        VisualMsgs_L visualMsgs;
        VisualMsgs_L collisionVisualMsgs;
        LightMsgs_L lightFactoryMsgs;
        LightMsgs_L lightModifyMsgs;
        PoseMsgs_M poseMsgs;
        SceneMsgs_L sceneMsgs;
        JointMsgs_L jointMsgs;
        LinkMsgs_L linkMsgs;
        ModelMsgs_L modelMsgs;
        SensorMsgs_L sensorMsgs;
        Visual_M visuals;
        SkeletonPoseMsgs_L skeletonPoseMsgs;
        std::mutex *receiveMutex;
        std::recursive_mutex poseMsgMutex;
        VisualPtr worldVisual;
        VisualPtr selectedVis;
        std::string selectionMode;
        std::map<std::string, Projector *> projectors;
        bool showCOMs;
        bool showInertias;
        bool showLinkFrames;
        bool showCollisions;
        bool showJoints;
        bool transparent;
        bool wireframe;
        bool initialized;
    };

    TransportProcessor() : dataPtr(new GazeboState)
    {
        this->dataPtr->initialized = false;
        this->dataPtr->showCOMs = false;
        this->dataPtr->showInertias = false;
        this->dataPtr->showLinkFrames = false;
        this->dataPtr->showCollisions = false;
        this->dataPtr->showJoints = false;
        this->dataPtr->transparent = false;
        this->dataPtr->wireframe = false;

        this->dataPtr->receiveMutex = new std::mutex();
    }

    ~TransportProcessor()
    {
        delete this->dataPtr->receiveMutex;
        this->dataPtr->receiveMutex = NULL;

        this->Clear();
    }

    void Clear()
    {
        this->dataPtr->modelMsgs.clear();
        this->dataPtr->visualMsgs.clear();
        this->dataPtr->lightFactoryMsgs.clear();
        this->dataPtr->lightModifyMsgs.clear();
        this->dataPtr->poseMsgs.clear();
        this->dataPtr->sceneMsgs.clear();
        this->dataPtr->jointMsgs.clear();
        this->dataPtr->linkMsgs.clear();
        this->dataPtr->sensorMsgs.clear();

        while(!this->dataPtr->visuals.empty())
            this->RemoveVisual(this->dataPtr->visuals.begin()->first);

        this->dataPtr->visuals.clear();

        if(this->dataPtr->worldVisual)
        {
            this->dataPtr->worldVisual->Fini();
            this->dataPtr->worldVisual.reset();
        }

        this->dataPtr->initialized = false;
    }

    void RemoveVisual(uint32_t _id)
    {
        // Delete the visual
        auto iter = this->dataPtr->visuals.find(_id);
        if(iter != this->dataPtr->visuals.end())
        {
            VisualPtr vis = iter->second;
            // Remove all projectors attached to the visual
            auto piter = this->dataPtr->projectors.begin();
            while(piter != this->dataPtr->projectors.end())
            {
                // Check to see if the projector is a child of the visual that is
                // being removed.
                if(piter->second->GetParent()->GetRootVisual()->GetName() == vis->GetRootVisual()->GetName())
                {
                    delete piter->second;
                    this->dataPtr->projectors.erase(piter++);
                }
                else
                    ++piter;
            }
            this->RemoveVisualizations(vis);

            vis->Fini();
            this->dataPtr->visuals.erase(iter);
            if(this->dataPtr->selectedVis && this->dataPtr->selectedVis->GetId() == vis->GetId())
                this->dataPtr->selectedVis.reset();
        }
    }

    void RemoveVisual(VisualPtr _vis) { this->RemoveVisual(_vis->GetId()); }

    void RemoveVisualizations(rendering::VisualPtr _vis)
    {
        std::vector<VisualPtr> toRemove;
        for(unsigned int i = 0; i < _vis->GetChildCount(); ++i)
        {
            rendering::VisualPtr childVis = _vis->GetChild(i);
            Visual::VisualType visType = childVis->GetType();
            if(visType == Visual::VT_PHYSICS || visType == Visual::VT_SENSOR || visType == Visual::VT_GUI)
            {
                // do not remove ModelManipulator's SelectionObj
                // FIXME remove this hardcoded check, issue #1832
                if(std::dynamic_pointer_cast<SelectionObj>(childVis) != NULL)
                    continue;

                toRemove.push_back(childVis);
            }
        }
        for(auto vis : toRemove)
            this->RemoveVisual(vis);
    }

    VisualPtr GetVisual(const uint32_t _id) const
    {
        auto iter = this->dataPtr->visuals.find(_id);
        if(iter != this->dataPtr->visuals.end())
            return iter->second;
        return VisualPtr();
    }

    VisualPtr GetVisual(const std::string &_name) const
    {
        VisualPtr result;

        Visual_M::const_iterator iter;
        for(iter = this->dataPtr->visuals.begin(); iter != this->dataPtr->visuals.end(); ++iter)
        {
            if(iter->second == nullptr)
            {
                continue;
            }

            if(iter->second != nullptr && iter->second->GetName() == _name)
                break;
        }

        if(iter != this->dataPtr->visuals.end())
            result = iter->second;
        else
        {
            std::string otherName = this->dataPtr->name + "::" + _name;
            for(iter = this->dataPtr->visuals.begin(); iter != this->dataPtr->visuals.end(); ++iter)
            {
                if(iter->second != nullptr && iter->second->GetName() == otherName)
                    break;
            }

            if(iter != this->dataPtr->visuals.end())
                result = iter->second;
        }

        return result;
    }

    void CreateCOMVisual(ConstLinkPtr &_msg, const VisualPtr &_linkVisual)
    {
        COMVisualPtr comVis(new COMVisual(_linkVisual->GetName() + "_COM_VISUAL__", _linkVisual));
        comVis->Load(_msg);
        comVis->SetVisible(this->dataPtr->showCOMs);
        this->dataPtr->visuals[comVis->GetId()] = comVis;
    }

    void CreateInertiaVisual(ConstLinkPtr &_msg, const VisualPtr &_linkVisual)
    {
        InertiaVisualPtr inertiaVis(new InertiaVisual(_linkVisual->GetName() + "_INERTIA_VISUAL__", _linkVisual));
        inertiaVis->Load(_msg);
        inertiaVis->SetVisible(this->dataPtr->showInertias);
        this->dataPtr->visuals[inertiaVis->GetId()] = inertiaVis;
    }

    void CreateLinkFrameVisual(ConstLinkPtr & /*_msg*/, const VisualPtr &_linkVisual)
    {
        LinkFrameVisualPtr linkFrameVis(new LinkFrameVisual(_linkVisual->GetName() + "_LINK_FRAME_VISUAL__", _linkVisual));
        linkFrameVis->Load();
        linkFrameVis->SetVisible(this->dataPtr->showLinkFrames);
        this->dataPtr->visuals[linkFrameVis->GetId()] = linkFrameVis;
    }

    void OnSkeletonPoseMsg(ConstPoseAnimationPtr &_msg)
    {
        std::lock_guard<std::mutex> lock(*this->dataPtr->receiveMutex);
        SkeletonPoseMsgs_L::iterator iter;

        // Find an old model message, and remove them
        for(iter = this->dataPtr->skeletonPoseMsgs.begin(); iter != this->dataPtr->skeletonPoseMsgs.end(); ++iter)
        {
            if((*iter)->model_name() == _msg->model_name())
            {
                this->dataPtr->skeletonPoseMsgs.erase(iter);
                break;
            }
        }

        this->dataPtr->skeletonPoseMsgs.push_back(_msg);
    }

    void OnModelMsg(ConstModelPtr &_msg)
    {
        std::lock_guard<std::mutex> lock(*this->dataPtr->receiveMutex);
        this->dataPtr->modelMsgs.push_back(_msg);
    }

    void OnResponse(ConstResponsePtr &_msg)
    {
        msgs::Scene sceneMsg;
        sceneMsg.ParseFromString(_msg->serialized_data());
        boost::shared_ptr<msgs::Scene> sm(new msgs::Scene(sceneMsg));

        std::lock_guard<std::mutex> lock(*this->dataPtr->receiveMutex);
        this->dataPtr->sceneMsgs.emplace_back(sm);
    }

    void OnPoseMsg(ConstPosesStampedPtr &_msg)
    {
        std::lock_guard<std::recursive_mutex> lock(this->dataPtr->poseMsgMutex);

        for(int i = 0; i < _msg->pose_size(); ++i)
        {
            auto iter = this->dataPtr->poseMsgs.find(_msg->pose(i).id());
            if(iter != this->dataPtr->poseMsgs.end())
                iter->second.CopyFrom(_msg->pose(i));
            else
                this->dataPtr->poseMsgs.insert(std::make_pair(_msg->pose(i).id(), _msg->pose(i)));
        }
    }

    bool ProcessVisualMsg(ConstVisualPtr &_msg, Visual::VisualType _type = Visual::VT_ENTITY)
    {
        bool result = false;
        auto iter = this->dataPtr->visuals.end();

        if(_msg->has_id())
        {
            iter = this->dataPtr->visuals.find(_msg->id());
        }
        else
        {
            VisualPtr vis = this->GetVisual(_msg->name());
            iter = vis ? this->dataPtr->visuals.find(vis->GetId()) : this->dataPtr->visuals.end();
        }

        if(_msg->has_delete_me() && _msg->delete_me())
        {
            if(iter != this->dataPtr->visuals.end())
            {
                this->dataPtr->visuals.erase(iter);
                result = true;
            }
        }
        else if(iter != this->dataPtr->visuals.end())
        {
            iter->second->UpdateFromMsg(_msg);
            result = true;
        }
        else
        {
            // std::cout << " a bit of a hack: Heightmap stuff" << std::endl;
            VisualPtr visual;

            // If the visual has a parent which is not the name of the scene...
            if(_msg->has_parent_name() && _msg->parent_name() != this->dataPtr->name)
            {
                if(_msg->has_id())
                    iter = this->dataPtr->visuals.find(_msg->id());
                else
                {
                    VisualPtr vis = this->GetVisual(_msg->name());
                    iter = vis ? this->dataPtr->visuals.find(vis->GetId()) : this->dataPtr->visuals.end();
                }

                if(iter != this->dataPtr->visuals.end())
                    gzerr << "Visual already exists. This shouldn't happen.\n";

                // Make sure the parent visual exists before trying to add a child
                // visual
                iter = this->dataPtr->visuals.find(_msg->parent_id());
                if(iter != this->dataPtr->visuals.end())
                {
                    visual.reset(new Visual(_msg->name(), iter->second));
                    if(_msg->has_id())
                        visual->SetId(_msg->id());
                }
            }
            else
            {
                // Add a visual that is attached to the scene root
                visual.reset(new Visual(_msg->name(), this->dataPtr->worldVisual));
                if(_msg->has_id())
                    visual->SetId(_msg->id());
            }

            if(visual)
            {
                result = true;
                visual->LoadFromMsg(_msg);
                visual->SetType(_type);

                this->dataPtr->visuals[visual->GetId()] = visual;
                if(visual->GetName().find("__COLLISION_VISUAL__") != std::string::npos || visual->GetName().find("__SKELETON_VISUAL__") != std::string::npos)
                {
                    visual->SetVisible(false);
                    visual->SetVisibilityFlags(GZ_VISIBILITY_GUI);
                }

                visual->ShowCOM(this->dataPtr->showCOMs);
                visual->ShowInertia(this->dataPtr->showInertias);
                visual->ShowLinkFrame(this->dataPtr->showLinkFrames);
                visual->ShowCollision(this->dataPtr->showCollisions);
                visual->ShowJoints(this->dataPtr->showJoints);
                if(visual->GetType() == Visual::VT_MODEL)
                    visual->SetTransparency(this->dataPtr->transparent ? 0.5f : 0.0f);
                visual->SetWireframe(this->dataPtr->wireframe);
            }
        }

        return result;
    }

    bool ProcessLinkMsg(ConstLinkPtr &_msg)
    {
        VisualPtr linkVis;

        if(_msg->has_id())
            linkVis = this->GetVisual(_msg->id());
        else
            linkVis = this->GetVisual(_msg->name());

        if(!linkVis)
        {
            gzerr << "No link visual with id[" << _msg->id() << "] and name[" << _msg->name() << "]\n";
            return false;
        }

        std::string linkName = linkVis->GetName();
        if(!this->GetVisual(linkName + "_COM_VISUAL__"))
        {
            this->CreateCOMVisual(_msg, linkVis);
        }

        if(!this->GetVisual(linkName + "_INERTIA_VISUAL__"))
        {
            this->CreateInertiaVisual(_msg, linkVis);
        }

        if(!this->GetVisual(linkName + "_LINK_FRAME_VISUAL__"))
        {
            this->CreateLinkFrameVisual(_msg, linkVis);
        }

        for(int i = 0; i < _msg->projector_size(); ++i)
        {
            std::string pname = _msg->name() + "::" + _msg->projector(i).name();

            if(this->dataPtr->projectors.find(pname) == this->dataPtr->projectors.end())
            {
                Projector *projector = new Projector(linkVis);
                projector->Load(_msg->projector(i));
                projector->Toggle();
                this->dataPtr->projectors[pname] = projector;
            }
        }

        return true;
    }

    bool ProcessJointMsg(ConstJointPtr &_msg)
    {
        VisualPtr childVis;

        if(_msg->has_child() && _msg->child() == "world")
            childVis = this->dataPtr->worldVisual;
        else if(_msg->has_child_id())
            childVis = this->GetVisual(_msg->child_id());

        if(!childVis)
            return false;

        JointVisualPtr jointVis(new JointVisual(_msg->name() + "_JOINT_VISUAL__", childVis));
        jointVis->Load(_msg);
        jointVis->SetVisible(this->dataPtr->showJoints);
        if(_msg->has_id())
            jointVis->SetId(_msg->id());

        this->dataPtr->visuals[jointVis->GetId()] = jointVis;

        return true;
    }

    bool ProcessModelMsg(const msgs::Model &_msg)
    {
        std::string modelName, linkName;

        modelName = _msg.name() + "::";
        for(int j = 0; j < _msg.visual_size(); ++j)
        {
            boost::shared_ptr<msgs::Visual> vm(new msgs::Visual(_msg.visual(j)));
            this->dataPtr->modelVisualMsgs.emplace_back(vm);
        }

        // Set the scale of the model visual
        if(_msg.has_scale())
        {
            // update scale using a visual msg
            boost::shared_ptr<msgs::Visual> vm(new msgs::Visual);
            if(_msg.has_id())
                vm->set_id(_msg.id());
            if(_msg.has_name())
                vm->set_name(_msg.name());
            vm->mutable_scale()->set_x(_msg.scale().x());
            vm->mutable_scale()->set_y(_msg.scale().y());
            vm->mutable_scale()->set_z(_msg.scale().z());
            this->dataPtr->modelVisualMsgs.emplace_back(vm);
        }

        for(int j = 0; j < _msg.joint_size(); ++j)
        {
            boost::shared_ptr<msgs::Joint> jm(new msgs::Joint(_msg.joint(j)));
            this->dataPtr->jointMsgs.emplace_back(jm);

            for(int k = 0; k < _msg.joint(j).sensor_size(); ++k)
            {
                boost::shared_ptr<msgs::Sensor> sm(new msgs::Sensor(_msg.joint(j).sensor(k)));
                this->dataPtr->sensorMsgs.emplace_back(sm);
            }
        }

        for(int j = 0; j < _msg.link_size(); ++j)
        {
            linkName = modelName + _msg.link(j).name();

            {
                std::lock_guard<std::recursive_mutex> lock(this->dataPtr->poseMsgMutex);
                if(_msg.link(j).has_pose())
                {
                    auto iter = this->dataPtr->poseMsgs.find(_msg.link(j).id());
                    if(iter != this->dataPtr->poseMsgs.end())
                        iter->second.CopyFrom(_msg.link(j).pose());
                    else
                        this->dataPtr->poseMsgs.insert(std::make_pair(_msg.link(j).id(), _msg.link(j).pose()));

                    this->dataPtr->poseMsgs[_msg.link(j).id()].set_name(linkName);
                    this->dataPtr->poseMsgs[_msg.link(j).id()].set_id(_msg.link(j).id());
                }
            }

            if(_msg.link(j).has_inertial())
            {
                boost::shared_ptr<msgs::Link> lm(new msgs::Link(_msg.link(j)));
                this->dataPtr->linkMsgs.emplace_back(lm);
            }

            if(_msg.link(j).visual_size() > 0)
            {
                // note: the first visual in the link is the link visual
                msgs::VisualPtr vm(new msgs::Visual(_msg.link(j).visual(0)));
                this->dataPtr->linkVisualMsgs.emplace_back(vm);
            }

            for(int k = 1; k < _msg.link(j).visual_size(); ++k)
            {
                boost::shared_ptr<msgs::Visual> vm(new msgs::Visual(_msg.link(j).visual(k)));
                this->dataPtr->visualMsgs.emplace_back(vm);
            }

            for(int k = 0; k < _msg.link(j).collision_size(); ++k)
            {
                for(int l = 0; l < _msg.link(j).collision(k).visual_size(); l++)
                {
                    boost::shared_ptr<msgs::Visual> vm(new msgs::Visual(_msg.link(j).collision(k).visual(l)));
                    this->dataPtr->collisionVisualMsgs.emplace_back(vm);
                }
            }

            for(int k = 0; k < _msg.link(j).sensor_size(); ++k)
            {
                boost::shared_ptr<msgs::Sensor> sm(new msgs::Sensor(_msg.link(j).sensor(k)));
                this->dataPtr->sensorMsgs.emplace_back(sm);
            }
        }

        for(int i = 0; i < _msg.model_size(); ++i)
        {
            boost::shared_ptr<msgs::Model> mm(new msgs::Model(_msg.model(i)));
            this->dataPtr->modelMsgs.emplace_back(mm);
        }

        return true;
    }

    bool ProcessSceneMsg(ConstScenePtr &_msg)
    {
        {
            std::lock_guard<std::recursive_mutex> lock(this->dataPtr->poseMsgMutex);
            for(int i = 0; i < _msg->model_size(); ++i)
            {
                auto iter = this->dataPtr->poseMsgs.find(_msg->model(i).id());
                if(iter != this->dataPtr->poseMsgs.end())
                    iter->second.CopyFrom(_msg->model(i).pose());
                else
                    this->dataPtr->poseMsgs.insert(std::make_pair(_msg->model(i).id(), _msg->model(i).pose()));

                this->dataPtr->poseMsgs[_msg->model(i).id()].set_name(_msg->model(i).name());
                this->dataPtr->poseMsgs[_msg->model(i).id()].set_id(_msg->model(i).id());

                this->ProcessModelMsg(_msg->model(i));
            }
        }

        for(int i = 0; i < _msg->light_size(); ++i)
        {
            boost::shared_ptr<msgs::Light> lm(new msgs::Light(_msg->light(i)));
            this->dataPtr->lightFactoryMsgs.emplace_back(lm);
        }

        for(int i = 0; i < _msg->joint_size(); ++i)
        {
            boost::shared_ptr<msgs::Joint> jm(new msgs::Joint(_msg->joint(i)));
            this->dataPtr->jointMsgs.emplace_back(jm);
        }

        if(_msg->has_ambient())
        {
            // ignore ambient color
        }

        if(_msg->has_background())
        {
            // ignore background
        }

        if(_msg->has_shadows())
        {
            // ignore shadows
        }

        if(_msg->has_grid())
        {
            // ignore grid
        }

        if(_msg->has_origin_visual())
        {
            // ignore origin
        }

        if(_msg->has_sky())
        {
            // ignore sky
        }

        if(_msg->has_fog())
        {
            // ignore fog
        }
        return true;
    }

    void PreRender()
    {
        static RequestMsgs_L::iterator rIter;
        static SceneMsgs_L::iterator sIter;
        static ModelMsgs_L::iterator modelIter;
        static VisualMsgs_L::iterator visualIter;
        static LightMsgs_L::iterator lightIter;
        static PoseMsgs_M::iterator pIter;
        static SkeletonPoseMsgs_L::iterator spIter;
        static JointMsgs_L::iterator jointIter;
        static SensorMsgs_L::iterator sensorIter;
        static LinkMsgs_L::iterator linkIter;

        SceneMsgs_L sceneMsgsCopy;
        ModelMsgs_L modelMsgsCopy;
        SensorMsgs_L sensorMsgsCopy;
        LightMsgs_L lightFactoryMsgsCopy;
        LightMsgs_L lightModifyMsgsCopy;
        VisualMsgs_L modelVisualMsgsCopy;
        VisualMsgs_L linkVisualMsgsCopy;
        VisualMsgs_L visualMsgsCopy;
        VisualMsgs_L collisionVisualMsgsCopy;
        JointMsgs_L jointMsgsCopy;
        LinkMsgs_L linkMsgsCopy;

        {
            std::lock_guard<std::mutex> lock(*this->dataPtr->receiveMutex);

            std::copy(this->dataPtr->sceneMsgs.begin(), this->dataPtr->sceneMsgs.end(), std::back_inserter(sceneMsgsCopy));
            this->dataPtr->sceneMsgs.clear();

            std::copy(this->dataPtr->modelMsgs.begin(), this->dataPtr->modelMsgs.end(), std::back_inserter(modelMsgsCopy));
            this->dataPtr->modelMsgs.clear();

            std::copy(this->dataPtr->sensorMsgs.begin(), this->dataPtr->sensorMsgs.end(), std::back_inserter(sensorMsgsCopy));
            this->dataPtr->sensorMsgs.clear();

            std::copy(this->dataPtr->lightFactoryMsgs.begin(), this->dataPtr->lightFactoryMsgs.end(), std::back_inserter(lightFactoryMsgsCopy));
            this->dataPtr->lightFactoryMsgs.clear();

            std::copy(this->dataPtr->lightModifyMsgs.begin(), this->dataPtr->lightModifyMsgs.end(), std::back_inserter(lightModifyMsgsCopy));
            this->dataPtr->lightModifyMsgs.clear();

            std::copy(this->dataPtr->modelVisualMsgs.begin(), this->dataPtr->modelVisualMsgs.end(), std::back_inserter(modelVisualMsgsCopy));
            this->dataPtr->modelVisualMsgs.clear();

            std::copy(this->dataPtr->linkVisualMsgs.begin(), this->dataPtr->linkVisualMsgs.end(), std::back_inserter(linkVisualMsgsCopy));
            this->dataPtr->linkVisualMsgs.clear();

            this->dataPtr->visualMsgs.sort(VisualMessageLessOp);
            std::copy(this->dataPtr->visualMsgs.begin(), this->dataPtr->visualMsgs.end(), std::back_inserter(visualMsgsCopy));
            this->dataPtr->visualMsgs.clear();

            std::copy(this->dataPtr->collisionVisualMsgs.begin(), this->dataPtr->collisionVisualMsgs.end(), std::back_inserter(collisionVisualMsgsCopy));
            this->dataPtr->collisionVisualMsgs.clear();

            std::copy(this->dataPtr->jointMsgs.begin(), this->dataPtr->jointMsgs.end(), std::back_inserter(jointMsgsCopy));
            this->dataPtr->jointMsgs.clear();

            std::copy(this->dataPtr->linkMsgs.begin(), this->dataPtr->linkMsgs.end(), std::back_inserter(linkMsgsCopy));
            this->dataPtr->linkMsgs.clear();
        }

        // Process the scene messages. DO THIS FIRST
        for(sIter = sceneMsgsCopy.begin(); sIter != sceneMsgsCopy.end();)
        {
            if(this->ProcessSceneMsg(*sIter))
            {
                if(!this->dataPtr->initialized)
                    RTShaderSystem::Instance()->UpdateShaders();
                this->dataPtr->initialized = true;
                sceneMsgsCopy.erase(sIter++);
            }
            else
                ++sIter;
        }

        // Process the model messages.
        if(!modelMsgsCopy.empty())
        {
            for(modelIter = modelMsgsCopy.begin(); modelIter != modelMsgsCopy.end();)
            {
                if(this->ProcessModelMsg(**modelIter))
                {
                    const msgs::Model &modelMsg = **modelIter;
                    if(modelMsg.visual_size() > 0)
                    {
                        for(int k = 0; k < modelMsg.visual_size(); k++)
                        {
                            boost::shared_ptr<msgs::Visual> msgCopy;
                            msgCopy.reset(new msgs::Visual(modelMsg.visual(k)));
                            visualMsgsCopy.emplace_back(msgCopy);
                        }
                    }
                    modelMsgsCopy.erase(modelIter++);
                }
                else
                    ++modelIter;
            }
        }

        // Process the model visual messages.
        for(visualIter = modelVisualMsgsCopy.begin(); visualIter != modelVisualMsgsCopy.end();)
        {
            if(this->ProcessVisualMsg(*visualIter, Visual::VT_MODEL))
                modelVisualMsgsCopy.erase(visualIter++);
            else
                ++visualIter;
        }

        // Process the link visual messages.
        for(visualIter = linkVisualMsgsCopy.begin(); visualIter != linkVisualMsgsCopy.end();)
        {
            if(this->ProcessVisualMsg(*visualIter, Visual::VT_LINK))
                linkVisualMsgsCopy.erase(visualIter++);
            else
                ++visualIter;
        }

        // Process the visual messages.
        for(visualIter = visualMsgsCopy.begin(); visualIter != visualMsgsCopy.end();)
        {
            for(visualIter = visualMsgsCopy.begin(); visualIter != visualMsgsCopy.end();)
            {
                if(this->ProcessVisualMsg(*visualIter))
                    visualMsgsCopy.erase(visualIter++);
                else
                    ++visualIter;
            }
        }

        // Process the joint messages.
        for(jointIter = jointMsgsCopy.begin(); jointIter != jointMsgsCopy.end();)
        {
            if(this->ProcessJointMsg(*jointIter))
                jointMsgsCopy.erase(jointIter++);
            else
                ++jointIter;
        }

        // Process the link messages.
        for(linkIter = linkMsgsCopy.begin(); linkIter != linkMsgsCopy.end();)
        {
            if(this->ProcessLinkMsg(*linkIter))
                linkMsgsCopy.erase(linkIter++);
            else
                ++linkIter;
        }

        {
            std::lock_guard<std::mutex> lock(*this->dataPtr->receiveMutex);

            std::copy(sceneMsgsCopy.begin(), sceneMsgsCopy.end(), std::front_inserter(this->dataPtr->sceneMsgs));
            std::copy(modelMsgsCopy.begin(), modelMsgsCopy.end(), std::front_inserter(this->dataPtr->modelMsgs));
            std::copy(sensorMsgsCopy.begin(), sensorMsgsCopy.end(), std::front_inserter(this->dataPtr->sensorMsgs));
            std::copy(lightFactoryMsgsCopy.begin(), lightFactoryMsgsCopy.end(), std::front_inserter(this->dataPtr->lightFactoryMsgs));
            std::copy(lightModifyMsgsCopy.begin(), lightModifyMsgsCopy.end(), std::front_inserter(this->dataPtr->lightModifyMsgs));
            std::copy(modelVisualMsgsCopy.begin(), modelVisualMsgsCopy.end(), std::front_inserter(this->dataPtr->modelVisualMsgs));
            std::copy(linkVisualMsgsCopy.begin(), linkVisualMsgsCopy.end(), std::front_inserter(this->dataPtr->linkVisualMsgs));
            std::copy(visualMsgsCopy.begin(), visualMsgsCopy.end(), std::front_inserter(this->dataPtr->visualMsgs));
            std::copy(collisionVisualMsgsCopy.begin(), collisionVisualMsgsCopy.end(), std::front_inserter(this->dataPtr->collisionVisualMsgs));
            std::copy(jointMsgsCopy.begin(), jointMsgsCopy.end(), std::front_inserter(this->dataPtr->jointMsgs));
            std::copy(linkMsgsCopy.begin(), linkMsgsCopy.end(), std::front_inserter(this->dataPtr->linkMsgs));
        }

        {
            std::lock_guard<std::recursive_mutex> lock(this->dataPtr->poseMsgMutex);

            // Process all the model messages last. Remove pose message from the list
            // only when a corresponding visual exits. We may receive pose updates
            // over the wire before  we recieve the visual
            pIter = this->dataPtr->poseMsgs.begin();
            while(pIter != this->dataPtr->poseMsgs.end())
            {
                auto iter = this->dataPtr->visuals.find(pIter->first);
                if(iter != this->dataPtr->visuals.end() && iter->second)
                {
                    // If an object is selected, don't let the physics engine move it.
                    if(!this->dataPtr->selectedVis || this->dataPtr->selectionMode != "move" ||
                       (iter->first != this->dataPtr->selectedVis->GetId() && !this->dataPtr->selectedVis->IsAncestorOf(iter->second)))
                    {
                        ignition::math::Pose3d pose = msgs::ConvertIgn(pIter->second);
                        GZ_ASSERT(iter->second, "Visual pointer is nullptr");
                        iter->second->SetPose(pose);
                        auto prev = pIter++;
                        this->dataPtr->poseMsgs.erase(prev);
                    }
                    else
                        ++pIter;
                }
                else
                    ++pIter;
            }

            // process skeleton pose msgs
            spIter = this->dataPtr->skeletonPoseMsgs.begin();
            while(spIter != this->dataPtr->skeletonPoseMsgs.end())
            {
                auto iter = this->dataPtr->visuals.find((*spIter)->model_id());
                for(int i = 0; i < (*spIter)->pose_size(); ++i)
                {
                    const msgs::Pose &pose_msg = (*spIter)->pose(i);
                    if(pose_msg.has_id())
                    {
                        auto iter2 = this->dataPtr->visuals.find(pose_msg.id());
                        if(iter2 != this->dataPtr->visuals.end())
                        {
                            // If an object is selected, don't let the physics engine move it.
                            if(!this->dataPtr->selectedVis || this->dataPtr->selectionMode != "move" ||
                               (iter->first != this->dataPtr->selectedVis->GetId() && !this->dataPtr->selectedVis->IsAncestorOf(iter->second)))
                            {
                                ignition::math::Pose3d pose = msgs::ConvertIgn(pose_msg);
                                iter2->second->SetPose(pose);
                            }
                        }
                    }
                }

                if(iter != this->dataPtr->visuals.end())
                {
                    iter->second->SetSkeletonPose(**spIter);
                    auto prev = spIter++;
                    this->dataPtr->skeletonPoseMsgs.erase(prev);
                }
                else
                    ++spIter;
            }
        }
    }

  private:
    std::unique_ptr<GazeboState> dataPtr;
};
}
}

#endif // GUACAMOLE_TRANSPORT_PROCESSOR_H
