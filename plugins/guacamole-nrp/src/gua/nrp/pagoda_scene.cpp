#include <gua/nrp/pagoda_joint_visual.hpp>
#include <gua/nrp/pagoda_scene.hpp>
namespace gua
{
namespace nrp
{
PagodaScene::PagodaScene() : _mutex_receive(), _mutex_scenegraph(), _mutex_pose_msgs() {}
PagodaScene::~PagodaScene()
{
    _msgs_model.clear();
    _msgs_visual.clear();
    _msgs_pose.clear();
    _msgs_scene.clear();
    _msgs_joint.clear();
    _msgs_light_factory.clear();
    _msgs_light_modify.clear();
    _msgs_link.clear();

    _visuals.clear();
    _lights.clear();

    _world_visual.reset();
}
void PagodaScene::set_root_node(node::Node *root_node)
{
    _root_node = root_node;
    _mutex_scenegraph.lock();
    _world_visual.reset(new PagodaVisual("world_visual", _root_node));
    _mutex_scenegraph.unlock();
}
void PagodaScene::on_skeleton_pose_msg(ConstPoseAnimationPtr &msg)
{
    std::lock_guard<std::mutex> lock(_mutex_receive);
    skeleton_msgs_list::iterator iter;

    // Find an old model message, and remove them
    for(iter = _msgs_skeleton_pose.begin(); iter != _msgs_skeleton_pose.end(); ++iter)
    {
        if((*iter)->model_name() == msg->model_name())
        {
            _msgs_skeleton_pose.erase(iter);
            break;
        }
    }

    _msgs_skeleton_pose.push_back(msg);
}
void PagodaScene::on_model_msg(ConstModelPtr &msg)
{
    std::lock_guard<std::mutex> lock(_mutex_receive);
    _msgs_model.push_back(msg);
}
void PagodaScene::on_response_msg(ConstResponsePtr &msg)
{
    gazebo::msgs::Scene sceneMsg;
    sceneMsg.ParseFromString(msg->serialized_data());
    boost::shared_ptr<gazebo::msgs::Scene> sm(new gazebo::msgs::Scene(sceneMsg));

    std::lock_guard<std::mutex> lock(_mutex_receive);
    _msgs_scene.emplace_back(sm);
}
void PagodaScene::on_pose_msg(ConstPosesStampedPtr &msg)
{
    std::lock_guard<std::recursive_mutex> lock(_mutex_pose_msgs);

    for(int i = 0; i < msg->pose_size(); ++i)
    {
        auto iter = _msgs_pose.find(msg->pose(i).id());
        if(iter != _msgs_pose.end())
            iter->second.CopyFrom(msg->pose(i));
        else
            _msgs_pose.insert(std::make_pair(msg->pose(i).id(), msg->pose(i)));
    }
}
bool PagodaScene::process_visual_msg(ConstVisualPtr &msg, PagodaVisual::VisualType type)
{
    bool result = false;
    auto iter = _visuals.end();

    if(msg->has_id())
    {
        iter = _visuals.find(msg->id());
    }
    else
    {
        ptr_visual vis = this->get_visual(msg->name());
        iter = vis ? _visuals.find(vis->get_id()) : _visuals.end();
    }

    if(msg->has_delete_me() && msg->delete_me())
    {
        if(iter != _visuals.end())
        {
            _visuals.erase(iter);
            result = true;
        }
    }
    else if(iter != _visuals.end())
    {
        iter->second->update_from_msg(msg);
        result = true;
    }
    else
    {
        // std::cout << " a bit of a hack: Heightmap stuff" << std::endl;
        ptr_visual visual;

        // If the visual has a parent which is not the name of the scene...
        if(msg->has_parent_name() && msg->parent_name() != _name)
        {
            if(msg->has_id())
                iter = _visuals.find(msg->id());
            else
            {
                ptr_visual vis = this->get_visual(msg->name());
                iter = vis ? _visuals.find(vis->get_id()) : _visuals.end();
            }

            if(iter != _visuals.end())
                std::cerr << "Visual already exists. This shouldn't happen.\n";

            // Make sure the parent visual exists before trying to add a child
            // visual
            iter = _visuals.find(msg->parent_id());
            if(iter != _visuals.end())
            {
                visual.reset(new PagodaVisual(msg->name(), iter->second));
                if(msg->has_id())
                    visual->set_id(msg->id());
            }
        }
        else
        {
            // Add a visual that is attached to the scene root
            visual.reset(new PagodaVisual(msg->name(), _world_visual));
            if(msg->has_id())
                visual->set_id(msg->id());
        }

        if(visual)
        {
            result = true;
            visual->update_from_msg(msg);
            visual->set_type(type);

            _visuals[visual->get_id()] = visual;
        }
    }

    return result;
}

ptr_visual PagodaScene::get_visual(const uint32_t id) const
{
    auto iter = _visuals.find(id);
    if(iter != _visuals.end())
        return iter->second;
    return ptr_visual();
}

ptr_visual PagodaScene::get_visual(const std::string &name) const
{
    ptr_visual result;

    visuals_map::const_iterator iter;
    for(iter = _visuals.begin(); iter != _visuals.end(); ++iter)
    {
        if(iter->second == nullptr)
        {
            continue;
        }

        if(iter->second != nullptr && iter->second->get_name() == name)
        {
            break;
        }
    }

    if(iter != _visuals.end())
    {
        result = iter->second;
    }
    else
    {
        std::string otherName = _name + "::" + name;
        for(iter = _visuals.begin(); iter != _visuals.end(); ++iter)
        {
            if(iter->second != nullptr && iter->second->get_name() == otherName)
                break;
        }

        if(iter != _visuals.end())
            result = iter->second;
    }

    return result;
}

bool PagodaScene::process_link_msg(ConstLinkPtr &msg)
{
    ptr_visual linkVis;

    if(msg->has_id())
        linkVis = this->get_visual(msg->id());
    else
        linkVis = this->get_visual(msg->name());

    if(!linkVis)
    {
        gzerr << "No link visual with id[" << msg->id() << "] and name[" << msg->name() << "]\n";
        return false;
    }

    return true;
}
bool PagodaScene::process_joint_msg(ConstJointPtr &msg)
{
    // TODO

    //    ptr_visual child_vis;
    //
    //    if(msg->has_child() && msg->child() == "world")
    //        child_vis = this->_world_visual;
    //    else if(msg->has_child_id())
    //        child_vis = this->get_visual(msg->child_id());
    //
    //    if(!child_vis)
    //        return false;
    //
    //    ptr_joint_visual joint_vis(new PagodaJointVisual(msg->name() + "_JOINT_VISUAL__", child_vis));
    //    joint_vis->update_from_joint_msg(msg);
    //
    //    if(msg->has_id())
    //        joint_vis->set_id(msg->id());
    //
    //    _visuals[joint_vis->get_id()] = joint_vis;

    return true;
}
bool PagodaScene::process_light_factory_msg(ConstLightPtr &msg)
{
    light_map::iterator iter;
    iter = _lights.find(msg->name());

    if (iter == _lights.end())
    {
        ptr_light light(new PagodaLight(msg->name(), _root_node));
        light->load_from_msg(msg);
        _lights[msg->name()] = light;
    }
    else
    {
        std::cerr << "Light [" << msg->name() << "] already exists." << " Use topic ~/light/modify to modify it." << std::endl;
        return false;
    }

    return true;
}
bool PagodaScene::process_light_modify_msg(ConstLightPtr &msg)
{
    light_map::iterator iter;
    iter = _lights.find(msg->name());

    if (iter == _lights.end())
    {
        std::cerr  << "Light [" << msg->name() << "] not found." << " Use topic ~/factory/light to spawn a new light." << std::endl;
        return false;
    }
    else
    {
        iter->second->load_from_msg(msg);
    }

    return true;
}
bool PagodaScene::process_model_msg(const gazebo::msgs::Model &msg)
{
    std::string modelName, linkName;

    modelName = msg.name() + "::";
    for(int j = 0; j < msg.visual_size(); ++j)
    {
        boost::shared_ptr<gazebo::msgs::Visual> vm(new gazebo::msgs::Visual(msg.visual(j)));
        _msgs_model_visual.emplace_back(vm);
    }

    // Set the scale of the model visual
    if(msg.has_scale())
    {
        // update scale using a visual msg
        boost::shared_ptr<gazebo::msgs::Visual> vm(new gazebo::msgs::Visual);
        if(msg.has_id())
            vm->set_id(msg.id());
        if(msg.has_name())
            vm->set_name(msg.name());
        vm->mutable_scale()->set_x(msg.scale().x());
        vm->mutable_scale()->set_y(msg.scale().y());
        vm->mutable_scale()->set_z(msg.scale().z());
        _msgs_model_visual.emplace_back(vm);
    }

    for(int j = 0; j < msg.joint_size(); ++j)
    {
        boost::shared_ptr<gazebo::msgs::Joint> jm(new gazebo::msgs::Joint(msg.joint(j)));
        _msgs_joint.emplace_back(jm);
    }

    for(int j = 0; j < msg.link_size(); ++j)
    {
        linkName = modelName + msg.link(j).name();

        {
            std::lock_guard<std::recursive_mutex> lock(_mutex_pose_msgs);
            if(msg.link(j).has_pose())
            {
                auto iter = _msgs_pose.find(msg.link(j).id());
                if(iter != _msgs_pose.end())
                    iter->second.CopyFrom(msg.link(j).pose());
                else
                    _msgs_pose.insert(std::make_pair(msg.link(j).id(), msg.link(j).pose()));

                _msgs_pose[msg.link(j).id()].set_name(linkName);
                _msgs_pose[msg.link(j).id()].set_id(msg.link(j).id());
            }
        }

        if(msg.link(j).has_inertial())
        {
            boost::shared_ptr<gazebo::msgs::Link> lm(new gazebo::msgs::Link(msg.link(j)));
            _msgs_link.emplace_back(lm);
        }

        if(msg.link(j).visual_size() > 0)
        {
            // note: the first visual in the link is the link visual
            gazebo::msgs::VisualPtr vm(new gazebo::msgs::Visual(msg.link(j).visual(0)));
            _msgs_link_visual.emplace_back(vm);
        }

        for(int k = 1; k < msg.link(j).visual_size(); ++k)
        {
            boost::shared_ptr<gazebo::msgs::Visual> vm(new gazebo::msgs::Visual(msg.link(j).visual(k)));
            _msgs_visual.emplace_back(vm);
        }
    }

    for(int i = 0; i < msg.model_size(); ++i)
    {
        boost::shared_ptr<gazebo::msgs::Model> mm(new gazebo::msgs::Model(msg.model(i)));
        _msgs_model.emplace_back(mm);
    }

    return true;
}
bool PagodaScene::process_scene_msg(ConstScenePtr &msg)
{
    {
        std::lock_guard<std::recursive_mutex> lock(_mutex_pose_msgs);
        for(int i = 0; i < msg->model_size(); ++i)
        {
            auto iter = _msgs_pose.find(msg->model(i).id());
            if(iter != _msgs_pose.end())
                iter->second.CopyFrom(msg->model(i).pose());
            else
                _msgs_pose.insert(std::make_pair(msg->model(i).id(), msg->model(i).pose()));

            _msgs_pose[msg->model(i).id()].set_name(msg->model(i).name());
            _msgs_pose[msg->model(i).id()].set_id(msg->model(i).id());

            this->process_model_msg(msg->model(i));
        }
    }

    for(int i = 0; i < msg->light_size(); ++i)
    {
        boost::shared_ptr<gazebo::msgs::Light> lm(new gazebo::msgs::Light(msg->light(i)));
        _msgs_light_factory.emplace_back(lm);
    }

    for(int i = 0; i < msg->joint_size(); ++i)
    {
        boost::shared_ptr<gazebo::msgs::Joint> jm(new gazebo::msgs::Joint(msg->joint(i)));
        _msgs_joint.emplace_back(jm);
    }

    if(msg->has_ambient())
    {
        // ignore ambient color
    }

    if(msg->has_background())
    {
        // ignore background
    }

    if(msg->has_shadows())
    {
        // ignore shadows
    }

    if(msg->has_grid())
    {
        // ignore grid
    }

    if(msg->has_origin_visual())
    {
        // ignore origin
    }

    if(msg->has_sky())
    {
        // ignore sky
    }

    if(msg->has_fog())
    {
        // ignore fog
    }
    return true;
}
void PagodaScene::pre_render()
{
    _mutex_scenegraph.lock();

    scene_msgs_list scene_msgs_copy;
    model_msgs_list model_msgs_copy;
    light_msgs_list light_factory_msgs_copy;
    light_msgs_list light_modify_msgs_copy;
    visual_msgs_list model_visual_msgs_copy;
    visual_msgs_list link_visual_msgs_copy;
    visual_msgs_list visual_msgs_copy;
    joint_msgs_list joint_msgs_copy;
    link_msgs_list link_msgs_copy;

    {
        std::lock_guard<std::mutex> lock(_mutex_receive);

        std::copy(_msgs_scene.begin(), _msgs_scene.end(), std::back_inserter(scene_msgs_copy));
        _msgs_scene.clear();

        std::copy(_msgs_model.begin(), _msgs_model.end(), std::back_inserter(model_msgs_copy));
        _msgs_model.clear();

        std::copy(_msgs_light_factory.begin(), _msgs_light_factory.end(), std::back_inserter(light_factory_msgs_copy));
        _msgs_light_factory.clear();

        std::copy(_msgs_light_modify.begin(), _msgs_light_modify.end(), std::back_inserter(light_modify_msgs_copy));
        _msgs_light_modify.clear();

        std::copy(_msgs_model_visual.begin(), _msgs_model_visual.end(), std::back_inserter(model_visual_msgs_copy));
        _msgs_model_visual.clear();

        std::copy(_msgs_link_visual.begin(), _msgs_link_visual.end(), std::back_inserter(link_visual_msgs_copy));
        _msgs_link_visual.clear();

        _msgs_visual.sort(VisualMessageLessOp);
        std::copy(_msgs_visual.begin(), _msgs_visual.end(), std::back_inserter(visual_msgs_copy));
        _msgs_visual.clear();

        std::copy(_msgs_joint.begin(), _msgs_joint.end(), std::back_inserter(joint_msgs_copy));
        _msgs_joint.clear();

        std::copy(_msgs_link.begin(), _msgs_link.end(), std::back_inserter(link_msgs_copy));
        _msgs_link.clear();
    }

    for(auto scene_msgs_iter = scene_msgs_copy.begin(); scene_msgs_iter != scene_msgs_copy.end();)
    {
        if(this->process_scene_msg(*scene_msgs_iter))
        {
            scene_msgs_copy.erase(scene_msgs_iter++);
        }
        else
            ++scene_msgs_iter;
    }

    if(!model_msgs_copy.empty())
    {
        for(auto model_msgs_iter = model_msgs_copy.begin(); model_msgs_iter != model_msgs_copy.end();)
        {
            if(this->process_model_msg(**model_msgs_iter))
            {
                const gazebo::msgs::Model &modelMsg = **model_msgs_iter;
                if(modelMsg.visual_size() > 0)
                {
                    for(int k = 0; k < modelMsg.visual_size(); k++)
                    {
                        boost::shared_ptr<gazebo::msgs::Visual> msgCopy;
                        msgCopy.reset(new gazebo::msgs::Visual(modelMsg.visual(k)));
                        visual_msgs_copy.emplace_back(msgCopy);
                    }
                }
                model_msgs_copy.erase(model_msgs_iter++);
            }
            else
                ++model_msgs_iter;
        }
    }

    // Process the light factory messages.
    for (auto light_factory_iter = light_factory_msgs_copy.begin(); light_factory_iter != light_factory_msgs_copy.end();)
    {
        if (this->process_light_factory_msg(*light_factory_iter))
            light_factory_msgs_copy.erase(light_factory_iter++);
        else
            ++light_factory_iter;
    }

    // Process the light modify messages.
    for (auto light_modify_iter = light_modify_msgs_copy.begin(); light_modify_iter != light_modify_msgs_copy.end();)
    {
        if (this->process_light_modify_msg(*light_modify_iter))
            light_modify_msgs_copy.erase(light_modify_iter++);
        else
            ++light_modify_iter;
    }

    for(auto model_visual_msgs_iter = model_visual_msgs_copy.begin(); model_visual_msgs_iter != model_visual_msgs_copy.end();)
    {
        if(this->process_visual_msg(*model_visual_msgs_iter, PagodaVisual::VT_MODEL))
            model_visual_msgs_copy.erase(model_visual_msgs_iter++);
        else
            ++model_visual_msgs_iter;
    }

    for(auto link_visual_msgs_iter = link_visual_msgs_copy.begin(); link_visual_msgs_iter != link_visual_msgs_copy.end();)
    {
        if(this->process_visual_msg(*link_visual_msgs_iter, PagodaVisual::VT_LINK))
            link_visual_msgs_copy.erase(link_visual_msgs_iter++);
        else
            ++link_visual_msgs_iter;
    }

    for(auto visual_msgs_iter = visual_msgs_copy.begin(); visual_msgs_iter != visual_msgs_copy.end();)
    {
        for(visual_msgs_iter = visual_msgs_copy.begin(); visual_msgs_iter != visual_msgs_copy.end();)
        {
            if(this->process_visual_msg(*visual_msgs_iter))
                visual_msgs_copy.erase(visual_msgs_iter++);
            else
                ++visual_msgs_iter;
        }
    }

    for(auto joint_msgs_iter = joint_msgs_copy.begin(); joint_msgs_iter != joint_msgs_copy.end();)
    {
        if(this->process_joint_msg(*joint_msgs_iter))
            joint_msgs_copy.erase(joint_msgs_iter++);
        else
            ++joint_msgs_iter;
    }

    for(auto link_msgs_iter = link_msgs_copy.begin(); link_msgs_iter != link_msgs_copy.end();)
    {
        if(this->process_link_msg(*link_msgs_iter))
            link_msgs_copy.erase(link_msgs_iter++);
        else
            ++link_msgs_iter;
    }

    {
        std::lock_guard<std::mutex> lock(_mutex_receive);

        std::copy(scene_msgs_copy.begin(), scene_msgs_copy.end(), std::front_inserter(_msgs_scene));
        std::copy(model_msgs_copy.begin(), model_msgs_copy.end(), std::front_inserter(_msgs_model));
        std::copy(light_factory_msgs_copy.begin(), light_factory_msgs_copy.end(), std::front_inserter(_msgs_light_factory));
        std::copy(light_modify_msgs_copy.begin(), light_modify_msgs_copy.end(), std::front_inserter(_msgs_light_modify));
        std::copy(model_visual_msgs_copy.begin(), model_visual_msgs_copy.end(), std::front_inserter(_msgs_model_visual));
        std::copy(link_visual_msgs_copy.begin(), link_visual_msgs_copy.end(), std::front_inserter(_msgs_link_visual));
        std::copy(visual_msgs_copy.begin(), visual_msgs_copy.end(), std::front_inserter(_msgs_visual));
        std::copy(joint_msgs_copy.begin(), joint_msgs_copy.end(), std::front_inserter(_msgs_joint));
        std::copy(link_msgs_copy.begin(), link_msgs_copy.end(), std::front_inserter(_msgs_link));
    }

    {
        std::lock_guard<std::recursive_mutex> lock(_mutex_pose_msgs);

        // Process all the model messages last. Remove pose message from the list
        // only when a corresponding visual exits. We may receive pose updates
        // over the wire before  we recieve the visual
        auto pose_msgs_iter = _msgs_pose.begin();
        while(pose_msgs_iter != _msgs_pose.end())
        {
            auto iter = _visuals.find(pose_msgs_iter->first);
            if(iter != _visuals.end() && iter->second)
            {
                ignition::math::Pose3d pose = gazebo::msgs::ConvertIgn(pose_msgs_iter->second);
                iter->second->set_pose(pose);

                auto prev = pose_msgs_iter++;
                _msgs_pose.erase(prev);
            }
            else
                ++pose_msgs_iter;
        }

        auto skeleton_pose_iter = _msgs_skeleton_pose.begin();
        while(skeleton_pose_iter != _msgs_skeleton_pose.end())
        {
            auto iter = _visuals.find((*skeleton_pose_iter)->model_id());
            for(int i = 0; i < (*skeleton_pose_iter)->pose_size(); ++i)
            {
                const gazebo::msgs::Pose &pose_msg = (*skeleton_pose_iter)->pose(i);
                if(pose_msg.has_id())
                {
                    auto iter2 = _visuals.find(pose_msg.id());
                    if(iter2 != _visuals.end())
                    {
                        ignition::math::Pose3d pose = gazebo::msgs::ConvertIgn(pose_msg);
                        iter2->second->set_pose(pose);
                    }
                }
            }

            if(iter != _visuals.end())
            {
                // TODO: review the necessity of the method
                // iter->second->set_skeleton_pose(**skeleton_pose_iter);
                auto prev = skeleton_pose_iter++;
                _msgs_skeleton_pose.erase(prev);
            }
            else
                ++skeleton_pose_iter;
        }
    }

    _mutex_scenegraph.unlock();
}
std::mutex &PagodaScene::get_mutex_scenegraph() {
    return _mutex_scenegraph;
}
void PagodaScene::on_light_factory_msg(ConstLightPtr &msg) {
    std::lock_guard<std::mutex> lock(_mutex_receive);
    _msgs_light_factory.emplace_back(msg);
}
void PagodaScene::on_light_modify_msg(ConstLightPtr &msg) {
    std::lock_guard<std::mutex> lock(_mutex_receive);
    _msgs_light_modify.emplace_back(msg);
}
}
}