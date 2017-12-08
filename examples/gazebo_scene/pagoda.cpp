#include "common.h"
#include "loader.cpp"
#include "log.cpp"
#include "transport_processor.cpp"

typedef std::map<uint32_t, std::shared_ptr<gua::node::TransformNode>> map_id_node;
typedef std::pair<uint32_t, std::shared_ptr<gua::node::TransformNode>> pair_id_node;

class Pagoda
{
  public:
    Pagoda()
    {
        _log = new Log("transport");
        _nodemap = new map_id_node();
        _tproc = boost::shared_ptr<gazebo::rendering::TransportProcessor>(new gazebo::rendering::TransportProcessor());
    }

    ~Pagoda()
    {
        halt_transport_layer();
        _worker.join();
        _tproc.reset();
    }

    void bind_scene_graph(gua::SceneGraph *sceneGraph)
    {
        _scene_graph = sceneGraph;
        _nodemap->clear();
    }

    void bind_transport_layer(int argc, char **argv)
    {
        _worker = std::thread([&] { this->_connect_to_transport_layer(argc, argv); });
    }

    const boost::shared_ptr<gazebo::rendering::TransportProcessor> &get_tproc() const { return _tproc; }

    void halt_transport_layer() { _worker_cv.notify_all(); }
    void lock_scenegraph() { _scenegraph_mutex.lock(); }
    void unlock_scenegraph() { _scenegraph_mutex.unlock(); }

  private:
    Loader _tml;
    gua::MaterialLoader _material_loader;
    Log *_log;

    map_id_node *_nodemap;

    std::mutex _scenegraph_mutex;
    gua::SceneGraph *_scene_graph = nullptr;

    std::thread _worker;
    std::mutex _worker_mutex;
    std::condition_variable _worker_cv;

    boost::shared_ptr<gazebo::rendering::TransportProcessor> _tproc;

    void _connect_to_transport_layer(int argc, char **argv)
    {
        Log log("worker");

        log.d("Gazebo client: setup");

        gazebo::client::setup(argc, argv);

        log.d("Gazebo client: init transport node");

        gazebo::transport::NodePtr node(new gazebo::transport::Node());
        node->Init();

        log.d("Gazebo client: subscription");

        gazebo::transport::SubscriberPtr sub_world = node->Subscribe("~/world_stats", &Pagoda::callback_world, this);
        gazebo::transport::SubscriberPtr sub_model = node->Subscribe("~/model/info", &Pagoda::callback_model_info, this);
        gazebo::transport::SubscriberPtr sub_pose_info = node->Subscribe("~/pose/info", &Pagoda::callback_pose_info, this);
        gazebo::transport::SubscriberPtr sub_material = node->Subscribe("~/material", &Pagoda::callback_material, this);

        gazebo::transport::PublisherPtr pub_request = node->Advertise<gazebo::msgs::Request>("~/request");

        gazebo::transport::SubscriberPtr sub_request = node->Subscribe("~/request", &Pagoda::callback_request, this);
        gazebo::transport::SubscriberPtr sub_response = node->Subscribe("~/response", &Pagoda::callback_response, this);
        gazebo::transport::SubscriberPtr sub_scene = node->Subscribe("~/scene", &Pagoda::callback_scene, this);
        gazebo::transport::SubscriberPtr sub_skeleton_pose_info = node->Subscribe("~/skeleton_pose/info", &Pagoda::callback_skeleton_pose_info, this);

        if(!pub_request->WaitForConnection(gazebo::common::Time(5, 0)))
        {
            log.e("Gazebo client: connection not established");
        }
        else
        {
            pub_request->Publish(*(gazebo::msgs::CreateRequest("scene_info")), false);
            log.d("Gazebo client: connection established");

            std::unique_lock<std::mutex> lk(this->_worker_mutex);
            this->_worker_cv.wait(lk);
        }

        pub_request->Fini();
        node->Fini();

        gazebo::client::shutdown();

        log.d("Gazebo client: over");
    }

    void callback_world(ConstWorldStatisticsPtr &_msg)
    {
        _log->d("callback_world");
        _log->d(_msg->DebugString().c_str());

        // TODO
    }

    void callback_material(ConstMaterialPtr &_msg)
    {
        _log->d("callback_material");
        _log->d(_msg->DebugString().c_str());

        // TODO
    }

    void callback_skeleton_pose_info(ConstPoseAnimationPtr &ptr)
    {
        _log->d("callback_skeleton_pose_info");
        _log->d(ptr->DebugString().c_str());

        _tproc->OnSkeletonPoseMsg(ptr);
    }

    // "~/model/info"
    void callback_model_info(ConstModelPtr &ptr)
    {
        _log->d("callback_model_info");
        _log->d(ptr->DebugString().c_str());

        _tproc->OnModelMsg(ptr);

        // _log->d("Add transform node");

        _scenegraph_mutex.lock();

        auto transform = _scene_graph->add_node<gua::node::TransformNode>("/transform", ptr->name());

        try
        {
            _nodemap->insert(pair_id_node(ptr->id(), transform));
        }
        catch(std::exception &e)
        {
            _log->e("Exception");
            _log->e(e.what());
        }

        double t_x = ptr->pose().position().x();
        double t_y = ptr->pose().position().y();
        double t_z = ptr->pose().position().z();

        double o_x = ptr->pose().orientation().x();
        double o_y = ptr->pose().orientation().y();
        double o_z = ptr->pose().orientation().z();
        double o_w = ptr->pose().orientation().w();

        double s_x = ptr->scale().x();
        double s_y = ptr->scale().y();
        double s_z = ptr->scale().z();

        gua::math::mat4 transform_mat = scm::math::make_translation(t_x, t_y, t_z) * scm::math::quatd(o_w, o_x, o_y, o_z).to_matrix() * scm::math::make_scale(s_x, s_y, s_z);

        for(const auto &it : ptr->visual())
        {
            if(it.type() == gazebo::msgs::Visual_Type::Visual_Type_MODEL)
            {
                // _log->d("visual: MODEL");

                // TODO: save geometry (?) position and orientation
                t_x = it.pose().position().x();
                t_y = it.pose().position().y();
                t_z = it.pose().position().z();

                o_x = it.pose().orientation().x();
                o_y = it.pose().orientation().y();
                o_z = it.pose().orientation().z();
                o_w = it.pose().orientation().w();

                transform_mat = transform_mat * scm::math::make_translation(t_x, t_y, t_z) * scm::math::quatd(o_w, o_x, o_y, o_z).to_matrix();

                break;
            }
        }

        transform->set_transform(transform_mat);

        // _log->d("Set transform matrix");

        for(const auto &it : ptr->link())
        {
            //_log->d("link");

            for(const auto &v_it : it.visual())
            {
                //_log->d("visual: VISUAL");

                if(v_it.type() == gazebo::msgs::Visual_Type::Visual_Type_VISUAL)
                {
                    std::shared_ptr<gua::node::Node> geometry_node;

                    switch(v_it.geometry().type())
                    {
                    case gazebo::msgs::Geometry_Type::Geometry_Type_BOX:
                    {
                        // TODO: use gazebo material

                        s_x = v_it.geometry().box().size().x();
                        s_y = v_it.geometry().box().size().y();
                        s_z = v_it.geometry().box().size().z();

                        std::shared_ptr<gua::Material> material_ptr = _material_loader.load_material("data/materials/box.mtl", "data/materials/box.mtl", false);

                        geometry_node = _tml.create_geometry_from_file("box", "data/objects/box.obj", material_ptr,
                                                                       gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE | gua::TriMeshLoader::LOAD_MATERIALS);

                        geometry_node->scale(s_x, s_y, s_z);

                        _scene_graph->add_node("/transform/" + ptr->name(), geometry_node);
                        geometry_node->set_draw_bounding_box(true);

                        break;
                    }
                    case gazebo::msgs::Geometry_Type::Geometry_Type_CYLINDER:
                    {
                        // TODO: use gazebo material

                        s_x = v_it.geometry().cylinder().length();
                        s_y = v_it.geometry().cylinder().radius();

                        geometry_node = _tml.create_geometry_from_file("cylinder", "data/objects/cylinder.obj",
                                                                       gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE | gua::TriMeshLoader::LOAD_MATERIALS);

                        geometry_node->scale(s_x, s_y, 1);

                        _scene_graph->add_node("/transform/" + ptr->name(), geometry_node);
                        geometry_node->set_draw_bounding_box(true);

                        break;
                    }
                    case gazebo::msgs::Geometry_Type::Geometry_Type_SPHERE:
                    {
                        // TODO: use gazebo material

                        s_x = v_it.geometry().sphere().radius();

                        geometry_node = _tml.create_geometry_from_file("sphere", "data/objects/sphere.obj",
                                                                       gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE | gua::TriMeshLoader::LOAD_MATERIALS);

                        geometry_node->scale(s_x, 1, 1);

                        _scene_graph->add_node("/transform/" + ptr->name(), geometry_node);
                        geometry_node->set_draw_bounding_box(true);

                        break;
                    }
                    case gazebo::msgs::Geometry_Type::Geometry_Type_PLANE:
                    {
                        // TODO: use gazebo material

                        s_x = v_it.geometry().plane().size().x();
                        s_y = v_it.geometry().plane().size().y();

                        o_x = v_it.geometry().plane().normal().x();
                        o_y = v_it.geometry().plane().normal().y();
                        o_z = v_it.geometry().plane().normal().z();

                        geometry_node = _tml.create_geometry_from_file("box", "data/objects/box.obj",
                                                                       gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE | gua::TriMeshLoader::LOAD_MATERIALS);

                        scm::math::vec3d normal = scm::math::vec3d(o_x, o_y, o_z);
                        scm::math::vec3d tangent_0 = scm::math::cross(scm::math::vec3d(o_x, o_y, o_z), scm::math::vec3d(1, 0, 0));
                        if(scm::math::dot(tangent_0, tangent_0) < 0.001)
                            tangent_0 = scm::math::cross(normal, scm::math::vec3d(0, 1, 0));
                        tangent_0 = scm::math::normalize(tangent_0);
                        scm::math::vec3d tangent_1 = scm::math::normalize(scm::math::cross(scm::math::vec3d(o_x, o_y, o_z), tangent_0));

                        scm::math::mat4d _transform = scm::math::mat4d::identity();

                        _transform.column(0) = scm::math::vec4d(tangent_0);
                        _transform.column(1) = scm::math::vec4d(tangent_1);
                        _transform.column(2) = scm::math::vec4d(normal);
                        _transform.column(3) = scm::math::vec4d(0, 0, 0, 1);

                        geometry_node->set_transform(_transform);
                        geometry_node->scale(s_x, s_y, 0.01);

                        _scene_graph->add_node("/transform/" + ptr->name(), geometry_node);
                        geometry_node->set_draw_bounding_box(true);

                        break;
                    }
                    case gazebo::msgs::Geometry_Type::Geometry_Type_IMAGE:
                    {
                        // TODO
                        break;
                    }
                    case gazebo::msgs::Geometry_Type::Geometry_Type_HEIGHTMAP:
                    {
                        // TODO
                        break;
                    }
                    case gazebo::msgs::Geometry_Type::Geometry_Type_MESH:
                        if(v_it.geometry().type() == gazebo::msgs::Geometry_Type::Geometry_Type_MESH)
                        {
                            s_x = v_it.geometry().mesh().scale().x();
                            s_y = v_it.geometry().mesh().scale().y();
                            s_z = v_it.geometry().mesh().scale().z();

                            std::string local_filename = v_it.geometry().mesh().filename();
                            // _log->d(local_filename.c_str());

                            // TODO: remove hack
                            if(local_filename == "model://virtual_room/meshes/room.dae")
                            {
                                break;
                            }

                            local_filename.replace(0, 8, "/home/xaf/.gazebo/models/");

                            geometry_node = _tml.create_geometry_from_file(v_it.geometry().mesh().filename(), local_filename,
                                                                           gua::TriMeshLoader::LOAD_MATERIALS); // gua::TriMeshLoader::NORMALIZE_SCALE
                            // gua::TriMeshLoader::NORMALIZE_POSITION

                            geometry_node->scale(s_x, s_y, s_z);

                            _scene_graph->add_node("/transform/" + ptr->name(), geometry_node);
                            geometry_node->set_draw_bounding_box(true);

                            // _log->d(("Add geometry node: " +
                            // v_it->geometry().mesh().filename() + " to " + "/transform/"
                            //    + ptr->name()).c_str());
                            break;
                        }
                    case gazebo::msgs::Geometry_Type::Geometry_Type_TRIANGLE_FAN:
                    {
                        // TODO
                        break;
                    }
                    case gazebo::msgs::Geometry_Type::Geometry_Type_LINE_STRIP:
                    {
                        // TODO
                        break;
                    }
                    case gazebo::msgs::Geometry_Type::Geometry_Type_POLYLINE:
                    {
                        // TODO
                        break;
                    }
                    case gazebo::msgs::Geometry_Type::Geometry_Type_EMPTY:
                    {
                        // TODO
                        break;
                    }
                    }
                }
            }
        }

        // _log->d("Scene graph:");

        // _log->d(print_scenegraph(_scene_graph->get_root(), 10).c_str());

        _scenegraph_mutex.unlock();
    }

    // "~/pose/info"
    void callback_pose_info(ConstPosesStampedPtr &ptr)
    {
        _log->d("callback_pose_info");
        _log->d(ptr->DebugString().c_str());

        _tproc->OnPoseMsg(ptr);

        for(const auto &it : ptr->pose())
        {
            map_id_node::const_iterator pos = _nodemap->find(it.id());
            if(pos != _nodemap->end())
            {
                _log->d("node found");

                double t_x = it.position().x();
                double t_y = it.position().y();
                double t_z = it.position().z();

                double o_x = it.orientation().x();
                double o_y = it.orientation().y();
                double o_z = it.orientation().z();
                double o_w = it.orientation().w();

                gua::math::mat4 transform_mat = scm::math::make_translation(t_x, t_y, t_z) * scm::math::quatd(o_w, o_x, o_y, o_z).to_matrix();

                pos->second->set_transform(transform_mat);
            }
            else
            {
                _log->d("map miss");
            }
        }

        _log->d(std::to_string(_nodemap->size()).c_str());
    }

    void callback_request(ConstRequestPtr &ptr)
    {
        _log->d("callback_request");
        _log->d(ptr->DebugString().c_str());
    }

    void callback_response(ConstResponsePtr &ptr)
    {
        _log->d("callback_response");
        _log->d(ptr->DebugString().c_str());

        gazebo::msgs::Scene sceneMsg;
        sceneMsg.ParseFromString(ptr->serialized_data());

        _log->d(sceneMsg.DebugString().c_str());

        _tproc->OnResponse(ptr);
    }

    void callback_scene(ConstScenePtr &ptr)
    {
        _log->d("callback_scene");
        _log->d(ptr->DebugString().c_str());

        _tproc->ProcessSceneMsg(ptr);
    }
};