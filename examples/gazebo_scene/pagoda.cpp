/******************************************************************************
 * guacamole - delicious VR                                                   *
 *                                                                            *
 * Copyright: (c) 2011-2013 Bauhaus-Universität Weimar                        *
 * Contact:   felix.lauer@uni-weimar.de / simon.schneegans@uni-weimar.de      *
 *                                                                            *
 * This program is free software: you can redistribute it and/or modify it    *
 * under the terms of the GNU General Public License as published by the Free *
 * Software Foundation, either version 3 of the License, or (at your option)  *
 * any later version.                                                         *
 *                                                                            *
 * This program is distributed in the hope that it will be useful, but        *
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY *
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License   *
 * for more details.                                                          *
 *                                                                            *
 * You should have received a copy of the GNU General Public License along    *
 * with this program. If not, see <http://www.gnu.org/licenses/>.             *
 *                                                                            *
 ******************************************************************************/
#include <gazebo/transport/transport.hh>
#include <gazebo/gazebo_client.hh>

#include <gua/guacamole.hpp>
#include <gua/renderer/DebugViewPass.hpp>
#include <gua/utils/Trackball.hpp>

#include <unordered_set>

typedef std::map<uint32_t, std::shared_ptr<gua::node::TransformNode>> map_id_node;
typedef std::pair<uint32_t, std::shared_ptr<gua::node::TransformNode>> pair_id_node;

class Pagoda {
public:

  std::string print_scenegraph(std::shared_ptr<gua::node::Node> const &subtree_root, int tree_depth = 0) {
      std::stringstream out;

      out << "\n";
      for (int tab_print_index = 0; tab_print_index < tree_depth; ++tab_print_index)
      {
          out << "  ";
      }

      out << subtree_root->get_name();

      auto const &all_children_nodes = subtree_root->get_children();

      for (auto const &subtree_child : all_children_nodes)
      {
          print_scenegraph(subtree_child, tree_depth + 1);
      }

      if (!tree_depth)
      {
          out << "\n\n";
      }

      return out.str();
  }

  class Log {

  public:
    enum LOG_LEVEL {
      INFO = 3,
      DEBUG = 2,
      WARNING = 1,
      ERROR = 0
    };

    explicit Log(const char *log_file) {
        _ofstream = std::ofstream();
        _ofstream.open(log_file, std::ios::out | std::ios::app);

        assert(_ofstream.is_open());
    }

    explicit Log(const char *log_file, LOG_LEVEL log_level) {
        _ofstream = std::ofstream();
        _ofstream.open(log_file, std::ios::out | std::ios::app);

        _log_level = log_level;

        assert(_ofstream.is_open());
    }

    ~Log() {
        _ofstream.close();
    }

    bool i(const char *msg) {
        return (_log_level >= LOG_LEVEL::INFO) && log(msg);
    }

    bool d(const char *msg) {
        return (_log_level >= LOG_LEVEL::DEBUG) && log(msg);
    }

    bool w(const char *msg) {
        return (_log_level >= LOG_LEVEL::WARNING) && log(msg);
    }

    bool e(const char *msg) {
        return (_log_level >= LOG_LEVEL::ERROR) && log(msg);
    }

  private:
    std::ofstream _ofstream;
    LOG_LEVEL _log_level = DEBUG;

    bool log(const char *msg) {
        if (!_ofstream.is_open())
            return false;

        _ofstream << std::endl;

        std::time_t timestamp = std::time(nullptr);
        _ofstream << std::asctime(std::localtime(&timestamp));
        _ofstream << std::endl;

        _ofstream << msg;

        _ofstream << std::endl;

        return !_ofstream.bad();
    }

  };

  Pagoda() {
      _log = new Log("log.txt");
      _nodemap = new std::map<uint32_t, std::shared_ptr<gua::node::TransformNode>>();
  }

  Pagoda(const char *log_file, Log::LOG_LEVEL log_level) {
      _log = new Log(log_file, log_level);
      _nodemap = new std::map<uint32_t, std::shared_ptr<gua::node::TransformNode>>();
  }

  void bind_scene_graph(gua::SceneGraph *sceneGraph) {
      _scene_graph = sceneGraph;
      _nodemap->clear();
  }

  void bind_transport_layer(int argc, char **argv) {
      std::thread gazebo_client_worker(
          [](Pagoda *pagoda, int _argc, char **_argv)
          {
            pagoda->_log->d("Gazebo client: setup");

            gazebo::client::setup(_argc, _argv);

            pagoda->_log->d("Gazebo client: init transport node");

            gazebo::transport::NodePtr node(new gazebo::transport::Node());
            node->Init();

            pagoda->_log->d("Gazebo client: subscription");

            // uncomment for topic namespaces
            //      gazebo::transport::TopicManager *tm = gazebo::transport::TopicManager::Instance();
            //      std::list<std::string> _namespaces;
            //      tm->GetTopicNamespaces(_namespaces);
            //      pagoda->_log->d("Available namespaces:");
            //      for (std::string _namespace : _namespaces) {
            //        pagoda->_log->d(_namespace.c_str());
            //      }

            gazebo::transport::SubscriberPtr sub_world = node->Subscribe("~/world_stats", &Pagoda::callback_world, pagoda);
            gazebo::transport::SubscriberPtr sub_model = node->Subscribe("~/model/info", &Pagoda::callback_model_info, pagoda);
            gazebo::transport::SubscriberPtr sub_pose_info = node->Subscribe("~/pose/info", &Pagoda::callback_pose_info, pagoda);

            gazebo::transport::PublisherPtr pub_request = node->Advertise<gazebo::msgs::Request>("~/request");

            gazebo::transport::SubscriberPtr sub_request = node->Subscribe("~/request", &Pagoda::callback_request, pagoda);
            gazebo::transport::SubscriberPtr sub_response = node->Subscribe("~/response", &Pagoda::callback_response, pagoda);
            gazebo::transport::SubscriberPtr sub_scene = node->Subscribe("~/scene", &Pagoda::callback_scene, pagoda);

            // gazebo::transport::SubscriberPtr sub_skeleton_pose_info = node->Subscribe("~/skeleton_pose/info", &Pagoda::callback_skeleton_pose_info, pagoda);
            // gazebo::transport::SubscriberPtr sub_material = node->Subscribe("~/material", &Pagoda::callback_material, pagoda);
            // gazebo::transport::SubscriberPtr sub_physics = node->Subscribe("~/physics", &Pagoda::callback_physics, pagoda);

            pub_request->WaitForConnection();
            pub_request->Publish(*(gazebo::msgs::CreateRequest("scene_info")), false);

            pagoda->_log->d("Gazebo client: ready");

            while (!pagoda->_should_gazebo_halt)
                gazebo::common::Time::MSleep(10);

            gazebo::client::shutdown();
          }, this, argc, argv);

      gazebo_client_worker.detach();
  }

  void halt_transport_layer() {
      _should_gazebo_halt = true;
  }

  void lock() {
      _mutex.lock();
  }

  void unlock() {
      _mutex.unlock();
  }

private:
  Log *_log = nullptr;
  gua::SceneGraph *_scene_graph = nullptr;
  map_id_node *_nodemap;
  bool _should_gazebo_halt = false;
  gua::TriMeshLoader _tml;
  std::mutex _mutex;

  void callback_world(ConstWorldStatisticsPtr &_msg) {
      _log->i("callback_world");
  }

//  void callback_material(ConstMaterialPtr &_msg) {
//    _log->i("callback_material");
//  }
//
//  void callback_physics(ConstPhysicsPtr &_msg) {
//    _log->i("callback_physics");
//  }
//
//  void callback_skeleton_pose_info(ConstPoseAnimationPtr &ptr) {
//      _log->d("callback_skeleton_pose_info");
//      _log->d(ptr->DebugString().c_str());
//  }

  // "~/model/info"
  void callback_model_info(ConstModelPtr &ptr) {
      _mutex.lock();

      _log->d("callback_model_info");
      _log->d(ptr->DebugString().c_str());

      // _log->d("Add transform node");

      auto transform = _scene_graph->add_node<gua::node::TransformNode>("/transform", ptr->name());

      try
      {
          _nodemap->insert(pair_id_node(ptr->id(), transform));
      }
      catch (std::exception &e)
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

      for (const auto &it : ptr->visual())
      {
          if (it.type() == gazebo::msgs::Visual_Type::Visual_Type_MODEL)
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

      for (const auto &it : ptr->link())
      {
          //_log->d("link");

          for (const auto &v_it : it.visual())
          {
              //_log->d("visual: VISUAL");

              if (v_it.type() == gazebo::msgs::Visual_Type::Visual_Type_VISUAL)
              {
                  std::shared_ptr<gua::node::Node> geometry_node;

                  switch (v_it.geometry().type())
                  {
                  case gazebo::msgs::Geometry_Type::Geometry_Type_BOX:
                  {

                      // TODO: use gazebo material

                      s_x = v_it.geometry().box().size().x();
                      s_y = v_it.geometry().box().size().y();
                      s_z = v_it.geometry().box().size().z();

                      geometry_node = _tml.create_geometry_from_file(
                          "box", "data/objects/box.obj",
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

                      geometry_node = _tml.create_geometry_from_file(
                          "cylinder", "data/objects/cylinder.obj",
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

                      geometry_node = _tml.create_geometry_from_file(
                          "sphere", "data/objects/sphere.obj",
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

                      geometry_node = _tml.create_geometry_from_file(
                          "box", "data/objects/box.obj",
                          gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE | gua::TriMeshLoader::LOAD_MATERIALS);

                      scm::math::vec3d normal = scm::math::vec3d(o_x, o_y, o_z);
                      scm::math::vec3d tangent_0 = scm::math::cross(scm::math::vec3d(o_x, o_y, o_z), scm::math::vec3d(1, 0, 0));
                      if (scm::math::dot(tangent_0, tangent_0) < 0.001)
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
                      if (v_it.geometry().type() == gazebo::msgs::Geometry_Type::Geometry_Type_MESH)
                      {
                          s_x = v_it.geometry().mesh().scale().x();
                          s_y = v_it.geometry().mesh().scale().y();
                          s_z = v_it.geometry().mesh().scale().z();

                          std::string local_filename = v_it.geometry().mesh().filename();
                          // _log->d(local_filename.c_str());

                          // TODO: remove hack
                          if (local_filename == "model://virtual_room/meshes/room.dae")
                          {
                              break;
                          }

                          local_filename.replace(0, 8, "/home/xaf/.gazebo/models/");

                          geometry_node = _tml.create_geometry_from_file(
                              v_it.geometry().mesh().filename(), local_filename,
                              gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE | gua::TriMeshLoader::LOAD_MATERIALS);

                          geometry_node->scale(s_x, s_y, s_z);

                          _scene_graph->add_node("/transform/" + ptr->name(), geometry_node);
                          geometry_node->set_draw_bounding_box(true);

                          // _log->d(("Add geometry node: " + v_it->geometry().mesh().filename() + " to " + "/transform/"
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

      _mutex.unlock();
  }

  // "~/pose/info"
  void callback_pose_info(ConstPosesStampedPtr &ptr) {
      _log->d("callback_pose_info");
      _log->d(ptr->DebugString().c_str());

      for (const auto &it : ptr->pose())
      {
          map_id_node::const_iterator pos = _nodemap->find(it.id());
          if (pos != _nodemap->end())
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

  void callback_request(ConstRequestPtr &ptr) {
      _log->d("callback_request");
      _log->d(ptr->DebugString().c_str());
  }

  void callback_response(ConstResponsePtr &ptr) {
      _log->d("callback_response");
      _log->d(ptr->DebugString().c_str());

      gazebo::msgs::Scene sceneMsg;
      sceneMsg.ParseFromString(ptr->serialized_data());

      _log->d(sceneMsg.DebugString().c_str());
  }

  void callback_scene(ConstScenePtr &ptr) {
      _log->d("callback_scene");
      _log->d(ptr->DebugString().c_str());
  }
};