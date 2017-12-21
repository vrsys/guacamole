#ifndef GUACAMOLE_PAGODA_SCENE_H
#define GUACAMOLE_PAGODA_SCENE_H

#include <functional>
#include <list>
#include <memory>
#include <string>
#include <unordered_set>

#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <boost/enable_shared_from_this.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/unordered/unordered_map.hpp>

#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>

#include <gazebo/msgs/msgs.hh>

#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Exception.hh>

#include <gazebo/transport/Node.hh>
#include <gazebo/transport/TransportIface.hh>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/TransportTypes.hh>

#include <gua/renderer/TriMeshLoader.hpp>
#include <gua/scenegraph/SceneGraph.hpp>

#include "pagoda_visual.hpp"

typedef std::map<uint32_t, ptr_visual> visuals_map;

typedef std::map<uint32_t, gazebo::msgs::Pose> pose_msgs_map;
typedef std::list<boost::shared_ptr<gazebo::msgs::Visual const>> visual_msgs_list;
typedef std::list<boost::shared_ptr<gazebo::msgs::Scene const>> scene_msgs_list;
typedef std::list<boost::shared_ptr<gazebo::msgs::Joint const>> joint_msgs_list;
typedef std::list<boost::shared_ptr<gazebo::msgs::Link const>> link_msgs_list;
typedef std::list<boost::shared_ptr<gazebo::msgs::Model const>> model_msgs_list;
typedef std::list<boost::shared_ptr<gazebo::msgs::PoseAnimation const>> skeleton_msgs_list;

class PagodaScene
{
  public:
    struct VisualMessageLess
    {
        bool operator()(const boost::shared_ptr<gazebo::msgs::Visual const> &_i, const boost::shared_ptr<gazebo::msgs::Visual const> &_j) { return _i->name().size() < _j->name().size(); }
    } VisualMessageLessOp;

    PagodaScene();
    ~PagodaScene();

    void set_scene_graph(gua::SceneGraph *scene_graph);

    const ptr_visual &get_world_visual() const;

    void on_skeleton_pose_msg(ConstPoseAnimationPtr &_msg);
    void on_model_msg(ConstModelPtr &_msg);
    void on_response_msg(ConstResponsePtr &_msg);
    void on_pose_msg(ConstPosesStampedPtr &_msg);

    bool process_visual_msg(ConstVisualPtr &msg, PagodaVisual::VisualType type = PagodaVisual::VT_ENTITY);
    bool process_link_msg(ConstLinkPtr &_msg);
    bool process_joint_msg(ConstJointPtr &msg);
    bool process_model_msg(const gazebo::msgs::Model &msg);
    bool process_scene_msg(ConstScenePtr &msg);

    void pre_render();

  private:
    visual_msgs_list _msgs_model_visual;
    visual_msgs_list _msgs_link_visual;
    visual_msgs_list _msgs_visual;
    pose_msgs_map _msgs_pose;
    scene_msgs_list _msgs_scene;
    joint_msgs_list _msgs_joint;
    link_msgs_list _msgs_link;
    model_msgs_list _msgs_model;
    skeleton_msgs_list _msgs_skeleton_pose;

    visuals_map _visuals;

    std::mutex _mutex_receive;
    std::recursive_mutex _mutex_pose_msgs;

    std::mutex _mutex_scenegraph;
    gua::SceneGraph *_scene_graph;

    std::string _name;
    ptr_visual _world_visual;

    ptr_visual get_visual(const uint32_t id) const;
    ptr_visual get_visual(const std::string &name) const;
};

#endif // GUACAMOLE_PAGODA_SCENE_H
