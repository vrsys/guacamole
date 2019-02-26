#ifndef GUACAMOLE_NRP_SCENE_H
#define GUACAMOLE_NRP_SCENE_H

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

#include <gua/nrp/nrp_light.hpp>
#include <gua/nrp/nrp_visual.hpp>
#include <gua/nrp/platform.hpp>

namespace gua
{
namespace nrp
{
class NRPNode;
class NRPInteractiveNode;
class NRPCameraNode;

typedef std::map<uint32_t, ptr_visual> visuals_map;
typedef std::map<std::string, ptr_light> light_map;

typedef std::map<uint32_t, gazebo::msgs::Pose> pose_msgs_map;
typedef std::list<boost::shared_ptr<gazebo::msgs::Visual const>> visual_msgs_list;
typedef std::list<boost::shared_ptr<gazebo::msgs::Scene const>> scene_msgs_list;
typedef std::list<boost::shared_ptr<gazebo::msgs::Light const>> light_msgs_list;
typedef std::list<boost::shared_ptr<gazebo::msgs::Link const>> link_msgs_list;
typedef std::list<boost::shared_ptr<gazebo::msgs::Model const>> model_msgs_list;
typedef std::list<boost::shared_ptr<gazebo::msgs::PoseAnimation const>> skeleton_msgs_list;

class GUA_NRP_DLL NRPScene
{
  public:
    struct VisualMessageLess
    {
        bool operator()(const boost::shared_ptr<gazebo::msgs::Visual const> &_i, const boost::shared_ptr<gazebo::msgs::Visual const> &_j) { return _i->name().size() < _j->name().size(); }
    } VisualMessageLessOp;

    NRPScene();
    ~NRPScene();

    void set_root_node(NRPNode *root_node);
    void set_interactive_node(NRPInteractiveNode *interactive_node);
    void set_cam_node(NRPCameraNode *cam_node);

    NRPInteractiveNode *get_interactive_node() const;
    NRPNode *get_root_node() const;

    void on_skeleton_pose_msg(ConstPoseAnimationPtr &msg);
    void on_model_msg(ConstModelPtr &msg);
    void on_scene_msg(ConstScenePtr &msg);
    void on_pose_msg(ConstPosesStampedPtr &msg);
    void on_light_factory_msg(ConstLightPtr &msg);
    void on_light_modify_msg(ConstLightPtr &msg);

    bool process_visual_msg(ConstVisualPtr &msg, NRPVisual::VisualType type = NRPVisual::VT_ENTITY);
    bool process_link_msg(ConstLinkPtr &msg);
    bool process_light_factory_msg(ConstLightPtr &msg);
    bool process_light_modify_msg(ConstLightPtr &msg);
    bool process_model_msg(const gazebo::msgs::Model &msg);
    bool process_scene_msg(ConstScenePtr &msg);

    std::mutex &get_mutex_scenegraph();

    void pre_render();

  private:
    visual_msgs_list _msgs_model_visual;
    visual_msgs_list _msgs_link_visual;
    visual_msgs_list _msgs_visual;
    pose_msgs_map _msgs_pose;
    scene_msgs_list _msgs_scene;
    light_msgs_list _msgs_light_factory;
    light_msgs_list _msgs_light_modify;
    link_msgs_list _msgs_link;
    model_msgs_list _msgs_model;
    skeleton_msgs_list _msgs_skeleton_pose;

    visuals_map _visuals;
    light_map _lights;

    std::mutex _mutex_receive;
    std::recursive_mutex _mutex_pose_msgs;

    std::mutex _mutex_scenegraph;
    bool _is_root_not_initialized = true;
    NRPNode *_root_node;
    NRPInteractiveNode *_interactive_node;
    NRPCameraNode *_cam_node;

    std::string _name;
    ptr_visual _world_visual;

    ptr_visual get_visual(const uint32_t id) const;
    ptr_visual get_visual(const std::string &name) const;
};
} // namespace nrp
} // namespace gua

#endif // GUACAMOLE_NRP_SCENE_H
