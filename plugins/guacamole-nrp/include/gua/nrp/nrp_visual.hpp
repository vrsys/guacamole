#ifndef GUACAMOLE_NRP_VISUAL_H
#define GUACAMOLE_NRP_VISUAL_H

#include <gazebo/common/Console.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/rendering/Material.hh>
#include <gazebo/rendering/Visual.hh>

#include "OgreMaterialManager.h"
#include "OgrePass.h"
#include "OgreTechnique.h"
#include "gazebo/rendering/Conversions.hh"

#include <gua/guacamole.hpp>

#include <gua/node/Node.hpp>
#include <gua/node/TransformNode.hpp>
#include <gua/renderer/MaterialLoader.hpp>
#include <gua/renderer/TriMeshLoader.hpp>
#include <memory>

#include <gua/nrp/platform.hpp>

namespace gua
{
namespace nrp
{
class NRPVisual;
class NRPScene;

typedef std::shared_ptr<NRPVisual> ptr_visual;

class GUA_NRP_DLL NRPVisual
{
  public:
    enum VisualType
    {
        VT_ENTITY,
        VT_MODEL,
        VT_LINK,
        VT_VISUAL
    };

    NRPVisual(const std::string &name, gua::node::Node *root_node);
    NRPVisual(const std::string &name, ptr_visual parent);
    ~NRPVisual();

    void set_id(uint32_t _id);
    void set_name(const std::string &name);
    void set_type(VisualType _type);

    uint32_t get_id() const;
    std::string get_name() const;
    VisualType get_type() const;
    const gazebo::math::Vector3 &get_scale() const;
    const ptr_visual get_parent() const;
    const std::shared_ptr<gua::node::TransformNode> get_node() const;

    void update_from_msg(const boost::shared_ptr<const gazebo::msgs::Visual> &msg);

    void set_scale(const gazebo::math::Vector3 &scale);
    void set_pose(const gazebo::math::Pose &pose);
    void set_material(gua::math::vec4 &ambient, gua::math::vec4 &diffuse, gua::math::vec4 &specular, gua::math::vec4 &emissive);

    const scm::math::mat4d flip_transform(const scm::math::mat4d &transform);

  protected:
    gua::TriMeshLoader _tml;

    uint32_t _id;
    std::string _name;
    VisualType _type;

    std::map<std::string, std::shared_ptr<gua::node::Node>> _attached_meshes;

    ptr_visual _parent;
    // std::vector<ptr_visual> _children;

    ignition::math::Vector3d _scale;

    std::shared_ptr<gua::node::TransformNode> _node;

    bool attach_mesh(const std::string &mesh_file_name, bool normalize_shape, gazebo::math::Vector3 &scale, scm::math::mat4d offset);
    void detach_meshes();
    bool get_material_colors_for_material_name(const std::string &material_name, gazebo::common::Color &ambient, gazebo::common::Color &diffuse, gazebo::common::Color &specular,
                                               gazebo::common::Color &emissive);
    std::string generate_random_name();
};
} // namespace nrp
} // namespace gua
#endif // GUACAMOLE_NRP_VISUAL_H