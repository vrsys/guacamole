#include <utility>

#include <gua/nrp/pagoda_scene.hpp>
namespace gua
{
namespace nrp
{
PagodaVisual::PagodaVisual(const std::string &name, node::Node *root_node) : _name(name), _parent(), _node()
{
    std::shared_ptr<gua::node::TransformNode> node = root_node->add_child(std::make_shared<gua::node::TransformNode>("/transform/" + name));
    _node.reset(node.get());

    _node->set_transform(gua::math::mat4::identity());
    _scale = gazebo::math::Vector3::One.Ign();
}
PagodaVisual::PagodaVisual(const std::string &name, ptr_visual parent) : _name(name), _parent(parent.get()), _node()
{
    auto node(std::make_shared<gua::node::TransformNode>(name));
    _node.reset(parent->get_node()->add_child(node).get());

    _node->set_transform(gua::math::mat4::identity());
    _scale = gazebo::math::Vector3::One.Ign();
}
PagodaVisual::~PagodaVisual() { _node.reset(); }

void PagodaVisual::set_name(const std::string &name) { _name = name; }
std::string PagodaVisual::get_name() const { return _name; }
uint32_t PagodaVisual::get_id() const { return _id; }
void PagodaVisual::set_id(uint32_t id) { _id = id; }
PagodaVisual::VisualType PagodaVisual::get_type() const { return _type; }
void PagodaVisual::set_type(PagodaVisual::VisualType type) { _type = type; }

void PagodaVisual::update_from_msg(const boost::shared_ptr<gazebo::msgs::Visual const> &msg)
{
    if(msg->has_pose())
    {
        set_pose(gazebo::msgs::ConvertIgn(msg->pose()));
    }

    if(msg->has_visible())
    {
        // ignore visibility
    }

    if(msg->has_scale())
    {
        set_scale(gazebo::msgs::ConvertIgn(msg->scale()));
    }

    if(msg->has_geometry() && msg->geometry().has_type())
    {
        detach_meshes();

        gazebo::math::Vector3 geom_scale(1, 1, 1);

        if(msg->geometry().type() == gazebo::msgs::Geometry::BOX)
        {
            geom_scale = gazebo::msgs::ConvertIgn(msg->geometry().box().size());

            attach_mesh(std::string(GUACAMOLE_INSTALL_DIR) + "/resources/geometry/primitive_box.obj", true, geom_scale, scm::math::mat4d::identity());
        }
        else if(msg->geometry().type() == gazebo::msgs::Geometry::CYLINDER)
        {
            geom_scale.x = msg->geometry().cylinder().radius() * 2.0;
            geom_scale.y = msg->geometry().cylinder().radius() * 2.0;
            geom_scale.z = msg->geometry().cylinder().length();

            attach_mesh(std::string(GUACAMOLE_INSTALL_DIR) + "/resources/geometry/primitive_cylinder.obj", true, geom_scale, scm::math::mat4d::identity());
        }
        else if(msg->geometry().type() == gazebo::msgs::Geometry::SPHERE)
        {
            geom_scale.x = geom_scale.y = geom_scale.z = msg->geometry().sphere().radius() * 2.0;

            attach_mesh(std::string(GUACAMOLE_INSTALL_DIR) + "/resources/geometry/primitive_sphere.obj", true, geom_scale, scm::math::mat4d::identity());
        }
        else if(msg->geometry().type() == gazebo::msgs::Geometry::PLANE)
        {
            if(msg->geometry().plane().has_size())
            {
                geom_scale.x = msg->geometry().plane().size().x();
                geom_scale.y = msg->geometry().plane().size().y();
                geom_scale.z = 0.1;
            }

            if(msg->geometry().plane().has_normal())
            {
                double o_x = msg->geometry().plane().normal().x();
                double o_y = msg->geometry().plane().normal().y();
                double o_z = msg->geometry().plane().normal().z();

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

                attach_mesh(std::string(GUACAMOLE_INSTALL_DIR) + "/resources/geometry/primitive_box.obj", true, geom_scale, _transform);
            }
            else
            {
                attach_mesh(std::string(GUACAMOLE_INSTALL_DIR) + "/resources/geometry/primitive_box.obj", true, geom_scale, scm::math::mat4d::identity());
            }
        }
        else if(msg->geometry().type() == gazebo::msgs::Geometry::IMAGE)
        {
            // TODO

            geom_scale.x = geom_scale.y = geom_scale.z = msg->geometry().image().scale();
        }
        else if(msg->geometry().type() == gazebo::msgs::Geometry::HEIGHTMAP)
        {
            // TODO

            geom_scale = gazebo::msgs::ConvertIgn(msg->geometry().heightmap().size());
        }
        else if(msg->geometry().type() == gazebo::msgs::Geometry::MESH)
        {
            std::string filename = msg->geometry().mesh().filename();
            std::string mesh_name = gazebo::common::find_file(filename);
            std::string submesh_name;
            bool center_submesh = false;

            if(mesh_name.empty())
            {
                geom_scale = gazebo::msgs::ConvertIgn(msg->geometry().box().size());
                attach_mesh(std::string(GUACAMOLE_INSTALL_DIR) + "/resources/geometry/primitive_box.obj", false, geom_scale, scm::math::mat4d::identity());
            }
            else
            {
                if(msg->geometry().mesh().has_submesh())
                    submesh_name = msg->geometry().mesh().submesh();
                if(msg->geometry().mesh().has_center_submesh())
                    center_submesh = msg->geometry().mesh().center_submesh();
            }

            if(msg->geometry().mesh().has_scale())
            {
                geom_scale = gazebo::msgs::ConvertIgn(msg->geometry().mesh().scale());
            }

            attach_mesh(mesh_name, center_submesh, geom_scale, scm::math::mat4d::identity());
        }
        else if(msg->geometry().type() == gazebo::msgs::Geometry::EMPTY || msg->geometry().type() == gazebo::msgs::Geometry::POLYLINE)
        {
            // do nothing for now - keep unit scale.
        }
        else
            gzerr << "Unknown geometry type[" << msg->geometry().type() << "]\n";
    }
}

bool PagodaVisual::attach_mesh(const std::string &mesh_name, bool normalize_shape, gazebo::math::Vector3 &scale, scm::math::mat4d offset)
{
    std::cout << "attach_mesh(" << mesh_name << ")" << std::endl;

    if(mesh_name.empty())
        return false;

    unsigned int flags = !normalize_shape ? gua::TriMeshLoader::LOAD_MATERIALS : gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::NORMALIZE_SCALE | gua::TriMeshLoader::NORMALIZE_POSITION;

    auto geometry_node = _tml.create_geometry_from_file(mesh_name, mesh_name, flags);
    // geometry_node->set_draw_bounding_box(true);

    _node->add_child(geometry_node);

    geometry_node->scale(scale.x, scale.y, scale.z);
    geometry_node->set_transform(flip_transform(geometry_node->get_transform()) * offset);

    return true;
}
void PagodaVisual::set_scale(const gazebo::math::Vector3 &scale)
{
    if(_scale == scale.Ign())
        return;

    _scale = scale.Ign();

    _node->scale(scale.x, scale.y, scale.z);
}
void PagodaVisual::set_pose(const gazebo::math::Pose &pose)
{
    std::cout << "set_pose(" << pose.pos.x << "," << pose.pos.y << "," << pose.pos.z << ")" << std::endl;
    std::cout << "set_rotation(" << pose.rot.w << "," << pose.rot.x << "," << pose.rot.y << "," << pose.rot.z << ")" << std::endl;

    scm::math::mat4d translation = scm::math::make_translation(pose.pos.x, pose.pos.y, pose.pos.z);
    scm::math::quatd quaternion = scm::math::quatd(pose.rot.w, pose.rot.x, pose.rot.y, pose.rot.z);

    _node->set_transform(translation * quaternion.to_matrix());
}
const scm::math::mat4d PagodaVisual::flip_transform(const scm::math::mat4d &transform)
{
    scm::math::mat4d transform_flipped = scm::math::mat4d::identity();
    scm::math::mat4d rot_90_x = scm::math::make_rotation(90., 1., 0., 0.);

    transform_flipped *= transform;
    transform_flipped *= rot_90_x;

    return transform_flipped;
}
const gazebo::math::Vector3 &PagodaVisual::get_scale() const { return _scale; }
const ptr_visual PagodaVisual::get_parent() const { return _parent; }
void PagodaVisual::detach_meshes() { _node->clear_children(); }
const std::shared_ptr<gua::node::TransformNode> PagodaVisual::get_node() const { return _node; }
}
}