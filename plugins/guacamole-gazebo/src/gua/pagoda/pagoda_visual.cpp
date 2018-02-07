#include <utility>

#include "../../../include/gua/pagoda/pagoda_scene.hpp"

PagodaVisual::PagodaVisual(const std::string &name, gua::SceneGraph *scene_graph) : _name(name), _parent(), _node()
{
    std::shared_ptr<gua::node::TransformNode> node = scene_graph->add_node<gua::node::TransformNode>("/transform", name);
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

            attach_mesh(std::string(GUACAMOLE_INSTALL_DIR) + "/resources/geometry/primitive_box.obj", true, geom_scale);
        }
        else if(msg->geometry().type() == gazebo::msgs::Geometry::CYLINDER)
        {
            geom_scale.x = msg->geometry().cylinder().radius() * 2.0;
            geom_scale.y = msg->geometry().cylinder().radius() * 2.0;
            geom_scale.z = msg->geometry().cylinder().length();

            attach_mesh(std::string(GUACAMOLE_INSTALL_DIR) + "/resources/geometry/primitive_cylinder.obj", true, geom_scale);
        }
        else if(msg->geometry().type() == gazebo::msgs::Geometry::SPHERE)
        {
            geom_scale.x = geom_scale.y = geom_scale.z = msg->geometry().sphere().radius() * 2.0;

            attach_mesh(std::string(GUACAMOLE_INSTALL_DIR) + "/resources/geometry/primitive_sphere.obj", true, geom_scale);
        }
        else if(msg->geometry().type() == gazebo::msgs::Geometry::PLANE)
        {
            // TODO
            //            o_x = v_it.geometry().plane().normal().x();
            //            o_y = v_it.geometry().plane().normal().y();
            //            o_z = v_it.geometry().plane().normal().z();
            //
            //            geometry_node = _tml.create_geometry_from_file(
            //                "box", "data/objects/box.obj",
            //                gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE | gua::TriMeshLoader::LOAD_MATERIALS);
            //
            //            scm::math::vec3d normal = scm::math::vec3d(o_x, o_y, o_z);
            //            scm::math::vec3d tangent_0 = scm::math::cross(scm::math::vec3d(o_x, o_y, o_z), scm::math::vec3d(1, 0, 0));
            //            if (scm::math::dot(tangent_0, tangent_0) < 0.001)
            //                tangent_0 = scm::math::cross(normal, scm::math::vec3d(0, 1, 0));
            //            tangent_0 = scm::math::normalize(tangent_0);
            //            scm::math::vec3d tangent_1 = scm::math::normalize(scm::math::cross(scm::math::vec3d(o_x, o_y, o_z), tangent_0));
            //
            //            scm::math::mat4d _transform = scm::math::mat4d::identity();
            //
            //            _transform.column(0) = scm::math::vec4d(tangent_0);
            //            _transform.column(1) = scm::math::vec4d(tangent_1);
            //            _transform.column(2) = scm::math::vec4d(normal);
            //            _transform.column(3) = scm::math::vec4d(0, 0, 0, 1);
            //
            //            geometry_node->set_transform(_transform);

            //            if(msg->geometry().plane().has_size())
            //            {
            //                geom_scale.x = msg->geometry().plane().size().x();
            //                geom_scale.y = msg->geometry().plane().size().y();
            //            }
        }
        else if(msg->geometry().type() == gazebo::msgs::Geometry::IMAGE)
        {
            geom_scale.x = geom_scale.y = geom_scale.z = msg->geometry().image().scale();
        }
        else if(msg->geometry().type() == gazebo::msgs::Geometry::HEIGHTMAP)
        {
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
                attach_mesh(std::string(GUACAMOLE_INSTALL_DIR) + "/resources/geometry/primitive_box.obj", false, geom_scale);
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

            attach_mesh(mesh_name, center_submesh, geom_scale);
        }
        else if(msg->geometry().type() == gazebo::msgs::Geometry::EMPTY || msg->geometry().type() == gazebo::msgs::Geometry::POLYLINE)
        {
            // do nothing for now - keep unit scale.
        }
        else
            gzerr << "Unknown geometry type[" << msg->geometry().type() << "]\n";
    }
}

bool PagodaVisual::attach_mesh(const std::string &mesh_name, bool normalize_shape, gazebo::math::Vector3 &scale)
{
    std::cout << "attach_mesh(" << mesh_name << ")" << std::endl;

    if(mesh_name.empty())
        return false;

    // _mesh_name = mesh_name;
    // _sub_mesh_name = sub_mesh;

    unsigned int flags = !normalize_shape ? gua::TriMeshLoader::LOAD_MATERIALS : gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::NORMALIZE_SCALE | gua::TriMeshLoader::NORMALIZE_POSITION;

    auto geometry_node = _tml.create_geometry_from_file(mesh_name, mesh_name, flags);
    // geometry_node->set_draw_bounding_box(true);

    _node->add_child(geometry_node);

    geometry_node->scale(scale.x, scale.y, scale.z);
    geometry_node->set_transform(flip_transform(geometry_node->get_transform()));

    return true;
}
void PagodaVisual::set_scale(const gazebo::math::Vector3 &scale)
{
    if(_scale == scale.Ign())
        return;

    _scale = scale.Ign();

    _node->scale(scale.x, scale.y, scale.z);
    // _node->set_transform(flip_transform(_node->get_transform()));
}
void PagodaVisual::set_pose(const gazebo::math::Pose &pose)
{
    std::cout << "set_pose(" << pose.pos.x << "," << pose.pos.y << "," << pose.pos.z << ")" << std::endl;
    std::cout << "set_rotation(" << pose.rot.w << "," << pose.rot.x << "," << pose.rot.y << "," << pose.rot.z << ")" << std::endl;

    scm::math::mat4d translation = scm::math::make_translation(pose.pos.x, pose.pos.y, pose.pos.z);
    scm::math::quatd quaternion = scm::math::quatd(pose.rot.w, pose.rot.x, pose.rot.y, pose.rot.z);

    _node->set_transform(translation * quaternion.to_matrix());
    // _node->set_transform(flip_transform(translation * quaternion.to_matrix()));
}
const scm::math::mat4d PagodaVisual::flip_transform(const scm::math::mat4d &transform)
{
    // TODO: devise pre matrix by examining side by side

    /* SCRATCHPAD START */

    //    scm::math::mat4d robot_position = scm::math::mat4d::identity() * scm::math::make_translation(0., 0., 0.15);
    //    scm::math::mat4d robot_rotation = scm::math::make_rotation(0.868112, 0.390607, -0.222017, -0.210987);
    //
    //    scm::math::mat4d robot = robot_position * robot_rotation;
    //
    //    scm::math::mat4d right_eye_position = scm::math::mat4d::identity() * scm::math::make_translation(-0.230682, -0.15136, 0.37049);
    //    scm::math::mat4d right_eye_rotation = scm::math::make_rotation(0.889883, -0.337649, -0.00798238, -0.306656);
    //
    //    scm::math::mat4d right_eye = right_eye_position * right_eye_rotation;
    //
    //    scm::math::mat4d eff_relative = scm::math::inverse(robot) * right_eye;
    //
    //    scm::math::mat4d exp_relative_translation = scm::math::mat4d::identity() * scm::math::make_translation(-0.0564, 0.034, 0.34685);
    //    scm::math::mat4d exp_relative_rotation = scm::math::quatd::from_euler(-1.5708, 0., 0.).to_matrix();
    //
    //    scm::math::mat4d exp_relative = exp_relative_translation * exp_relative_rotation;
    //
    //    std::cout << std::endl;
    //    std::cout << "Effective: " << eff_relative.m00 << ", " << eff_relative.m01 << ", " << eff_relative.m02 << ", "<<eff_relative.m03 << ", " << std::endl;
    //    std::cout << "Effective: " << eff_relative.m04 << ", " << eff_relative.m05 << ", " << eff_relative.m06 << ", "<<eff_relative.m07 << ", " << std::endl;
    //    std::cout << "Effective: " << eff_relative.m08 << ", " << eff_relative.m09 << ", " << eff_relative.m10 << ", "<<eff_relative.m11 << ", " << std::endl;
    //    std::cout << "Effective: " << eff_relative.m12 << ", " << eff_relative.m13 << ", " << eff_relative.m14 << ", "<<eff_relative.m15 << std::endl;
    //
    //    std::cout << "Expected: " << exp_relative.m00 << ", " << exp_relative.m01 << ", " << exp_relative.m02 << ", "<<exp_relative.m03 << ", " << std::endl;
    //    std::cout << "Expected: " << exp_relative.m04 << ", " << exp_relative.m05 << ", " << exp_relative.m06 << ", "<<exp_relative.m07 << ", " << std::endl;
    //    std::cout << "Expected: " << exp_relative.m08 << ", " << exp_relative.m09 << ", " << exp_relative.m10 << ", "<<exp_relative.m11 << ", " << std::endl;
    //    std::cout << "Expected: " << exp_relative.m12 << ", " << exp_relative.m13 << ", " << exp_relative.m14 << ", "<<exp_relative.m15 << std::endl;
    //    std::cout << std::endl;

    //    scm::math::vec3d translation = gua::math::get_translation(relative);
    //    scm::math::quatd rotation_quat = scm::math::quatd::from_matrix(gua::math::get_rotation(relative));

    //    double roll, pitch, yaw;
    //
    //    // roll (x-axis rotation)
    //    double sinr = +2.0 * (rotation_quat.w * rotation_quat.x + rotation_quat.y * rotation_quat.z);
    //    double cosr = +1.0 - 2.0 * (rotation_quat.x * rotation_quat.x + rotation_quat.y * rotation_quat.y);
    //    roll = atan2(sinr, cosr);
    //
    //    // pitch (y-axis rotation)
    //    double sinp = +2.0 * (rotation_quat.w * rotation_quat.y - rotation_quat.z * rotation_quat.x);
    //    if(fabs(sinp) >= 1)
    //        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    //    else
    //        pitch = asin(sinp);
    //
    //    // yaw (z-axis rotation)
    //    double siny = +2.0 * (rotation_quat.w * rotation_quat.z + rotation_quat.x * rotation_quat.y);
    //    double cosy = +1.0 - 2.0 * (rotation_quat.y * rotation_quat.y + rotation_quat.z * rotation_quat.z);
    //    yaw = atan2(siny, cosy);
    //
    //    std::cout << std::endl;
    //    std::cout << "Position: " << translation.x << ", " << translation.y << ", " << translation.z << std::endl;
    //    std::cout << "Rotation: " << roll << ", " << pitch << ", " << yaw << std::endl;
    //    std::cout << std::endl;

    /* SCRATCHPAD END */

    scm::math::mat4d transform_flipped = scm::math::mat4d::identity();

    /* ROTATION */

    scm::math::mat4d rot_90_x = scm::math::make_rotation(90., 1., 0., 0.);
    //scm::math::mat4d rot_90_z = scm::math::make_rotation(90.0, 0., 0., 1.);
    //scm::math::mat4d rot_90_y = scm::math::make_rotation(90.0, 0., 1., 0.);

    //    gua::math::mat4d pre = scm::math::inverse(scm::math::mat4d(0.99942, -0.0040342, -0.03377, 0.,
    //                                                               0.00324, 0.999717, -0.02356, 0.,
    //                                                               0.03385, 0.0234378, 0.99915, 0.,
    //                                                               -0.17511, -0.184189, -0.11825, 1.));

    //    gua::math::mat4d post = scm::math::inverse(scm::math::mat4d(0.99942, -0.00468, -0.03376, 0.,
    //                                                                0.00388397, 0.999717, -0.0234632, 0.,
    //                                                                0.03386, 0.02332, 0.99915, 0.,
    //                                                                -0.18371, -0.19253, -0.12382, 1.));

    // scm::math::mat4d rot_180_z = scm::math::inverse(scm::math::make_rotation(180., 0., 0., 1.));

    /* FLIP */

    // scm::math::mat4d flip_y = scm::math::mat4d(1.0, 0., 0., 0., 0., -1.0, 0., 0., 0., 0., 1.0, 0., 0., 0., 0., 1.0);

    /* MIRROR */

    // scm::math::mat4d mirror_z = scm::math::inverse(scm::math::make_scale(1., 1., -1.));

    transform_flipped *= transform;
    //transform_flipped *= rot_90_y;// * rot_90_z;
    transform_flipped *= rot_90_x;

    /* FLIPPED QUAT */

    //    gua::math::vec3d translation = gua::math::get_translation(transform);
    //    gua::math::quatd quaternion = gua::math::quatd::from_matrix(gua::math::get_rotation(transform));
    //    gua::math::vec3d scaling = gua::math::get_scale(transform);

    //    double angle;
    //    gua::math::vec3d axis;
    //    quaternion.retrieve_axis_angle(angle, axis);
    //
    //    transform_flipped *= scm::math::make_translation(translation.x, translation.z, translation.y);
    //    transform_flipped *= quaternion.from_axis(angle, gua::math::vec3d(axis.x, axis.z, axis.y)).to_matrix();
    //    transform_flipped *= scm::math::make_scale(scaling.x, scaling.z, scaling.y);

    return transform_flipped;
}
const gazebo::math::Vector3 &PagodaVisual::get_scale() const { return _scale; }
const ptr_visual PagodaVisual::get_parent() const { return _parent; }
void PagodaVisual::detach_meshes()
{
    _node->clear_children();

    // _mesh_name = "";
    // _sub_mesh_name = "";
}
const std::shared_ptr<gua::node::TransformNode> PagodaVisual::get_node() const { return _node; }
