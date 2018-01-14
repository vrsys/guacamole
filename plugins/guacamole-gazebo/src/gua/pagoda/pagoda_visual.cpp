#include <utility>

#include "../../../include/gua/pagoda/pagoda_scene.hpp"

PagodaVisual::PagodaVisual(const std::string &name, gua::SceneGraph *scene_graph) : _name(name), _parent(), _node()
{
    std::shared_ptr<gua::node::TransformNode> node = scene_graph->add_node<gua::node::TransformNode>("/transform", name);
    _node.reset(node.get());
}
PagodaVisual::PagodaVisual(const std::string &name, ptr_visual parent) : _name(name), _parent(parent.get()), _node()
{
    auto node(std::make_shared<gua::node::TransformNode>(name));
    _node.reset(parent->get_node()->add_child(node).get());
}
PagodaVisual::~PagodaVisual() {}

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
            attach_mesh(std::string(GUACAMOLE_INSTALL_DIR) + "/resources/geometry/primitive_box.obj", "", false);
            geom_scale = gazebo::msgs::ConvertIgn(msg->geometry().box().size());
        }
        else if(msg->geometry().type() == gazebo::msgs::Geometry::CYLINDER)
        {
            attach_mesh(std::string(GUACAMOLE_INSTALL_DIR) + "/resources/geometry/primitive_cylinder.obj", "", false);
            geom_scale.x = msg->geometry().cylinder().radius() * 2.0;
            geom_scale.y = msg->geometry().cylinder().radius() * 2.0;
            geom_scale.z = msg->geometry().cylinder().length();
        }
        else if(msg->geometry().type() == gazebo::msgs::Geometry::SPHERE)
        {
            attach_mesh(std::string(GUACAMOLE_INSTALL_DIR) + "/resources/geometry/primitive_sphere.obj", "", false);
            geom_scale.x = geom_scale.y = geom_scale.z = msg->geometry().sphere().radius() * 2.0;
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

            if(msg->geometry().plane().has_size())
            {
                geom_scale.x = msg->geometry().plane().size().x();
                geom_scale.y = msg->geometry().plane().size().y();
            }
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
                attach_mesh(std::string(GUACAMOLE_INSTALL_DIR) + "/resources/geometry/primitive_box.obj", "", false);
                geom_scale = gazebo::msgs::ConvertIgn(msg->geometry().box().size());
            }
            else
            {
                if(msg->geometry().mesh().has_submesh())
                    submesh_name = msg->geometry().mesh().submesh();
                if(msg->geometry().mesh().has_center_submesh())
                    center_submesh = msg->geometry().mesh().center_submesh();
            }

            attach_mesh(mesh_name, submesh_name, center_submesh);

            if(msg->geometry().mesh().has_scale())
            {
                geom_scale = gazebo::msgs::ConvertIgn(msg->geometry().mesh().scale());
            }
        }
        else if(msg->geometry().type() == gazebo::msgs::Geometry::EMPTY || msg->geometry().type() == gazebo::msgs::Geometry::POLYLINE)
        {
            // do nothing for now - keep unit scale.
        }
        else
            gzerr << "Unknown geometry type[" << msg->geometry().type() << "]\n";

        set_scale(geom_scale * _scale / get_derived_scale());
    }
}

bool PagodaVisual::attach_mesh(const std::string &mesh_name, const std::string &sub_mesh, bool center_submesh, const std::string &obj_name)
{
    std::cout << "attach_mesh(" << mesh_name << ")" << std::endl;

    if(mesh_name.empty())
        return false;

    _mesh_name = mesh_name;
    _sub_mesh_name = sub_mesh;

    unsigned int flags = !center_submesh ? gua::TriMeshLoader::LOAD_MATERIALS : gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::NORMALIZE_SCALE | gua::TriMeshLoader::NORMALIZE_POSITION;

    auto geometry_node = _tml.create_geometry_from_file(mesh_name, mesh_name, flags);
    geometry_node->set_draw_bounding_box(true);

    _node->add_child(geometry_node);

    return true;
}

void PagodaVisual::update_geom_size(const ignition::math::Vector3d &scale)
{
    for(auto &iter : _children)
    {
        iter->update_geom_size(scale * iter->get_scale().Ign());
    }
}

void PagodaVisual::set_scale(const gazebo::math::Vector3 &scale)
{
    if(_scale == scale.Ign())
        return;

    update_geom_size(get_derived_scale() / _scale * scale.Ign());

    _scale = scale.Ign();

    _node->scale(_scale.X(), _scale.Y(), _scale.Z());
}
void PagodaVisual::set_pose(const gazebo::math::Pose &pose)
{
    std::cout << "set_pose()" << std::endl;

    gua::math::mat4 transform_mat = scm::math::make_translation(pose.pos.x, pose.pos.y, pose.pos.z);
    transform_mat = transform_mat * scm::math::quatd(pose.rot.w, pose.rot.x, pose.rot.y, pose.rot.z).to_matrix();

    _node->set_transform(transform_mat);
}
const gazebo::math::Vector3 &PagodaVisual::get_scale() const { return _scale; }

ignition::math::Vector3d PagodaVisual::get_derived_scale() const
{
    ignition::math::Vector3d derivedScale = _scale;

    //    ptr_visual worldVis = _scene->get_world_visual();
    //    ptr_visual vis = _parent;
    //
    //    while(vis && vis != worldVis)
    //    {
    //        derivedScale = derivedScale * vis->get_scale().Ign();
    //        vis = vis->get_parent();
    //    }

    return derivedScale;
}
const ptr_visual PagodaVisual::get_parent() const { return _parent; }
void PagodaVisual::detach_meshes()
{
    _node->clear_children();

    _mesh_name = "";
    _sub_mesh_name = "";
}
const std::shared_ptr<gua::node::TransformNode> PagodaVisual::get_node() const { return _node; }
