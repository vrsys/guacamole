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
            std::cerr << "Unknown geometry type[" << msg->geometry().type() << "]\n";

        if(msg->has_material())
        {
            gazebo::msgs::Material material = msg->material();
            gua::math::vec4 ambient, diffuse, specular, emissive;

            if(material.has_script() && material.script().has_name())
            {
                gazebo::common::Color mat_ambient, mat_diffuse, mat_specular, mat_emissive;

                bool material_colors = get_material_colors_for_material_name(material.script().name(), mat_ambient, mat_diffuse, mat_specular, mat_emissive);

                if(material_colors)
                {
                    ambient = gua::math::vec4(mat_ambient.r, mat_ambient.g, mat_ambient.b, mat_ambient.a);
                    diffuse = gua::math::vec4(mat_diffuse.r, mat_diffuse.g, mat_diffuse.b, mat_diffuse.a);
                    specular = gua::math::vec4(mat_specular.r, mat_specular.g, mat_specular.b, mat_specular.a);
                    emissive = gua::math::vec4(mat_emissive.r, mat_emissive.g, mat_emissive.b, mat_emissive.a);
                }
                else
                {
                    std::cerr << "Material not found in group General: " << material.script().name() << std::endl;
                }
            }else{
                if(material.has_ambient())
                {
                    ambient = gua::math::vec4(material.ambient().r(), material.ambient().g(), material.ambient().b(), material.ambient().a());
                }

                if(material.has_diffuse())
                {
                    diffuse = gua::math::vec4(material.diffuse().r(), material.diffuse().g(), material.diffuse().b(), material.diffuse().a());
                }

                if(material.has_specular())
                {
                    specular = gua::math::vec4(material.specular().r(), material.specular().g(), material.specular().b(), material.specular().a());
                }

                if(material.has_emissive())
                {
                    emissive = gua::math::vec4(material.emissive().r(), material.emissive().g(), material.emissive().b(), material.emissive().a());
                }
            }

            set_material(ambient, diffuse, specular, emissive);
        }
    }
}

bool PagodaVisual::attach_mesh(const std::string &mesh_name, bool normalize_shape, gazebo::math::Vector3 &scale, scm::math::mat4d offset)
{
    if(mesh_name.empty())
        return false;

    unsigned int flags = !normalize_shape ? gua::TriMeshLoader::LOAD_MATERIALS : gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::NORMALIZE_SCALE | gua::TriMeshLoader::NORMALIZE_POSITION;

    static auto &chrs = "0123456789"
                        "abcdefghijklmnopqrstuvwxyz"
                        "ABCDEFGHIJKLMNOPQRSTUVWXYZ";

    thread_local static std::mt19937 rg{std::random_device{}()};
    thread_local static std::uniform_int_distribution<std::string::size_type> pick(0, sizeof(chrs) - 2);

    std::string mesh_random_name;

    int length = 8;

    mesh_random_name.reserve(length);

    while(length--)
        mesh_random_name += chrs[pick(rg)];

    std::shared_ptr<node::Node> geometry_node = _tml.create_geometry_from_file(mesh_random_name, mesh_name, flags);

    // geometry_node->set_draw_bounding_box(true);

    // std::cout << "attach_mesh(" << mesh_name << " , " << mesh_random_name << ")" << std::endl;

    _node->add_child(geometry_node);

    geometry_node->scale(scale.x, scale.y, scale.z);
    geometry_node->set_transform(flip_transform(geometry_node->get_transform()) * offset);

    return true;
}

void PagodaVisual::set_material(gua::math::vec4 &ambient, gua::math::vec4 &diffuse, gua::math::vec4 &specular, gua::math::vec4 &emissive)
{
    // TODO: shading specifics are not accounted for

    for(const std::shared_ptr<node::Node> &geometry_node : _node->get_children())
    {
        std::stack<std::shared_ptr<node::Node>> trav_stack;
        trav_stack.push(geometry_node);

        while(!trav_stack.empty())
        {
            std::shared_ptr<node::Node> top = trav_stack.top();
            trav_stack.pop();

            for(const std::shared_ptr<node::Node> &child : top->get_children())
            {
                trav_stack.push(child);
            }

            std::shared_ptr<gua::node::TriMeshNode> tm_candidate = std::dynamic_pointer_cast<gua::node::TriMeshNode>(top);
            if(tm_candidate)
            {
                std::shared_ptr<Material> material = gua::MaterialShaderDatabase::instance()->lookup("gua_default_material")->make_new_material();
                material->set_uniform("Color", ambient);
                tm_candidate->set_material(material);

                //std::cout << "Material set to: " << ambient << std::endl;
            }
        }
    }
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
    // std::cout << "set_pose(" << pose.pos.x << "," << pose.pos.y << "," << pose.pos.z << ")" << std::endl;
    // std::cout << "set_rotation(" << pose.rot.w << "," << pose.rot.x << "," << pose.rot.y << "," << pose.rot.z << ")" << std::endl;

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
bool PagodaVisual::get_material_colors_for_material_name(const std::string &material_name, gazebo::common::Color &ambient, gazebo::common::Color &diffuse, gazebo::common::Color &specular,
                                                         gazebo::common::Color &emissive)
{
    Ogre::MaterialPtr material_ptr;

    if(Ogre::MaterialManager::getSingleton().resourceExists(material_name))
    {
        material_ptr = Ogre::MaterialManager::getSingleton().getByName(material_name, "General");

        if(material_ptr.isNull())
            return false;

        Ogre::Technique *technique = material_ptr->getTechnique(0);
        if(technique && technique->getNumPasses() > 0)
        {
            Ogre::Pass *pass = technique->getPass(0);
            if(pass)
            {
                ambient = gazebo::rendering::Conversions::Convert(pass->getAmbient());
                diffuse = gazebo::rendering::Conversions::Convert(pass->getDiffuse());
                specular = gazebo::rendering::Conversions::Convert(pass->getSpecular());
                emissive = gazebo::rendering::Conversions::Convert(pass->getSelfIllumination());
                return true;
            }
        }
    }

    return false;
}
}
}