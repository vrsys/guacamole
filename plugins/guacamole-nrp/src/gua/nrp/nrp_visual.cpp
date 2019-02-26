#include <utility>

#include <gua/nrp/nrp_scene.hpp>
#include <gua/renderer/PBSMaterialFactory.hpp>
namespace gua
{
namespace nrp
{
NRPVisual::NRPVisual(const std::string &name, node::Node *root_node) : _name(name), _parent(), _node(), _attached_meshes()
{
    std::shared_ptr<gua::node::TransformNode> node = root_node->add_child(std::make_shared<gua::node::TransformNode>(name));
    _node.reset(node.get());

    _node->set_transform(gua::math::mat4::identity());
    _scale = gazebo::math::Vector3::One.Ign();

    if(_name.find("interactive") != std::string::npos)
    {
        _node->get_tags().add_tag("invisible");
    }
}
NRPVisual::NRPVisual(const std::string &name, ptr_visual parent) : _name(name), _parent(parent.get()), _node(), _attached_meshes()
{
    std::shared_ptr<gua::node::TransformNode> node = parent->get_node()->add_child(std::make_shared<gua::node::TransformNode>(name));
    _node.reset(node.get());

    _node->set_transform(gua::math::mat4::identity());
    _scale = gazebo::math::Vector3::One.Ign();

    if(_name.find("interactive") != std::string::npos)
    {
        _node->get_tags().add_tag("invisible");
    }
}
NRPVisual::~NRPVisual() { _node.reset(); }

void NRPVisual::set_name(const std::string &name) { _name = name; }
std::string NRPVisual::get_name() const { return _name; }
uint32_t NRPVisual::get_id() const { return _id; }
void NRPVisual::set_id(uint32_t id) { _id = id; }
NRPVisual::VisualType NRPVisual::get_type() const { return _type; }
void NRPVisual::set_type(NRPVisual::VisualType type) { _type = type; }

void NRPVisual::update_from_msg(const boost::shared_ptr<gazebo::msgs::Visual const> &msg)
{
    if(_name.find("interactive") != std::string::npos)
    {
        return;
    }

#if GUA_DEBUG == 1
    auto start = std::chrono::high_resolution_clock::now();
#endif

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
        // detach_meshes();

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
            }
            else
            {
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
#if GUA_DEBUG == 1
    auto end = std::chrono::high_resolution_clock::now();
    float update_from_msg = std::chrono::duration<float, std::milli>(end - start).count();

    if(update_from_msg > 0.01f)
    {
        std::cout << "update_from_msg: " << update_from_msg << std::endl;
    }
#endif
}

bool NRPVisual::attach_mesh(const std::string &mesh_file_name, bool normalize_shape, gazebo::math::Vector3 &scale, scm::math::mat4d offset)
{
    if(mesh_file_name.empty())
        return false;

    std::shared_ptr<gua::node::Node> geometry_node = _node->add_child(std::make_shared<gua::node::TransformNode>());

    auto attached_trimesh = _attached_meshes.find(mesh_file_name);

    if(attached_trimesh == _attached_meshes.cend())
    {
        unsigned int flags = gua::TriMeshLoader::LOAD_MATERIALS | gua::TriMeshLoader::PARSE_HIERARCHY | gua::TriMeshLoader::MAKE_PICKABLE;

        flags |= (!normalize_shape ? 0 : gua::TriMeshLoader::NORMALIZE_SCALE | gua::TriMeshLoader::NORMALIZE_POSITION);

        auto mesh = _tml.create_geometry_from_file(generate_random_name(), mesh_file_name, flags);
        geometry_node->add_child(mesh);

        // geometry_node->set_draw_bounding_box(true);

        // std::cout << "attach_mesh(" << mesh_file_name << " , " << mesh_random_name << ")" << std::endl;

        _node->add_child(geometry_node);

        std::pair<std::string, std::shared_ptr<gua::node::Node>> entry(mesh_file_name, geometry_node);
        _attached_meshes.insert(entry);
    }
    else
    {
        geometry_node = attached_trimesh->second;
        geometry_node->set_transform(scm::math::mat4d::identity());
    }

    geometry_node->scale(scale.x, scale.y, scale.z);
    geometry_node->set_transform(flip_transform(geometry_node->get_transform()) * offset);

    return true;
}

void NRPVisual::set_material(gua::math::vec4 &ambient, gua::math::vec4 &diffuse, gua::math::vec4 &specular, gua::math::vec4 &emissive)
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
                auto material(gua::PBSMaterialFactory::create_material(static_cast<gua::PBSMaterialFactory::Capabilities>(
                    gua::PBSMaterialFactory::COLOR_VALUE | gua::PBSMaterialFactory::METALNESS_VALUE | gua::PBSMaterialFactory::ROUGHNESS_VALUE | gua::PBSMaterialFactory::EMISSIVITY_VALUE)));
                material->set_uniform("Color", gua::math::vec4f((float)ambient.r, (float)ambient.g, (float)ambient.b, 1.f));

                float max_spec = std::max(specular.r, std::max(specular.g, specular.b));
                float max_diff = std::max(diffuse.r, std::max(diffuse.g, diffuse.b));
                float mtro = (max_spec - max_diff + 1.f) / 2.f;

                material->set_uniform("Metalness", mtro);
                material->set_uniform("Roughness", 1.f - mtro);
                material->set_uniform("Emissivity", (float)(emissive.r + emissive.g + emissive.b) / 3.f);

                tm_candidate->set_material(material);

#if 0
                auto material = gua::MaterialShaderDatabase::instance()->lookup("overwrite_color")->make_new_material();

                material->set_uniform("color", gua::math::vec3f((float)ambient.r, (float)ambient.g, (float)ambient.b));
                material->set_uniform("metalness", (float)(specular.r + specular.g + specular.b) / 3.f);
                material->set_uniform("roughness", (float)(diffuse.r + diffuse.g + diffuse.b) / 3.f);
                material->set_uniform("emissivity", (float)(emissive.r + emissive.g + emissive.b) / 3.f);

                tm_candidate->set_material(material);
#endif

                // std::cout << "Material set to: " << ambient << std::endl;
            }
        }
    }
}

void NRPVisual::set_scale(const gazebo::math::Vector3 &scale)
{
    if(_scale == scale.Ign())
        return;

    _scale = scale.Ign();

    _node->scale(scale.x, scale.y, scale.z);
}
void NRPVisual::set_pose(const gazebo::math::Pose &pose)
{
    // std::cout << "set_pose(" << pose.pos.x << "," << pose.pos.y << "," << pose.pos.z << ")" << std::endl;
    // std::cout << "set_rotation(" << pose.rot.w << "," << pose.rot.x << "," << pose.rot.y << "," << pose.rot.z << ")" << std::endl;

    scm::math::mat4d translation = scm::math::make_translation(pose.pos.x, pose.pos.y, pose.pos.z);
    scm::math::quatd quaternion = scm::math::quatd(pose.rot.w, pose.rot.x, pose.rot.y, pose.rot.z);

    _node->set_transform(translation * quaternion.to_matrix());
}
const scm::math::mat4d NRPVisual::flip_transform(const scm::math::mat4d &transform)
{
    scm::math::mat4d transform_flipped = scm::math::mat4d::identity();
    scm::math::mat4d rot_90_x = scm::math::make_rotation(90., 1., 0., 0.);

    transform_flipped *= transform;
    transform_flipped *= rot_90_x;

    return transform_flipped;
}
const gazebo::math::Vector3 &NRPVisual::get_scale() const { return _scale; }
const ptr_visual NRPVisual::get_parent() const { return _parent; }
void NRPVisual::detach_meshes()
{
    _node->clear_children();
    _attached_meshes.clear();
}
const std::shared_ptr<gua::node::TransformNode> NRPVisual::get_node() const { return _node; }
bool NRPVisual::get_material_colors_for_material_name(const std::string &material_name, gazebo::common::Color &ambient, gazebo::common::Color &diffuse, gazebo::common::Color &specular,
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
std::string NRPVisual::generate_random_name()
{
    static auto &chrs = "0123456789"
                        "abcdefghijklmnopqrstuvwxyz"
                        "ABCDEFGHIJKLMNOPQRSTUVWXYZ";

    thread_local static std::mt19937 rg{std::random_device{}()};
    thread_local static std::uniform_int_distribution<std::string::size_type> pick(0, sizeof(chrs) - 2);

    std::string mesh_random_name;

    int length = 8;

    mesh_random_name.reserve((uint8_t)length);

    while(length--)
        mesh_random_name += chrs[pick(rg)];

    return mesh_random_name;
}
} // namespace nrp
} // namespace gua