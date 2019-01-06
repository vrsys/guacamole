#include <gua/nrp/nrp_light.hpp>
#include <gua/scenegraph.hpp>
namespace gua
{
namespace nrp
{
NRPLight::NRPLight(const std::string &name, node::Node *root_node)
{
    std::shared_ptr<gua::node::LightNode> node = root_node->add_child(std::make_shared<gua::node::LightNode>(name));
    _node.reset(node.get());
    _node->set_transform(gua::math::mat4::identity());

    _scale = 1.0f;
    _direction = scm::math::mat4d::identity();
}
NRPLight::~NRPLight() { _node.reset(); }
void NRPLight::load_from_msg(const boost::shared_ptr<const gazebo::msgs::Light> &msg)
{
    if(msg->has_cast_shadows())
    {
        _node->data.set_enable_shadows(msg->cast_shadows());

        if(msg->cast_shadows())
        {
            _node->data.set_shadow_map_size(4096);
            _node->data.set_max_shadow_dist(20.f);
            _node->data.set_shadow_offset(0.005f);
            _node->data.set_shadow_cascaded_splits({0.3f, 0.7f, 1.f, 10.f});
            _node->data.set_shadow_near_clipping_in_sun_direction(0.f);
            _node->data.set_shadow_far_clipping_in_sun_direction(20.f);
        }
    }

    if(msg->has_range() && int(msg->range()) != 0)
    {
        _scale = msg->range();
    }

    // specular component predominantly weak

    if(msg->has_specular())
    {
        //_node->data.set_enable_specular_shading(true);

        // specular color not used if diffuse is given
        _node->data.set_color(gua::utils::Color3f(msg->specular().r(), msg->specular().g(), msg->specular().b()));
    }
    else
    {
        _node->data.set_enable_specular_shading(false);
    }

    if(msg->has_diffuse())
    {
        _node->data.set_enable_diffuse_shading(true);
        _node->data.set_color(gua::utils::Color3f(msg->diffuse().r(), msg->diffuse().g(), msg->diffuse().b()));
    }
    else
    {
        _node->data.set_enable_diffuse_shading(false);
    }

    if(msg->has_type())
    {
        switch(msg->type())
        {
        default:
        case gazebo::msgs::Light_LightType_POINT:
        {
            _node->data.set_type(gua::node::LightNode::Type::POINT);

            _node->data.set_falloff(2.f);
            _node->data.set_softness(2.f);

            _node->data.set_brightness(20.f * std::max(msg->diffuse().r(), std::max(msg->diffuse().g(), msg->diffuse().b())));

            break;
        }
        case gazebo::msgs::Light_LightType_DIRECTIONAL:
        {
            _node->data.set_type(gua::node::LightNode::Type::SUN);

            _scale = 1.0f;

            _node->data.set_falloff(2.f);
            _node->data.set_softness(2.f);

            if(msg->has_direction())
            {
                _node->data.set_enable_godrays(true);

                set_direction(msg->direction());
            }

            _node->data.set_brightness(10.f * std::max(msg->diffuse().r(), std::max(msg->diffuse().g(), msg->diffuse().b())));

            break;
        }
        case gazebo::msgs::Light_LightType_SPOT:
        {
            _node->data.set_type(gua::node::LightNode::Type::SPOT);

            _node->data.set_falloff(2.f);
            _node->data.set_softness(2.f);

            _node->data.set_brightness(20.f * std::max(msg->diffuse().r(), std::max(msg->diffuse().g(), msg->diffuse().b())));

            break;
        }
        }
    }

    if(msg->has_pose())
    {
        set_pose(gazebo::msgs::ConvertIgn(msg->pose()));
    }

    // TODO: figure out proxy geometry

     auto light_proxy_geometry(_tml.create_geometry_from_file("light_proxy", std::string(GUACAMOLE_INSTALL_DIR) + "/resources/geometry/primitive_sphere.obj",
                                                             gua::TriMeshLoader::NORMALIZE_POSITION | gua::TriMeshLoader::NORMALIZE_SCALE));
     light_proxy_geometry->scale(0.02);
    _node->add_child(light_proxy_geometry);
}
void NRPLight::set_pose(const gazebo::math::Pose &pose)
{
    scm::math::mat4d translation = scm::math::make_translation(pose.pos.x, pose.pos.y, pose.pos.z);
    scm::math::quatd quaternion = scm::math::quatd(pose.rot.w, pose.rot.x, pose.rot.y, pose.rot.z);
    scm::math::mat4d scale = scm::math::make_scale(_scale, _scale, _scale);

    _node->set_transform(translation * quaternion.to_matrix() * _direction * scale);
}
void NRPLight::set_direction(const gazebo::msgs::Vector3d &direction)
{
    scm::math::vec3d orientation_direction(direction.x(), direction.y(), direction.z());
    orientation_direction = scm::math::normalize(orientation_direction);
    double angle = scm::math::acos(scm::math::dot(orientation_direction, scm::math::vec3d(0, 0, -1.)));
    scm::math::vec3d axis = scm::math::normalize(scm::math::cross(orientation_direction, scm::math::vec3d(0, 0, -1.)));
    _direction = scm::math::make_rotation(-angle * 57.295779513, axis);

#if GUA_DEBUG == 1
    std::cout << angle << std::endl;
    std::cout << axis << std::endl;
#endif

    // scm::math::quatd quaternion = scm::math::quatd::from_euler(direction.x(), direction.y(), direction.z());
    // _direction = quaternion.to_matrix();
}
}
}