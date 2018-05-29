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
            _node->data.set_max_shadow_dist(1000);
            _node->data.set_shadow_cascaded_splits({0.3f, 0.7f, 1.0f, 10.0f});
            _node->data.set_shadow_near_clipping_in_sun_direction(0.0f);
            _node->data.set_shadow_far_clipping_in_sun_direction(1000.0f);
        }
    }

    if(msg->has_range() && int(msg->range()) != 0)
    {
        _scale = msg->range();
    }

    // specular component predominantly weak
    _node->data.set_enable_specular_shading(false);

    if(msg->has_specular())
    {
        //_node->data.set_enable_specular_shading(true);

        // specular color not used if diffuse is given
        _node->data.set_color(gua::utils::Color3f(msg->specular().r(), msg->specular().g(), msg->specular().b()));
    }
    //else
    //{
        //_node->data.set_enable_specular_shading(false);
    //}

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

            _node->data.set_brightness(20);

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

            _node->data.set_brightness(10 * (msg->diffuse().r() + msg->diffuse().g() + msg->diffuse().b()) / 3.0f);

            break;
        }
        case gazebo::msgs::Light_LightType_SPOT:
        {
            _node->data.set_type(gua::node::LightNode::Type::SPOT);

            _node->data.set_falloff(2.f);
            _node->data.set_softness(2.f);

            _node->data.set_brightness(20);

            break;
        }
        }
    }

    if(msg->has_pose())
    {
        set_pose(gazebo::msgs::ConvertIgn(msg->pose()));
    }
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
    scm::math::quatd quaternion = scm::math::quatd::from_euler(direction.x(), direction.y(), direction.z());
    _direction = quaternion.to_matrix();
}
}
}