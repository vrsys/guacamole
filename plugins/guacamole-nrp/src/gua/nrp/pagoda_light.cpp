#include <gua/nrp/pagoda_light.hpp>
#include <gua/scenegraph.hpp>
namespace gua
{
namespace nrp
{
PagodaLight::PagodaLight(const std::string &name, node::Node *root_node)
{
    std::shared_ptr<gua::node::LightNode> node = root_node->add_child(std::make_shared<gua::node::LightNode>(name));
    _node.reset(node.get());
    _node->set_transform(gua::math::mat4::identity());
}
PagodaLight::~PagodaLight() { _node.reset(); }
void PagodaLight::load_from_msg(const boost::shared_ptr<const gazebo::msgs::Light> &msg)
{
    _node->data.set_enable_shadows(msg->cast_shadows());

    if(msg->cast_shadows())
    {
        _node->data.set_shadow_map_size(512);
        _node->data.set_shadow_cascaded_splits({0.1f, 1.f, 2.f, 5.f});
        _node->data.set_shadow_near_clipping_in_sun_direction(0.0f);
        _node->data.set_shadow_far_clipping_in_sun_direction(100.0f);
    }

    if(msg->has_type())
    {
        switch(msg->type())
        {
        case gazebo::msgs::Light_LightType_POINT:
        {
            _node->data.set_type(gua::node::LightNode::Type::POINT);

            _node->data.set_falloff(2.f);
            _node->data.set_softness(2.f);

            break;
        }
        case gazebo::msgs::Light_LightType_DIRECTIONAL:
        {
            _node->data.set_type(gua::node::LightNode::Type::SUN);

            break;
        }
        case gazebo::msgs::Light_LightType_SPOT:
        {
            _node->data.set_type(gua::node::LightNode::Type::SPOT);

            _node->data.set_falloff(2.f);
            _node->data.set_softness(2.f);

            break;
        }
        default:
            _node->data.set_type(gua::node::LightNode::Type::POINT);
            break;
        }
    }

    if(msg->has_specular())
    {
        _node->data.set_enable_specular_shading(true);

        // specular color not used if diffuse is given
        _node->data.set_color(gua::utils::Color3f(msg->specular().r(), msg->specular().g(), msg->specular().b()));
    }

    if(msg->has_diffuse())
    {
        _node->data.set_enable_diffuse_shading(true);
        _node->data.set_color(gua::utils::Color3f(msg->diffuse().r(), msg->diffuse().g(), msg->diffuse().b()));
    }

    if(msg->has_direction())
    {
        _node->data.set_enable_godrays(true);

        // TODO: make use?
        // msg->direction();
    }

    // TODO: linear, quadratic, constant - can we really incorporate that? With falloff?

    if(msg->has_range())
    {
        _scale = msg->range();
    }

    if(msg->has_pose())
    {
        set_pose(gazebo::msgs::ConvertIgn(msg->pose()));
    }
}
void PagodaLight::set_pose(const gazebo::math::Pose &pose)
{
    scm::math::mat4d translation = scm::math::make_translation(pose.pos.x, pose.pos.y, pose.pos.z);
    scm::math::quatd quaternion = scm::math::quatd(pose.rot.w, pose.rot.x, pose.rot.y, pose.rot.z);
    scm::math::mat4d scale = scm::math::make_scale(_scale, _scale, _scale);

    _node->set_transform(translation * quaternion.to_matrix() * scale);
}
}
}