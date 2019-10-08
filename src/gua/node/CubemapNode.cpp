/******************************************************************************
 * guacamole - delicious VR                                                   *
 *                                                                            *
 * Copyright: (c) 2011-2013 Bauhaus-Universität Weimar                        *
 * Contact:   felix.lauer@uni-weimar.de / simon.schneegans@uni-weimar.de      *
 *                                                                            *
 * This program is free software: you can redistribute it and/or modify it    *
 * under the terms of the GNU General Public License as published by the Free *
 * Software Foundation, either version 3 of the License, or (at your option)  *
 * any later version.                                                         *
 *                                                                            *
 * This program is distributed in the hope that it will be useful, but        *
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY *
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License   *
 * for more details.                                                          *
 *                                                                            *
 * You should have received a copy of the GNU General Public License along    *
 * with this program. If not, see <http://www.gnu.org/licenses/>.             *
 *                                                                            *
 ******************************************************************************/

// class header
#include <gua/node/CubemapNode.hpp>

// guacamole headers
#include <gua/scenegraph/NodeVisitor.hpp>
#include <gua/databases.hpp>
#include <gua/renderer/TextureDistance.hpp>
#include <gua/renderer/Frustum.hpp>
#include <gua/math/BoundingBoxAlgo.hpp>

// c++ headers
#include <limits>
#include <climits>

namespace gua
{
namespace node
{
CubemapNode::CubemapNode(std::string const& name, Configuration const& configuration, math::mat4 const& transform) : SerializableNode(name, transform), config(configuration)
{
    m_NewTextureData = std::make_shared<std::atomic<bool>>(false);
    m_MinDistance.distance = -1.0;
    m_WeightedMinDistance.distance = -1.0;

    m_Weights = std::vector<float>(config.get_resolution() * config.get_resolution() * 6, 1.0f);
    create_weights(math::vec3(0.0, 0.0, 0.0), math::vec3(0.0, 0.0, 0.0));

    m_DistortionWeights = std::vector<float>(config.get_resolution() * config.get_resolution(), 1.0f);
    create_distortion_weights();

    update_bounding_box();
}

/* virtual */ void CubemapNode::accept(NodeVisitor& visitor) { visitor.visit(this); }

void CubemapNode::update_bounding_box() const
{
    auto bbox = math::BoundingBox<math::vec3>(math::vec3(-config.far_clip()), math::vec3(config.far_clip()));
    bounding_box_ = transform(bbox, world_transform_);

    for(auto child : get_children())
    {
        bounding_box_.expandBy(child->get_bounding_box());
    }
}

float CubemapNode::get_min_distance()
{
    if(m_NewTextureData->load())
    {
        find_min_distance();
    }
    return m_MinDistance.distance;
}

float CubemapNode::get_weighted_min_distance()
{
    if(m_NewTextureData->load())
    {
        find_min_distance();
    }
    return m_WeightedMinDistance.distance;
}

math::vec3 CubemapNode::get_min_distance_position()
{
    if(m_NewTextureData->load())
    {
        find_min_distance();
    }
    return m_MinDistance.world_position;
}

float CubemapNode::get_distance_by_local_direction(math::vec3 const& dir) const
{
    math::mat4 screen_transform(scm::math::make_translation(0., 0., -0.5));
    std::vector<math::mat4> screen_transforms({screen_transform,
                                               scm::math::make_rotation(180., 0., 1., 0.) * screen_transform,
                                               scm::math::make_rotation(90., 1., 0., 0.) * screen_transform,
                                               scm::math::make_rotation(-90., 1., 0., 0.) * screen_transform,
                                               scm::math::make_rotation(90., 0., 1., 0.) * screen_transform,
                                               scm::math::make_rotation(-90., 0., 1., 0.) * screen_transform});

    for(int i = 0; i < 6; ++i)
    {
        auto frustum(Frustum::perspective(math::mat4::identity(), screen_transforms[i], config.near_clip(), config.far_clip()));
        if(frustum.contains(dir))
        {
            math::vec4 view_point(frustum.get_view() * math::vec4(dir.x, dir.y, dir.z, 1.0));
            math::vec4 proj_point(frustum.get_projection() * view_point);

            return acces_texture_data(i, math::vec2(proj_point.x, proj_point.y));
        }
    }

    return -1.0;
}

void CubemapNode::create_weights(math::vec3 const& view_direction, math::vec3 const& move_direction)
{
    int _pixel_count = config.get_resolution() * config.get_resolution() * 6;
    for(int i = 0; i < _pixel_count; ++i)
    {
        math::vec2 tex_coords = math::vec2(i % (config.resolution() * 6), i / (config.resolution() * 6));
        int side = tex_coords.x / config.resolution();
        math::vec2 xy(float(tex_coords.x) / config.resolution(), float(tex_coords.y) / config.resolution());
        xy.x = fmod(xy.x, 1.0);
        xy -= 0.5;

        math::vec3 point_on_face;
        switch(side)
        {
        case 0:
            point_on_face = math::vec3(xy.x, xy.y, -0.5);
            break;
        case 1:
            point_on_face = math::vec3(-xy.x, xy.y, 0.5);
            break;
        case 2:
            point_on_face = math::vec3(xy.x, 0.5, xy.y);
            break;
        case 3:
            point_on_face = math::vec3(xy.x, -0.5, -xy.y);
            break;
        case 4:
            point_on_face = math::vec3(-0.5, xy.y, -xy.x);
            break;
        case 5:
            point_on_face = math::vec3(0.5, xy.y, xy.x);
            break;
        }
        math::vec3 point_direction = scm::math::normalize(point_on_face);

        m_Weights[i] = 1.0;

        // Weight by ViewDirection
        float view_lenght(scm::math::length(view_direction));
        if(view_lenght > 0.0)
        {
            float cosin_factor = scm::math::dot(point_direction, scm::math::normalize(view_direction)); // alligned: 1.0  not alligned -1.0
            float weighting_strength(5.0);
            m_Weights[i] += weighting_strength * (cosin_factor - 1.0) * -0.5; // alligned: +0.0  not alligned +2.0
        }

        // Weight by MoveDirection
        float move_lenght(scm::math::length(move_direction));
        if(move_lenght > 0.0)
        {
            float cosin_factor = scm::math::dot(point_direction, scm::math::normalize(move_direction));
            float weighting_strength(5.0);
            m_Weights[i] += weighting_strength * (cosin_factor - 1.0) * -0.5;
        }
    }
}

void CubemapNode::create_distortion_weights()
{
    int _res = config.resolution();

    float sum_of_angle_areas(0.0);

    for(int x = 0; x < _res; ++x)
    {
        for(int y = 0; y < _res; ++y)
        {
            float l_x = float(x) / _res - 0.5;
            float u_x = float(x + 1.0) / _res - 0.5;
            float l_y = float(y) / _res - 0.5;
            float u_y = float(y + 1.0) / _res - 0.5;

            float l_x_a = atan(l_x / 0.5);
            float u_x_a = atan(u_x / 0.5);
            float l_y_a = atan(l_y / 0.5);
            float u_y_a = atan(u_y / 0.5);

            l_x_a = (l_x_a / (2 * M_PI)) * 360.0;
            u_x_a = (u_x_a / (2 * M_PI)) * 360.0;
            l_y_a = (l_y_a / (2 * M_PI)) * 360.0;
            u_y_a = (u_y_a / (2 * M_PI)) * 360.0;

            float dxa = fabs(u_x_a - l_x_a);
            float dya = fabs(u_y_a - l_y_a);

            float angle_area = dxa * dya;
            sum_of_angle_areas += angle_area;

            m_DistortionWeights[y * _res + x] = angle_area;
        }
    }

    float mean_area = sum_of_angle_areas / (_res * _res);

    for(int x = 0; x < _res; ++x)
    {
        for(int y = 0; y < _res; ++y)
        {
            m_DistortionWeights[y * _res + x] = m_DistortionWeights[y * _res + x] / mean_area;
        }
    }
}

math::vec3 CubemapNode::get_push_back(float radius, float softness)
{
    auto texture = std::dynamic_pointer_cast<TextureDistance>(TextureDatabase::instance()->lookup(config.get_texture_name()));
    math::vec3 pushback(0.0, 0.0, 0.0);

    if(texture)
    {
        std::vector<float> const& data = texture->get_data();

        unsigned samples(0);

        for(const float& f : data)
        {
            unsigned index = &f - &data[0];

            if((f < radius) && (f != -1.f))
            {
                math::vec2ui tex_coords(index % (config.resolution() * 6), index / (config.resolution() * 6));
                math::vec3 direction(calculate_direction_from_tex_coords(tex_coords));
                direction *= -1.0;

                float intrusion_factor = (radius - f) / radius;
                intrusion_factor = pow(intrusion_factor, 2);

                float _softness = pow(softness, 2);

                int distortion_index = (tex_coords.y * config.resolution()) + (tex_coords.x % config.resolution());
                float weight = -1.0 * exp(-1.0 * intrusion_factor / _softness) + 1.0;

                pushback += direction * weight * m_DistortionWeights[distortion_index];
                ++samples;
            }
        }
        // if (samples > 0){
        //   pushback /= samples;
        // }
    }
    if(scm::math::length(pushback) == 0.0)
    {
        return pushback;
    }
    else
    {
        return pushback /= pow(config.get_resolution(), 2) * 6;
    }
    // return pushback;
}

math::vec3 CubemapNode::get_pull_in(float inner_radius, float outer_radius, float softness)
{
    auto texture = std::dynamic_pointer_cast<TextureDistance>(TextureDatabase::instance()->lookup(config.get_texture_name()));
    math::vec3 pullin(0.0, 0.0, 0.0);

    if(texture)
    {
        std::vector<float> const& data = texture->get_data();

        unsigned samples(0);

        for(const float& f : data)
        {
            unsigned index = &f - &data[0];

            if((f > inner_radius) && (f < outer_radius))
            {
                math::vec2ui tex_coords(index % (config.resolution() * 6), index / (config.resolution() * 6));
                math::vec3 direction(calculate_direction_from_tex_coords(tex_coords));
                // direction *= -1.0;

                // float intrusion_factor = (outer_radius - f) / (outer_radius - inner_radius);
                // float intrusion_factor = (f - inner_radius) / (outer_radius - inner_radius);
                // intrusion_factor = pow(intrusion_factor, 2);

                // float _softness = pow(softness, 2);

                // float weight = -1.0 * exp( -1.0 * intrusion_factor / _softness) + 1.0;
                int distortion_index = (tex_coords.y * config.resolution()) + (tex_coords.x % config.resolution());

                pullin += direction * m_DistortionWeights[distortion_index];
                ;
                // ++samples;
            }
        }
        // if (samples > 0){
        //   pullin /= samples;
        // }
    }
    if(scm::math::length(pullin) == 0.0)
    {
        return pullin;
    }
    else
    {
        // return pullin /= pow(config.get_resolution(), 2) * 6;
        return scm::math::normalize(pullin);
    }
    // return pullin;
}

void CubemapNode::find_min_distance()
{
    auto texture = std::dynamic_pointer_cast<TextureDistance>(TextureDatabase::instance()->lookup(config.get_texture_name()));
    if(texture)
    {
        std::vector<float> const& data = texture->get_data();
        *(m_NewTextureData) = false;

        Distance_Info min_distance;
        min_distance.distance = std::numeric_limits<float>::max();

        Distance_Info weighted_min_distance;
        weighted_min_distance.distance = std::numeric_limits<float>::max();

        for(const float& f : data)
        {
            unsigned index = &f - &data[0];
            float w_f = f * m_Weights[index];
            if((f < min_distance.distance) && (f != -1.f))
            {
                min_distance.distance = f;
                min_distance.tex_coords = math::vec2(index % (config.resolution() * 6), index / (config.resolution() * 6));
            }
            if((w_f < weighted_min_distance.distance) && (f != -1.f))
            {
                weighted_min_distance.distance = w_f;
                weighted_min_distance.tex_coords = math::vec2(index % (config.resolution() * 6), index / (config.resolution() * 6));
            }
        }

        if(min_distance.distance != std::numeric_limits<float>::max())
        {
            min_distance.world_position = project_back_to_world_coords(min_distance);
            m_MinDistance = min_distance;
            m_WeightedMinDistance = weighted_min_distance;
            return;
        }
    }
    // m_MinDistance.distance = -1.0;
    // m_WeightedMinDistance.distance = -1.0;
}

math::vec3 CubemapNode::project_back_to_world_coords(Distance_Info const& di) const
{
    math::vec3 direction(calculate_direction_from_tex_coords(di.tex_coords));

    math::vec4 v4_direction(direction.x, direction.y, direction.z, 0.0);
    math::mat4 rotation(math::get_rotation(world_transform_));

    v4_direction = rotation * v4_direction;
    direction = scm::math::normalize(math::vec3(v4_direction.x, v4_direction.y, v4_direction.z));

    math::vec3 center(gua::math::get_translation(world_transform_));

    return center + (direction * di.distance);
}

math::vec3 CubemapNode::calculate_direction_from_tex_coords(math::vec2ui const& tex_coords) const
{
    int side = tex_coords.x / config.resolution();
    math::vec2 xy(float(tex_coords.x) / config.resolution(), float(tex_coords.y) / config.resolution());
    xy.x = fmod(xy.x, 1.0);
    xy -= 0.5;

    math::vec4 point_on_face;
    switch(side)
    {
    case 0:
        point_on_face = math::vec4(xy.x, xy.y, -0.5, 1.0);
        break;
    case 1:
        point_on_face = math::vec4(-xy.x, xy.y, 0.5, 1.0);
        break;
    case 2:
        point_on_face = math::vec4(xy.x, 0.5, xy.y, 1.0);
        break;
    case 3:
        point_on_face = math::vec4(xy.x, -0.5, -xy.y, 1.0);
        break;
    case 4:
        point_on_face = math::vec4(-0.5, xy.y, -xy.x, 1.0);
        break;
    case 5:
        point_on_face = math::vec4(0.5, xy.y, xy.x, 1.0);
        break;
    }

    // point_on_face = world_transform_ *  point_on_face;

    // math::vec3 center( gua::math::get_translation(world_transform_) );
    // math::vec3 direction( math::vec3(point_on_face.x, point_on_face.y, point_on_face.z) - center );
    math::vec3 direction(math::vec3(point_on_face.x, point_on_face.y, point_on_face.z));

    return scm::math::normalize(direction);
}

float CubemapNode::acces_texture_data(unsigned side, math::vec2 coords) const
{
    unsigned row(config.resolution() * ((coords.y + 1) / 2.0));
    unsigned colm(config.resolution() * ((coords.x + 1) / 2.0));

    auto texture = std::dynamic_pointer_cast<TextureDistance>(TextureDatabase::instance()->lookup(config.get_texture_name()));
    if(texture)
    {
        std::vector<float> const& v = texture->get_data();
        return v[row * config.resolution() * 6 + side * config.resolution() + colm];
    }
    else
    {
        return -1.0;
    }
}

std::shared_ptr<Node> CubemapNode::copy() const { return std::make_shared<CubemapNode>(*this); }

} // namespace node
} // namespace gua
