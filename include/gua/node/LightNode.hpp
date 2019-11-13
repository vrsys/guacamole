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

#ifndef GUA_LIGHT_NODE_HPP
#define GUA_LIGHT_NODE_HPP

#include <gua/platform.hpp>
#include <gua/node/SerializableNode.hpp>
#include <gua/utils/configuration_macro.hpp>

#include <gua/utils/Color3f.hpp>

#include <string>

namespace gua
{
namespace node
{
/**
 * This class is used to represent a light in the SceneGraph.
 *
 * \ingroup gua_scenegraph
 */
class GUA_DLL LightNode : public SerializableNode
{
  public:
    enum class Type
    {
        POINT = 0,
        SPOT = 1,
        SUN = 2
    };

    struct Configuration
    {
        // 0 - point, 1 - spot, 2 - sun
        GUA_ADD_PROPERTY(Type, type, Type::POINT);
        /**
         * The color of the light source.
         * It's possible to use negative values and values > 1.
         */
        GUA_ADD_PROPERTY(utils::Color3f, color, utils::Color3f(1.f, 1.f, 1.f));

        /**
         * The intensity of the light source.
         */
        GUA_ADD_PROPERTY(float, brightness, 10.0f);

        /**
         * The exponent of distance attenuation.
         * E.g. a value of 2 means quadratic falloff, 1 means linear falloff
         */
        GUA_ADD_PROPERTY(float, falloff, 1.f);

        /**
         * The exponent of radial attenuation.
         * E.g. a value of 2 means quadratic falloff, 1 means linear falloff
         */
        GUA_ADD_PROPERTY(float, softness, 0.5f);

        /**
         * Triggers whether the light casts shadows.
         */
        GUA_ADD_PROPERTY(bool, enable_shadows, false);

        /**
         * Triggers volumetric screen-space effects for the light source.
         */
        GUA_ADD_PROPERTY(bool, enable_godrays, false);

        /**
         * Triggers whether or not the light source has influence on objects'
         * diffuse shading.
         */
        GUA_ADD_PROPERTY(bool, enable_diffuse_shading, true);

        /**
         * Triggers whether or not the light source has influence on objects'
         * specular shading.
         */
        GUA_ADD_PROPERTY(bool, enable_specular_shading, true);

        /**
         * Sets the size in pixel of the texture used for shadow map generation.
         * Choose wisely!
         */
        GUA_ADD_PROPERTY(unsigned, shadow_map_size, 512);

        /**
         * Maximum shadow distance. The shadow will fade to the given distance
         * and no shadow will be calculated beyond. Use zero to disable this
         * feature.
         */
        GUA_ADD_PROPERTY(float, max_shadow_dist, 100.f);

        /**
         * Sets the offset between a shadow casting object and the shadow's edge.
         * Increase this value to reduce artefacts (especially shadow acne).
         */
        GUA_ADD_PROPERTY(float, shadow_offset, 0.001f);

        /**
         * Sets the split distance-to-camera values for rendering cascaded shadow
         * maps in world coordinates. Only available for sun lights.
         */
        GUA_ADD_PROPERTY(std::vector<float>, shadow_cascaded_splits, std::vector<float>({0.1f, 2, 10, 50}));

        /**
         * Sets the value used for additional near and far clipping when renering
         * cascaded shadow maps. Only available for sun lights. If both are set to
         * zero, only objects wchich are visible in the viewing frustum will cast
         * shadows.
         */
        GUA_ADD_PROPERTY(float, shadow_near_clipping_in_sun_direction, 1.f);
        GUA_ADD_PROPERTY(float, shadow_far_clipping_in_sun_direction, 100.f);
    };

    /**
     * The LightNode's configuration.
     */
    Configuration data;

    /**
     * Constructor.
     *
     * This constructs an empty LightNode.
     *
     */
    LightNode() {}

    /**
     * Constructor.
     *
     * This constructs a LightNode with the given parameters.
     *
     * \param name           The name of the new LightNode.
     * \param configuration  A configuration struct to define the LightNode's
     *                       properties.
     * \param transform      A matrix to describe the LightNode's
     *                       transformation.
     */
    LightNode(std::string const& name, Configuration const& configuration = Configuration(), math::mat4 const& transform = math::mat4::identity());

    /**
     * Accepts a visitor and calls concrete visit method.
     *
     * This method implements the visitor pattern for Nodes.
     *
     * \param visitor  A visitor to process the LightNode's data.
     */
    void accept(NodeVisitor& visitor) override;

    /**
     * Updates a LightNode's BoundingBox.
     *
     * The bounding box is updated according to the transformation matrices of
     * all children.
     */
    void update_bounding_box() const override;

    inline virtual std::string get_type_string() const {return "<LightNode>";}
  private:
    std::shared_ptr<Node> copy() const override;
};

} // namespace node
} // namespace gua

#endif // GUA_POINT_LIGHT_NODE_HPP
