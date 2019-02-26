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

#ifndef GUA_CUBEMAP_NODE
#define GUA_CUBEMAP_NODE

#include <gua/platform.hpp>
#include <gua/node/SerializableNode.hpp>
#include <gua/utils/configuration_macro.hpp>

#include <string>
#include <atomic>

namespace gua
{
namespace node
{
/**
 * This class is used to represent an ------- node in the SceneGraph.
 *
 * \ingroup gua_scenegraph
 */
class GUA_DLL CubemapNode : public SerializableNode
{
  public:
    enum class RenderMode
    {
        COMPLETE = 0,
        ONE_SIDE_PER_FRAME = 1
    };

    struct Configuration
    {
        /**
         * Sets wether for this node is a depth map created or not
         */
        GUA_ADD_PROPERTY(bool, active, true);

        /**
         * Sets the size in pixel of each side of the texture used for depth cube map generation.
         */
        GUA_ADD_PROPERTY(unsigned, resolution, 64);

        /**
         * near clipping distance for depth value generation
         */
        GUA_ADD_PROPERTY(float, near_clip, 0.1f);

        /**
         * far clipping distance for depth value generation
         */
        GUA_ADD_PROPERTY(float, far_clip, 10.f);

        /**
         * render mode for depth calculation
         * one side per frame improves performance, but the data gets up to 5 frames old
         * 0 - COMPLETE, 1 - ONE_SIDE_PER_FRAME
         */
        GUA_ADD_PROPERTY(RenderMode, render_mode, RenderMode::COMPLETE);

        /**
         * Name of the depth texture in the texture database
         */
        GUA_ADD_PROPERTY(std::string, texture_name, "depth_map");

        // a user-defined view id, can be used to customize material parameters for
        // objects rendered by this camera
        GUA_ADD_PROPERTY(int, view_id, 0);

        // limits the rendered object to a set defined by the mask
        GUA_ADD_PROPERTY(Mask, mask, Mask());
    };

    struct Distance_Info
    {
      public:
        float distance;
        math::vec2ui tex_coords;
        math::vec3 world_position;
        Distance_Info() : distance(-1.0), tex_coords(0, 0), world_position(0.f, 0.f, 0.f) {}
    };

    /**
     * The CubemapNode's configuration.
     */
    Configuration config;

    /**
     * Constructor.
     *
     * This constructs an empty CubemapNode.
     *
     */
    CubemapNode(){};

    /**
     * Constructor.
     *
     * This constructs a CubemapNode with the given parameters.
     *
     * \param name           The name of the new CubemapNode.
     * \param configuration  A configuration struct to define the CubemapNodes's
     *                       properties.
     * \param transform      A matrix to describe the CubemapNode's
     *                       transformation.
     */
    CubemapNode(std::string const& name, Configuration const& configuration = Configuration(), math::mat4 const& transform = math::mat4::identity());

    /**
     * Accepts a visitor and calls concrete visit method.
     *
     * This method implements the visitor pattern for Nodes.
     *
     * \param visitor  A visitor to process the CubemapNode's data.
     */
    void accept(NodeVisitor& visitor) override;
    void update_bounding_box() const override;

    float get_min_distance();
    float get_weighted_min_distance();
    math::vec3 get_min_distance_position();
    float get_distance_by_local_direction(math::vec3 const& dir) const;

    void create_weights(math::vec3 const& view_direction, math::vec3 const& move_direction);

    math::vec3 get_push_back(float radius, float softness);
    math::vec3 get_pull_in(float inner_radius, float outer_radius, float softness);

    std::shared_ptr<std::atomic<bool>> m_NewTextureData;

  private:
    Distance_Info m_MinDistance;
    Distance_Info m_WeightedMinDistance;

    std::vector<float> m_Weights;
    std::vector<float> m_DistortionWeights;

    void find_min_distance();
    math::vec3 calculate_direction_from_tex_coords(math::vec2ui const& tex_coords) const;
    math::vec3 project_back_to_world_coords(Distance_Info const& di) const;
    float acces_texture_data(unsigned side, math::vec2 coords) const;

    void create_distortion_weights();

    std::shared_ptr<Node> copy() const override;
};

} // namespace node
} // namespace gua

#endif // GUA_CUBEMAP_NODE
