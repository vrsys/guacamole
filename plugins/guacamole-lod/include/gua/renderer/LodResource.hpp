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

#ifndef GUA_LOD_RESOURCE_HPP
#define GUA_LOD_RESOURCE_HPP

// guacamole headers
#include <gua/renderer/Lod.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/GeometryResource.hpp>
#include <gua/utils/KDTree.hpp>

// external headers
#include <scm/core/math.h>

#include <scm/gl_core/gl_core_fwd.h>
#include <scm/gl_core/data_types.h>
#include <scm/gl_core/state_objects.h>

#include <scm/gl_util/primitives/primitives_fwd.h>
#include <scm/gl_util/primitives/geometry.h>

#include <scm/core/platform/platform.h>
#include <scm/core/utilities/platform_warning_disable.h>

#include <lamure/types.h>
#include <lamure/ren/model_database.h>
#include <lamure/ren/cut_database.h>
#include <lamure/ren/cut.h>
#include <lamure/ren/dataset.h>

namespace gua
{
namespace node
{
class PLodNode;
class MLodNode;
}; // namespace node

/**
 * Stores a point cloud model with Lod.
 *
 * This class simply a wrapper for accessing models of PBR library
 */
class GUA_LOD_DLL LodResource : public GeometryResource
{
  public: // c'tor /d'tor
    LodResource(lamure::model_t model_id, bool is_pickable, math::mat4 const& local_transform);

    ~LodResource();

  public: // methods
    /*virtual*/ void draw(RenderContext const& context) const;

    /**
     * Draws the point cloud.
     *
     * Draws the point cloud to the given context.
     *
     * \param context  The RenderContext to draw onto.
     */
    void draw(RenderContext const& ctx,
              lamure::context_t context_id,
              lamure::view_t view_id,
              lamure::model_t model_id,
              scm::gl::vertex_array_ptr const& vertex_array,
              std::unordered_set<lamure::node_t> const& nodes_in_frustum,
              scm::gl::primitive_topology const,
              scm::math::mat4d model_view_matrix = math::mat4d(),
              bool draw_sorted = false) const;

    void draw_instanced(uint32_t instance_count, RenderContext const& ctx,
                        lamure::context_t context_id,
                        lamure::view_t view_id,
                        lamure::model_t model_id,
                        scm::gl::vertex_array_ptr const& vertex_array,
                        std::unordered_set<lamure::node_t> const& nodes_in_frustum,
                        scm::gl::primitive_topology const type,
                        scm::math::mat4d model_view_matrix = math::mat4d(),
                        bool draw_sorted = false
                        ) const;

    math::mat4 const& local_transform() const;

    void ray_test(Ray const& ray, int options, node::Node* owner, std::set<PickResult>& hits);

  private:
    bool is_pickable_;
    lamure::model_t model_id_;
    math::mat4 local_transform_;

    //mutable istd::unordered_map<std::size_t, scm::gl::buffer_ptr> indirect_buffer_per_context_; 

    // data collection per context
    std::unordered_map<std::size_t, scm::gl::buffer_ptr > data_collection_per_context_;
};

} // namespace gua

#endif // GUA_LOD_RESSOURCE_HPP
