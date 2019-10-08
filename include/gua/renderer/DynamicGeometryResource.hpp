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

#ifndef GUA_DYNAMIC_GEOMETRY_RESOURCE_HPP
#define GUA_DYNAMIC_GEOMETRY_RESOURCE_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/GeometryResource.hpp>
#include <gua/utils/DynamicGeometry.hpp>
#include <gua/utils/KDTree.hpp>

// external headers
#include <scm/gl_core.h>

#include <mutex>
#include <thread>

#include <vector>

namespace gua
{
struct RenderContext;
class DynamicGeometryResource;

/**
 * Stores geometry data.
 *
 * A dynamic geometry can be loaded from an *.lob file and the draw onto multiple
 * contexts.
 * Do not use this class directly, it is just used by the Geometry class to
 * store the individual meshes of a file.
 */

class DynamicGeometryResource : public GeometryResource
{
  public:
    /**
     * Default constructor.
     *
     * Creates a new and empty DynamicGeometryResource.
     */
    DynamicGeometryResource(scm::gl::primitive_topology vertex_rendering_mode);

    /**
     * Constructor from an *.lob strip.
     *
     * Initializes the strip from a given *.lob strip.
     *
     * \param mesh             The dynamic geometry to load the data from.
     */
    DynamicGeometryResource(std::shared_ptr<DynamicGeometry> dynamic_geometry_ptr, bool build_kd_tree, scm::gl::primitive_topology vertex_rendering_mode);

    // TODO conrstuct with

    /**
     * Draws the dynamic geometry.
     *
     * Draws the dynamic geometry to the given context.
     *
     * \param context          The RenderContext to draw onto.
     */
    virtual void draw(RenderContext& context) const = 0;

    void ray_test(Ray const& ray, int options, node::Node* owner, std::set<PickResult>& hits) override;

    // void resolve_vertex_updates(RenderContext& ctx);

    void make_clean_flags_dirty();

    void compute_bounding_box();

    inline unsigned int num_occupied_vertex_slots() const { return dynamic_geometry_ptr_->num_occupied_vertex_slots; }
    inline unsigned int vertex_reservoir_size() const { return dynamic_geometry_ptr_->vertex_reservoir_size; }

    void compute_consistent_normals() const;

    virtual void compile_buffer_string(std::string& buffer_string) = 0;
    virtual void uncompile_buffer_string(std::string const& buffer_string) = 0;

    void push_vertex(DynamicGeometry::Vertex const& in_vertex);
    void pop_front_vertex();
    void pop_back_vertex();

    void clear_vertices();

    // ephra
    void set_vertex_rendering_mode(scm::gl::primitive_topology const& render_mode);

    void forward_queued_vertices(std::vector<scm::math::vec3f> const& queued_positions,
                                 std::vector<scm::math::vec4f> const& queued_colors,
                                 std::vector<float> const& queued_thicknesses //,
                                 // std::vector<scm::math::vec3f> const& queued_normals
    );

    math::vec3 get_vertex(unsigned int i) const;

  protected:
    KDTree kd_tree_;
    mutable std::mutex dynamic_geometry_update_mutex_;
    scm::gl::primitive_topology vertex_rendering_mode_; //= scm::gl::PRIMITIVE_LINE_STRIP_ADJACENCY;
    //  alternativ muss es zu scm::gl::PRIMITIVE_LINE_LIST gesetzt werde koennen

    mutable std::map<unsigned, bool> clean_flags_per_context_;
    std::shared_ptr<DynamicGeometry> dynamic_geometry_ptr_;

  private:
    virtual void upload_to(RenderContext& context) const;

    // ephra

    // std::vector<dynamic_geometry_update_job*> dynamic_geometry_update_queue_;
};

} // namespace gua

#endif // GUA_DYNAMIC_GEOMETRY_RESOURCE_HPP
