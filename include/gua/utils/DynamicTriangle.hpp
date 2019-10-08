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

#ifndef GUA_DYNAMIC_TRIANGLE_HPP
#define GUA_DYNAMIC_TRIANGLE_HPP

// guacamole headers
#include <gua/config.hpp>
#include <gua/platform.hpp>

#include <gua/utils/DynamicGeometry.hpp>
#include <gua/utils/DynamicGeometryImporter.hpp>

// external headers
#include <scm/gl_core.h>
#include <scm/core/math/quat.h>

#include <mutex>
#include <vector>

namespace gua
{
/**
 * @brief holds vertex information of one mesh
 */
struct GUA_DLL DynamicTriangle : DynamicGeometry
{
  public:
    // create empty geometry for dynamic usage
    DynamicTriangle(unsigned int initial_geometry_buffer_size = 1000);

    DynamicTriangle(DynamicGeometryObject const& dynamic_geometry_object);

    /**
     * @brief holds information of a vertex
     */
    struct TriVertex : public DynamicGeometry::Vertex
    {
        TriVertex(
            float x = 0.0f, float y = 0.0f, float z = 0.0f, float col_r = 0.0f, float col_g = 0.0f, float col_b = 0.0f, float col_a = 1.0f, float thickness = 1.0f, float u = 1.0f, float v = 1.0f)
            : Vertex(x, y, z, col_r, col_g, col_b, col_a, thickness), uv(u, v)
        {
        }

        bool operator==(TriVertex const& rhs)
        {
            if(pos[0] == rhs.pos[0] && pos[1] == rhs.pos[1] && pos[2] == rhs.pos[2] && col[0] == rhs.col[0] && col[1] == rhs.col[1] && col[2] == rhs.col[2] && col[3] == rhs.col[3] &&
               thick == rhs.thick && uv[0] == rhs.uv[0] && uv[1] == rhs.uv[1])
            {
                return true;
            }

            return false;
        }

        bool operator!=(TriVertex const& rhs) { return !((*this) == rhs); }

        scm::math::vec2f uv;
    };

    void compute_consistent_normals() const;

    void compile_buffer_string(std::string& buffer_string) override;
    void uncompile_buffer_string(std::string const& buffer_string) override;

    bool push_vertex(TriVertex const& v_to_push);
    bool update_vertex(int vertex_idx, TriVertex const& v_to_update);

    bool pop_back_vertex() override;
    bool pop_front_vertex() override;
    bool clear_vertices() override;

    void forward_queued_vertices(std::vector<scm::math::vec3f> const& queued_positions,
                                 std::vector<scm::math::vec4f> const& queued_colors,
                                 std::vector<float> const& queued_thicknesses,
                                 std::vector<scm::math::vec2f> const& queued_uv);

    /**
     * @brief writes vertex info to given buffer
     *
     * @param vertex_buffer buffer to write to
     */
    void copy_to_buffer(TriVertex* vertex_buffer) const;

    /**
     * @brief returns vertex layout for mesh vertex
     * @return schism vertex format
     */
    scm::gl::vertex_format get_vertex_format() const override;

    mutable std::vector<scm::math::vec2f> uvs;

  protected:
    void enlarge_reservoirs();
};

} // namespace gua

#endif // GUA_DYNAMIC_TRIANGLE_HPP
