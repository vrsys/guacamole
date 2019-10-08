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

#ifndef GUA_LINE_STRIP_HPP
#define GUA_LINE_STRIP_HPP

// guacamole headers
#include <gua/config.hpp>
#include <gua/platform.hpp>

#include <gua/utils/LineStripImporter.hpp>

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
struct GUA_DLL LineStrip
{
  public:
    // create empty line strip for dynamic usage
    LineStrip(unsigned int initial_line_buffer_size = 1000);

    // convert representation of LineStripImporter to LineStrip object
    LineStrip(LineObject const& line_object);

    /**
     * @brief holds information of a vertex
     */
    struct Vertex
    {
        Vertex(float x = 0.0f,
               float y = 0.0f,
               float z = 0.0f,
               float col_r = 0.0f,
               float col_g = 0.0f,
               float col_b = 0.0f,
               float col_a = 1.0f,
               float thickness = 1.0f,
               float nor_x = 0.0f,
               float nor_y = 1.0f,
               float nor_z = 0.0f)
            : pos(x, y, z), col(col_r, col_g, col_b, col_a), thick(thickness), nor(nor_x, nor_y, nor_z)
        {
        }

        bool operator==(Vertex const& rhs)
        {
            if(pos[0] == rhs.pos[0] && pos[1] == rhs.pos[1] && pos[2] == rhs.pos[2] && col[0] == rhs.col[0] && col[1] == rhs.col[1] && col[2] == rhs.col[2] && col[3] == rhs.col[3] &&
               thick == rhs.thick && nor[0] == rhs.nor[0] && nor[1] == rhs.nor[1] && nor[2] == rhs.nor[2])
            {
                return true;
            }

            return false;
        }

        bool operator!=(Vertex const& rhs) { return !((*this) == rhs); }

        scm::math::vec3f pos;
        scm::math::vec4f col;
        float thick;
        scm::math::vec3f nor;
    };

    void compute_consistent_normals() const;

    void compile_buffer_string(std::string& buffer_string);
    void uncompile_buffer_string(std::string const& buffer_string);

    bool push_vertex(Vertex const& v_to_push);
    bool pop_back_vertex();
    bool pop_front_vertex();
    bool clear_vertices();

    void forward_queued_vertices(std::vector<scm::math::vec3f> const& queued_positions,
                                 std::vector<scm::math::vec4f> const& queued_colors,
                                 std::vector<float> const& queued_thicknesses,
                                 std::vector<scm::math::vec3f> const& queued_normals);

    /**
     * @brief writes vertex info to given buffer
     *
     * @param vertex_buffer buffer to write to
     */
    void copy_to_buffer(Vertex* vertex_buffer) const;

    /**
     * @brief returns vertex layout for mesh vertex
     * @return schism vertex format
     */
    virtual scm::gl::vertex_format get_vertex_format() const;

    std::vector<scm::math::vec3f> positions;
    std::vector<scm::math::vec4f> colors;
    std::vector<float> thicknesses;
    mutable std::vector<scm::math::vec3f> normals;

    int vertex_reservoir_size;
    int num_occupied_vertex_slots;

    std::map<unsigned, bool> gpu_dirty_flags_per_context_;

  protected:
    void enlarge_reservoirs();
};

} // namespace gua

#endif // GUA_LINE_STRIP_HPP
