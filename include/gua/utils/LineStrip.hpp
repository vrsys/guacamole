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

// external headers
#include <scm/gl_core.h>
#include <scm/core/math/quat.h>
#include <vector>

struct aiMesh;

namespace gua {

/**
 * @brief holds vertex information of one mesh
 */
struct GUA_DLL LineStrip {
 public:
  LineStrip(unsigned int initial_line_buffer_size = 0);


  /**
   * @brief holds information of a vertex
   */
  struct Vertex {
    scm::math::vec3f pos;
    scm::math::vec4f col;
    float            thick;
  };

  void push_vertex();
  void pop_vertex();

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

  unsigned int vertex_reservoir_size;
  unsigned int num_occupied_vertex_slots;

protected:
  void enlarge_reservoirs(unsigned int new_reservoir_size);
/*
 protected:


  std::vector<unsigned> construct(FbxMesh& mesh, int material_index);


  template<typename T>
  static std::function<unsigned(temp_vert const&)> get_access_function(FbxLayerElementTemplate<T> const& layer);
*/

};


}

#endif //GUA_LINE_STRIP_HPP
