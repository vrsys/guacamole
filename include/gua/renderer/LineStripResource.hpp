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

#ifndef GUA_LINE_STRIP_RESOURCE_HPP
#define GUA_LINE_STRIP_RESOURCE_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/GeometryResource.hpp>
#include <gua/utils/LineStrip.hpp>
#include <gua/utils/KDTree.hpp>

// external headers
#include <scm/gl_core.h>

#include <mutex>
#include <thread>

#include <vector>

#include <zmq.hpp>

namespace gua {

struct RenderContext;

/**
 * Stores geometry data.
 *
 * A line strip can be loaded from an *.lob file and the draw onto multiple
 * contexts.
 * Do not use this class directly, it is just used by the Geometry class to
 * store the individual meshes of a file.
 */





class LineStripResource : public GeometryResource {
 public:

  /**
   * Default constructor.
   *
   * Creates a new and empty Line Strip.
   */
   LineStripResource();
   ~LineStripResource();

   //creates a linestrip resource which can be updated over the network
   LineStripResource(uint16_t recv_socket_port);


  /**
   * Constructor from an *.lob strip.
   *
   * Initializes the strip from a given *.lob strip.
   *
   * \param mesh             The line strip to load the data from.
   */
   LineStripResource(LineStrip const& line_strip, bool build_kd_tree);

  /**
   * Draws the line strip.
   *
   * Draws the line strip to the given context.
   *
   * \param context          The RenderContext to draw onto.
   */
  void draw(RenderContext& context) const;
  void draw(RenderContext& context, bool render_vertices_as_points) const;

  void issue_buffer_swap() const;

  void ray_test(Ray const& ray, int options,
                node::Node* owner, std::set<PickResult>& hits) override;

  void convert_per_thread(unsigned const voxel_recv, unsigned char const* buff, unsigned const tid);
  void receive_streaming_update();

  inline unsigned int num_occupied_vertex_slots() const { return line_strip_.num_occupied_vertex_slots; }
  inline unsigned int vertex_reservoir_size() const { return line_strip_.vertex_reservoir_size; }
  
  math::vec3 get_vertex(unsigned int i) const;

  bool get_is_net_node() const;

 private:

  void upload_to(RenderContext& context) const;
  void upload_front_buffer_to(RenderContext& ctx) const;

  KDTree kd_tree_;
  LineStrip line_strip_;

  //the following variables are associated with the streaming version of the
  //point and line geometry
  bool thread_needs_to_be_joined = false;
  bool is_net_node = false;
  std::shared_ptr<std::thread>   socket_receiving_thread_ptr;
  std::shared_ptr<zmq::context_t> zmq_context_ptr;
  std::shared_ptr<zmq::socket_t>  zmq_receive_socket_ptr;
  mutable std::vector<streaming_voxel> socket_to_cpu_buffer; //back buffer
  mutable std::vector<streaming_voxel> cpu_to_gpu_buffer; //front buffer
  bool is_resource_alive = false;
  mutable std::atomic<bool> needs_double_buffer_swap{false};
  //mutable bool needs_double_buffer_swap = false;
  uint64_t currently_written_voxels_back = 0;
  uint64_t currently_available_voxels_front = 0;
  mutable boost::mutex buffer_swap_mutex;

  unsigned const max_voxels_per_packet = 6000;
  unsigned const byte_per_voxel = 10;
  unsigned const byte_of_header = 40;
  unsigned const max_bufflen = max_voxels_per_packet * byte_per_voxel + byte_of_header;
  unsigned const max_num_packets = 16;
};

}

#endif  // GUA_LINE_STRIP_RESOURCE_HPP
