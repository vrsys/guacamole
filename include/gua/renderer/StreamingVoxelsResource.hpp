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

#ifndef GUA_STREAMING_VOXELS_RESOURCE_HPP
#define GUA_STREAMING_VOXELS_RESOURCE_HPP

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





class StreamingVoxelsResource : public LineStripResource {
 public:

  /**
   * Default constructor.
   *
   * Creates a new and empty Line Strip.
   */
   StreamingVoxelsResource();
   ~StreamingVoxelsResource();

   //creates a linestrip resource which can be updated over the network
   StreamingVoxelsResource(uint16_t recv_socket_port, std::string const& feedback_ip = "", uint16_t feedback_port = 0);



  /**
   * Draws the line strip.
   *
   * Draws the line strip to the given context.
   *
   * \param context          The RenderContext to draw onto.
   */
  //void draw(RenderContext& context) const;
  void draw(RenderContext& context, bool render_vertices_as_points) const override;

  void issue_buffer_swap() const;
/*
  void ray_test(Ray const& ray, int options,
                node::Node* owner, std::set<PickResult>& hits) override;
*/
  void convert_per_thread(unsigned const voxel_recv, unsigned char const* buff, unsigned const tid);
  void receive_streaming_update();

  
  //math::vec3 get_vertex(unsigned int i) const;

  bool get_is_net_node() const;
  //float get_voxel_thickness() const;

  void set_desired_voxel_thickness(float des_vox_thick);
  float  get_desired_voxel_thickness() const;
/*
  void push_vertex(float pos_x, float pos_y, float pos_z,
                   float col_r, float col_g, float col_b, float col_a,
                   float thickness);
  void pop_vertex();
*/
 private:

  //void upload_to(RenderContext& context) const;
  void upload_front_buffer_to(RenderContext& ctx) const;
/*
  KDTree kd_tree_;
  LineStrip line_strip_;
  bool gpu_resource_rewrite_needed_ = true;
  bool gpu_resource_resize_needed_ = true;
*/

  //the following variables are associated with the streaming version of the
  //point and line geometry
  bool thread_needs_to_be_joined_ = false;
  bool is_net_node_ = false;
  bool is_feedback_port_open_ = false;
  std::shared_ptr<std::thread>   socket_receiving_thread_ptr_;
  std::shared_ptr<zmq::context_t> zmq_context_ptr_;
  std::shared_ptr<zmq::socket_t>  zmq_receive_socket_ptr_;
  std::shared_ptr<zmq::socket_t>  zmq_feedback_sender_socket_ptr_;
  mutable std::vector<streaming_voxel> socket_to_cpu_buffer_; //back buffer
  mutable std::vector<streaming_voxel> cpu_to_gpu_buffer_; //front buffer
  bool is_resource_alive_ = false;
  mutable std::atomic<bool> needs_double_buffer_swap_{false};
  //mutable bool needs_double_buffer_swap = false;
  uint64_t currently_written_voxels_back_ = 0;
  uint64_t currently_available_voxels_front_ = 0;
  mutable boost::mutex buffer_swap_mutex_;

  float voxel_thickness_ = 0.0;
  float desired_voxel_thickness_ = 0.008;

  struct voxel_packet_header {
    unsigned const max_voxels_per_packet_ = 6000;
    unsigned const byte_per_voxel_ = 10;
    unsigned const byte_of_header_ = 48;
    unsigned const max_bufflen_ = max_voxels_per_packet_ * byte_per_voxel_ + byte_of_header_;
    unsigned const max_num_packets_ = 16;
  };

  voxel_packet_header voxel_packet_header_;
};

}

#endif  // GUA_STREAMING_VOXELS_RESOURCE_HPP