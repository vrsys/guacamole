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
#include <gua/renderer/LineStripResource.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/node/LineStripNode.hpp>
#include <gua/utils/Logger.hpp>

#include <scm/gl_core/constants.h>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

LineStripResource::LineStripResource()
    : kd_tree_(), line_strip_() {}

////////////////////////////////////////////////////////////////////////////////

LineStripResource::~LineStripResource() {
  std::cout << "DESTRUKTOR IS CALLED\n";

  is_resource_alive = false;

  if(thread_needs_to_be_joined) {
    socket_receiving_thread_ptr->join();
  }
}

////////////////////////////////////////////////////////////////////////////////

   //creates a linestrip resource which can be updated over the network
LineStripResource::LineStripResource(uint16_t recv_socket_port) {
  zmq_context_ptr = std::make_shared<zmq::context_t>(1);
  std::string specified_receiver_port = std::to_string(recv_socket_port);
  std::string receiver_socket_string = "tcp://127.0.0.1:" + specified_receiver_port;
  std::cout << "OPENING SOCKET WITH THE FOLLOWING ADDRESS: " << receiver_socket_string << "\n";
  zmq_receive_socket_ptr = std::make_shared<zmq::socket_t>( *zmq_context_ptr, ZMQ_SUB );
  zmq_receive_socket_ptr->connect(receiver_socket_string);
  const char* filter = "";
  zmq_receive_socket_ptr->setsockopt(ZMQ_SUBSCRIBE, filter, strlen(filter));

  thread_needs_to_be_joined = true;
  is_net_node = true;
  is_resource_alive = true;
  socket_receiving_thread_ptr = std::make_shared<std::thread>(&LineStripResource::receive_streaming_update, this);

}

void LineStripResource::convert_per_thread(unsigned const voxel_recv, unsigned char const* buff, unsigned const tid = 0) {
  
  std::cout << "CONVERT PER THREAD CALL\n";
  for(unsigned v = 0; v < voxel_recv; ++v){
    unsigned char r;
    unsigned char g;
    unsigned char b;
  
    unsigned short x;
    unsigned short y;
    unsigned short z;
  
    streaming_voxel new_voxel_to_create;

    uint8_t size_of_unsigned_short = sizeof(unsigned short);
    uint8_t size_of_unsigned_char = sizeof(unsigned char);

    // copy "voxel into buff"
    size_t buff_index = v * 10 + byte_of_header;
    memcpy(&new_voxel_to_create.qz_pos[0], &buff[buff_index], size_of_unsigned_short);
    buff_index += size_of_unsigned_short;
    memcpy(&new_voxel_to_create.qz_pos[0], &buff[buff_index], size_of_unsigned_short);
    buff_index += size_of_unsigned_short;
    memcpy(&new_voxel_to_create.qz_pos[0], &buff[buff_index], size_of_unsigned_short);
    buff_index += size_of_unsigned_short;
    
    memcpy(&new_voxel_to_create.color[0], &buff[buff_index], size_of_unsigned_char);
    buff_index += size_of_unsigned_char;
    memcpy(&new_voxel_to_create.color[1], &buff[buff_index], size_of_unsigned_char);
    buff_index += size_of_unsigned_char;
    memcpy(&new_voxel_to_create.color[2], &buff[buff_index], size_of_unsigned_char);
    buff_index += size_of_unsigned_char;
    
    unsigned char joint_id = 0; // not used but needed by protocol
    memcpy(&joint_id, &buff[buff_index], size_of_unsigned_char);
    buff_index += size_of_unsigned_char;
    
/*
    voxel.pos[0] = voxelsize_sent * x + bbx_min[0];
    voxel.pos[1] = voxelsize_sent * y + bbx_min[1];
    voxel.pos[2] = voxelsize_sent * z + bbx_min[2];
    
    voxel.rgb[0] = (1.0/255.0) * r;
    voxel.rgb[1] = (1.0/255.0) * g;
    voxel.rgb[2] = (1.0/255.0) * b;
*/
    //leave everything quantized and unnormalized
    socket_to_cpu_buffer.emplace_back(new_voxel_to_create);
    /*
    if(use_a){
      voxel_data[tid].emplace_back(voxel);
    }
    else{
      voxel_datab[tid].emplace_back(voxel);
    }
    */
  }
  
}

void LineStripResource::receive_streaming_update() {

  while(is_resource_alive) {
  size_t counter = 0;

    while(needs_double_buffer_swap.load()) {
      //std::cout << "waiting " << ++counter << " \n";
      ;
    }


    //std::cout << "start recv" << std::endl;

    size_t time_stamp = 0;
    unsigned voxel_count = 0;
    
    unsigned char temp_streambuff[max_bufflen];
    unsigned packet_number = 0;
    unsigned voxel_number = 0;
    unsigned number_of_packets = 0;
    unsigned number_of_packets_processed = 0;
    size_t current_timestamp = 0;

    size_t num_byte_of_header = 40;

    double voxelsize_sent = 0.008;

//    if (line_strip_.num_occupied_vertex_slots > 0) {
//    bounding_box_ = math::BoundingBox<math::vec3>();

    //for (int v(0); v < line_strip_.num_occupied_vertex_slots; ++v) {

    //}


    while (true) {
      //std::cout << "LOOPING\n";
      zmq::message_t request;
      zmq_receive_socket_ptr->recv(&request);

      size_t bytes_received = request.size();
      size_t received_voxel_bytes = bytes_received - num_byte_of_header;

      memcpy((void*) &temp_streambuff[0], (const void*) request.data(), received_voxel_bytes);
    


    //std::cout << "MESSAGE OF SIZE " << bytes_received << " received\n";

        //std::cout << "bytes_received: " << bytes_received << std::endl;
      if(bytes_received > num_byte_of_header) {
        size_t buff_index = 0;
        memcpy(&voxelsize_sent, &temp_streambuff[currently_written_voxels_back + buff_index], sizeof(voxelsize_sent));
        voxelsize_sent = 0.008;
        buff_index += sizeof(voxelsize_sent);
        memcpy(&packet_number, &temp_streambuff[buff_index], sizeof(packet_number));
        buff_index += sizeof(packet_number);
        memcpy(&voxel_count, &temp_streambuff[buff_index], sizeof(voxel_count));
        buff_index += sizeof(voxel_count);

        math::vec3 bbx_min, bbx_max;

        float bb_min[3]{0.0, 0.0, 0.0};
        float bb_max[3]{0.0, 0.0, 0.0};
        memcpy(&bb_min[0], &temp_streambuff[buff_index], sizeof(float));
        buff_index += sizeof(float);
        memcpy(&bb_min[1], &temp_streambuff[buff_index], sizeof(float));
        buff_index += sizeof(float);
        memcpy(&bb_min[2], &temp_streambuff[buff_index], sizeof(float));
        buff_index += sizeof(float);
        memcpy(&bb_max[0], &temp_streambuff[buff_index], sizeof(float));
        buff_index += sizeof(float);
        memcpy(&bb_max[1], &temp_streambuff[buff_index], sizeof(float));
        buff_index += sizeof(float);
        memcpy(&bb_max[2], &temp_streambuff[buff_index], sizeof(float));
        buff_index += sizeof(float);

        std::cout << "RECEIVED BB MIN: " << bb_min[0] << ", " << bb_min[1] << ", " << bb_min[2] << "\n";
        std::cout << "RECEIVED BB MAX: " << bb_max[0] << ", " << bb_max[1] << ", " << bb_max[2] << "\n";

        bounding_box_ = math::BoundingBox<math::vec3>();

        bounding_box_.expandBy(bbx_min);
        bounding_box_.expandBy(bbx_max);

        if(packet_number == 0){
          number_of_packets = (unsigned) std::ceil( (1.0f * voxel_count) / max_voxels_per_packet);
          std::cout << number_of_packets << std::endl;
          current_timestamp = time_stamp;

          for(unsigned pid = 0; pid != max_num_packets; ++pid){
            socket_to_cpu_buffer.clear();
          }
          number_of_packets_processed = 0;
        }
        
        unsigned const voxel_recv = (bytes_received - byte_of_header)/byte_per_voxel;

        std::cout << "Voxel count: " << voxel_count << "\n";
        convert_per_thread(voxel_recv, temp_streambuff);
        std::cout << "After convert per thread\n";
        //replace
        //threadGroup.create_thread(boost::bind(convert_per_thread, voxel_recv, number_of_packets_processed, buff));

        ++number_of_packets_processed;
        std::cout << packet_number << " -> " << number_of_packets_processed << " of " << number_of_packets << std::endl;
        if(number_of_packets_processed == number_of_packets){
          break;
        }

      } else {
        break;
      }


  /*
    std::string message = std::string(static_cast<char*>(request.data()), request.size());

    std::cout << "Message received! ("<< message.size() << " bytes)" << std::endl;
    //std::cout << message << std::endl;
  */
  }

    //threadGroup.join_all();
  //std::cout << "end recv" << std::endl;

//    std::cout << "sending feedback, voxelsize_want: " << voxelsize_want << std::endl;
//    g_send->send(&voxelsize_want, sizeof(voxelsize_want));

  needs_double_buffer_swap.store(true);
}

  std::cout << "Jaaa you know I died\n";

  

}

//LineStripResource::update_from_stream() {
//}
////////////////////////////////////////////////////////////////////////////////


LineStripResource::LineStripResource(LineStrip const& line_strip, bool build_kd_tree)
    : kd_tree_(), line_strip_(line_strip) {
  if (line_strip_.num_occupied_vertex_slots > 0) {
    bounding_box_ = math::BoundingBox<math::vec3>();

    for (int v(0); v < line_strip_.num_occupied_vertex_slots; ++v) {
      bounding_box_.expandBy(math::vec3{line_strip_.positions[v]});
    }

    if (build_kd_tree) {
      //kd_tree_.generate(line_strip);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////

void LineStripResource::upload_to(RenderContext& ctx) const {
  RenderContext::LineStrip clinestrip{};
  clinestrip.vertex_topology = scm::gl::PRIMITIVE_LINE_STRIP_ADJACENCY;
  clinestrip.vertex_reservoir_size = line_strip_.vertex_reservoir_size;
  clinestrip.num_occupied_vertex_slots = line_strip_.num_occupied_vertex_slots;


  if (line_strip_.vertex_reservoir_size == 0) {
    Logger::LOG_WARNING << "Unable to load LineStrip! Has no vertex data." << std::endl;
    return;
  }

 
  clinestrip.vertices =
      ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                                       scm::gl::USAGE_DYNAMIC_DRAW,
                                       (line_strip_.vertex_reservoir_size+3) * sizeof(LineStrip::Vertex),
                                       0);

  LineStrip::Vertex* data(static_cast<LineStrip::Vertex*>(ctx.render_context->map_buffer(
      clinestrip.vertices, scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER)));

  line_strip_.copy_to_buffer(data);

  //std::cout << buffer_content << ""
  ctx.render_context->unmap_buffer(clinestrip.vertices);

  clinestrip.vertex_array = ctx.render_device->create_vertex_array(
      line_strip_.get_vertex_format(),
      {clinestrip.vertices});
  ctx.line_strips[uuid()] = clinestrip;

  ctx.render_context->apply();
}

////////////////////////////////////////////////////////////////////////////////

void LineStripResource::upload_front_buffer_to(RenderContext& ctx) const {

 
  uint32_t num_vertices_to_upload = cpu_to_gpu_buffer.size();

  RenderContext::LineStrip clinestrip{};

  clinestrip.vertices =
      ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                                       scm::gl::USAGE_STREAM_DRAW,
                                       num_vertices_to_upload * sizeof(streaming_voxel),
                                       &cpu_to_gpu_buffer[0]);

  clinestrip.num_occupied_vertex_slots = num_vertices_to_upload;

//  LineStrip::Vertex* data(static_cast<LineStrip::Vertex*>(ctx.render_context->map_buffer(
//      clinestrip.vertices, scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER)));

//  line_strip_.copy_to_buffer(data);

  //std::cout << buffer_content << ""
//  ctx.render_context->unmap_buffer(clinestrip.vertices);

  clinestrip.vertex_array = ctx.render_device->create_vertex_array(
      line_strip_.get_streaming_vertex_format(),
      {clinestrip.vertices});
  ctx.line_strips[uuid()] = clinestrip;

  ctx.render_context->apply();
}

////////////////////////////////////////////////////////////////////////////////

void LineStripResource::draw(RenderContext& ctx) const {
  //DUMMY
}


////////////////////////////////////////////////////////////////////////////////

void LineStripResource::draw(RenderContext& ctx, bool render_vertices_as_points) const {
  
  if(!is_net_node) {

    auto iter = ctx.line_strips.find(uuid());
    if (iter == ctx.line_strips.end()) {
      // upload to GPU if neccessary
      upload_to(ctx);
      iter = ctx.line_strips.find(uuid());
    }
  


    ctx.render_context->bind_vertex_array(iter->second.vertex_array);
    //ctx.render_context->bind_index_buffer(iter->second.indices, iter->second.indices_topology, iter->second.indices_type);
    ctx.render_context->apply_vertex_input();
  
    if(!render_vertices_as_points) {
      //ctx.render_context->draw_arrays(scm::gl::PRIMITIVE_LINE_LOOP, 0, iter->second.num_occupied_vertex_slots+2);
      ctx.render_context->draw_arrays(iter->second.vertex_topology, 0, iter->second.num_occupied_vertex_slots+3);
    } else {
      ctx.render_context->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, 1, iter->second.num_occupied_vertex_slots);
    }
  } else {
    //std::cout << "WOULD RENDER NET NODE RESOURCE\n";

    //std::cout << "Same.\n";

    std::cout << "Before swap: " << "Front size: " << socket_to_cpu_buffer.size() << "    Back Size: " << cpu_to_gpu_buffer.size() << "\n";
    issue_buffer_swap();

    auto iter = ctx.line_strips.find(uuid());
    if (iter == ctx.line_strips.end()) {
      // upload to GPU if neccessary
      upload_front_buffer_to(ctx);
      iter = ctx.line_strips.find(uuid());
    }


    ctx.render_context->bind_vertex_array(iter->second.vertex_array);
    //ctx.render_context->bind_index_buffer(iter->second.indices, iter->second.indices_topology, iter->second.indices_type);
    ctx.render_context->apply_vertex_input();
  /*
    if(!render_vertices_as_points) {
      //ctx.render_context->draw_arrays(scm::gl::PRIMITIVE_LINE_LOOP, 0, iter->second.num_occupied_vertex_slots+2);
      ctx.render_context->draw_arrays(iter->second.vertex_topology, 0, iter->second.num_occupied_vertex_slots+3);
    } else {*/
      ctx.render_context->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, 0, iter->second.num_occupied_vertex_slots);
    //}

    std::cout << "Swap.\n";
    std::cout << "After swap: " << "Front size: " << socket_to_cpu_buffer.size() << "    Back Size: " << cpu_to_gpu_buffer.size() << "\n";



    std::cout << "ESTABLISHED BOUNDING BOX: " << bounding_box_.min << "\n";
  }
}


/////////////////////////////////////////////////

void LineStripResource::issue_buffer_swap() const {
  if(needs_double_buffer_swap.load()){
    std::swap(socket_to_cpu_buffer, cpu_to_gpu_buffer);
    needs_double_buffer_swap.store(false);


    //voxelsize_want = voxelsize;
  }
  
}

////////////////////////////////////////////////////////////////////////////////


void LineStripResource::ray_test(Ray const& ray, int options,
                    node::Node* owner, std::set<PickResult>& hits) {

  //kd_tree_.ray_test(ray, line_strip_, options, owner, hits);
}

////////////////////////////////////////////////////////////////////////////////

math::vec3 LineStripResource::get_vertex(unsigned int i) const {
  return math::vec3(
      line_strip_.positions[i].x, line_strip_.positions[i].y, line_strip_.positions[i].z);
}

////////////////////////////////////////////////////////////////////////////////

}
