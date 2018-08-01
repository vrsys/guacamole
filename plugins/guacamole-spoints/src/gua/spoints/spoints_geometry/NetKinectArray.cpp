#include <gua/spoints/spoints_geometry/NetKinectArray.hpp>
#include <gua/spoints/spoints_geometry/codec/point_types.h>

#include <boost/assign/list_of.hpp>


#include <zmq.hpp>
#include <iostream>
#include <mutex>

namespace spoints {

NetKinectArray::NetKinectArray(const std::string& server_endpoint,
                               const std::string& feedback_endpoint)
  : m_mutex_(),
    m_running_(true),
    //m_feedback_running_(true),
    m_server_endpoint_(server_endpoint),
    m_feedback_endpoint_(feedback_endpoint),
    m_buffer_(/*(m_colorsize_byte + m_depthsize_byte) * m_calib_files.size()*/),
    m_buffer_back_( /*(m_colorsize_byte + m_depthsize_byte) * m_calib_files.size()*/),
    m_need_cpu_swap_{false},
    //m_feedback_need_swap_{false},
    m_recv_()
{
 
  m_recv_ = std::thread([this]() { readloop(); });

  //m_send_feedback_ = std::thread([this]() {sendfeedbackloop();});

  //std::cout << "CREATED A NETKINECTARRAY!\n";
}

NetKinectArray::~NetKinectArray()
{
  m_running_ = false;
  m_recv_.join();
}

void 
NetKinectArray::draw_vertex_colored_points(gua::RenderContext const& ctx) {

  auto const& current_point_layout = point_layout_per_context_[ctx.id];

  if( current_point_layout != nullptr ) {
    scm::gl::context_vertex_input_guard vig(ctx.render_context);

    ctx.render_context->bind_vertex_array(current_point_layout);
    ctx.render_context->apply();
    
    size_t const& current_num_points_to_draw = num_vertex_colored_points_to_draw_per_context_[ctx.id];
    ctx.render_context->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST,
                                    0,
                                    current_num_points_to_draw);

    ctx.render_context->reset_vertex_input();
  }
}

void 
NetKinectArray::draw_vertex_colored_triangle_soup(gua::RenderContext const& ctx) {

  auto const& current_point_layout = point_layout_per_context_[ctx.id];

  if( current_point_layout != nullptr ) {
    scm::gl::context_vertex_input_guard vig(ctx.render_context);

    ctx.render_context->bind_vertex_array(current_point_layout);
    ctx.render_context->apply();
    
    size_t vertex_offset =   num_vertex_colored_points_to_draw_per_context_[ctx.id];

    size_t const& current_num_tri_vertices_to_draw = num_vertex_colored_tris_to_draw_per_context_[ctx.id] * 3;
    
    std::cout << "ABOUT TO DRAW NUM VERTICES: " << current_num_tri_vertices_to_draw << "\n";

    ctx.render_context->draw_arrays(scm::gl::PRIMITIVE_TRIANGLE_LIST,
                                    vertex_offset,
                                    current_num_tri_vertices_to_draw);

    ctx.render_context->reset_vertex_input();
  }
}

void 
NetKinectArray::draw_textured_triangle_soup(gua::RenderContext const& ctx) {

  auto const& current_point_layout = point_layout_per_context_[ctx.id];

  if( current_point_layout != nullptr ) {
    scm::gl::context_vertex_input_guard vig(ctx.render_context);

    ctx.render_context->bind_vertex_array(current_point_layout);
    ctx.render_context->apply();
    
    size_t vertex_offset =   num_vertex_colored_points_to_draw_per_context_[ctx.id]
                           + num_vertex_colored_tris_to_draw_per_context_[ctx.id] * 3;

    size_t const& current_num_tri_vertices_to_draw = num_textured_tris_to_draw_per_context_[ctx.id] * 3;

    ctx.render_context->draw_arrays(scm::gl::PRIMITIVE_TRIANGLE_LIST,
                                    vertex_offset,
                                    current_num_tri_vertices_to_draw);

    ctx.render_context->reset_vertex_input();
  }
}

std::string 
NetKinectArray::get_socket_string() const {
  return m_feedback_endpoint_;
};

void 
NetKinectArray::push_matrix_package(spoints::camera_matrix_package const& cam_mat_package) {
  submitted_camera_matrix_package_ = cam_mat_package;

  encountered_context_ids_for_feedback_frame_.insert(submitted_camera_matrix_package_.k_package.render_context_id);
}

bool
NetKinectArray::update(gua::RenderContext const& ctx, gua::math::BoundingBox<gua::math::vec3>& in_out_bb) {
  {
    auto& current_encountered_frame_count = encountered_frame_counts_per_context_[ctx.id];

    if (current_encountered_frame_count != ctx.framecount) {
      current_encountered_frame_count = ctx.framecount;
    } else {
    
      return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex_);
    if(m_need_cpu_swap_.load()){
      //start of synchro point
      m_buffer_.swap(m_buffer_back_);
      //std::swap(m_voxel_size_, m_voxel_size_back_);

      std::swap(m_received_vertex_colored_points_back_, m_received_vertex_colored_points_);
      std::swap(m_received_vertex_colored_tris_back_, m_received_vertex_colored_tris_);
      std::swap(m_received_textured_tris_back_, m_received_textured_tris_);



      //end of synchro point
      m_need_cpu_swap_.store(false);
    }

    if(m_need_gpu_swap_[ctx.id].load()) {
      size_t total_num_bytes_to_copy = m_buffer_.size();

      if(0 != total_num_bytes_to_copy) {

        //size_t sizeof_point = 4*sizeof(float);
        //size_t sizeof_point = 2*sizeof(uint32_t);

        size_t sizeof_vertex_colored_point = 2*sizeof(uint32_t);
        size_t sizeof_vertex_colored_tri   = 3*sizeof_vertex_colored_point;
        size_t sizeof_textured_tri         = 3*sizeof_vertex_colored_point;

  

        num_vertex_colored_points_to_draw_per_context_[ctx.id] = m_received_vertex_colored_points_;
        num_vertex_colored_tris_to_draw_per_context_[ctx.id]   = m_received_vertex_colored_tris_;
        num_textured_tris_to_draw_per_context_[ctx.id]         = m_received_textured_tris_;

        //num_points_to_draw_per_context_[ctx.id] = total_num_bytes_to_copy / sizeof_point;

        auto& current_is_vbo_created = is_vbo_created_per_context_[ctx.id];

        auto& current_net_data_vbo = net_data_vbo_per_context_[ctx.id];
        if(!current_is_vbo_created) {
          //std::cout << "CREATED BUFFER WITH NUM BYTES: " << total_num_bytes_to_copy << "\n";
          current_net_data_vbo = ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER, scm::gl::USAGE_STREAM_DRAW, total_num_bytes_to_copy, &m_buffer_[0]);


        } else {
          in_out_bb = gua::math::BoundingBox<scm::math::vec3d>();

          //std::cout << "RESIZED BUFFER WITH NUM BYTES: " << total_num_bytes_to_copy << "\n";
          ctx.render_device->resize_buffer(current_net_data_vbo, total_num_bytes_to_copy);

          //size_t total_num_points_in_buffer = total_num_bytes_to_copy / sizeof_point;
          float* mapped_net_data_vbo_ = (float*) ctx.render_device->main_context()->map_buffer(current_net_data_vbo, scm::gl::access_mode::ACCESS_WRITE_ONLY);
          memcpy((char*) mapped_net_data_vbo_, (char*) &m_buffer_[0], total_num_bytes_to_copy);
          //do not set bounding box to avoid culling problems during scene graph serialization
/*
          for(size_t dim_idx = 0; dim_idx < 3; ++dim_idx) {
            in_out_bb.min[dim_idx] = latest_received_bb_min[dim_idx];
            in_out_bb.max[dim_idx] = latest_received_bb_max[dim_idx];
          }
*/

          remote_server_screen_width_to_return_ = remote_server_screen_width_;
          remote_server_screen_height_to_return_ = remote_server_screen_height_;

          ctx.render_device->main_context()->unmap_buffer(current_net_data_vbo);
        }

        if(!current_is_vbo_created) {
          auto& current_point_layout = point_layout_per_context_[ctx.id];
          /*current_point_layout = ctx.render_device->create_vertex_array(scm::gl::vertex_format
                                                                      (0, 0, scm::gl::TYPE_VEC4F, sizeof_point),
                                                                      boost::assign::list_of(current_net_data_vbo));
          */
          /*
          current_point_layout = ctx.render_device->create_vertex_array(scm::gl::vertex_format
                                                                                (0, 0, scm::gl::TYPE_VEC2UI, sizeof_point, scm::gl::INT_PURE),
                                                                                boost::assign::list_of(current_net_data_vbo));
          */

          
          size_t size_of_vertex = 2 * sizeof(uint32_t);
          current_point_layout = ctx.render_device->create_vertex_array(scm::gl::vertex_format
                                                                       (0, 0, scm::gl::TYPE_VEC2UI, size_of_vertex, scm::gl::INT_PURE),
                                                                        boost::assign::list_of(current_net_data_vbo));
          

          current_is_vbo_created = true;

        }


      }
      
      m_need_gpu_swap_[ctx.id].store(false);
      return true;
    }
  }
  return false;
}


float NetKinectArray::get_voxel_size() const {
  return m_voxel_size_;
}

void NetKinectArray::readloop() {
  // open multicast listening connection to server and port
  zmq::context_t ctx(1); // means single threaded
  zmq::socket_t  socket(ctx, ZMQ_SUB); // means a subscriber

  socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);

  int conflate_messages = 1;
  socket.setsockopt(ZMQ_CONFLATE, &conflate_messages, sizeof(conflate_messages));

  std::string endpoint("tcp://" + m_server_endpoint_);
  socket.connect(endpoint.c_str());

  const unsigned message_size = sizeof(size_t);//(m_colorsize_byte + m_depthsize_byte) * m_calib_files.size();

  size_t header_byte_size = 100;
  std::vector<uint8_t> header_data(header_byte_size, 0);

  while (m_running_) {
    
    zmq::message_t zmqm(message_size);
    socket.recv(&zmqm); // blocking
    
    while (m_need_cpu_swap_) {
      ;
    }

    memcpy((unsigned char*) &header_data[0], (unsigned char*) zmqm.data(), header_byte_size);

    size_t num_voxels_received{0};

    // num points -> 8 byte
    // bb_min     -> 3*4 byte
    // bb_max     -> 3*4 byte
    // remote_server_screen_width  -> 4  byte
    // remote_server_screen_height -> 4  byte
    //--------------------------------------
    //                                40 byte 
    // m_voxel_size                -> 4 byte

    std::cout << "ABOUT TO READ: " << m_received_vertex_colored_points_back_ << "\n";

    size_t header_data_offset = 0;
    memcpy((unsigned char*) &num_voxels_received, (unsigned char*) &header_data[header_data_offset], sizeof(size_t) );
    header_data_offset += sizeof(size_t);
    memcpy((unsigned char*) &latest_received_bb_min[0], (unsigned char*) &header_data[header_data_offset], 3 * sizeof(float));
    header_data_offset += 3 * sizeof(float);
    memcpy((unsigned char*) &latest_received_bb_max[0], (unsigned char*) &header_data[header_data_offset], 3 * sizeof(float));
    header_data_offset += 3 * sizeof(float);
    
    memcpy((unsigned char*) &m_received_vertex_colored_points_back_, (unsigned char*) &header_data[header_data_offset], sizeof(uint32_t));
    header_data_offset +=  sizeof(uint32_t);
    memcpy((unsigned char*) &m_received_vertex_colored_tris_back_, (unsigned char*) &header_data[header_data_offset], sizeof(uint32_t));
    header_data_offset +=  sizeof(uint32_t);
    memcpy((unsigned char*) &m_received_textured_tris_back_, (unsigned char*) &header_data[header_data_offset], sizeof(uint32_t));
    header_data_offset +=  sizeof(uint32_t);
/*
    memcpy((char*) &remote_server_screen_width_, (char*)&header_data[header_data_offset], sizeof(unsigned));
    header_data_offset += sizeof(unsigned);
    memcpy((char*) &remote_server_screen_height_, (char*)&header_data[header_data_offset], sizeof(unsigned));
    header_data_offset += sizeof(unsigned);

    memcpy((char*) &m_voxel_size_back_, (char*)&header_data[header_data_offset], sizeof(float));
    header_data_offset += sizeof(float);
  */  
    std::cout << "NUM COL POINTS RECEIVED: " << m_received_vertex_colored_points_back_ << "\n";
    std::cout << "NUM COL TRIS RECEIVED: " << m_received_vertex_colored_tris_back_ << "\n";
    std::cout << "NUM TEXTURED TRIS RECEIVED: " << m_received_textured_tris_back_ << "\n";

    size_t total_num_received_primitives = m_received_vertex_colored_points_back_ + m_received_vertex_colored_tris_back_ + m_received_textured_tris_back_;

    if(total_num_received_primitives > 10000000) {
      return;
    }
    //size_t data_points_byte_size = num_voxels_received * 2 * sizeof(uint32_t);//sizeof(gua::point_types::XYZ32_RGB8);
    //size_t data_points_byte_size = num_voxels_received * 3 * sizeof(uint32_t);//sizeof(gua::point_types::XYZ32_RGB8);

    size_t vertex_colored_points_byte_size = m_received_vertex_colored_points_back_ * 2 * sizeof(uint32_t);//sizeof(gua::point_types::XYZ32_RGB8);
    size_t vertex_colored_tris_byte_size   = m_received_vertex_colored_tris_back_ * 3 * 2 * sizeof(uint32_t);//sizeof(gua::point_types::XYZ32_RGB8);
    size_t textured_tris_byte_size         = m_received_textured_tris_back_       * 3 * 2 * sizeof(uint32_t);//sizeof(gua::point_types::XYZ32_RGB8);

    size_t total_payload_byte_size = vertex_colored_points_byte_size + vertex_colored_tris_byte_size + textured_tris_byte_size;
    m_buffer_back_.resize(total_payload_byte_size);

    memcpy((unsigned char*) &m_buffer_back_[0], ((unsigned char*) zmqm.data()) + header_byte_size, total_payload_byte_size);

    std::cout << "COPYIING NUM BYTES TO BUFFER: " << total_payload_byte_size << "\n";
  
    { // swap
      std::lock_guard<std::mutex> lock(m_mutex_);
      m_need_cpu_swap_.store(true);

      for( auto& entry : m_need_gpu_swap_) {
        entry.second.store(true);
      }
    //mutable std::unordered_map<std::size_t,
    }
    
  }

}


}
