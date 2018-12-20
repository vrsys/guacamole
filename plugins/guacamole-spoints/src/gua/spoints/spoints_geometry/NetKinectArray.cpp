#include <gua/spoints/spoints_geometry/NetKinectArray.hpp>
#include <gua/spoints/spoints_geometry/codec/point_types.h>

#include <boost/assign/list_of.hpp>


#include <gua/spoints/sgtp/SGTP.h>

#include <zmq.hpp>
#include <iostream>
#include <mutex>

namespace spoints {

#define ONE_D_TEXTURE_ATLAS_SIZE 2048

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


    ctx.render_context->draw_arrays(scm::gl::PRIMITIVE_TRIANGLE_LIST,
                                    vertex_offset,
                                    current_num_tri_vertices_to_draw);

    ctx.render_context->reset_vertex_input();
  }
}

void 
NetKinectArray::draw_textured_triangle_soup(gua::RenderContext const& ctx, std::shared_ptr<gua::ShaderProgram>& shader_program) {

  auto const& current_point_layout = point_layout_per_context_[ctx.id];

  if( current_point_layout != nullptr ) {

    auto const& current_texture_atlas = texture_atlas_per_context_[ctx.id];

    scm::gl::context_vertex_input_guard vig(ctx.render_context);

    ctx.render_context->bind_vertex_array(current_point_layout);
    

    if( nullptr == linear_sampler_state_ ) {
      linear_sampler_state_ = ctx.render_device
        ->create_sampler_state(scm::gl::FILTER_MIN_MAG_LINEAR, scm::gl::WRAP_CLAMP_TO_EDGE);
    }

    ctx.render_context->bind_texture(current_texture_atlas, linear_sampler_state_, 0);

    shader_program->set_uniform(ctx, int(m_triangle_texture_atlas_size_), "texture_space_triangle_size");

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
      m_texture_buffer_.swap(m_texture_buffer_back_);
      //std::swap(m_voxel_size_, m_voxel_size_back_);

      std::swap(m_received_vertex_colored_points_back_, m_received_vertex_colored_points_);
      std::swap(m_received_vertex_colored_tris_back_, m_received_vertex_colored_tris_);
      std::swap(m_received_textured_tris_back_, m_received_textured_tris_);
      std::swap(m_received_kinect_timestamp_back_, m_received_kinect_timestamp_);
      std::swap(m_received_reconstruction_time_back_, m_received_reconstruction_time_);

      std::swap(m_texture_payload_size_in_byte_back_, m_texture_payload_size_in_byte_);
      std::swap(m_triangle_texture_atlas_size_back_, m_triangle_texture_atlas_size_);
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
        auto& current_texture_atlas = texture_atlas_per_context_[ctx.id];



        if(!current_is_vbo_created) {
          size_t initial_vbo_size = 10000000;

          current_net_data_vbo = ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER, scm::gl::USAGE_STREAM_DRAW, initial_vbo_size, 0);
          current_texture_atlas = ctx.render_device->create_texture_2d(scm::math::vec2ui(ONE_D_TEXTURE_ATLAS_SIZE, ONE_D_TEXTURE_ATLAS_SIZE), scm::gl::FORMAT_BGR_8, 1, 1, 1);
        }

          float* mapped_net_data_vbo_ = (float*) ctx.render_device->main_context()->map_buffer(current_net_data_vbo, scm::gl::access_mode::ACCESS_WRITE_ONLY);
          memcpy((char*) mapped_net_data_vbo_, (char*) &m_buffer_[0], total_num_bytes_to_copy);


          remote_server_screen_width_to_return_ = remote_server_screen_width_;
          remote_server_screen_height_to_return_ = remote_server_screen_height_;

          ctx.render_device->main_context()->unmap_buffer(current_net_data_vbo);
          

          uint32_t total_num_pixels_to_upload = m_texture_payload_size_in_byte_ / 3;

          uint32_t num_pixels_u_direction = ( (total_num_pixels_to_upload / ONE_D_TEXTURE_ATLAS_SIZE) > 0) ?  ONE_D_TEXTURE_ATLAS_SIZE : total_num_pixels_to_upload;
          uint32_t num_pixels_v_direction = ( (total_num_pixels_to_upload) / ONE_D_TEXTURE_ATLAS_SIZE == 0) ? total_num_pixels_to_upload : total_num_pixels_to_upload/ONE_D_TEXTURE_ATLAS_SIZE;

          auto region_to_update = scm::gl::texture_region(scm::math::vec3ui(0, 0, 0), scm::math::vec3ui(num_pixels_u_direction, num_pixels_v_direction, 1));
          ctx.render_device->main_context()->update_sub_texture(current_texture_atlas, region_to_update, 0, scm::gl::FORMAT_BGR_8, (void*) &m_texture_buffer_[0]);


        if(!current_is_vbo_created) {
          auto& current_point_layout = point_layout_per_context_[ctx.id];

          
          size_t size_of_vertex = 2 * sizeof(uint32_t);
          current_point_layout = ctx.render_device->create_vertex_array(scm::gl::vertex_format
                                                                       (0, 0, scm::gl::TYPE_VEC2UI, size_of_vertex, scm::gl::INT_PURE),
                                                                        boost::assign::list_of(current_net_data_vbo));
          

          current_is_vbo_created = true;

        }


      } else {
        num_vertex_colored_points_to_draw_per_context_[ctx.id] = 0;
        num_vertex_colored_tris_to_draw_per_context_[ctx.id]   = 0;
        num_textured_tris_to_draw_per_context_[ctx.id]         = 0;
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

  //size_t header_byte_size = 100;
  //std::vector<uint8_t> header_data(header_byte_size, 0);

  while (m_running_) {
    
    zmq::message_t zmqm(message_size);
    socket.recv(&zmqm); // blocking
    
    while (m_need_cpu_swap_) {
      ;
    }

    SGTP::header_data_t message_header;

    std::size_t const HEADER_SIZE = SGTP::HEADER_BYTE_SIZE;

    memcpy((char*)&message_header, (unsigned char*) zmqm.data(), SGTP::HEADER_BYTE_SIZE);
    //memcpy((unsigned char*) &header_data[0], (unsigned char*) zmqm.data(), header_byte_size);

    size_t num_voxels_received{0};

    for(uint32_t dim_idx = 0; dim_idx < 3; ++dim_idx) {
      latest_received_bb_min[dim_idx] = message_header.global_bb_min[dim_idx];
      latest_received_bb_max[dim_idx] = message_header.global_bb_max[dim_idx];
    }

    num_voxels_received                    = message_header.num_points;
    m_received_vertex_colored_points_back_ = message_header.num_vertex_col_points;
    m_received_vertex_colored_tris_back_   = message_header.num_vertex_col_triangles;
    m_received_textured_tris_back_         = message_header.num_textured_triangles;
    m_texture_payload_size_in_byte_back_   = message_header.texture_payload_size;

    m_triangle_texture_atlas_size_back_  = message_header.texture_space_triangle_size;
    m_received_kinect_timestamp_back_    = message_header.timestamp;
    m_received_reconstruction_time_back_ = message_header.geometry_creation_time_in_ms;



    size_t total_num_received_primitives = m_received_vertex_colored_points_back_ + m_received_vertex_colored_tris_back_ + m_received_textured_tris_back_;

    if(total_num_received_primitives > 50000000) {
      return;
    }
    //size_t data_points_byte_size = num_voxels_received * 2 * sizeof(uint32_t);//sizeof(gua::point_types::XYZ32_RGB8);
    //size_t data_points_byte_size = num_voxels_received * 3 * sizeof(uint32_t);//sizeof(gua::point_types::XYZ32_RGB8);

    size_t vertex_colored_points_byte_size = m_received_vertex_colored_points_back_ * SGTP::VERTEX_COL_POINT_SIZE ;//sizeof(gua::point_types::XYZ32_RGB8);
    size_t vertex_colored_tris_byte_size   = m_received_vertex_colored_tris_back_   * SGTP::VERTEX_COL_TRIANGLE_SIZE;//sizeof(gua::point_types::XYZ32_RGB8);
    size_t textured_tris_byte_size         = m_received_textured_tris_back_         * SGTP::TEXTURED_TRIANGLE_SIZE;//sizeof(gua::point_types::XYZ32_RGB8);

    //std::cout << "RECEIVED TRIANGLES: " << m_received_vertex_colored_tris_back_ + m_received_textured_tris_back_ << "\n";

    size_t total_payload_byte_size = vertex_colored_points_byte_size + vertex_colored_tris_byte_size + textured_tris_byte_size;
    m_buffer_back_.resize(total_payload_byte_size);

    memcpy((unsigned char*) &m_buffer_back_[0], ((unsigned char*) zmqm.data()) + HEADER_SIZE, total_payload_byte_size);

    m_texture_buffer_back_.resize(m_texture_payload_size_in_byte_back_);
    memcpy((unsigned char*) &m_texture_buffer_back_[0], ((unsigned char*) zmqm.data()) + HEADER_SIZE + total_payload_byte_size, m_texture_payload_size_in_byte_back_);
  
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
