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
NetKinectArray::draw(gua::RenderContext const& ctx) {

  auto const& current_point_layout = point_layout_per_context_[ctx.id];

  if( current_point_layout != nullptr ) {
    scm::gl::context_vertex_input_guard vig(ctx.render_context);

    ctx.render_context->bind_vertex_array(current_point_layout);
    ctx.render_context->apply();
    
    size_t const& current_num_points_to_draw = num_points_to_draw_per_context_[ctx.id];
    ctx.render_context->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST,
                                    0,
                                    current_num_points_to_draw);

    ctx.render_context->reset_vertex_input();
  }
}

std::string 
NetKinectArray::get_socket_string() {
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

      //end of synchro point
      m_need_cpu_swap_.store(false);
    }

    if(m_need_gpu_swap_[ctx.id].load()) {
      size_t total_num_bytes_to_copy = m_buffer_.size();

      if(0 != total_num_bytes_to_copy) {

        size_t sizeof_point = 4*sizeof(float);

        num_points_to_draw_per_context_[ctx.id] = total_num_bytes_to_copy / sizeof_point;

        auto& current_is_vbo_created = is_vbo_created_per_context_[ctx.id];

        auto& current_net_data_vbo = net_data_vbo_per_context_[ctx.id];
        if(!current_is_vbo_created) {
          //std::cout << "CREATED BUFFER WITH NUM BYTES: " << total_num_bytes_to_copy << "\n";
          current_net_data_vbo = ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER, scm::gl::USAGE_STREAM_DRAW, total_num_bytes_to_copy, &m_buffer_[0]);


        } else {
          in_out_bb = gua::math::BoundingBox<scm::math::vec3d>();

          //std::cout << "RESIZED BUFFER WITH NUM BYTES: " << total_num_bytes_to_copy << "\n";
          ctx.render_device->resize_buffer(current_net_data_vbo, total_num_bytes_to_copy);

          size_t total_num_points_in_buffer = total_num_bytes_to_copy / sizeof_point;
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
          current_point_layout = ctx.render_device->create_vertex_array(scm::gl::vertex_format
                                                                      (0, 0, scm::gl::TYPE_VEC4F, sizeof_point),
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

/*
void
NetKinectArray::update_feedback(gua::RenderContext const& ctx) {
  {

    //std::cout << !m_feedback_need_swap_.load() << "\n";

    if( true) {
      std::lock_guard<std::mutex> lock(m_feedback_mutex_);

      std::swap(submitted_camera_matrix_package_back_, submitted_camera_matrix_package_);
      //std::swap(m_matrix_package, m_matrix_package_back);

      bool is_stereo = submitted_camera_matrix_package_back_.k_package.stereo_mode;
      unsigned framecount = submitted_camera_matrix_package_back_.k_package.framecount;
      unsigned view_uuid = submitted_camera_matrix_package_back_.k_package.view_uuid;


      //std::cout << "FCT: " << submitted_camera_matrix_package_back_.k_package.framecount << "\n"; 

      if(submitted_camera_matrix_package_back_.k_package.framecount != last_frame_count_) {
        m_feedback_need_swap_.store(true);
        std::swap(matrix_packages_to_submit_, matrix_packages_to_collect_);
        matrix_packages_to_collect_.clear();
        last_frame_count_ = submitted_camera_matrix_package_back_.k_package.framecount;
      }

      bool detected_matrix_identity = false;

      for(auto const& curr_matrix_package : matrix_packages_to_collect_) {
        if( !memcmp ( &curr_matrix_package, &submitted_camera_matrix_package_back_.mat_package, sizeof(matrix_package) ) ) {
          detected_matrix_identity = true;
          break;
        }
      }

      if(!detected_matrix_identity) {
        matrix_packages_to_collect_.push_back(submitted_camera_matrix_package_back_.mat_package);
      }

    } else {
      last_omitted_frame_count_ = submitted_camera_matrix_package_back_.k_package.framecount;
    }

  }
}
*/
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
    size_t header_data_offset = 0;
    memcpy((unsigned char*) &num_voxels_received, (unsigned char*) &header_data[header_data_offset], sizeof(size_t) );
    header_data_offset += sizeof(size_t);
    memcpy((unsigned char*) &latest_received_bb_min[0], (unsigned char*) &header_data[header_data_offset], 3 * sizeof(float));
    header_data_offset += 3 * sizeof(float);
    memcpy((unsigned char*) &latest_received_bb_max[0], (unsigned char*) &header_data[header_data_offset], 3 * sizeof(float));
    header_data_offset += 3 * sizeof(float);
    memcpy((char*) &remote_server_screen_width_, (char*)&header_data[header_data_offset], sizeof(unsigned));
    header_data_offset += sizeof(unsigned);
    memcpy((char*) &remote_server_screen_height_, (char*)&header_data[header_data_offset], sizeof(unsigned));
    header_data_offset += sizeof(unsigned);
    
    std::cout << "NUM VOXELS RECEIVED: " << num_voxels_received << "\n";

    size_t data_points_byte_size = num_voxels_received * sizeof(gua::point_types::XYZ32_RGB8);
    //if(m_buffer_back.size() < data_points_byte_size) {
      m_buffer_back_.resize(data_points_byte_size);
   // }


    //memcpy((unsigned char*) m_buffer_back.data(), (unsigned char*) zmqm.data(), message_size);
    memcpy((unsigned char*) &m_buffer_back_[0], ((unsigned char*) zmqm.data()) + header_byte_size, data_points_byte_size);

  
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



/*
void NetKinectArray::sendfeedbackloop() {
  
  // open multicast listening connection to server and port
  zmq::context_t ctx(1); // means single threaded
  zmq::socket_t  socket(ctx, ZMQ_PUB); // means a subscriber

  int conflate_messages  = 1;
  socket.setsockopt(ZMQ_CONFLATE, &conflate_messages, sizeof(conflate_messages));
  //socket.set
  std::string endpoint(std::string("tcp://") + m_feedback_endpoint_);

  try { 
    socket.bind(endpoint.c_str()); 
  } 
  catch (const std::exception& e) {
    std::cout << "Failed to bind feedback socket\n";
    return;
  }

  while (m_feedback_running_) {
    

    if(m_feedback_need_swap_.load()) { // swap
      std::lock_guard<std::mutex> lock(m_feedback_mutex_);

      size_t feedback_header_byte = 100;



      uint32_t num_recorded_matrix_packages = 0;


      num_recorded_matrix_packages +=  matrix_packages_to_submit_.size();

      //HEADER DATA SO FAR:

      // 00000000 uint32_t num_matrices

      


      zmq::message_t zmqm(feedback_header_byte + num_recorded_matrix_packages * sizeof(matrix_package) );


      memcpy((char*)zmqm.data(), (char*)&(num_recorded_matrix_packages), sizeof(uint32_t));     
      memcpy( ((char*)zmqm.data()) + (feedback_header_byte), (char*)&(matrix_packages_to_submit_[0]), (num_recorded_matrix_packages) *  sizeof(matrix_package) );

      //std::cout << "actually recorded matrices: " << num_recorded_matrix_packages << "\n";

      // send feedback
      socket.send(zmqm); // blocking

      m_feedback_need_swap_.store(false);

    }
    
  }

}
*/



}
