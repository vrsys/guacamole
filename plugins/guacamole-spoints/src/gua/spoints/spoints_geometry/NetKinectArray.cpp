#include <gua/spoints/spoints_geometry/NetKinectArray.hpp>
#include <gua/spoints/spoints_geometry/codec/point_types.h>

#include <boost/assign/list_of.hpp>


#include <zmq.hpp>
#include <iostream>
#include <mutex>

namespace spoints {

NetKinectArray::NetKinectArray(const std::string& server_endpoint,
                               const std::string& feedback_endpoint)
  : m_mutex(),
    m_running(true),
    m_feedback_running(true),
    m_server_endpoint(server_endpoint),
    m_feedback_endpoint(feedback_endpoint),
    m_buffer( /*(m_colorsize_byte + m_depthsize_byte) * m_calib_files.size()*/),
    m_buffer_back( /*(m_colorsize_byte + m_depthsize_byte) * m_calib_files.size()*/),
    m_need_swap{false},
    m_feedback_need_swap{false},
    m_recv()
{
 
  m_recv = std::thread([this]() { readloop(); });

  m_send_feedback = std::thread([this]() {sendfeedbackloop();});

}

NetKinectArray::~NetKinectArray()
{
  m_running = false;
  m_recv.join();
}

void 
NetKinectArray::draw(gua::RenderContext const& ctx) {

  if( point_layout_ != nullptr ) {
    scm::gl::context_vertex_input_guard vig(ctx.render_context);

    ctx.render_context->bind_vertex_array(point_layout_);
    ctx.render_context->apply();
    

    ctx.render_context->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST,
                                    0,
                                    num_points_to_draw);

    ctx.render_context->reset_vertex_input();
  }
}

void 
NetKinectArray::push_matrix_package(bool is_camera, std::size_t view_uuid, bool stereo_mode, matrix_package mp) {
  std::swap(m_matrix_package, mp);
  current_feedback_is_camera_status = is_camera;
  current_feedback_view_uuid = view_uuid;
  current_feedback_is_stereo_mode = stereo_mode;

  std::cout << "NetKinectArray: " << current_feedback_is_camera_status << "\n";
}

bool
NetKinectArray::update(gua::RenderContext const& ctx) {
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    if(m_need_swap.load()){
      //start of synchro point
      m_buffer.swap(m_buffer_back);

      //end of synchro point
      m_need_swap.store(false);

      size_t total_num_bytes_to_copy = m_buffer.size();

      size_t sizeof_point = 4*sizeof(float);

      num_points_to_draw = total_num_bytes_to_copy / sizeof_point;

      if(!is_vbo_created) {
        net_data_vbo_ = ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER, scm::gl::USAGE_STREAM_DRAW, total_num_bytes_to_copy, &m_buffer[0]);

        is_vbo_created = true;

      } else {
        ctx.render_device->resize_buffer(net_data_vbo_, total_num_bytes_to_copy);

        float* mapped_net_data_vbo_ = (float*) ctx.render_device->main_context()->map_buffer(net_data_vbo_, scm::gl::access_mode::ACCESS_WRITE_ONLY);
        memcpy((char*) mapped_net_data_vbo_, (char*) &m_buffer[0], total_num_bytes_to_copy);
        ctx.render_device->main_context()->unmap_buffer(net_data_vbo_);
      }

        point_layout_ = ctx.render_device->create_vertex_array(scm::gl::vertex_format
          (0, 0, scm::gl::TYPE_VEC4F, sizeof_point),
          boost::assign::list_of(net_data_vbo_));

        m_buffer.clear();
      return true;
    }
  }
  return false;
}

void
NetKinectArray::update_feedback(gua::RenderContext const& ctx) {
  {

    std::cout << !m_feedback_need_swap.load() << "\n";

    if(!m_feedback_need_swap.load()) {
      std::lock_guard<std::mutex> lock(m_feedback_mutex);
      std::swap(m_matrix_package, m_matrix_package_back);


      std::cout << "while updating feedback: " << current_feedback_is_camera_status << "\n";

      std::cout << "MONO MODE: " << int(current_feedback_is_stereo_mode) << "\n";

      if( camera_group_to_uuid_to_matrix_package_list[current_feedback_is_camera_status][current_feedback_view_uuid].end() != camera_group_to_uuid_to_matrix_package_list[current_feedback_is_camera_status][current_feedback_view_uuid].find(current_feedback_is_stereo_mode) ) {
        
        std::cout << "detected identity and was actual camera\n";
        std::swap(camera_group_to_uuid_to_matrix_package_list_back, camera_group_to_uuid_to_matrix_package_list);
        camera_group_to_uuid_to_matrix_package_list.clear();
        camera_group_to_uuid_to_matrix_package_list[current_feedback_is_camera_status][current_feedback_view_uuid][current_feedback_is_stereo_mode].push_back(m_matrix_package_back);
        m_feedback_need_swap.store(true);

      } else {

        auto& known_feedback_statii
          = camera_group_to_uuid_to_matrix_package_list[current_feedback_is_camera_status][current_feedback_view_uuid][current_feedback_is_stereo_mode];

        bool detected_matrix_identity = false;


        for(auto const& curr_matrix_package : known_feedback_statii) {
          if( !memcmp ( &curr_matrix_package, &m_matrix_package_back, sizeof(matrix_package) ) ) {
            detected_matrix_identity = true;
            break;
          }
        }

        if( camera_group_to_uuid_to_matrix_package_list[current_feedback_is_camera_status][current_feedback_view_uuid].end() != camera_group_to_uuid_to_matrix_package_list[current_feedback_is_camera_status][current_feedback_view_uuid].find(current_feedback_is_stereo_mode) ) {
          std::cout << "detected identity and was actual camera\n";
          std::swap(camera_group_to_uuid_to_matrix_package_list_back, camera_group_to_uuid_to_matrix_package_list);
          camera_group_to_uuid_to_matrix_package_list.clear();
          camera_group_to_uuid_to_matrix_package_list[current_feedback_is_camera_status][current_feedback_view_uuid][current_feedback_is_stereo_mode].push_back(m_matrix_package_back);
          m_feedback_need_swap.store(true);            
        } else {

          if(!detected_matrix_identity) {
            std::cout << "did not detect identity\n";
            known_feedback_statii.push_back(m_matrix_package_back);
          } else {
            if(current_feedback_is_camera_status) {






            } else {
              std::cout << "Not an actual camera. Adding more matrix packages.\n\n";
            }

          }
        }

      }


      


    }

  }
}

void NetKinectArray::readloop() {
  // open multicast listening connection to server and port
  zmq::context_t ctx(1); // means single threaded
  zmq::socket_t  socket(ctx, ZMQ_SUB); // means a subscriber

  socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);
#if ZMQ_VERSION_MAJOR < 3
  int64_t hwm = 1;
  socket.setsockopt(ZMQ_HWM, &hwm, sizeof(hwm));
#else
  int hwm = 1;
  socket.setsockopt(ZMQ_RCVHWM, &hwm, sizeof(hwm));
#endif
  std::string endpoint("tcp://" + m_server_endpoint);
  socket.connect(endpoint.c_str());

  //const unsigned message_size = (m_colorsize_byte + m_depthsize_byte) * m_calib_files.size();

  const unsigned message_size = sizeof(size_t);//(m_colorsize_byte + m_depthsize_byte) * m_calib_files.size();

  size_t header_byte_size = 100;
  std::vector<uint8_t> header_data(header_byte_size, 0);

  while (m_running) {
    
    zmq::message_t zmqm(message_size);
    socket.recv(&zmqm); // blocking
    
    while (m_need_swap) {
      ;
    }

    memcpy((unsigned char*) &header_data[0], (unsigned char*) zmqm.data(), header_byte_size);

    size_t num_voxels_received{0};

    memcpy((unsigned char*) &num_voxels_received, (unsigned char*) &header_data[0], sizeof(size_t) );

    size_t data_points_byte_size = num_voxels_received * sizeof(gua::point_types::XYZ32_RGB8);
    //if(m_buffer_back.size() < data_points_byte_size) {
      m_buffer_back.resize(data_points_byte_size);
   // }

    //memcpy((unsigned char*) m_buffer_back.data(), (unsigned char*) zmqm.data(), message_size);
    memcpy((unsigned char*) &m_buffer_back[0], ((unsigned char*) zmqm.data()) + header_byte_size, data_points_byte_size);

  
    { // swap
      std::lock_guard<std::mutex> lock(m_mutex);
      m_need_swap.store(true);
    }
    
  }

}




void NetKinectArray::sendfeedbackloop() {
  
  // open multicast listening connection to server and port
  zmq::context_t ctx(1); // means single threaded
  zmq::socket_t  socket(ctx, ZMQ_PUB); // means a subscriber

  int hwm = 1;
  socket.setsockopt(ZMQ_SNDHWM, &hwm, sizeof(hwm));

  std::string endpoint(std::string("tcp://") + m_feedback_endpoint);
  socket.bind(endpoint.c_str());

  while (m_feedback_running) {
    

    if(m_feedback_need_swap.load()) { // swap
      std::lock_guard<std::mutex> lock(m_feedback_mutex);

      size_t feedback_header_byte = 100;



      uint32_t num_recorded_matrix_packages = 0;

      std::vector<matrix_package> flattened_matrix_packages;

      for ( auto const& camera_status_list : camera_group_to_uuid_to_matrix_package_list_back) {
        for ( auto const& view_ids : camera_status_list.second ) {
            for ( auto const& camera_modes : view_ids.second ) {
              //for( auto const& matrix_vector : view_ids.second) {
              std::cout << "cam status: " << camera_status_list.first << " with " << camera_modes.second.size() << " elements\n";
              num_recorded_matrix_packages += camera_modes.second.size();

              for(auto const& matrix : camera_modes.second) {
                flattened_matrix_packages.push_back(matrix);
              }
              ////}
            }
        }
      }

      //HEADER DATA SO FAR:

      /* 00000000 uint32_t num_matrices

      */

      zmq::message_t zmqm(feedback_header_byte + num_recorded_matrix_packages * sizeof(matrix_package) );


      std::cout << "GOING TO SEND " << num_recorded_matrix_packages <<  " MATRICES\n";

      //std::cout << "actually recorded matrices: " << num_recorded_matrix_packages << "\n";

      // fill two matrices (current feedback to TSDF-reconstruction)
      // 1. matrix: modelview matrix of avatar 
      // 2. matrix: projection matrix
      //memcpy((char*)zmqm.data(), (char*)&m_matrix_package_back, 32*sizeof(float));

      memcpy((char*)zmqm.data(), (char*)&(num_recorded_matrix_packages), sizeof(uint32_t));     
      memcpy( ((char*)zmqm.data()) + (feedback_header_byte), (char*)&(flattened_matrix_packages[0]), (num_recorded_matrix_packages) *  sizeof(matrix_package) );



      for ( auto const& camera_status_list : camera_group_to_uuid_to_matrix_package_list_back) {
        for ( auto const& view_ids : camera_status_list.second ) {
          //for( auto const& matrix_vector : view_ids.second) {
            std::cout << "cam status: " << camera_status_list.first << " with " << view_ids.second.size() << " elements\n";
            num_recorded_matrix_packages += view_ids.second.size();
          ////}
        }
      }
      std::cout << "actually recorded matrices: " << num_recorded_matrix_packages << "\n";

      // send feedback
      socket.send(zmqm); // blocking

      // tell the main thread, that the feedback information was received
      m_feedback_need_swap.store(false);

    }
    
  }

}



}
