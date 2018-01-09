#include <gua/spoints/spoints_geometry/NetKinectArray.hpp>
#include <gua/spoints/spoints_geometry/codec/point_types.h>

#include <boost/assign/list_of.hpp>


#include <zmq.hpp>
#include <iostream>
#include <mutex>

namespace spoints {

NetKinectArray::NetKinectArray(const std::string& server_endpoint,
                               const std::string& feedback_endpoint)
  : m_encoder(),
    m_voxels(),
    m_mutex(),
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
  PointCloudGridEncoder encoder;
 
  m_recv = std::thread([this]() { readloop(); });

  m_send_feedback = std::thread([this]() {sendfeedbackloop();});

}

NetKinectArray::~NetKinectArray()
{
  m_running = false;
  m_recv.join();
}

void 
NetKinectArray:: draw(gua::RenderContext const& ctx) {

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
NetKinectArray::push_matrix_package(matrix_package mp) {
  m_matrix_package.swap(mp);
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
    std::lock_guard<std::mutex> lock(m_feedback_mutex);
    m_matrix_package.swap(m_matrix_package_back);

    m_feedback_need_swap.store(true);
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

  while (m_running) {
    
    zmq::message_t zmqm;
    socket.recv(&zmqm); // blocking
    m_encoder.decode(zmqm, &m_voxels);

    while (m_need_swap) {
      ;
    }

    std::cout << "VOXELS received " << m_voxels.size() << std::endl;
    size_t data_points_byte_size = m_voxels.size() * sizeof(gua::point_types::XYZ32_RGB8);
    if(m_buffer_back.size() < data_points_byte_size) {
      m_buffer_back.resize(data_points_byte_size);
    }

    memcpy((unsigned char*) &m_buffer_back[0], (unsigned char*) m_voxels.data(), data_points_byte_size);
  
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

      zmq::message_t zmqm(32*sizeof(float));

      // fill two matrices (current feedback to TSDF-reconstruction)
      // 1. matrix: modelview matrix of avatar 
      // 2. matrix: projection matrix
      memcpy((char*)zmqm.data(), (char*)&m_matrix_package_back, 32*sizeof(float));


      // send feedback
      socket.send(zmqm); // blocking

      // tell the main thread, that the feedback information was received
      m_feedback_need_swap.store(false);

    }
    
  }

}



}
