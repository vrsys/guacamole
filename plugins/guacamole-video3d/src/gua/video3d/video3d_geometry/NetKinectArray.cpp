#include <gua/video3d/video3d_geometry/NetKinectArray.hpp>

#include <zmq.hpp>
#include <RGBDCompressor.h>
#include <MultiRGBDStreamClient.h>
#include <MultiRGBDStreamHeader.h>

#include <iostream>
#include <mutex>

namespace video3d {

NetKinectArray::NetKinectArray(const std::vector<std::shared_ptr<KinectCalibrationFile>>& calib_files,
                               const std::string& server_endpoint, unsigned colorsize_byte, unsigned depthsize_byte)
  : m_mutex(),
    m_running(true),
    m_server_endpoint(server_endpoint),
    m_calib_files(calib_files),
    m_colorsize_byte(colorsize_byte),
    m_depthsize_byte(depthsize_byte),
    m_buffer     (/* (m_colorsize_byte + m_depthsize_byte) * m_calib_files.size()*/),
    m_buffer_back(/* (m_colorsize_byte + m_depthsize_byte) * m_calib_files.size()*/),
    m_need_swap(false),
    m_recv(),
    m_feedPack(),
    m_debug_message()
{
  m_recv = std::thread([this]() { readloop(); });
}

NetKinectArray::~NetKinectArray(){
  m_running = false;
  m_recv.join();
}

bool NetKinectArray::update() {
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    if(m_need_swap){
      m_buffer.swap(m_buffer_back);
      m_need_swap = false;
      return true;
    }
  }
  return false;
}


void NetKinectArray::readloop() {

  RGBDSizes size;
  const unsigned num_streams(m_calib_files.size());
  float* debug_values           = new float [size.debug_size];
  float* feedback_val           = new float [size.feedback_size];

  // init server 
  kinect::MultiRGBDStreamClient c(
    m_server_endpoint,
    num_streams,
    size.color_width,
    size.color_height,
    size.colorsize_byte,
    size.depth_width,
    size.depth_height);

  // open multicast listening connection to server and port
  zmq::context_t ctx(1); // means single threaded
  zmq::socket_t  socket(ctx, ZMQ_SUB);    // means a subscriber
  zmq::socket_t  socket_d(ctx, ZMQ_SUB);    // means a subscriber
  zmq::socket_t  socket_f(ctx, ZMQ_PUB);  // means a Publisher

  socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);
  socket_d.setsockopt(ZMQ_SUBSCRIBE, "", 0);

#if ZMQ_VERSION_MAJOR < 3
  int64_t hwm = 1;
  socket.setsockopt(ZMQ_HWM, &hwm, sizeof(hwm));
  socket_d.setsockopt(ZMQ_HWM, &hwm, sizeof(hwm));
  socket_f.setsockopt(ZMQ_HWM, &hwm, sizeof(hwm));
#else
  int hwm = 1;
  socket.setsockopt(ZMQ_RCVHWM, &hwm, sizeof(hwm));
  socket_d.setsockopt(ZMQ_RCVHWM, &hwm, sizeof(hwm));
  socket_f.setsockopt(ZMQ_RCVHWM, &hwm, sizeof(hwm));
#endif
  std::string endpoint("tcp://" + m_server_endpoint);
  std::string endpoint_d("tcp://141.54.147.57:7051");
  std::string endpoint_f("tcp://141.54.147.57:7001");
  socket.connect(endpoint.c_str());
  socket_d.connect(endpoint_d.c_str());
  socket_f.bind(endpoint_f.c_str());



  //const unsigned message_size = (m_colorsize_byte + m_depthsize_byte) * m_calib_files.size();
  //std::cout << m_calib_files.size() << "; " <<  m_colorsize_byte << " : " << size.colorsize_byte << "; " <<  m_depthsize_byte << " : " << size.depthsize_byte <<std::endl;
  
  unsigned cont = 0;
  while (m_running) {

    //zmq::message_t zmqm(message_size);
    //socket.recv(&zmqm); // blocking
    
    c.receive();
    c.waitForDecode();

    while (m_need_swap) {
      ;
    }

    
    zmq::message_t zmqm_d(size.debug_byte);
    socket_d.recv(&zmqm_d, ZMQ_NOBLOCK);
    bool got_debug = false;
    if(zmqm_d.size() == size.debug_byte){  
      memcpy((float*) debug_values, zmqm_d.data(), size.debug_byte);

      set_debug_message(  std::to_string(debug_values[1])  + ", "+ std::to_string(debug_values[2]) + ", "+ std::to_string(debug_values[3]) + ", "
                        + std::to_string(debug_values[7])  + ", "+ std::to_string(debug_values[8]) + ", "+ std::to_string(debug_values[9]) + ", "
                        + std::to_string(debug_values[10]) + ", "+ std::to_string(debug_values[4]) + ", "+ std::to_string(debug_values[5]) + ", "
                        + std::to_string(debug_values[6])  + ", "+ std::to_string(debug_values[0]));
      /*
      std::cout << "\n   > Debug information: "                   << std::endl;
      std::cout << "\t > total_MegaBitPerSecond@30Hz: "           << debug_values[1]<< std::endl;
      std::cout << "\t > total_MegaBitPerSecond@30Hz_color: "     << debug_values[2]<< std::endl;
      std::cout << "\t > total_MegaBitPerSecond@30Hz_depth:"      << debug_values[3]<< std::endl;
      std::cout << "\t > total_byte_base: "                       << debug_values[7]<< std::endl;
      std::cout << "\t > total_byte_enc: "                        << debug_values[8]<< std::endl;
      std::cout << "\t > total_byte_enc_color: "                  << debug_values[9]<< std::endl;
      std::cout << "\t > total_byte_enc_depth: "                  << debug_values[10]<< std::endl;
      std::cout << "\t > total_compression_ratio_percent: "       << debug_values[4]<< std::endl;
      std::cout << "\t > total_compression_ratio_color_percent: " << debug_values[5]<< std::endl;
      std::cout << "\t > total_compression_ratio_depth_percent: " << debug_values[6]<< std::endl;
      std::cout << "\t > total_time: "                            << debug_values[0]<< std::endl;
      */
      got_debug = true;
    }


    zmq::message_t zmqm_f(size.feedback_byte);
    feedback_val[0] = get_feedback_global_comp_lvl();
    feedback_val[1] = get_feedback_depth_comp_lvl();
    feedback_val[2] = get_feedback_color_comp_lvl();
    memcpy(zmqm_f.data(), (float*) feedback_val, size.feedback_byte);
    socket_f.send(zmqm_f);


    const unsigned message_size = (size.colorsize_byte + size.depthsize_byte) * num_streams; 
    if(m_buffer_back.size() < message_size) {
      m_buffer_back.resize(message_size);
    }
    
    unsigned offset = 0;
    for(unsigned i = 0; i < num_streams; ++i){
      memcpy((unsigned char*) &m_buffer_back[0] + offset, c.getColor(i), size.colorsize_byte);
      offset += size.colorsize_byte;
      memcpy((unsigned char*) &m_buffer_back[0] + offset, c.getDepth(i), size.depthsize_byte);
      offset += size.depthsize_byte;
    } 

    //memcpy((unsigned char*) m_buffer_back.data(), (unsigned char*) zmqm.data(), message_size);
    { // swap
      std::lock_guard<std::mutex> lock(m_mutex);
      m_need_swap = true;
    }
  }
}

}
