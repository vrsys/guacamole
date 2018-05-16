#include <gua/video3d/video3d_geometry/NetKinectArray.hpp>

#include <zmq.hpp>
#include <RGBDCompressor.h>
#include <MultiRGBDStreamClient.h>
#include <MultiRGBDStreamHeader.h>

#include <iostream>
#include <chrono>
#include <mutex>

namespace video3d {

template<typename T>
const std::string toString(const T& value) {
  std::stringstream ss;
  ss.precision(3);
  ss << value;
  return ss.str();
}

NetKinectArray::NetKinectArray(const std::vector<std::shared_ptr<KinectCalibrationFile>>& calib_files,
                               const std::string& server_endpoint, const std::string& feedback_port, const std::string& debug_port, 
                               unsigned colorsize_byte, unsigned depthsize_byte)
  : m_mutex(),
    m_running(true),
    m_server_endpoint(server_endpoint),
    m_feedback_port(feedback_port),
    m_debug_port(debug_port),
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
  std::string endpoint_d("tcp://" + m_debug_port);
  std::string endpoint_f("tcp://" + m_feedback_port);
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

    
    zmq::message_t zmqm_d;//(size.debug_byte*sizeof(float));
    socket_d.recv(&zmqm_d, ZMQ_NOBLOCK);
    
    if(zmqm_d.size() == size.debug_byte){ 

      memcpy((float*) debug_values, zmqm_d.data(), size.debug_byte);

      float total_MegaBitPerSecond_30Hz             = debug_values[1];
      float total_MegaBitPerSecond_30Hz_color       = debug_values[2];
      float total_MegaBitPerSecond_30Hz_depth       = debug_values[3];
      float total_byte_base                         = debug_values[7];
      float total_byte_enc                          = debug_values[8];
      float total_byte_enc_color                    = debug_values[9];
      float total_byte_enc_depth                    = debug_values[10];
      float total_compression_ratio_percent         = debug_values[4];
      float total_compression_ratio_color_percent   = debug_values[5];
      float total_compression_ratio_depth_percent   = debug_values[6];
      float total_time                              = debug_values[0];
<<<<<<< HEAD
      int comp_ratio = (int) (100.0f / total_compression_ratio_percent);
      int comp_ratio_depth = (int) (100.0f / total_compression_ratio_depth_percent);
      int comp_ratio_color = (int) (100.0f / total_compression_ratio_color_percent);
=======
>>>>>>> lib_rgbd
      //float enc_time                                = debug_values[11];
      //float mask_time                               = debug_values[12];

      float Mbits_Compressed        =  total_MegaBitPerSecond_30Hz/30;
      float Mbits_Compressed_30f    =  total_MegaBitPerSecond_30Hz;
      float comp_color_ratio        = (total_MegaBitPerSecond_30Hz_color/30)/Mbits_Compressed;
      float comp_depth_ratio        = (total_MegaBitPerSecond_30Hz_depth/30)/Mbits_Compressed;
      float Mbits_Uncompressed      = (8 * total_byte_base)/(1024 * 1024);
      float Mbits_Uncompressed_30f  = (8 * 30.0 * total_byte_base)/(1024 * 1024);

      set_debug_message("{    \"<i>Mbits Uncompressed</i>\" : \""           + toString<float>(Mbits_Uncompressed) + 
                          "\",\"<i>Mbits Uncompressed@30 Frames</i>\" : \"" + toString<float>(Mbits_Uncompressed_30f) + 
                          "\",\"<i>Mbits Compressed</i>\" : \""             + toString<float>(Mbits_Compressed) + 
                          "\",\"<i>Mbits Compressed@30 Frames</i>\" : \""   + toString<float>(Mbits_Compressed_30f) + 
                          "\",\"<i>Remaining Size</i>\" : \"~"              + toString<float>(total_compression_ratio_percent) + "%" +
                          "\",\"<i>Compression Ratio</i>\" : \"~1:"         + toString<float>(comp_ratio) +
                          "\",\"Remaining Size (color)\" : \"~"      + toString<float>(total_compression_ratio_color_percent) + "%" +
                          "\",\"Compression Ratio (color)\" : \"~1:" + toString<float>(comp_ratio_color) +
                          "\",\"Remaining Size (depth)\" : \"~"      + toString<float>(total_compression_ratio_depth_percent) + "%" +
                          "\",\"Compression Ratio (depth)\" : \"~1:" + toString<float>(comp_ratio_depth) +
                          "\",\"Color ratio\" : \""                  + toString<float>(comp_color_ratio) + 
                          "\",\"Depth ratio\" : \""                  + toString<float>(comp_depth_ratio) + 
                          "\"}");
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
