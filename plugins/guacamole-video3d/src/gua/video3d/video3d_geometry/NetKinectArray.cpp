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
    m_recv()
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
  zmq::socket_t  socket_f(ctx, ZMQ_PUB);  // means a Publisher

  socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);
#if ZMQ_VERSION_MAJOR < 3
  int64_t hwm = 1;
  socket.setsockopt(ZMQ_HWM, &hwm, sizeof(hwm));
  socket_f.setsockopt(ZMQ_HWM, &hwm, sizeof(hwm));
#else
  int hwm = 1;
  socket.setsockopt(ZMQ_RCVHWM, &hwm, sizeof(hwm));
  socket_f.setsockopt(ZMQ_RCVHWM, &hwm, sizeof(hwm));
#endif
  std::string endpoint("tcp://" + m_server_endpoint);
  std::string endpoint_f("tcp://141.54.147.108:7001");
  socket.connect(endpoint.c_str());
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

    zmq::message_t zmqm_f(6*sizeof(float));
    std::vector<float> test_inputs;
    test_inputs.push_back(cont);
    cont += 1;
    test_inputs.push_back(cont);
    cont += 1;
    test_inputs.push_back(cont);
    cont += 1;
    test_inputs.push_back(cont);
    cont += 1;
    test_inputs.push_back(cont);
    cont += 1;
    test_inputs.push_back(cont);
    cont += 1;
    
    memcpy(zmqm_f.data(), test_inputs.data(), 6*sizeof(float));    
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
