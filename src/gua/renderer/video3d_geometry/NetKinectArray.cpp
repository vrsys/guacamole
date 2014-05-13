#include <gua/renderer/video3d_geometry/NetKinectArray.hpp>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>

#include <zmq.hpp>

#include <iostream>

namespace video3d{

  NetKinectArray::NetKinectArray(const std::vector<std::shared_ptr<KinectCalibrationFile>>& calib_files,
				 const std::string& server_endpoint, unsigned colorsize_byte, unsigned depthsize_byte)
    : m_mutex(new boost::mutex),
      m_recv(0),
      m_running(true),
      m_server_endpoint(server_endpoint),
      m_calib_files(calib_files),
      m_colorsize_byte(colorsize_byte),
      m_depthsize_byte(depthsize_byte),
      m_buffer(0),
      m_buffer_back(0),
      m_need_swap(false),
      m_need_swap2(false)
      
  {
    init();
  }

  NetKinectArray::~NetKinectArray()
  {}

  bool
  NetKinectArray::update(){

    {
      boost::mutex::scoped_lock lock(*m_mutex);
      if(m_need_swap && m_need_swap2){
	std::swap(m_buffer, m_buffer_back);
	m_need_swap = false;
	m_need_swap2 = false;
	return true;
      }
    }
    return false;
  }

  unsigned char*
  NetKinectArray::getBuffer(){
    return m_buffer;
  }

  void
  NetKinectArray::init(){


    m_buffer = new unsigned char [ (m_colorsize_byte + m_depthsize_byte) * m_calib_files.size() ];
    m_buffer_back = new unsigned char [ (m_colorsize_byte + m_depthsize_byte) * m_calib_files.size() ];
    m_recv = new boost::thread(boost::bind(&NetKinectArray::readloop, this));
  }


  void
  NetKinectArray::readloop(){


    // open multicast listening connection to server and port
    zmq::context_t ctx(1); // means single threaded
    zmq::socket_t  socket(ctx, ZMQ_SUB); // means a subscriber

    socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    int hwm = 1;
    socket.setsockopt(ZMQ_RCVHWM, &hwm, sizeof(hwm));
    
    std::string endpoint("tcp://" + m_server_endpoint);
    socket.connect(endpoint.c_str());
    
    const unsigned message_size = (m_colorsize_byte + m_depthsize_byte) * m_calib_files.size();
    while(m_running){

      zmq::message_t zmqm(message_size);
      socket.recv(&zmqm); // blocking

      while(m_need_swap && m_need_swap2){
	;
      }

      memcpy((unsigned char*) m_buffer_back, (unsigned char*) zmqm.data(), message_size);
      { // swap
        boost::mutex::scoped_lock lock(*m_mutex);
        m_need_swap = true;
        m_need_swap2 = true;
      }

    }
  }


}
