#ifndef VIDEO3D_NETKINECTARRAY_HPP
#define VIDEO3D_NETKINECTARRAY_HPP

#include <gua/renderer/video3d_geometry/KinectCalibrationFile.hpp>

namespace boost{
  class thread;
  class mutex;
}

namespace video3d{


  class NetKinectArray{
  
  public:
    NetKinectArray(const std::vector<std::shared_ptr<KinectCalibrationFile>>& calib_files,
		   const std::string& server_endpoint, unsigned colorsize_byte, unsigned depthsize_byte);
    ~NetKinectArray();

    bool update();
    unsigned char* getBuffer();

  private:
    void init();
    void readloop();


    boost::mutex* m_mutex;
    boost::thread* m_recv;
    bool           m_running;
    const std::string m_server_endpoint;
    std::vector<std::shared_ptr<KinectCalibrationFile>> m_calib_files;
    unsigned m_colorsize_byte;
    unsigned m_depthsize_byte;
    unsigned char* m_buffer;
    unsigned char* m_buffer_back;
    bool m_need_swap;
    bool m_need_swap2;

  };


}


#endif // #ifndef VIDEO3D_NETKINECTARRAY_HPP
