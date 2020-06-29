#ifndef VIDEO3D_NETKINECTARRAY_HPP
#define VIDEO3D_NETKINECTARRAY_HPP

#include <gua/video3d/video3d_geometry/KinectCalibrationFile.hpp>
#include <gua/video3d/Video3DResource.hpp>

#include <atomic>
#include <mutex>
#include <thread>

namespace video3d
{
class NetKinectArray
{
  public:
    NetKinectArray(const std::vector<std::shared_ptr<KinectCalibrationFile>>& calib_files, const std::string& server_endpoint, unsigned colorsize_byte, unsigned depthsize_byte);
    ~NetKinectArray();

    bool update(uint8_t* mapped_pbo_back_pointer, ::gua::Video3DResource const& video3d_ressource);
    inline unsigned char* getBuffer() { return m_buffer.data(); }

  private:
    void readloop();

    std::mutex m_mutex;
    std::atomic<bool> m_running;
    const std::string m_server_endpoint;
    std::vector<std::shared_ptr<KinectCalibrationFile>> m_calib_files;
    unsigned m_colorsize_byte;
    unsigned m_depthsize_byte;
    std::vector<uint8_t> m_buffer;
    std::vector<uint8_t> m_buffer_back;
    std::atomic<bool> m_need_swap;
    std::thread m_recv;
};

} // namespace video3d

#endif // #ifndef VIDEO3D_NETKINECTARRAY_HPP
