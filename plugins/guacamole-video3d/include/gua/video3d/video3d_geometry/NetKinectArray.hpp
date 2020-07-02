#ifndef VIDEO3D_NETKINECTARRAY_HPP
#define VIDEO3D_NETKINECTARRAY_HPP

#include <gua/video3d/video3d_geometry/KinectCalibrationFile.hpp>
#include <gua/video3d/Video3DResource.hpp>

#include <atomic>
#include <mutex>
#include <thread>

#ifdef GUACAMOLE_ENABLE_TURBOJPEG
#include <turbojpeg.h>
#endif //GUACAMOLE_ENABLE_TURBOJPEG

namespace video3d
{
class NetKinectArray
{
  public:
    NetKinectArray(const std::vector<std::shared_ptr<KinectCalibrationFile>>& calib_files, const std::string& server_endpoint, unsigned colorsize_byte, unsigned depthsize_byte);
    ~NetKinectArray();

    bool update(uint8_t* mapped_pbo_back_pointer, ::gua::Video3DResource const& video3d_ressource);
    bool decompress_images(uint8_t* mapped_pbo_back_pointer, ::gua::Video3DResource const& video3d_ressource, unsigned char* buff);
    inline unsigned char* getBuffer() { return m_buffer.data(); }

  private:
    void readloop();

    std::mutex m_mutex;
    std::atomic<bool> m_running;
    const std::string m_server_endpoint;
    std::vector<std::shared_ptr<KinectCalibrationFile>> m_calib_files;

#ifdef GUACAMOLE_ENABLE_TURBOJPEG
    std::vector<tjhandle> m_jpeg_decompressor_per_layer;
#endif //GUACAMOLE_ENABLE_TURBOJPEG

    unsigned m_colorsize_byte;
    unsigned m_depthsize_byte;

    bool is_jpeg_compressed_back = false;
    bool is_jpeg_compressed      = false;

    std::vector<uint8_t> m_buffer;
    std::vector<uint8_t> m_buffer_back;
    std::atomic<bool> m_need_swap;
    std::thread m_recv;
};

} // namespace video3d

#endif // #ifndef VIDEO3D_NETKINECTARRAY_HPP
