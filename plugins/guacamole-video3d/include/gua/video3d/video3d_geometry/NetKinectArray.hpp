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

    void copy_decompressed_jpeg_images(uint8_t* mapped_pbo_back_pointer_back);

    bool update(uint8_t* mapped_pbo_back_pointer /*, ::gua::Video3DResource const& video3d_ressource*/);
    bool decompress_images();
    inline unsigned char* getBuffer() { return m_buffer.data(); }

    bool try_register_resource(::gua::Video3DResource* video3d_ressource);
  private:
    void readloop();

    std::mutex m_mutex;
    std::mutex m_mutex_decompression;

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

    std::vector<uint8_t> m_decompressed_images;
    std::vector<uint8_t> m_decompressed_images_back;
    std::vector<uint8_t> m_buffer_to_decompress;
    std::vector<uint8_t> m_buffer_to_decompress_back;
    std::vector<uint8_t> m_buffer;
    std::vector<uint8_t> m_buffer_back;
    std::atomic<bool> m_need_swap_decompression;
    std::atomic<bool> m_need_swap;

    std::atomic<bool> m_needs_decompression;
    std::atomic<bool> m_needs_decompression_back;
    std::atomic<bool> m_has_decompressed_images;

    std::thread m_recv;
    std::thread m_decompress;

    ::gua::Video3DResource* registered_video_3d_resource = nullptr;
};

} // namespace video3d

#endif // #ifndef VIDEO3D_NETKINECTARRAY_HPP
