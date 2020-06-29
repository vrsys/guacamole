#include <gua/video3d/video3d_geometry/NetKinectArray.hpp>
#include <gua/video3d/Video3DResource.hpp>

#include <zmq.hpp>

#include <iostream>
#include <mutex>

namespace video3d
{
NetKinectArray::NetKinectArray(const std::vector<std::shared_ptr<KinectCalibrationFile>>& calib_files, const std::string& server_endpoint, unsigned colorsize_byte, unsigned depthsize_byte)
    : m_mutex(), m_running(true), m_server_endpoint(server_endpoint), m_calib_files(calib_files), m_colorsize_byte(colorsize_byte), m_depthsize_byte(depthsize_byte),
      m_buffer((m_colorsize_byte + m_depthsize_byte) * m_calib_files.size()), m_buffer_back((m_colorsize_byte + m_depthsize_byte) * m_calib_files.size()), m_need_swap(false), m_recv()
{
    m_recv = std::thread([this]() { readloop(); });
}

NetKinectArray::~NetKinectArray()
{
    m_running.store(false);
    m_recv.join();
}

bool NetKinectArray::update(uint8_t* mapped_pbo_back_pointer_back, ::gua::Video3DResource const& video3d_ressource)
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if(m_need_swap.load())
        {
            m_buffer.swap(m_buffer_back);
            m_need_swap.store(false);

            {

                unsigned char* buff = m_buffer.data();
                int64_t current_write_offset = 0;
                for(unsigned i = 0; i < video3d_ressource.number_of_cameras(); ++i)
                {
                    memcpy((char*) &mapped_pbo_back_pointer_back[current_write_offset], (void*) buff, video3d_ressource.color_size());
                    buff += video3d_ressource.color_size();
                    current_write_offset += video3d_ressource.color_size();
                    memcpy((char*) &mapped_pbo_back_pointer_back[current_write_offset], (void*) buff, video3d_ressource.depth_size_byte());
                    buff += video3d_ressource.depth_size_byte();
                    current_write_offset += video3d_ressource.depth_size_byte();
                }

            }


            return true;
        }
    }
    return false;
}

void NetKinectArray::readloop()
{
    // open multicast listening connection to server and port
    zmq::context_t ctx(1);              // means single threaded
    zmq::socket_t socket(ctx, ZMQ_SUB); // means a subscriber

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

    const unsigned message_size = (m_colorsize_byte + m_depthsize_byte) * m_calib_files.size();

    while(m_running)
    {
        zmq::message_t zmqm(message_size);
        socket.recv(&zmqm); // blocking

        while(true)
        {
            std::lock_guard<std::mutex> lock(m_mutex);

            if(!m_need_swap.load() || !m_running.load())
            {
                break;
            }
        }

        memcpy((unsigned char*)m_buffer_back.data(), (unsigned char*)zmqm.data(), message_size);
        { // swap
            std::lock_guard<std::mutex> lock(m_mutex);
            m_need_swap.store(true);
        }
    }
}
} // namespace video3d
