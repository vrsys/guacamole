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
#ifdef GUACAMOLE_ENABLE_TURBOJPEG
    m_jpeg_decompressor_per_layer.resize(calib_files.size(), 0);
#endif
    m_recv = std::thread([this]() { readloop(); });
}

NetKinectArray::~NetKinectArray()
{
    m_running.store(false);
    m_recv.join();
}

bool NetKinectArray::decompress_images(uint8_t* mapped_pbo_back_pointer_back, ::gua::Video3DResource const& video3d_ressource, unsigned char* buff) {
                        for(uint32_t sensor_layer_idx = 0; sensor_layer_idx < video3d_ressource.number_of_cameras(); ++sensor_layer_idx)
                    {
                        //if(m_jpeg_decompressor_per_layer.end() == m_jpeg_decompressor_per_layer.find(sensor_layer_idx))
                        if(0 == m_jpeg_decompressor_per_layer[sensor_layer_idx])
                        {
                            m_jpeg_decompressor_per_layer[sensor_layer_idx] = tjInitDecompress();
                            if(m_jpeg_decompressor_per_layer[sensor_layer_idx] == NULL)
                            {
                                std::cout << "ERROR INITIALIZING DECOMPRESSOR\n";
                            }
                        }
                    }
        
                    uint64_t color_write_offset = 0;
                    //#pragma omp parallel for num_threads(4)


                    std::vector<unsigned long int> jpeg_sizes(video3d_ressource.number_of_cameras(), 0);


                    unsigned long int buff_read_offset = 0;
                    size_t sizeof_unsigned_long_int = sizeof(unsigned long int);
                    for(uint32_t sensor_layer_idx = 0; sensor_layer_idx < video3d_ressource.number_of_cameras(); ++sensor_layer_idx) {
                        memcpy(&jpeg_sizes[sensor_layer_idx], (void*) &buff[buff_read_offset], sizeof_unsigned_long_int);
                        buff_read_offset += sizeof_unsigned_long_int + jpeg_sizes[sensor_layer_idx];
                    }

                    #pragma omp parallel for
                    for(uint32_t sensor_layer_idx = 0; sensor_layer_idx < video3d_ressource.number_of_cameras(); ++sensor_layer_idx) 
                    {
                        

                        //memcpy((char*)&m_tj_compressed_image_buffer_per_layer_[sensor_layer_idx][0], (char*)&m_texture_buffer_back_compressed_[byte_offset_to_current_image], jpeg_size);



                        auto& current_decompressor_handle = m_jpeg_decompressor_per_layer[sensor_layer_idx];
    

                        int header_width = {0};
                        int header_height = {0};
                        int header_subsamp = {0};

                        int error_handles = {0};

                        unsigned long int current_jpeg_size = jpeg_sizes[sensor_layer_idx];

                       //std::cout << "Sizeof jpeg size" << std::endl;

                        //memcpy(&jpeg_size, (void*) buff, sizeof(jpeg_size));

                        //buff += sizeof(jpeg_size);

                        //std::cout << "jpeg size: " << jpeg_size << std::endl;

                        unsigned long int jpeg_payload_offset_to_current_image = 0;

                        for(uint32_t preceeding_sensor_layer_idx = 0; preceeding_sensor_layer_idx < sensor_layer_idx; ++preceeding_sensor_layer_idx) {
                            jpeg_payload_offset_to_current_image += sizeof_unsigned_long_int + jpeg_sizes[preceeding_sensor_layer_idx];
                        }

                        //skip the bytes for the size of the image
                        jpeg_payload_offset_to_current_image += sizeof_unsigned_long_int;

                        error_handles = tjDecompressHeader2(current_decompressor_handle, (unsigned char*) &buff[jpeg_payload_offset_to_current_image], current_jpeg_size, &(header_width), &(header_height), &(header_subsamp) );
                    


                        if(error_handles < 0) {
                            std::cout << tjGetErrorStr() << std::endl;
                        }


                        std::vector<unsigned char> test(header_width * header_height * 3);
                        
                        error_handles = tjDecompress2(current_decompressor_handle,
                                      (unsigned char*) &buff[jpeg_payload_offset_to_current_image],
                                      current_jpeg_size,
                                      &mapped_pbo_back_pointer_back[sensor_layer_idx * (video3d_ressource.color_size() + video3d_ressource.depth_size_byte() ) ],
                                      header_width,
                                      0,
                                      header_height,
                                      TJPF_RGB,
                                      0);
                        
                        if(error_handles < 0) {
                            std::cout << tjGetErrorStr() << std::endl;
                        }
            

                        //memcpy(&mapped_pbo_back_pointer_back[color_write_offset], &test[0], uint64_t(2560)*1440*3);
                        ///color_write_offset += video3d_ressource.color_size();
                        //color_write_offset += video3d_ressource.depth_size_byte();
                        //buff += jpeg_size;

                        //std::cout << "Decompressed params: " << header_width << "  " << header_height << std::endl;

                    }

}

bool NetKinectArray::update(uint8_t* mapped_pbo_back_pointer_back, ::gua::Video3DResource const& video3d_ressource)
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if(m_need_swap.load())
        {
            m_buffer.swap(m_buffer_back);
            std::swap(is_jpeg_compressed_back, is_jpeg_compressed);

            m_need_swap.store(false);

            {
                unsigned char* buff = m_buffer.data();
                int64_t current_write_offset = 0;
                for(unsigned i = 0; i < video3d_ressource.number_of_cameras(); ++i)
                {
                    if(!is_jpeg_compressed) {
                        memcpy((char*) &mapped_pbo_back_pointer_back[current_write_offset], (void*) buff, video3d_ressource.color_size());
                        buff += video3d_ressource.color_size();
                        //leave the buffer alone for a moment
                    }
                    current_write_offset += video3d_ressource.color_size();
                    memcpy((char*) &mapped_pbo_back_pointer_back[current_write_offset], (void*) buff, video3d_ressource.depth_size_byte());
                    buff += video3d_ressource.depth_size_byte();
                    current_write_offset += video3d_ressource.depth_size_byte();
                }


#ifdef GUACAMOLE_ENABLE_TURBOJPEG
                if(is_jpeg_compressed) {
                    decompress_images(mapped_pbo_back_pointer_back, video3d_ressource, buff);
                }
#endif
                

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

/*
#if ZMQ_VERSION_MAJOR < 3
    int64_t hwm = 1;
    socket.setsockopt(ZMQ_HWM, &hwm, sizeof(hwm));
#else
    int hwm = 1;
    socket.setsockopt(ZMQ_RCVHWM, &hwm, sizeof(hwm));
#endif
*/
    int conflate_messages = 1;
    socket.setsockopt(ZMQ_CONFLATE, &conflate_messages, sizeof(conflate_messages));

    std::string endpoint("tcp://" + m_server_endpoint);
    socket.connect(endpoint.c_str());

    const unsigned message_size = (m_colorsize_byte + m_depthsize_byte) * m_calib_files.size();

    double elapsed_time = 0.0;
    uint32_t num_avatars_received = 0;
    while(m_running)
    {
        auto start_receive_avatar = std::chrono::system_clock::now();

        const unsigned message_size = sizeof(size_t);

        zmq::message_t zmqm;//(message_size);
        socket.recv(&zmqm); // blocking

        ++num_avatars_received;
        while(true)
        {
            std::lock_guard<std::mutex> lock(m_mutex);

            if(!m_need_swap.load() || !m_running.load())
            {
                break;
            }
        }

        uint64_t received_num_bytes = zmqm.size();

        memcpy((unsigned char*)m_buffer_back.data(), (unsigned char*)zmqm.data(), received_num_bytes);

        if(received_num_bytes < (m_colorsize_byte/4.0 + m_depthsize_byte) * m_calib_files.size()  ) {
            std::cout << "Probably JPEG compressed! " << std::endl;
            std::cout << "Message size: " << received_num_bytes << std::endl;
            is_jpeg_compressed_back = true;
        } else {
            is_jpeg_compressed_back = false;
        } //else {

        { // swap
            std::lock_guard<std::mutex> lock(m_mutex);
            m_need_swap.store(true);
        }
        //}

        auto end_receive_avatar = std::chrono::system_clock::now();

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_receive_avatar - start_receive_avatar);

        elapsed_time += elapsed.count();

        if(elapsed_time > 1000.0) {
            std::cout << "Received " << num_avatars_received << " Avatars in the last second" << std::endl;
            num_avatars_received = 0;
            elapsed_time -= 1000.0;
        }
    }
}
} // namespace video3d
