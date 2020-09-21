#include <gua/video3d/video3d_geometry/NetKinectArray.hpp>
#include <gua/video3d/Video3DResource.hpp>

#include <zmq.hpp>

#include <iostream>
#include <mutex>

namespace video3d
{
NetKinectArray::NetKinectArray(const std::vector<std::shared_ptr<KinectCalibrationFile>>& calib_files, const std::string& server_endpoint, unsigned colorsize_byte, unsigned depthsize_byte)
    : m_mutex(), m_mutex_decompression(), m_running(true), m_needs_decompression(false), m_needs_decompression_back(false), m_has_decompressed_images(false), m_server_endpoint(server_endpoint), m_calib_files(calib_files), m_colorsize_byte(colorsize_byte), m_depthsize_byte(depthsize_byte),
      m_decompressed_images( (m_colorsize_byte + m_depthsize_byte) * m_calib_files.size() ), m_decompressed_images_back(m_decompressed_images.size()), m_buffer_to_decompress((m_colorsize_byte + m_depthsize_byte  ) * m_calib_files.size()), m_buffer_to_decompress_back((m_colorsize_byte + m_depthsize_byte  ) * m_calib_files.size()), m_buffer((m_colorsize_byte + m_depthsize_byte) * m_calib_files.size()), m_buffer_back((m_colorsize_byte + m_depthsize_byte) * m_calib_files.size()), m_need_swap(false), m_recv()
{
#ifdef GUACAMOLE_ENABLE_TURBOJPEG
    m_jpeg_decompressor_per_layer.resize(calib_files.size(), 0);
    //m_decompress = std::thread([this]() {} )
#endif
    m_recv = std::thread([this]() { readloop(); });
    m_decompress = std::thread([this]() { decompress_images(); } );
}

NetKinectArray::~NetKinectArray()
{
    m_running.store(false);
    m_recv.join();
    m_decompress.join();
}


void NetKinectArray::copy_decompressed_jpeg_images(uint8_t* mapped_pbo_back_pointer_back) {
    memcpy(mapped_pbo_back_pointer_back, m_decompressed_images.data(), m_decompressed_images.size());
}

bool NetKinectArray::decompress_images() {

    double elapsed_time = 0.0;
    uint32_t num_avatars_decompressed = 0;
    while(true) {
        //std::cout << "Busy decompression " << std::endl;

        auto start_decompress_avatar = std::chrono::system_clock::now();

        // we got the signal from the update thread and start working
        if(m_needs_decompression_back.load()) {
            auto const& video3d_ressource = *registered_video_3d_resource;




            //m_buffer_to_decompress.swap(m_buffer_to_decompress_back);
            //m_has_decompressed_images.store(true);



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


            std::vector<unsigned long int> jpeg_sizes(video3d_ressource.number_of_cameras(), 0);


            unsigned long int buff_read_offset = video3d_ressource.number_of_cameras() * video3d_ressource.depth_size_byte();
            size_t sizeof_unsigned_long_int = sizeof(unsigned long int);
            for(uint32_t sensor_layer_idx = 0; sensor_layer_idx < video3d_ressource.number_of_cameras(); ++sensor_layer_idx) {
                memcpy(&jpeg_sizes[sensor_layer_idx], (void*) &m_buffer_to_decompress_back[buff_read_offset], sizeof_unsigned_long_int);
                buff_read_offset += sizeof_unsigned_long_int + jpeg_sizes[sensor_layer_idx];
            }

            #pragma omp parallel for
            for(uint32_t sensor_layer_idx = 0; sensor_layer_idx < video3d_ressource.number_of_cameras(); ++sensor_layer_idx) 
            {
                auto& current_decompressor_handle = m_jpeg_decompressor_per_layer[sensor_layer_idx];


                int header_width = {0};
                int header_height = {0};
                int header_subsamp = {0};

                int error_handles = {0};

                unsigned long int current_jpeg_size = jpeg_sizes[sensor_layer_idx];

                unsigned long int jpeg_payload_offset_to_current_image = 0;

                for(uint32_t preceeding_sensor_layer_idx = 0; preceeding_sensor_layer_idx < sensor_layer_idx; ++preceeding_sensor_layer_idx) {
                    jpeg_payload_offset_to_current_image += sizeof_unsigned_long_int + jpeg_sizes[preceeding_sensor_layer_idx];
                }

                jpeg_payload_offset_to_current_image += video3d_ressource.number_of_cameras() * video3d_ressource.depth_size_byte();

                //skip the bytes for the size of the image
                jpeg_payload_offset_to_current_image += sizeof_unsigned_long_int;

                error_handles = tjDecompressHeader2(current_decompressor_handle, (unsigned char*) &m_buffer_to_decompress_back[jpeg_payload_offset_to_current_image], current_jpeg_size, &(header_width), &(header_height), &(header_subsamp) );
            


                if(error_handles < 0) {
                    std::cout << tjGetErrorStr() << std::endl;
                }


                std::vector<unsigned char> test(header_width * header_height * 3);
                


                error_handles = tjDecompress2(current_decompressor_handle,
                              (unsigned char*) &m_buffer_to_decompress_back[jpeg_payload_offset_to_current_image],
                              current_jpeg_size,
                              &m_decompressed_images_back[sensor_layer_idx * (video3d_ressource.color_size() + video3d_ressource.depth_size_byte()) ],
                              //&mapped_pbo_back_pointer_back[sensor_layer_idx * (video3d_ressource.color_size() + video3d_ressource.depth_size_byte() ) ],
                              header_width,
                              0,
                              header_height,
                              TJPF_RGB,
                              TJFLAG_FASTDCT);
                
                if(error_handles < 0) {
                    std::cout << tjGetErrorStr() << std::endl;
                }

            }

            
            //unsigned char* buff = m_buffer_to_decompress.data();
            #pragma omp parallel for
            for(uint32_t sensor_layer_idx = 0; sensor_layer_idx < video3d_ressource.number_of_cameras(); ++sensor_layer_idx) 
            {
                uint64_t depth_image_write_offset = (sensor_layer_idx + 1 ) * (video3d_ressource.color_size() + video3d_ressource.depth_size_byte()) - video3d_ressource.depth_size_byte();
                uint64_t depth_image_read_offset = (sensor_layer_idx) * (video3d_ressource.depth_size_byte());

                memcpy(&m_decompressed_images_back[depth_image_write_offset], &m_buffer_to_decompress_back[depth_image_read_offset], video3d_ressource.depth_size_byte());
            }            

            {
                std::lock_guard<std::mutex> lock(m_mutex_decompression);
                m_decompressed_images.swap(m_decompressed_images_back);
            }
            m_has_decompressed_images.store(true);

            num_avatars_decompressed+=1;
            m_needs_decompression_back.store(false);
        }
       // }


        auto end_decompress_avatar = std::chrono::system_clock::now();

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_decompress_avatar - start_decompress_avatar);

        elapsed_time += elapsed.count();

        if(elapsed_time > 1000.0) {
            //std::cout << "Decompressed " << num_avatars_decompressed << " Avatars in the last second" << std::endl;
            num_avatars_decompressed = 0;
            elapsed_time -= 1000.0;
        }

    }

}

bool NetKinectArray::update(uint8_t* mapped_pbo_back_pointer_back/*, ::gua::Video3DResource const& video3d_ressource*/)
{

    auto const& video3d_resource = *registered_video_3d_resource;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if(m_need_swap.load())
        {
            m_buffer.swap(m_buffer_back);
            std::swap(is_jpeg_compressed_back, is_jpeg_compressed);

            m_need_swap.store(false);

            
#ifdef GUACAMOLE_ENABLE_TURBOJPEG
        if(is_jpeg_compressed) {
            bool return_val = true;
            if(!m_needs_decompression_back.load()) {
                            
            // we give our buffer to the decompression and tell it that there is work to do
                m_buffer_to_decompress_back.swap(m_buffer);
                m_needs_decompression_back.store(true);
            }

            if(m_has_decompressed_images.load()) {
                // get latest decompressed buffers
                std::lock_guard<std::mutex> lock(m_mutex_decompression);
                copy_decompressed_jpeg_images(mapped_pbo_back_pointer_back);
            } else {
                return_val = false;
            }

            return return_val;
        } else  
#endif


        {
            unsigned char* buff = m_buffer.data();
            int64_t current_write_offset = 0;
            for(unsigned i = 0; i < video3d_resource.number_of_cameras(); ++i)
            {
                if(!is_jpeg_compressed) {
                    memcpy((char*) &mapped_pbo_back_pointer_back[current_write_offset], (void*) buff, video3d_resource.color_size());
                    buff += video3d_resource.color_size();
                    //leave the buffer alone for a moment
                }
                current_write_offset += video3d_resource.color_size();
                memcpy((char*) &mapped_pbo_back_pointer_back[current_write_offset], (void*) buff, video3d_resource.depth_size_byte());
                buff += video3d_resource.depth_size_byte();
                current_write_offset += video3d_resource.depth_size_byte();
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

    int conflate_messages = 1;
    socket.setsockopt(ZMQ_CONFLATE, &conflate_messages, sizeof(conflate_messages));

    std::string endpoint("tcp://" + m_server_endpoint);
    socket.connect(endpoint.c_str());

    // const unsigned message_size = (m_colorsize_byte + m_depthsize_byte) * m_calib_files.size();

    double elapsed_time = 0.0;
    uint32_t num_avatars_received = 0;
    while(m_running)
    {
        auto start_receive_avatar = std::chrono::system_clock::now();

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
            is_jpeg_compressed_back = true;
        } else {
            is_jpeg_compressed_back = false;
        } //else {

        { // swap
            std::lock_guard<std::mutex> lock(m_mutex);
            m_need_swap.store(true);
        }


        auto end_receive_avatar = std::chrono::system_clock::now();

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_receive_avatar - start_receive_avatar);

        elapsed_time += elapsed.count();

        if(elapsed_time > 1000.0) {
            //std::cout << "Received " << num_avatars_received << " Avatars in the last second" << std::endl;
            num_avatars_received = 0;
            elapsed_time -= 1000.0;
        }
    }
}

bool NetKinectArray::try_register_resource(::gua::Video3DResource* video3d_resource_ptr) {
    if(nullptr == registered_video_3d_resource) {
        registered_video_3d_resource = video3d_resource_ptr;
        return true;
    }

    return false;
}


} // namespace video3d
