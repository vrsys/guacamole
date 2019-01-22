#include <gua/spoints/spoints_geometry/NetKinectArray.hpp>
#include <gua/spoints/spoints_geometry/codec/point_types.h>

#include <boost/assign/list_of.hpp>


#include <gua/spoints/sgtp/SGTP.h>

#include <zmq.hpp>
#include <iostream>
#include <mutex>

#include <turbojpeg.h>

namespace spoints {

#define ONE_D_TEXTURE_ATLAS_SIZE 2048

NetKinectArray::NetKinectArray(const std::string& server_endpoint,
                               const std::string& feedback_endpoint)
  : m_mutex_(),
    m_running_(true),
    //m_feedback_running_(true),
    m_server_endpoint_(server_endpoint),
    m_feedback_endpoint_(feedback_endpoint),
    m_buffer_(/*(m_colorsize_byte + m_depthsize_byte) * m_calib_files.size()*/),
    m_buffer_back_( /*(m_colorsize_byte + m_depthsize_byte) * m_calib_files.size()*/),
    m_need_cpu_swap_{false},
    m_need_calibration_cpu_swap_{false},
    //m_feedback_need_swap_{false},
    m_recv_()
{
  m_recv_ = std::thread([this]() { readloop(); });
}

NetKinectArray::~NetKinectArray()
{
  m_running_ = false;
  m_recv_.join();
}


void 
NetKinectArray::draw_textured_triangle_soup(gua::RenderContext const& ctx, std::shared_ptr<gua::ShaderProgram>& shader_program) {

  auto const& current_point_layout = point_layout_per_context_[ctx.id];

  if( current_point_layout != nullptr ) {

    auto const& current_texture_atlas = texture_atlas_per_context_[ctx.id];

    auto const& current_inv_xyz_pointers = inv_xyz_calibs_per_context_[ctx.id];
    auto const& current_uv_pointers = uv_calibs_per_context_[ctx.id];

    scm::gl::context_vertex_input_guard vig(ctx.render_context);

    ctx.render_context->bind_vertex_array(current_point_layout);
    

    if( nullptr == linear_sampler_state_ ) {
      linear_sampler_state_ = ctx.render_device
        ->create_sampler_state(scm::gl::FILTER_MIN_MAG_LINEAR, scm::gl::WRAP_CLAMP_TO_EDGE);
    }

    auto& current_net_data_vbo = net_data_vbo_per_context_[ctx.id];
    


    ctx.render_context->bind_texture(current_texture_atlas, linear_sampler_state_, 0);

    //if(m_bound_calibration_data_.end() == m_bound_calibration_data_.find(ctx.id) ) {
      for(uint32_t sensor_idx = 0; sensor_idx < m_num_sensors_; ++sensor_idx) {
        auto const& current_individual_inv_xyz_texture = current_inv_xyz_pointers[sensor_idx];
        ctx.render_context->bind_texture(current_individual_inv_xyz_texture, linear_sampler_state_, sensor_idx + 1);
        
        //shader_program->set_uniform(ctx, int(sensor_idx), "inv_xyz_volumes[" + std::to_string(sensor_idx)+"]");     
        auto const& current_individual_uv_texture = current_uv_pointers[sensor_idx];
        ctx.render_context->bind_texture(current_individual_uv_texture, linear_sampler_state_, m_num_sensors_ + sensor_idx + 1); 
      }
      //m_bound_calibration_data_[ctx.id] = true;
    //}
    shader_program->set_uniform(ctx, m_inverse_vol_to_world_mat_, "inv_vol_to_world_matrix");    
    shader_program->set_uniform(ctx, m_lod_scaling_, "scaling_factor");

    size_t initial_vbo_size = 10000000;
    
    //compressed_LQ_one_pass_program_->storage_buffer("bvh_auxiliary_struct", 1);
    shader_program->set_uniform(ctx, int(3), "Out_Sorted_Vertex_Tri_Data");
    ctx.render_device->main_context()->bind_storage_buffer(current_net_data_vbo, 3, 0, initial_vbo_size);
    //ctx.render_device->main_context()->set_storage_buffers( std::vector<scm::gl::render_context::buffer_binding>{scm::gl::BIND_STORAGE_BUFFER} );
    
    ctx.render_device->main_context()->apply_storage_buffer_bindings();
    shader_program->set_uniform(ctx, int(m_triangle_texture_atlas_size_), "texture_space_triangle_size");
    //shader_program->set_uniform(ctx, 10, "Out_Sorted_Vertex_Tri_Data");

    uint32_t triangle_offset_for_current_layer = 0;
    uint32_t num_triangles_to_draw_for_current_layer = 0;

    for(int layer_idx = 0; layer_idx < 4; ++layer_idx) {
      num_triangles_to_draw_for_current_layer = m_num_best_triangles_for_sensor_layer_[layer_idx];

      shader_program->set_uniform(ctx, int(layer_idx), "current_sensor_layer");

      ctx.render_context->apply();
      
      size_t vertex_offset = triangle_offset_for_current_layer * 3;
      size_t num_vertices_to_draw = num_triangles_to_draw_for_current_layer * 3;
      size_t const current_num_tri_vertices_to_draw = num_textured_tris_to_draw_per_context_[ctx.id] * 3;


      ctx.render_context->draw_arrays(scm::gl::PRIMITIVE_TRIANGLE_LIST,
                                      vertex_offset,
                                      num_vertices_to_draw);

      triangle_offset_for_current_layer += num_triangles_to_draw_for_current_layer;
    }

    ctx.render_context->reset_vertex_input();
  }
}

std::string 
NetKinectArray::get_socket_string() const {
  return m_feedback_endpoint_;
};

void 
NetKinectArray::push_matrix_package(spoints::camera_matrix_package const& cam_mat_package) {
  submitted_camera_matrix_package_ = cam_mat_package;

  encountered_context_ids_for_feedback_frame_.insert(submitted_camera_matrix_package_.k_package.render_context_id);
}

bool
NetKinectArray::update(gua::RenderContext const& ctx, gua::math::BoundingBox<gua::math::vec3>& in_out_bb) {
  {
    auto& current_encountered_frame_count = encountered_frame_counts_per_context_[ctx.id];

    if (current_encountered_frame_count != ctx.framecount) {
      current_encountered_frame_count = ctx.framecount;
    } else {
    
      return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex_);
    if(m_need_calibration_cpu_swap_.load()) {
      m_calibration_.swap(m_calibration_back_);

      std::swap(m_inv_xyz_calibration_res_, m_inv_xyz_calibration_res_back_); 
      std::swap(m_uv_calibration_res_, m_uv_calibration_res_back_);
      std::swap(m_num_sensors_, m_num_sensors_back_);
      std::swap(m_inverse_vol_to_world_mat_back_, m_inverse_vol_to_world_mat_);

      m_need_calibration_cpu_swap_.store(false);
    }

    if(m_need_calibration_gpu_swap_[ctx.id].load()) {

      inv_xyz_calibs_per_context_[ctx.id] = std::vector<scm::gl::texture_3d_ptr>(m_num_sensors_, nullptr);
      uv_calibs_per_context_[ctx.id] = std::vector<scm::gl::texture_3d_ptr>(m_num_sensors_, nullptr);

      std::size_t current_read_offset = 0;
      std::size_t const num_bytes_per_inv_xyz_volume 
        = 4 * sizeof(float) * 
          m_inv_xyz_calibration_res_[0] * m_inv_xyz_calibration_res_[1] * m_inv_xyz_calibration_res_[2];

      auto const volumetric_inv_xyz_region_to_update 
        = scm::gl::texture_region(scm::math::vec3ui(0, 0, 0), 
          scm::math::vec3ui(m_inv_xyz_calibration_res_[0], m_inv_xyz_calibration_res_[1], m_inv_xyz_calibration_res_[2]));

      auto const volumetric_uv_region_to_update 
        = scm::gl::texture_region(scm::math::vec3ui(0, 0, 0), 
          scm::math::vec3ui(m_uv_calibration_res_[0], m_uv_calibration_res_[1], m_uv_calibration_res_[2]));

      std::size_t const num_bytes_per_uv_volume 
        = 2 * sizeof(float) * 
          m_uv_calibration_res_[0] * m_uv_calibration_res_[1] * m_uv_calibration_res_[2];

      for(uint32_t sensor_idx = 0; sensor_idx < m_num_sensors_; ++sensor_idx) {
        // create and update calibration volume
        auto& current_inv_xyz_calibration_volume_ptr = inv_xyz_calibs_per_context_[ctx.id][sensor_idx];
        current_inv_xyz_calibration_volume_ptr 
          = ctx.render_device->create_texture_3d(scm::math::vec3ui(m_inv_xyz_calibration_res_[0], 
                                                                   m_inv_xyz_calibration_res_[1],
                                                                   m_inv_xyz_calibration_res_[2]), 
                                                                   scm::gl::FORMAT_RGBA_32F);

        ctx.render_device->main_context()->update_sub_texture(current_inv_xyz_calibration_volume_ptr, volumetric_inv_xyz_region_to_update, 0, 
                                                              scm::gl::FORMAT_RGBA_32F, (void*) &m_calibration_[current_read_offset]);
        current_read_offset += num_bytes_per_inv_xyz_volume;

        //=======================================================================================================

        // create and update calibration volume
        auto& current_uv_calibration_volume_ptr = uv_calibs_per_context_[ctx.id][sensor_idx];
        current_uv_calibration_volume_ptr 
          = ctx.render_device->create_texture_3d(scm::math::vec3ui(m_uv_calibration_res_[0], 
                                                                   m_uv_calibration_res_[1],
                                                                   m_uv_calibration_res_[2]), 
                                                                   scm::gl::FORMAT_RG_32F);

        ctx.render_device->main_context()->update_sub_texture(current_uv_calibration_volume_ptr, volumetric_uv_region_to_update, 0, 
                                                              scm::gl::FORMAT_RG_32F, (void*) &m_calibration_[current_read_offset]);
        current_read_offset += num_bytes_per_uv_volume;

      }

      //std::cout << "Loaded Volume Textures\n";
      //current_texture_atlas = ctx.render_device->create_texture_3d(scm::math::vec3ui(texture_width, texture_height), scm::gl::FORMAT_BGR_8, 1, 1, 1);
      m_need_calibration_gpu_swap_[ctx.id].store(false);
      m_received_calibration_[ctx.id].store(true);
      return true;
    }



    if(m_need_cpu_swap_.load()){
      //start of synchro point
      m_buffer_.swap(m_buffer_back_);
      m_texture_buffer_.swap(m_texture_buffer_back_);
      //std::swap(m_voxel_size_, m_voxel_size_back_);


      std::swap(m_received_textured_tris_back_, m_received_textured_tris_);
      std::swap(m_received_kinect_timestamp_back_, m_received_kinect_timestamp_);
      std::swap(m_received_reconstruction_time_back_, m_received_reconstruction_time_);

      std::swap(m_texture_payload_size_in_byte_back_, m_texture_payload_size_in_byte_);
      std::swap(m_triangle_texture_atlas_size_back_, m_triangle_texture_atlas_size_);

      std::swap(m_num_best_triangles_for_sensor_layer_, 
                m_num_best_triangles_for_sensor_layer_back_);

      std::swap(m_lod_scaling_back_, m_lod_scaling_);

      std::swap(m_texture_space_bounding_boxes_back_,
                m_texture_space_bounding_boxes_);

      //end of synchro point
      m_need_cpu_swap_.store(false);
    }

    if(m_need_gpu_swap_[ctx.id].load()) {
      size_t total_num_bytes_to_copy = m_buffer_.size();

      if(0 != total_num_bytes_to_copy) {

        //size_t sizeof_point = 4*sizeof(float);
        //size_t sizeof_point = 2*sizeof(uint32_t);

/*        size_t sizeof_vertex_colored_point = 2*sizeof(uint32_t);
        size_t sizeof_vertex_colored_tri   = 3*sizeof_vertex_colored_point;
        size_t sizeof_textured_tri         = 3*sizeof_vertex_colored_point;
*/
        size_t sizeof_vertex_colored_point = 3*sizeof(float);
        size_t sizeof_vertex_colored_tri   = 3*3*sizeof(float);
        size_t sizeof_textured_tri         = 3*3*sizeof(float);

        num_textured_tris_to_draw_per_context_[ctx.id]         = m_received_textured_tris_;

        //num_points_to_draw_per_context_[ctx.id] = total_num_bytes_to_copy / sizeof_point;

        auto& current_is_vbo_created = is_vbo_created_per_context_[ctx.id];

        auto& current_empty_vbo = net_data_vbo_per_context_[ctx.id];
        auto& current_net_data_vbo = net_data_vbo_per_context_[ctx.id];
        auto& current_texture_atlas = texture_atlas_per_context_[ctx.id];


        if(!current_is_vbo_created) {
          current_empty_vbo = ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER, scm::gl::USAGE_STATIC_DRAW, 0, 0);

          size_t initial_vbo_size = 10000000;

          current_net_data_vbo = ctx.render_device->create_buffer(scm::gl::BIND_STORAGE_BUFFER, scm::gl::USAGE_DYNAMIC_COPY, initial_vbo_size, 0);
          

          size_t texture_width = 1280*2;
          size_t texture_height = 720*2;

          current_texture_atlas = ctx.render_device->create_texture_2d(scm::math::vec2ui(texture_width, texture_height), scm::gl::FORMAT_BGR_8, 1, 1, 1);
          //current_texture_atlas = ctx.render_device->create_texture_2d(scm::math::vec2ui(ONE_D_TEXTURE_ATLAS_SIZE, ONE_D_TEXTURE_ATLAS_SIZE), scm::gl::FORMAT_BGR_8, 1, 1, 1);
        }

          size_t initial_vbo_size = 10000000;
          ctx.render_device->main_context()->bind_storage_buffer(current_net_data_vbo, 3, 0, initial_vbo_size);

          ctx.render_device->main_context()->apply_storage_buffer_bindings();

          float dummy_data[9] = {0.0, 0.0, 0.0, 0.5, 1.0, 0.0, 1.0, 0.0, 0.0};
          //float dummy_data[9] = {0.0, 0.0, 0.5, 0.5, 0.0, 0.5, 0.5, 1.0, 0.5};


          float* mapped_net_data_vbo_ = (float*) ctx.render_device->main_context()->map_buffer(current_net_data_vbo, scm::gl::access_mode::ACCESS_WRITE_ONLY);
          memcpy((char*) mapped_net_data_vbo_, (char*) &m_buffer_[0], total_num_bytes_to_copy);
          //memcpy((char*) mapped_net_data_vbo_, (char*) &dummy_data[0], 9*sizeof(float));


          //std::cout << ""

          remote_server_screen_width_to_return_ = remote_server_screen_width_;
          remote_server_screen_height_to_return_ = remote_server_screen_height_;

          ctx.render_device->main_context()->unmap_buffer(current_net_data_vbo);
          

          uint32_t total_num_pixels_to_upload = m_texture_payload_size_in_byte_ / 3;

//          uint32_t num_pixels_u_direction = ( (total_num_pixels_to_upload / ONE_D_TEXTURE_ATLAS_SIZE) > 0) ?  ONE_D_TEXTURE_ATLAS_SIZE : total_num_pixels_to_upload;
//          uint32_t num_pixels_v_direction = ( (total_num_pixels_to_upload) / ONE_D_TEXTURE_ATLAS_SIZE == 0) ? total_num_pixels_to_upload : total_num_pixels_to_upload/ONE_D_TEXTURE_ATLAS_SIZE;

          uint32_t num_pixels_u_direction = 1280*2;
          uint32_t num_pixels_v_direction = 720*2;

          uint32_t byte_offset_per_texture_data_for_layers[16];
          std::vector<scm::gl::texture_region> regions_to_update(16);


          for(uint32_t layer_to_update_idx = 0; layer_to_update_idx < 16; ++layer_to_update_idx) {
              //initialize offset with 0 or value of last region and update incrementally
              if(0 == layer_to_update_idx) {
                byte_offset_per_texture_data_for_layers[layer_to_update_idx] = 0;
              } else {
                byte_offset_per_texture_data_for_layers[layer_to_update_idx] = byte_offset_per_texture_data_for_layers[layer_to_update_idx-1];
              
                if(m_num_best_triangles_for_sensor_layer_[layer_to_update_idx]) {
                  uint32_t layer_offset = 4 * (layer_to_update_idx-1);
                  uint32_t prev_bb_pixel_coverage =   (1 + m_texture_space_bounding_boxes_[layer_offset + 2] - m_texture_space_bounding_boxes_[layer_offset + 0])
                                                    * (1 + m_texture_space_bounding_boxes_[layer_offset + 3] - m_texture_space_bounding_boxes_[layer_offset + 1]);


                  byte_offset_per_texture_data_for_layers[layer_to_update_idx] += prev_bb_pixel_coverage * 3;
                }
              }


              if(m_num_best_triangles_for_sensor_layer_[layer_to_update_idx] > 0) {
                uint32_t current_layer_offset = 4 * (layer_to_update_idx);
                //uint32_t prev_bb_pixel_coverage =   (1 + m_texture_space_bounding_boxes_[layer_offset + 2] - m_texture_space_bounding_boxes_[layer_offset + 0])
                //                                    * (1 + m_texture_space_bounding_boxes_[layer_offset + 3] - m_texture_space_bounding_boxes_[layer_offset + 1]);


                auto current_region_to_update = 
                  scm::gl::texture_region(scm::math::vec3ui(m_texture_space_bounding_boxes_[current_layer_offset + 0]    , 
                                                            m_texture_space_bounding_boxes_[current_layer_offset + 1]
                                                            ,     0), 
                                          scm::math::vec3ui((m_texture_space_bounding_boxes_[current_layer_offset + 2] + 1 - m_texture_space_bounding_boxes_[current_layer_offset + 0]), 
                                                             (m_texture_space_bounding_boxes_[current_layer_offset + 3] + 1 - m_texture_space_bounding_boxes_[current_layer_offset + 1])
                                                             , 1));
              
                size_t current_read_offset = byte_offset_per_texture_data_for_layers[layer_to_update_idx];

                std::cout << "Trying to update the following region: " << "\n";
                std::cout      << m_texture_space_bounding_boxes_[current_layer_offset + 0] << ", " << m_texture_space_bounding_boxes_[current_layer_offset + 1] <<
                          ", " << m_texture_space_bounding_boxes_[current_layer_offset + 2] << ", " << m_texture_space_bounding_boxes_[current_layer_offset + 3] << "\n";

                std::cout << "Trying to read with offset: " << byte_offset_per_texture_data_for_layers[layer_to_update_idx] << " / " 
                                                            << m_texture_payload_size_in_byte_ << "\n";

                ctx.render_device->main_context()->update_sub_texture(current_texture_atlas, 
                                                                      current_region_to_update, 0, scm::gl::FORMAT_BGR_8, 
                                                                      (void*) &m_texture_buffer_[current_read_offset] );
                                                                      //(void*) &m_texture_buffer_[0] );
          
              }


          }

          //auto region_to_update = scm::gl::texture_region(scm::math::vec3ui(0, 0, 0), scm::math::vec3ui(num_pixels_u_direction, num_pixels_v_direction, 1));
          


          std::cout << "Pixels to upload: " << total_num_pixels_to_upload << "\n";
          //to replace
          //ctx.render_device->main_context()->update_sub_texture(current_texture_atlas, region_to_update, 0, scm::gl::FORMAT_BGR_8, (void*) &m_texture_buffer_[0]);


        if(!current_is_vbo_created) {
          auto& current_point_layout = point_layout_per_context_[ctx.id];

          size_t size_of_vertex = 3 * sizeof(float);

          current_point_layout = ctx.render_device->create_vertex_array(scm::gl::vertex_format
                                                                       (0, 0, scm::gl::TYPE_VEC3F, 0),
                                                                        boost::assign::list_of(current_empty_vbo));
          //deprecated
          //size_t size_of_vertex = 2 * sizeof(uint32_t);
          //current_point_layout = ctx.render_device->create_vertex_array(scm::gl::vertex_format
          //                                                             (0, 0, scm::gl::TYPE_VEC2UI, size_of_vertex, scm::gl::INT_PURE),
          //                                                              boost::assign::list_of(current_net_data_vbo));
          current_is_vbo_created = true;

        }


      } else {
        num_textured_tris_to_draw_per_context_[ctx.id]         = 0;
      }
      
      m_need_gpu_swap_[ctx.id].store(false);
      return true;
    }
  }
  return false;
}


/*float NetKinectArray::get_voxel_size() const {
  return m_voxel_size_;
}*/


void NetKinectArray::_decompress_and_rewrite_message() {
  //std::cout << "COPYING " << m_texture_payload_size_in_byte_back_ << " byte into texture source\n";
  //memcpy((unsigned char*) &m_texture_buffer_back_[0], ((unsigned char*) zmqm.data()) + HEADER_SIZE + total_payload_byte_size, m_texture_payload_size_in_byte_back_);
  
  // allocate 50 mb for compressed data 
  uint8_t* compressed_image_buffer = tjAlloc(1024*1024 * 50);

  int header_width, header_height, header_subsamp,;

  auto& current_decompressor_handle = m_jpeg_decompressor_per_layer[kinect_layer_idx];
  tjDecompressHeader2(current_decompressor_handle, compressed_image[kinect_layer_idx], _jpegSize[kinect_layer_idx],
                      &header_width, &header_height, &header_subsamp);

  tjDecompress2(current_decompressor_handle, compressed_image[kinect_layer_idx], _jpegSize[kinect_layer_idx], &texture_write_pos[byte_offset_to_current_image],
                header_width, 0, header_height, TJPF_BGR, TJFLAG_FASTDCT);

  tjFree(compressed_image_buffer);

}

void NetKinectArray::readloop() {
  // open multicast listening connection to server and port
  zmq::context_t ctx(1); // means single threaded
  zmq::socket_t  socket(ctx, ZMQ_SUB); // means a subscriber

  socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);

  int conflate_messages = 1;
  socket.setsockopt(ZMQ_CONFLATE, &conflate_messages, sizeof(conflate_messages));

  std::string endpoint("tcp://" + m_server_endpoint_);
  socket.connect(endpoint.c_str());

  const unsigned message_size = sizeof(size_t);//(m_colorsize_byte + m_depthsize_byte) * m_calib_files.size();

  //size_t header_byte_size = 100;
  //std::vector<uint8_t> header_data(header_byte_size, 0);

  while (m_running_) {
    
    zmq::message_t zmqm(message_size);
    socket.recv(&zmqm); // blocking
    
    while (m_need_cpu_swap_) {
      ;
    }

    SGTP::header_data_t message_header;

    std::size_t const HEADER_SIZE = SGTP::HEADER_BYTE_SIZE;

    memcpy((char*)&message_header, (unsigned char*) zmqm.data(), SGTP::HEADER_BYTE_SIZE);
    //memcpy((unsigned char*) &header_data[0], (unsigned char*) zmqm.data(), header_byte_size);

    //size_t num_voxels_received{0};
    //std::cout << "Received Data\n";
    
    if(message_header.is_calibration_data) {
      //std::cout << "ISSSSSSSSSSSSSSSS CALIBRATION DATA\n";

        //std::cout << "Total Calibration payload: " << message_header.total_payload << "\n";

        for(int dim_idx = 0; dim_idx < 3; ++dim_idx) {
          m_inv_xyz_calibration_res_back_[dim_idx] = message_header.inv_xyz_volume_res[dim_idx];
          m_uv_calibration_res_back_[dim_idx]      = message_header.uv_volume_res[dim_idx];
        }

        m_num_sensors_back_ = message_header.num_sensors;

        m_calibration_back_.resize(message_header.total_payload);

        memcpy((char*) &m_calibration_back_[0], ((char*) zmqm.data()) + HEADER_SIZE, message_header.total_payload);

        // memcpy inv_model_to_world_mat

        memcpy((char*) &m_inverse_vol_to_world_mat_back_[0], 
               (char*)message_header.inv_vol_to_world_mat,
                16 * sizeof(float));

        { // swap
          std::lock_guard<std::mutex> lock(m_mutex_);
          m_need_calibration_cpu_swap_.store(true);

          for( auto& entry : m_need_calibration_gpu_swap_) {
            entry.second.store(true);
          }
          //mutable std::unordered_map<std::size_t,
        }


    } else {
      //std::cout << "ISSSSSSSSSSSSSSSS AVATAR DATA\n";

        for(uint32_t dim_idx = 0; dim_idx < 3; ++dim_idx) {
          latest_received_bb_min[dim_idx] = message_header.global_bb_min[dim_idx];
          latest_received_bb_max[dim_idx] = message_header.global_bb_max[dim_idx];
        }

        m_received_textured_tris_back_         = message_header.num_textured_triangles;
        m_texture_payload_size_in_byte_back_   = message_header.texture_payload_size;

        m_received_kinect_timestamp_back_    = message_header.timestamp;
        m_received_reconstruction_time_back_ = message_header.geometry_creation_time_in_ms;

        m_lod_scaling_back_                  = message_header.lod_scaling;

        uint16_t const MAX_LAYER_IDX = 16;
        for(int layer_idx = 0; layer_idx < MAX_LAYER_IDX; ++layer_idx) {
          m_num_best_triangles_for_sensor_layer_back_[layer_idx] =
            message_header.num_best_triangles_per_sensor[layer_idx];


          m_texture_space_bounding_boxes_back_[4*layer_idx + 0]
            = message_header.tex_bounding_box[layer_idx].min.u;
          m_texture_space_bounding_boxes_back_[4*layer_idx + 1]
            = message_header.tex_bounding_box[layer_idx].min.v;
          m_texture_space_bounding_boxes_back_[4*layer_idx + 2]
            = message_header.tex_bounding_box[layer_idx].max.u;
          m_texture_space_bounding_boxes_back_[4*layer_idx + 3]
            = message_header.tex_bounding_box[layer_idx].max.v;
            //= message_header.texture_bounding_boxes[4*layer_idx + bb_component_idx];
          
        }
        


        size_t total_num_received_primitives = m_received_textured_tris_back_;

        if(total_num_received_primitives > 50000000) {
          return;
        }

        size_t textured_tris_byte_size  = m_received_textured_tris_back_         * 3*3*sizeof(float);//sizeof(gua::point_types::XYZ32_RGB8);


        //std::cout << "RECEIVED TRIANGLES: " << m_received_textured_tris_back_ << "\n";

        //std::cout << "ZMQ MESSAGE SIZE: " << zmqm.size() << "\n";
        

        size_t total_payload_byte_size = textured_tris_byte_size;

        //std::cout << "BYTES TO COPY   : " << HEADER_SIZE + total_payload_byte_size + m_texture_payload_size_in_byte_back_ << "\n";

        m_buffer_back_.resize(total_payload_byte_size);

        memcpy((unsigned char*) &m_buffer_back_[0], ((unsigned char*) zmqm.data()) + HEADER_SIZE, total_payload_byte_size);

        //m_texture_buffer_back_.resize(m_texture_payload_size_in_byte_back_);

        std::cout << "COPYING " << m_texture_payload_size_in_byte_back_ << " byte into texture source\n";
        memcpy((unsigned char*) &m_texture_buffer_back_[0], ((unsigned char*) zmqm.data()) + HEADER_SIZE + total_payload_byte_size, m_texture_payload_size_in_byte_back_);
      
        _decompress_and_rewrite_message();

        { // swap
          std::lock_guard<std::mutex> lock(m_mutex_);
          m_need_cpu_swap_.store(true);

          for( auto& entry : m_need_gpu_swap_) {
            entry.second.store(true);
          }
          //mutable std::unordered_map<std::size_t,
        }


    }
 

  
    
  }

}


}
