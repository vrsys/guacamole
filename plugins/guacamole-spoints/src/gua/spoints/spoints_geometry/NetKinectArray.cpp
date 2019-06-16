#include <gua/utils/Logger.hpp>

#include <gua/spoints/SPointsFeedbackCollector.hpp>
#include <gua/spoints/spoints_geometry/NetKinectArray.hpp>
#include <gua/spoints/spoints_geometry/codec/point_types.h>

#include <boost/assign/list_of.hpp>

#include <iostream>
#include <mutex>
#include <zmq.hpp>

#include <gua/spoints/sgtp/SGTP.h>
// fastlz for inflation of geometry data
#include <lz4/lz4.h>

namespace spoints
{
NetKinectArray::NetKinectArray(const std::string& server_endpoint, const std::string& feedback_endpoint)
    : m_mutex_(), m_running_(true),
      // m_feedback_running_(true),
      m_server_endpoint_(server_endpoint), m_feedback_endpoint_(feedback_endpoint), m_buffer_(/*(m_colorsize_byte + m_depthsize_byte) * m_calib_files.size()*/),
      m_buffer_back_(/*(m_colorsize_byte + m_depthsize_byte) * m_calib_files.size()*/),
      // m_feedback_need_swap_{false},
      m_recv_()
{
    m_recv_ = std::thread([this]() { _readloop(); });
}

NetKinectArray::~NetKinectArray()
{
#ifdef GUACAMOLE_ENABLE_TURBOJPEG
    if(nullptr != m_tj_compressed_image_buffer_)
    {
        tjFree(m_tj_compressed_image_buffer_);
    }
#endif //GUACAMOLE_ENABLE_TURBOJPEG
    m_running_ = false;
    m_recv_.join();
}

void NetKinectArray::draw_textured_triangle_soup(gua::RenderContext const& ctx, std::shared_ptr<gua::ShaderProgram>& shader_program)
{
    //auto const& current_point_layout = ;

    if(point_layout_per_context_[ctx.id] != nullptr)
    {
        if(!m_model_descriptor_.is_fully_encoded_vertex_data)
        {

            if( (!is_calibration_data_created_per_context_[ctx.id]) || (!is_vbo_created_per_context_[ctx.id]) ) {
                return;
            }

            //auto const& current_texture_atlas = texture_atlas_per_context_[ctx.id];

            //auto const& current_inv_xyz_pointers = inv_xyz_calibs_per_context_[ctx.id];
            //auto const& current_uv_pointers = uv_calibs_per_context_[ctx.id];

            scm::gl::context_vertex_input_guard vig(ctx.render_context);

            ctx.render_context->bind_vertex_array(point_layout_per_context_[ctx.id]);
            ctx.render_context->apply_vertex_input();


            //auto& current_net_data_vbo = net_data_vbo_per_context_[ctx.id];

            ctx.render_context->bind_texture(texture_atlas_per_context_[ctx.id], linear_sampler_state_, 0);
            shader_program->set_uniform(ctx, 0, "color_texture_atlas");

            // if(m_bound_calibration_data_.end() == m_bound_calibration_data_.find(ctx.id) ) {
            for(uint32_t sensor_idx = 0; sensor_idx < m_calibration_descriptor_.num_sensors; ++sensor_idx)
            {
                auto const& current_individual_inv_xyz_texture = inv_xyz_calibs_per_context_[ctx.id][sensor_idx];
                ctx.render_context->bind_texture(current_individual_inv_xyz_texture, linear_sampler_state_, sensor_idx + 1);

                std::string uniform_inv_xyz_name = "inv_xyz_volumes[" + std::to_string(sensor_idx) + "]";
                int uniform_inv_xyz_value = sensor_idx + 1;
                shader_program->set_uniform(ctx, uniform_inv_xyz_value, uniform_inv_xyz_name);

                // shader_program->set_uniform(ctx, int(sensor_idx), "inv_xyz_volumes[" + std::to_string(sensor_idx)+"]");
                auto const& current_individual_uv_texture = uv_calibs_per_context_[ctx.id][sensor_idx];
                ctx.render_context->bind_texture(current_individual_uv_texture, linear_sampler_state_, m_calibration_descriptor_.num_sensors + sensor_idx + 1);
            
                std::string uniform_uv_name = "uv_volumes[" + std::to_string(sensor_idx) + "]";
                int uniform_uv_value = m_calibration_descriptor_.num_sensors + sensor_idx + 1;
                shader_program->set_uniform(ctx, uniform_uv_value, uniform_uv_name);
            }
            // m_bound_calibration_data_[ctx.id] = true;
            //}

            float lod_scaling_for_context = m_current_lod_scaling_per_context_[ctx.id];

            shader_program->set_uniform(ctx, m_calibration_descriptor_.inverse_vol_to_world_mat, "inv_vol_to_world_matrix");
            shader_program->set_uniform(ctx, lod_scaling_for_context, "scaling_factor");


            scm::math::vec3 tight_geometry_bb_min_for_context = m_current_tight_geometry_bb_min_per_context_[ctx.id];
            scm::math::vec3 tight_geometry_bb_max_for_context = m_current_tight_geometry_bb_max_per_context_[ctx.id];

            shader_program->set_uniform(ctx, tight_geometry_bb_min_for_context, "tight_bb_min");
            shader_program->set_uniform(ctx, tight_geometry_bb_max_for_context, "tight_bb_max");

            shader_program->set_uniform(ctx, int(3), "Out_Sorted_Vertex_Tri_Data");
            ctx.render_context->bind_storage_buffer(net_data_vbo_per_context_[ctx.id], 3, 0, INITIAL_VBO_SIZE);
            //ctx.render_context->set_storage_buffers( std::vector<scm::gl::render_context::buffer_binding>{scm::gl::BIND_STORAGE_BUFFER} );

            ctx.render_context->apply_storage_buffer_bindings();

            uint32_t triangle_offset_for_current_layer = 0;
            uint32_t num_triangles_to_draw_for_current_layer = 0;

            auto& num_best_triangles_for_sensor_layer_for_context = m_current_num_best_triangles_for_sensor_layer_per_context_[ctx.id];

            for(int layer_idx = 0; layer_idx < 4; ++layer_idx)
            {
                num_triangles_to_draw_for_current_layer = num_best_triangles_for_sensor_layer_for_context[layer_idx];

                shader_program->set_uniform(ctx, int(layer_idx), "current_sensor_layer");

                ctx.render_context->apply();

                size_t const vertex_offset = triangle_offset_for_current_layer * 3;
                size_t const num_vertices_to_draw = num_triangles_to_draw_for_current_layer * 3;

                ctx.render_context->draw_arrays(scm::gl::PRIMITIVE_TRIANGLE_LIST, vertex_offset, num_vertices_to_draw);

                triangle_offset_for_current_layer += num_triangles_to_draw_for_current_layer;
            }
        }
        else
        {
            scm::gl::context_vertex_input_guard vig(ctx.render_context);

            auto& current_net_data_vbo = net_data_vbo_per_context_[ctx.id];

            //auto const& current_texture_atlas = texture_atlas_per_context_[ctx.id];
            ctx.render_context->bind_texture(texture_atlas_per_context_[ctx.id], linear_sampler_state_, 0);


            shader_program->set_uniform(ctx, int(3), "Out_Sorted_Vertex_Tri_Data");
            //ctx.render_context->bind_storage_buffer(current_net_data_vbo, 3, 0, INITIAL_VBO_SIZE);

            //ctx.render_context->apply_storage_buffer_bindings();

            ctx.render_context->bind_vertex_array(point_layout_per_context_[ctx.id]);
            ctx.render_context->apply_vertex_input();

            size_t const num_vertices_to_draw = m_model_descriptor_.received_textured_tris * 3;
            ctx.render_context->draw_arrays(scm::gl::PRIMITIVE_TRIANGLE_LIST, 0, num_vertices_to_draw);

            //std::cout << "Drawing fully encoded vertex data\n";
        }

        ctx.render_context->reset_vertex_input();
    }
}

std::string NetKinectArray::get_socket_string() const { return m_feedback_endpoint_; };

void NetKinectArray::push_matrix_package(spoints::camera_matrix_package const& cam_mat_package)
{
    submitted_camera_matrix_package_ = cam_mat_package;

    encountered_context_ids_for_feedback_frame_.insert(submitted_camera_matrix_package_.k_package.render_context_id);
}

void NetKinectArray::_try_swap_calibration_data_cpu() {
    std::lock_guard<std::mutex> lock(m_mutex_);

    if(m_need_calibration_cpu_swap_.load()) {
        if(num_clients_gpu_swapping_.load()) {
            return;
        }
        ++num_clients_cpu_swapping_;
        //std::lock_guard<std::mutex> lock(m_mutex_);
        m_calibration_.swap(m_calibration_back_);

        std::swap(m_calibration_descriptor_, m_calibration_descriptor_back_);

        --num_clients_cpu_swapping_;
        m_need_calibration_cpu_swap_.store(false);
    }
}

bool NetKinectArray::_try_swap_calibration_data_gpu(gua::RenderContext const& ctx) {
        bool current_thread_need_calibration_gpu_swap = false;
        
        {
            //std::lock_guard<std::mutex> lock(m_mutex_);
            current_thread_need_calibration_gpu_swap = m_need_calibration_gpu_swap_[ctx.id].load();
        }  



        if(current_thread_need_calibration_gpu_swap) {

            if(m_received_calibration_[ctx.id]) {
                m_need_calibration_gpu_swap_[ctx.id].store(false);
            } else {
                
                if(num_clients_cpu_swapping_.load()) {
                    return false;
                }

                std::lock_guard<std::mutex> lock(m_mutex_);
                ++num_clients_gpu_swapping_;

                //{
                    //std::lock_guard<std::mutex> lock(m_mutex_);
                //    inv_xyz_calibs_per_context_[ctx.id] = std::vector<scm::gl::texture_3d_ptr>(m_calibration_descriptor_.num_sensors, nullptr);
                //    uv_calibs_per_context_[ctx.id] = std::vector<scm::gl::texture_3d_ptr>(m_calibration_descriptor_.num_sensors, nullptr);
                //}

                std::size_t current_read_offset = 0;



                auto const volumetric_inv_xyz_region_to_update =
                    scm::gl::texture_region(scm::math::vec3ui(0, 0, 0), scm::math::vec3ui(m_calibration_descriptor_.inv_xyz_calibration_res[0], 
                                                                                          m_calibration_descriptor_.inv_xyz_calibration_res[1], 
                                                                                          m_calibration_descriptor_.inv_xyz_calibration_res[2]));

                auto const volumetric_uv_region_to_update =
                    scm::gl::texture_region(scm::math::vec3ui(0, 0, 0), scm::math::vec3ui(m_calibration_descriptor_.uv_calibration_res[0], 
                                                                                          m_calibration_descriptor_.uv_calibration_res[1], 
                                                                                          m_calibration_descriptor_.uv_calibration_res[2]));

                uint64_t total_num_voxels_uv_calibration_volume = 1;

                for(unsigned int dim_idx = 0; dim_idx < 3; ++dim_idx) {
                    total_num_voxels_uv_calibration_volume *= m_calibration_descriptor_.uv_calibration_res[dim_idx];
                }

                uint32_t constexpr num_channels_uv_calibration_volume = 2;
                std::size_t const num_bytes_per_uv_volume = num_channels_uv_calibration_volume * sizeof(float) * total_num_voxels_uv_calibration_volume;

                for(uint32_t sensor_idx = 0; sensor_idx < m_calibration_descriptor_.num_sensors; ++sensor_idx)
                {

                    //std::lock_guard<std::mutex> lock(m_mutex_);

                    // create and update calibration volume
                    //std::cout << "Trying to create calib volume of size"
                    auto& current_inv_xyz_calibration_volume_ptr = inv_xyz_calibs_per_context_[ctx.id][sensor_idx];



                    uint64_t total_num_voxels_inv_xyz_calibration_volume = 1;

                    for(unsigned int dim_idx = 0; dim_idx < 3; ++dim_idx) {
                        total_num_voxels_inv_xyz_calibration_volume *= m_calibration_descriptor_.inv_xyz_calibration_res[dim_idx];
                    }
                    uint64_t const num_channels_inv_xyz_volume = 4;
                    std::size_t const num_bytes_per_inv_xyz_volume = num_channels_inv_xyz_volume * sizeof(float) * total_num_voxels_inv_xyz_calibration_volume;


                    ctx.render_context->update_sub_texture(
                        current_inv_xyz_calibration_volume_ptr, volumetric_inv_xyz_region_to_update, 
                        0, scm::gl::FORMAT_RGBA_32F, (void*)&m_calibration_[current_read_offset]);
                    current_read_offset += num_bytes_per_inv_xyz_volume;

                    //=======================================================================================================

                    // create and update calibration volume
                    auto& current_uv_calibration_volume_ptr = uv_calibs_per_context_[ctx.id][sensor_idx];

                    ctx.render_context->update_sub_texture(
                        current_uv_calibration_volume_ptr, volumetric_uv_region_to_update, 0, scm::gl::FORMAT_RG_32F, (void*)&m_calibration_[current_read_offset]);
                    current_read_offset += num_bytes_per_uv_volume;
                }

                // std::cout << "Loaded Volume Textures\n";
                // current_texture_atlas = ctx.render_device->create_texture_3d(scm::math::vec3ui(texture_width, texture_height), scm::gl::FORMAT_BGR_8, 1, 1, 1);
                

                --num_clients_gpu_swapping_;

                {

                    is_calibration_data_created_per_context_[ctx.id] = true;
                    m_need_calibration_gpu_swap_[ctx.id].store(false);
                    m_received_calibration_[ctx.id].store(true);
                }
                return true;

            }

        }

        return true;
}

void NetKinectArray::_try_swap_model_data_cpu() {
    std::lock_guard<std::mutex> lock(m_mutex_);

    if(m_need_model_cpu_swap_.load())
    {
        
        if(num_clients_gpu_swapping_.load()) {
            return;
        }

        ++num_clients_cpu_swapping_;

        //std::lock_guard<std::mutex> lock(m_mutex_);
        // start of synchro point
        m_buffer_.swap(m_buffer_back_);
        m_texture_buffer_.swap(m_texture_buffer_back_);
        // std::swap(m_voxel_size_, m_voxel_size_back_);

        std::swap(m_model_descriptor_back_, m_model_descriptor_);


        std::swap(m_num_best_triangles_for_sensor_layer_, m_num_best_triangles_for_sensor_layer_back_);

        std::swap(m_lod_scaling_back_, m_lod_scaling_);

        std::swap(m_texture_space_bounding_boxes_back_, m_texture_space_bounding_boxes_);

        std::swap(m_tight_geometry_bb_min_, m_tight_geometry_bb_min_back_);
        std::swap(m_tight_geometry_bb_max_, m_tight_geometry_bb_max_back_);

        //std::swap(m_is_fully_encoded_vertex_data_back_, m_is_fully_encoded_vertex_data_);

        // end of synchro point
        --num_clients_cpu_swapping_;
        m_need_model_cpu_swap_.store(false);
    }
}

bool NetKinectArray::update(gua::RenderContext const& ctx, gua::math::BoundingBox<gua::math::vec3>& in_out_bb, scm::math::vec3ui const& inv_xyz_vol_res, scm::math::vec3ui const& uv_vol_res)
{

                            //std::lock_guard<std::mutex> lock(m_mutex_);


    bool some_gpu_update_went_wrong = false;
    
    if(!is_vbo_created_per_context_[ctx.id])
    {
        std::lock_guard<std::mutex> lock(m_mutex_);

        
        
        if(!empty_vbo_per_context_[ctx.id]) {
            empty_vbo_per_context_[ctx.id] = ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER, scm::gl::USAGE_STATIC_DRAW, INITIAL_VBO_SIZE, 0);
            ctx.render_context->apply_vertex_input();
            if(!empty_vbo_per_context_[ctx.id]) {
                some_gpu_update_went_wrong = true;
                std::cout << "SOME UPDATE OF GPU BUFFERS WENT WRONG" << std::endl;
            }
        }
        
        // size_t size_of_vertex = 2 * sizeof(uint32_t);
        if(!point_layout_per_context_[ctx.id]) {
            point_layout_per_context_[ctx.id] = ctx.render_device->create_vertex_array(scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC3F, 3*sizeof(float)), boost::assign::list_of(empty_vbo_per_context_[ctx.id]));
            if(!point_layout_per_context_[ctx.id]) {
                some_gpu_update_went_wrong = true;
                std::cout << "SOME UPDATE OF GPU BUFFERS WENT WRONG" << std::endl;
            }
        }



        
        if(!net_data_vbo_per_context_[ctx.id]) {
            net_data_vbo_per_context_[ctx.id] = ctx.render_device->create_buffer(scm::gl::BIND_STORAGE_BUFFER, scm::gl::USAGE_DYNAMIC_COPY, INITIAL_VBO_SIZE, 0);
            if(!net_data_vbo_per_context_[ctx.id]) {
                some_gpu_update_went_wrong = true;
                std::cout << "SOME UPDATE OF GPU BUFFERS WENT WRONG" << std::endl;
            }
        }
        
        

        
        ctx.render_context->apply();

        if(nullptr == linear_sampler_state_)
        {
            linear_sampler_state_ = ctx.render_device->create_sampler_state(scm::gl::FILTER_MIN_MAG_LINEAR, scm::gl::WRAP_CLAMP_TO_EDGE);
        }


        size_t texture_width = 1280 * 2;
        size_t texture_height = 720 * 2;

        if(!texture_atlas_per_context_[ctx.id]) {
            texture_atlas_per_context_[ctx.id] = ctx.render_device->create_texture_2d(scm::math::vec2ui(texture_width, texture_height), scm::gl::FORMAT_BGR_8, 1, 1, 1);
            if(!texture_atlas_per_context_[ctx.id]) {
                some_gpu_update_went_wrong = true;
                std::cout << "SOME UPDATE OF GPU BUFFERS WENT WRONG" << std::endl;
            }
        }

        if(some_gpu_update_went_wrong) {
            std::cout << "SOME UPDATE OF GPU BUFFERS WENT WRONG" << std::endl;
        }


        is_vbo_created_per_context_[ctx.id] = true;
        //is_vbo_created_per_context_[ctx.id] = true;
    }
    


    {


        
        bool some_gpu_texture_update_went_wrong = false;

            //std::lock_guard<std::mutex> lock(m_mutex_);
            
            if(!are_textures_created_per_context_[ctx.id]) {
                for(uint32_t sensor_idx = 0; sensor_idx < 4; ++sensor_idx)
                {



                    auto& current_inv_xyz_calibration_volume_ptr = inv_xyz_calibs_per_context_[ctx.id][sensor_idx];
                    //std::lock_guard<std::mutex> lock(m_mutex_);

                    // create and update calibration volume
                    //std::cout << "Trying to create calib volume of size"


                    if(!current_inv_xyz_calibration_volume_ptr) {
                        current_inv_xyz_calibration_volume_ptr =
                            ctx.render_device->create_texture_3d(scm::math::vec3ui(inv_xyz_vol_res[0], 
                                                                                   inv_xyz_vol_res[1], 
                                                                                   inv_xyz_vol_res[2]), 
                                                                                   scm::gl::FORMAT_RGBA_32F);
                        if(!current_inv_xyz_calibration_volume_ptr) {
                            std::cout << "THIS TEXTURE UPDATE WENT WRONG" << std::endl;
                            some_gpu_texture_update_went_wrong = true;
                        }
                    }



                    //=======================================================================================================

                    // create and update calibration volume
                    auto& current_uv_calibration_volume_ptr = uv_calibs_per_context_[ctx.id][sensor_idx];

                    if(!current_uv_calibration_volume_ptr) {
                    current_uv_calibration_volume_ptr =
                        ctx.render_device->create_texture_3d(scm::math::vec3ui(uv_vol_res[0], 
                                                                               uv_vol_res[1], 
                                                                               uv_vol_res[2]),
                                                                               scm::gl::FORMAT_RG_32F);
                        if(!current_uv_calibration_volume_ptr) {
                            std::cout << "THIS TEXTURE UPDATE WENT WRONG2" << std::endl;
                            some_gpu_texture_update_went_wrong = true;
                        }
                    }


                }
                are_textures_created_per_context_[ctx.id] = true;
            }

        //std::cout << "SOME TEXTURE UPDATE WENT WRONG" << std::endl;

    }




    //std::cout << "Trying to update something else" << std::endl;

    //std::lock_guard<std::mutex> lock(m_mutex_);
    {
        {
            //std::lock_guard<std::mutex> lock(m_mutex_);
            auto current_encountered_frame_count = encountered_frame_counts_per_context_[ctx.id];

            if(current_encountered_frame_count != ctx.framecount)
            {
                current_encountered_frame_count = ctx.framecount;
            }
            else
            {
                return false;
            }
        }


        _try_swap_calibration_data_cpu();

        bool got_calibration_data = _try_swap_calibration_data_gpu(ctx);

        if(!got_calibration_data) {
            return false;
        }

        _try_swap_model_data_cpu();

        bool current_thread_need_model_gpu_swap = false;

        {
            //std::lock_guard<std::mutex> lock(m_mutex_);
            current_thread_need_model_gpu_swap = m_need_model_gpu_swap_[ctx.id].load();
        }

        if(current_thread_need_model_gpu_swap)
        {


            if(num_clients_cpu_swapping_.load()) {
                return false;
            }

            ++num_clients_gpu_swapping_;
            size_t total_num_bytes_to_copy = 0;

            if(false == m_model_descriptor_.is_fully_encoded_vertex_data)
            {
                total_num_bytes_to_copy = m_model_descriptor_.received_textured_tris * 3 * 3 * sizeof(uint16_t);
            }
            else
            {
                total_num_bytes_to_copy =  m_model_descriptor_.received_textured_tris * 3 * 5 * sizeof(float);
            }

            if(0 != total_num_bytes_to_copy)
            {


                num_textured_tris_to_draw_per_context_[ctx.id] = m_model_descriptor_.received_textured_tris;
                m_current_num_best_triangles_for_sensor_layer_per_context_[ctx.id] = m_num_best_triangles_for_sensor_layer_;


                //std::lock_guard<std::mutex> lock(m_mutex_);
                //auto& current_is_vbo_created = is_vbo_created_per_context_[ctx.id];



                
                
                ctx.render_context->bind_storage_buffer(net_data_vbo_per_context_[ctx.id], 3, 0, INITIAL_VBO_SIZE);

                ctx.render_context->apply_storage_buffer_bindings();
                
                float* mapped_net_data_vbo_ = (float*)ctx.render_context->map_buffer(net_data_vbo_per_context_[ctx.id], scm::gl::access_mode::ACCESS_WRITE_ONLY);
                memcpy((char*)mapped_net_data_vbo_, (char*)&m_buffer_[0], total_num_bytes_to_copy);


                ctx.render_context->unmap_buffer(net_data_vbo_per_context_[ctx.id]);
                
                
                m_current_lod_scaling_per_context_[ctx.id] = m_lod_scaling_;

                m_current_tight_geometry_bb_min_per_context_[ctx.id] = m_tight_geometry_bb_min_;
                m_current_tight_geometry_bb_max_per_context_[ctx.id] = m_tight_geometry_bb_max_;

                uint32_t byte_offset_per_texture_data_for_layers[MAX_LAYER_IDX];

                for(uint32_t layer_to_update_idx = 0; layer_to_update_idx < MAX_LAYER_IDX; ++layer_to_update_idx)
                {
                    // initialize offset with 0 or value of last region and update incrementally
                    if(0 == layer_to_update_idx)
                    {
                        byte_offset_per_texture_data_for_layers[layer_to_update_idx] = 0;
                    }
                    else
                    {
                        byte_offset_per_texture_data_for_layers[layer_to_update_idx] = byte_offset_per_texture_data_for_layers[layer_to_update_idx - 1];

                        if(m_num_best_triangles_for_sensor_layer_[layer_to_update_idx])
                        {
                            uint32_t layer_offset = 4 * (layer_to_update_idx - 1);
                            uint32_t prev_bb_pixel_coverage = (1 + m_texture_space_bounding_boxes_[layer_offset + 2] - m_texture_space_bounding_boxes_[layer_offset + 0]) *
                                                              (1 + m_texture_space_bounding_boxes_[layer_offset + 3] - m_texture_space_bounding_boxes_[layer_offset + 1]);

                            byte_offset_per_texture_data_for_layers[layer_to_update_idx] += prev_bb_pixel_coverage * 3;
                        }
                    }

                    if(m_num_best_triangles_for_sensor_layer_[layer_to_update_idx] > 0)
                    {
                        uint32_t current_layer_offset = 4 * (layer_to_update_idx);

                        auto current_region_to_update =
                            scm::gl::texture_region(scm::math::vec3ui(m_texture_space_bounding_boxes_[current_layer_offset + 0], m_texture_space_bounding_boxes_[current_layer_offset + 1], 0),
                                                    scm::math::vec3ui((m_texture_space_bounding_boxes_[current_layer_offset + 2] + 1 - m_texture_space_bounding_boxes_[current_layer_offset + 0]),
                                                                      (m_texture_space_bounding_boxes_[current_layer_offset + 3] + 1 - m_texture_space_bounding_boxes_[current_layer_offset + 1]),
                                                                      1));

                        size_t current_read_offset = byte_offset_per_texture_data_for_layers[layer_to_update_idx];


                        ctx.render_context->update_sub_texture(
                            texture_atlas_per_context_[ctx.id], current_region_to_update, 0, scm::gl::FORMAT_BGR_8, (void*)&m_texture_buffer_[current_read_offset]);
                    }
                }

            }
            else
            {
                num_textured_tris_to_draw_per_context_[ctx.id] = 0;
            }

            --num_clients_gpu_swapping_;
            m_need_model_gpu_swap_[ctx.id].store(false);
            return true;
        }
    }
    return false;
}

    /*float NetKinectArray::get_voxel_size() const {
      return m_voxel_size_;
    }*/

#ifdef GUACAMOLE_ENABLE_TURBOJPEG
bool NetKinectArray::_decompress_and_rewrite_message(std::vector<std::size_t> const& byte_offset_to_jpeg_windows)
{
    int num_decompressed_bytes = LZ4_decompress_safe((const char*)&m_buffer_back_compressed_[0], (char*)&m_buffer_back_[0], m_buffer_back_compressed_.size(), m_buffer_back_.size());

    if(nullptr == m_tj_compressed_image_buffer_)
    {
        m_tj_compressed_image_buffer_ = tjAlloc(1024 * 1024 * 50);
    }

    std::size_t byte_offset_to_current_image = 0;

    std::size_t total_image_byte_size = 0;

    std::size_t decompressed_image_offset = 0;

    for(uint32_t sensor_layer_idx = 0; sensor_layer_idx < 4; ++sensor_layer_idx)
    {
        total_image_byte_size += byte_offset_to_jpeg_windows[sensor_layer_idx];
    }

    for(uint32_t sensor_layer_idx = 0; sensor_layer_idx < 4; ++sensor_layer_idx)
    {
        if(m_jpeg_decompressor_per_layer.end() == m_jpeg_decompressor_per_layer.find(sensor_layer_idx))
        {
            m_jpeg_decompressor_per_layer[sensor_layer_idx] = tjInitDecompress();
            if(m_jpeg_decompressor_per_layer[sensor_layer_idx] == NULL)
            {
                std::cout << "ERROR INITIALIZING DECOMPRESSOR\n";
            }
        }

        long unsigned int jpeg_size = byte_offset_to_jpeg_windows[sensor_layer_idx];

        memcpy((char*)&m_tj_compressed_image_buffer_[0], (char*)&m_texture_buffer_back_[byte_offset_to_current_image], jpeg_size);

        int header_width, header_height, header_subsamp;

        auto& current_decompressor_handle = m_jpeg_decompressor_per_layer[sensor_layer_idx];

        int error_handle = tjDecompressHeader2(current_decompressor_handle, &m_tj_compressed_image_buffer_[0], jpeg_size, &header_width, &header_height, &header_subsamp);

        if(-1 == error_handle)
        {
            std::cout << "ERROR DECOMPRESSING JPEG\n";
            std::cout << "Error was: " << tjGetErrorStr() << "\n";

            std::cout << "JPEG SIZE IN BYTE:" << jpeg_size << "\n";
            std::cout << "Skipping frame again 6.\n";
            return false;
        }

        tjDecompress2(current_decompressor_handle,
                      &m_tj_compressed_image_buffer_[0],
                      jpeg_size,
                      &m_decompressed_image_buffer_[decompressed_image_offset],
                      header_width,
                      0,
                      header_height,
                      TJPF_BGR,
                      TJFLAG_FASTDCT);

        uint32_t copied_image_byte = header_height * header_width * 3;

        byte_offset_to_current_image += jpeg_size;
        decompressed_image_offset += copied_image_byte;
    }

    memcpy((char*)m_texture_buffer_back_.data(), (char*)m_decompressed_image_buffer_.data(), decompressed_image_offset);

    return true;
}
#endif //GUACAMOLE_ENABLE_TURBOJPEG

void NetKinectArray::_readloop()
{
    // open multicast listening connection to server and port
    zmq::context_t ctx(1);              // means single threaded
    zmq::socket_t socket(ctx, ZMQ_SUB); // means a subscriber

    socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);

    int conflate_messages = 1;
    socket.setsockopt(ZMQ_CONFLATE, &conflate_messages, sizeof(conflate_messages));

    std::string endpoint("tcp://" + m_server_endpoint_);
    socket.connect(endpoint.c_str());

    const unsigned message_size = sizeof(size_t); //(m_colorsize_byte + m_depthsize_byte) * m_calib_files.size();

    // size_t header_byte_size = 100;
    // std::vector<uint8_t> header_data(header_byte_size, 0);


    zmq::message_t zmqm;

    while(m_running_)
    {

        socket.recv(&zmqm); // blocking

        while(m_need_model_cpu_swap_ || num_clients_gpu_swapping_.load() || num_clients_cpu_swapping_.load())
        {
            ;
        }

        SGTP::header_data_t message_header;

        std::size_t const HEADER_SIZE = SGTP::HEADER_BYTE_SIZE;

        memcpy((char*)&message_header, (unsigned char*)zmqm.data(), SGTP::HEADER_BYTE_SIZE);

        if(message_header.is_calibration_data)
        {
            for(int dim_idx = 0; dim_idx < 3; ++dim_idx)
            {
                m_calibration_descriptor_back_.inv_xyz_calibration_res[dim_idx] = message_header.inv_xyz_volume_res[dim_idx];
                m_calibration_descriptor_back_.uv_calibration_res[dim_idx] = message_header.uv_volume_res[dim_idx];
            }

            m_calibration_descriptor_back_.num_sensors = message_header.num_sensors;

            m_calibration_back_.resize(message_header.total_payload);

            memcpy((char*)&m_calibration_back_[0], ((char*)zmqm.data()) + HEADER_SIZE, message_header.total_payload);

            // memcpy inv_model_to_world_mat

            memcpy((char*)&m_calibration_descriptor_back_.inverse_vol_to_world_mat[0], 
                   (char*)message_header.inv_vol_to_world_mat, 16 * sizeof(float));

            { // swap
                std::lock_guard<std::mutex> lock(m_mutex_);
                m_need_calibration_cpu_swap_.store(true);

                for(auto& entry : m_need_calibration_gpu_swap_)
                {
                    entry.store(true);
                }
                // mutable std::unordered_map<std::size_t,
            }
        }
        else
        {
            message_header.fill_texture_byte_offsets_to_bounding_boxes();

            for(uint32_t dim_idx = 0; dim_idx < 3; ++dim_idx)
            {
                m_tight_geometry_bb_min_back_[dim_idx] = message_header.global_bb_min[dim_idx];
                m_tight_geometry_bb_max_back_[dim_idx] = message_header.global_bb_max[dim_idx];
            }

            m_model_descriptor_back_.is_fully_encoded_vertex_data = message_header.is_fully_encoded_vertex_data;
            m_model_descriptor_back_.received_textured_tris = message_header.num_textured_triangles;
            m_model_descriptor_back_.texture_payload_size_in_byte = message_header.texture_payload_size;

            m_model_descriptor_back_.received_kinect_timestamp = message_header.timestamp;
            m_model_descriptor_back_.received_reconstruction_time = message_header.geometry_creation_time_in_ms;

            m_model_descriptor_back_.total_message_payload_in_byte = zmqm.size();

            auto passed_microseconds_to_request = message_header.passed_microseconds_since_request;

            auto timestamp_during_reception = std::chrono::system_clock::now();
            auto reference_timestamp = ::gua::SPointsFeedbackCollector::instance()->get_reference_timestamp();

            auto start_to_reply_diff = timestamp_during_reception - reference_timestamp;

            int64_t passed_microseconds_to_reply = std::chrono::duration<double>(start_to_reply_diff).count() * 1000000;

            int64_t total_latency_in_microseconds = passed_microseconds_to_reply - passed_microseconds_to_request;

            m_model_descriptor_back_.request_reply_latency_ms = total_latency_in_microseconds / 1000.0f;

            m_lod_scaling_back_ = message_header.lod_scaling;



            if(!m_model_descriptor_back_.is_fully_encoded_vertex_data)
            {
                m_model_descriptor_back_.received_textured_tris = 0;

                for(int layer_idx = 0; layer_idx < MAX_LAYER_IDX; ++layer_idx)
                {
                    m_num_best_triangles_for_sensor_layer_back_[layer_idx] = message_header.num_best_triangles_per_sensor[layer_idx];

                    m_model_descriptor_back_.received_textured_tris += message_header.num_best_triangles_per_sensor[layer_idx];

                    //memcpy(&m_texture_space_bounding_boxes_back_[4 * layer_idx + 0], &message_header.tex_bounding_box[layer_idx], sizeof(message_header.tex_bounding_box[0]));
                    m_texture_space_bounding_boxes_back_[4 * layer_idx + 0] = message_header.tex_bounding_box[layer_idx].min.u;
                    m_texture_space_bounding_boxes_back_[4 * layer_idx + 1] = message_header.tex_bounding_box[layer_idx].min.v;
                    m_texture_space_bounding_boxes_back_[4 * layer_idx + 2] = message_header.tex_bounding_box[layer_idx].max.u;
                    m_texture_space_bounding_boxes_back_[4 * layer_idx + 3] = message_header.tex_bounding_box[layer_idx].max.v;
                    //= message_header.texture_bounding_boxes[4*layer_idx + bb_component_idx];
                }
            }
            else
            {
                m_num_best_triangles_for_sensor_layer_back_[0] = m_model_descriptor_back_.received_textured_tris;

                m_texture_space_bounding_boxes_back_[0] = message_header.tex_bounding_box[0].min.u;
                m_texture_space_bounding_boxes_back_[1] = message_header.tex_bounding_box[0].min.v;
                m_texture_space_bounding_boxes_back_[2] = message_header.tex_bounding_box[0].max.u;
                m_texture_space_bounding_boxes_back_[3] = message_header.tex_bounding_box[0].max.v;
            }

            size_t total_num_received_primitives = m_model_descriptor_back_.received_textured_tris;

            if(total_num_received_primitives > 50000000)
            {
                return;
            }

            std::size_t size_of_vertex = 0;

            if(false == m_model_descriptor_back_.is_fully_encoded_vertex_data)
            {
                size_of_vertex = 3 * sizeof(uint16_t);
            }
            else
            {
                size_of_vertex = 5 * sizeof(float);
            }

            size_t textured_tris_byte_size = total_num_received_primitives * 3 * size_of_vertex;

            size_t total_uncompressed_geometry_payload_byte_size = textured_tris_byte_size;

            m_buffer_back_.resize(total_uncompressed_geometry_payload_byte_size*4);

            size_t const total_encoded_geometry_byte_size = zmqm.size() - (m_model_descriptor_back_.texture_payload_size_in_byte + HEADER_SIZE);

            if(message_header.is_data_compressed)
            {
                m_buffer_back_compressed_.resize(total_encoded_geometry_byte_size);
                memcpy((unsigned char*)&m_buffer_back_compressed_[0], ((unsigned char*)zmqm.data()) + HEADER_SIZE, total_encoded_geometry_byte_size);
            }
            else
            {
                memcpy((unsigned char*)&m_buffer_back_[0], ((unsigned char*)zmqm.data()) + HEADER_SIZE, total_encoded_geometry_byte_size);
            }

            memcpy((unsigned char*)&m_texture_buffer_back_[0], ((unsigned char*)zmqm.data()) + HEADER_SIZE + total_encoded_geometry_byte_size, m_model_descriptor_back_.texture_payload_size_in_byte);

            std::vector<std::size_t> byte_offset_to_jpeg_windows(MAX_LAYER_IDX, 0);

            for(uint32_t sensor_layer_idx = 0; sensor_layer_idx < 4; ++sensor_layer_idx)
            {
                byte_offset_to_jpeg_windows[sensor_layer_idx] = message_header.jpeg_bytes_per_sensor[sensor_layer_idx];
            }

            if(message_header.is_data_compressed)
            {
#ifdef GUACAMOLE_ENABLE_TURBOJPEG
                bool decompressed_successfully = _decompress_and_rewrite_message(byte_offset_to_jpeg_windows);
                if(!decompressed_successfully) {
                    continue;
                }
#else
                gua::Logger::LOG_WARNING << "TurboJPEG not available. Compile with option ENABLE_TURBOJPEG" << std::endl;
#endif // GUACAMOLE_ENABLE_TURBOJPEG
            }

            { // swap
                std::lock_guard<std::mutex> lock(m_mutex_);
                m_need_model_cpu_swap_.store(true);

                for(auto& entry : m_need_model_gpu_swap_)
                {
                    entry.second.store(true);
                }
                // mutable std::unordered_map<std::size_t,
            }
        }
    }
}

} // namespace spoints
