#ifndef SPOINTS_NETKINECTARRAY_HPP
#define SPOINTS_NETKINECTARRAY_HPP

#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/ShaderProgram.hpp>

#include <gua/math/BoundingBox.hpp>
#include <gua/math/math.hpp>

#include <atomic>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#ifdef GUACAMOLE_ENABLE_TURBOJPEG
#include <turbojpeg.h>
#endif //GUACAMOLE_ENABLE_TURBOJPEG

namespace spoints
{
struct key_package
{
    bool is_camera;
    std::size_t view_uuid;
    bool stereo_mode;
    std::size_t framecount;
    std::size_t render_context_id;
};

struct matrix_package
{
    float model_matrix[16];
    float viewprojection_matrix[16];
    float modelview_matrix[16];
    float projection_matrix[16];
    uint32_t res_xy[2];
    int32_t camera_type; // mono = 0, left = 1, right = 2
    int32_t uuid;
    bool calibration_request;
};

// is_camera, view_uuid, stereo_mode, current_package);

struct camera_matrix_package
{
    key_package k_package;
    matrix_package mat_package;

    bool operator<(camera_matrix_package const& rhs)
    {
        if(k_package.is_camera < rhs.k_package.is_camera)
        {
            return true;
        }
        else if(rhs.k_package.is_camera < k_package.is_camera)
        {
            return false;
        }
        else
        {
            if(k_package.view_uuid < rhs.k_package.view_uuid)
            {
                return true;
            }
            else if(rhs.k_package.view_uuid < k_package.view_uuid)
            {
                return false;
            }
            else
            {
                if(k_package.stereo_mode < rhs.k_package.stereo_mode)
                {
                    return true;
                }
                else if(rhs.k_package.stereo_mode < k_package.stereo_mode)
                {
                    if(k_package.framecount < rhs.k_package.framecount)
                    {
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
            }
        }
    }
};

struct SPointsStats
{
    SPointsStats() : m_received_triangles(0), m_received_timestamp_ms(0.0f), m_received_reconstruction_time_ms(-1.0f), m_request_reply_latency_ms(-1.0f), m_total_message_payload_in_byte(0) {}

    SPointsStats(uint32_t in_received_tris, float in_received_timestamp, float in_received_recon_time, float in_request_reply_latency_ms, uint32_t in_total_message_payload_in_byte)
        : m_received_triangles(in_received_tris), m_received_timestamp_ms(in_received_timestamp), m_received_reconstruction_time_ms(in_received_recon_time),
          m_request_reply_latency_ms(in_request_reply_latency_ms), m_total_message_payload_in_byte(in_total_message_payload_in_byte)
    {
    }

    uint32_t m_received_triangles = 0;
    float m_received_timestamp_ms = -1.0f;
    float m_received_reconstruction_time_ms = 0.0f;
    float m_request_reply_latency_ms = -1.0f;
    uint32_t m_total_message_payload_in_byte = 0;
};

struct SPointsCalibrationDescriptor {
    uint32_t num_sensors = 0;
    std::array<uint32_t, 3> inv_xyz_calibration_res;
    std::array<uint32_t, 3> uv_calibration_res;
    scm::math::mat4f inverse_vol_to_world_mat;
};

struct SPointsModelDescriptor {
    uint32_t received_textured_tris = 0.0;
    float received_kinect_timestamp = 0.0;
    float received_reconstruction_time = 0.0;
  
    float request_reply_latency_ms = -1.0;
    int32_t triangle_texture_atlas_size = 0.0;
   
    uint32_t texture_payload_size_in_byte = 0;
   

    uint32_t total_message_payload_in_byte = 0;

    bool is_fully_encoded_vertex_data = false;
};

class NetKinectArray
{
  public:
    NetKinectArray(const std::string& server_endpoint, const std::string& feedback_endpoint = "");
    ~NetKinectArray();

    void draw_vertex_colored_points(gua::RenderContext const& ctx);
    void draw_vertex_colored_triangle_soup(gua::RenderContext const& ctx);
    void draw_textured_triangle_soup(gua::RenderContext const& ctx, std::shared_ptr<gua::ShaderProgram>& shader_program);
    bool update(gua::RenderContext const& ctx, gua::math::BoundingBox<gua::math::vec3>& in_out_bb);
    void update_feedback(gua::RenderContext const& ctx);

    bool is_vertex_data_fully_encoded() { return m_model_descriptor_.is_fully_encoded_vertex_data; }

    inline unsigned char* getBuffer() { return m_buffer_.data(); }

    //unsigned get_remote_server_screen_width() const { return remote_server_screen_width_to_return_; }
    //unsigned get_remote_server_screen_height() const { return remote_server_screen_height_to_return_; }

    std::string get_socket_string() const;
    // float       get_voxel_size() const;

#ifdef GUACAMOLE_ENABLE_TURBOJPEG
    std::unordered_map<uint32_t, tjhandle> m_jpeg_decompressor_per_layer;
#endif //GUACAMOLE_ENABLE_TURBOJPEG

    SPointsStats get_latest_spoints_stats()
    {
        std::lock_guard<std::mutex> lock(m_mutex_);

        return SPointsStats{m_model_descriptor_.received_textured_tris, 
                            m_model_descriptor_.received_kinect_timestamp, 
                            m_model_descriptor_.received_reconstruction_time, 
                            m_model_descriptor_.request_reply_latency_ms, 
                            m_model_descriptor_.total_message_payload_in_byte};
    }

    // void push_matrix_package(bool is_camera, std::size_t view_uuid, bool is_stereo_mode, matrix_package mp);
    void push_matrix_package(spoints::camera_matrix_package const& cam_mat_package);

    bool has_calibration(gua::RenderContext const& ctx) { return m_received_calibration_[ctx.id].load(); }

  // helper functions
  private:
#ifdef GUACAMOLE_ENABLE_TURBOJPEG
    bool _decompress_and_rewrite_message(std::vector<std::size_t> const& byte_offset_to_jpeg_windows);
#endif //GUACAMOLE_ENABLE_TURBOJPEG
    void _readloop();

    void _try_swap_calibration_data_cpu();

    bool _try_swap_calibration_data_gpu(gua::RenderContext const& ctx);

    void _try_swap_model_data_cpu();

    // receiving geometry


  // member variables
  private: 


    static size_t constexpr INITIAL_VBO_SIZE = 50000000;
    static uint16_t constexpr MAX_LAYER_IDX = 16;
    static uint64_t constexpr MAX_NUM_SUPPORTED_CONTEXTS = 12;

    std::mutex m_mutex_;
    std::atomic<bool> m_running_;
    std::string const m_server_endpoint_;
    std::string const m_feedback_endpoint_;
    std::vector<uint8_t> m_buffer_;
    std::vector<uint8_t> m_buffer_back_;
    std::vector<uint8_t> m_buffer_back_compressed_;

    std::vector<uint8_t> m_texture_buffer_ = std::vector<uint8_t>(11059200, 0);
    std::vector<uint8_t> m_texture_buffer_back_ = std::vector<uint8_t>(11059200, 0);

    uint8_t* m_tj_compressed_image_buffer_ = nullptr;
    std::array<uint8_t, 1024 * 1024 * 100> m_decompressed_image_buffer_;

    std::vector<uint8_t> m_calibration_;
    std::vector<uint8_t> m_calibration_back_;

    std::atomic<bool> m_need_calibration_cpu_swap_{false};



    std::vector<std::atomic<bool>> m_need_calibration_gpu_swap_ = std::vector<std::atomic<bool>>(MAX_NUM_SUPPORTED_CONTEXTS);
    std::vector<std::atomic<bool>> m_received_calibration_ = std::vector<std::atomic<bool>>(MAX_NUM_SUPPORTED_CONTEXTS);

    std::vector<bool> m_bound_calibration_data_ = std::vector<bool>(MAX_NUM_SUPPORTED_CONTEXTS, false);

   // mutable std::unordered_map<std::size_t, std::atomic<bool>> m_need_calibration_gpu_swap_;
   // mutable std::unordered_map<std::size_t, std::atomic<bool>> m_received_calibration_;

   // mutable std::unordered_map<std::size_t, bool> m_bound_calibration_data_;


    SPointsCalibrationDescriptor m_calibration_descriptor_;
    SPointsCalibrationDescriptor m_calibration_descriptor_back_;

    SPointsModelDescriptor m_model_descriptor_;
    SPointsModelDescriptor m_model_descriptor_back_;

    std::array<uint32_t, 16> m_num_best_triangles_for_sensor_layer_;
    std::array<uint32_t, 16> m_num_best_triangles_for_sensor_layer_back_;

    std::unordered_map<std::size_t, std::array<uint32_t, 16>> m_current_num_best_triangles_for_sensor_layer_per_context_;

    scm::math::vec3 m_tight_geometry_bb_min_back_;
    scm::math::vec3 m_tight_geometry_bb_min_;
    scm::math::vec3 m_tight_geometry_bb_max_back_;
    scm::math::vec3 m_tight_geometry_bb_max_;

    std::unordered_map<std::size_t, scm::math::vec3> m_current_tight_geometry_bb_min_per_context_;
    std::unordered_map<std::size_t, scm::math::vec3> m_current_tight_geometry_bb_max_per_context_;

    std::array<uint32_t, 4 * MAX_LAYER_IDX> m_texture_space_bounding_boxes_;
    std::array<uint32_t, 4 * MAX_LAYER_IDX> m_texture_space_bounding_boxes_back_;



    float m_lod_scaling_ = 1.0f;
    float m_lod_scaling_back_ = 1.0f;

    mutable std::unordered_map<std::size_t, float> m_current_lod_scaling_per_context_;

    std::atomic<bool> m_need_model_cpu_swap_{false};
    mutable std::unordered_map<std::size_t, std::atomic<bool>> m_need_model_gpu_swap_;
    std::thread m_recv_;

    // sending matrices
    std::mutex m_feedback_mutex_;
    // bool       m_feedback_running_;
    const std::string m_server_feedback_endpoint_;
    matrix_package m_matrix_package_;
    matrix_package m_matrix_package_back_;

    spoints::camera_matrix_package submitted_camera_matrix_package_;
    spoints::camera_matrix_package submitted_camera_matrix_package_back_;

    bool current_feedback_is_camera_status_ = true;
    std::size_t current_feedback_view_uuid_ = {0};
    bool current_feedback_is_stereo_mode_;

    // std::map<key_package, matrix_package> cam_matrix_packages;
    std::vector<matrix_package> finalized_matrix_packages_to_submit_;
    std::vector<matrix_package> matrix_packages_to_submit_;
    std::vector<matrix_package> matrix_packages_to_collect_;

    std::size_t last_frame_count_ = std::numeric_limits<std::size_t>::max();
    std::size_t last_omitted_frame_count_ = std::numeric_limits<std::size_t>::max();

    std::array<float, 3> latest_received_bb_min;
    std::array<float, 3> latest_received_bb_max;

    volatile std::atomic<int> num_clients_cpu_swapping_{0};
    volatile std::atomic<int> num_clients_gpu_swapping_{0};

    scm::gl::sampler_state_ptr linear_sampler_state_;

    // mutable std::unordered_set<std::size_t> known_context_ids_;
    mutable std::unordered_set<std::size_t> encountered_context_ids_for_feedback_frame_;
    //mutable std::vector<std::size_t> encountered_context_ids_for_feedback_frame_(100);



    //mutable std::unordered_map<std::size_t, scm::gl::vertex_array_ptr> point_layout_per_context_;
    //mutable std::unordered_map<std::size_t, scm::gl::buffer_ptr> net_data_vbo_per_context_;
    // used for attributeless rendering
    //mutable std::unordered_map<std::size_t, scm::gl::buffer_ptr> empty_vbo_per_context_;

    mutable std::vector<scm::gl::vertex_array_ptr> point_layout_per_context_ = std::vector<scm::gl::vertex_array_ptr>(50, nullptr);
    
    mutable std::vector<scm::gl::buffer_ptr> net_data_vbo_per_context_ = std::vector<scm::gl::buffer_ptr>(MAX_NUM_SUPPORTED_CONTEXTS, nullptr);
    mutable std::vector<scm::gl::buffer_ptr> empty_vbo_per_context_ = std::vector<scm::gl::buffer_ptr>(MAX_NUM_SUPPORTED_CONTEXTS, nullptr);




    mutable std::vector<scm::gl::texture_2d_ptr> texture_atlas_per_context_ = std::vector<scm::gl::texture_2d_ptr>(MAX_NUM_SUPPORTED_CONTEXTS, nullptr);

    //mutable std::unordered_map<std::size_t, std::vector<scm::gl::texture_3d_ptr>> inv_xyz_calibs_per_context_;
    mutable std::vector<std::vector<scm::gl::texture_3d_ptr>> inv_xyz_calibs_per_context_ = std::vector<std::vector<scm::gl::texture_3d_ptr>>(MAX_NUM_SUPPORTED_CONTEXTS, std::vector<scm::gl::texture_3d_ptr>(4, nullptr) );
    mutable std::vector<std::vector<scm::gl::texture_3d_ptr>> uv_calibs_per_context_ = std::vector<std::vector<scm::gl::texture_3d_ptr>>(MAX_NUM_SUPPORTED_CONTEXTS, std::vector<scm::gl::texture_3d_ptr>(4, nullptr) );

    //mutable std::unordered_map<std::size_t, std::vector<scm::gl::texture_3d_ptr>> uv_calibs_per_context_;

    //mutable std::unordered_map<std::size_t, std::size_t> net_data_vbo_size_per_context_;
    mutable std::vector<std::size_t> net_data_vbo_size_per_context_ = std::vector<std::size_t>(MAX_NUM_SUPPORTED_CONTEXTS, 0);

    mutable std::vector<std::size_t> num_vertex_colored_points_to_draw_per_context_ = std::vector<std::size_t>(MAX_NUM_SUPPORTED_CONTEXTS, 0);
    mutable std::vector<std::size_t> num_vertex_colored_tris_to_draw_per_context_ = std::vector<std::size_t>(MAX_NUM_SUPPORTED_CONTEXTS, 0);
    mutable std::vector<std::size_t> num_textured_tris_to_draw_per_context_ = std::vector<std::size_t>(MAX_NUM_SUPPORTED_CONTEXTS, 0);

    mutable std::vector<bool> is_vbo_created_per_context_ = std::vector<bool>(MAX_NUM_SUPPORTED_CONTEXTS, false);

    mutable std::vector<bool> are_textures_created_per_context_ = std::vector<bool>(MAX_NUM_SUPPORTED_CONTEXTS, false);
    

    mutable std::vector<bool> is_calibration_data_created_per_context_ = std::vector<bool>(MAX_NUM_SUPPORTED_CONTEXTS, false);

    mutable std::vector<std::size_t> encountered_frame_counts_per_context_ = std::vector<std::size_t>(MAX_NUM_SUPPORTED_CONTEXTS, 0);
    //mutable std::unordered_map<std::size_t, std::size_t> num_vertex_colored_points_to_draw_per_context_;
    //mutable std::unordered_map<std::size_t, std::size_t> num_vertex_colored_tris_to_draw_per_context_;
    //mutable std::unordered_map<std::size_t, std::size_t> num_textured_tris_to_draw_per_context_;

    //mutable std::unordered_map<std::size_t, bool> is_vbo_created_per_context_; // = false;

    //mutable std::unordered_map<std::size_t, std::size_t> encountered_frame_counts_per_context_; // = false;
};

} // namespace spoints

#endif // #ifndef SPOINTS_NETKINECTARRAY_HPP
