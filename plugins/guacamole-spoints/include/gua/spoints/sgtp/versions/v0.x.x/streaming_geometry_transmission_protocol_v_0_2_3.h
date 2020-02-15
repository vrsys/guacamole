#ifndef SGTP_PROTOCOL_V_0_2_3
#define SGTP_PROTOCOL_V_0_2_3
// streaming geometry transmission protocol v0.2.3

#include <cstdint>
#include <chrono>
#include <limits>

using chrono_timestamp = std::chrono::time_point<std::chrono::system_clock>;

namespace SGTP {
  uint8_t     const _MAX_NUM_SENSORS = 16;
  std::size_t const _TEXTURE_DIMENSION_X = 2*2560;
  std::size_t const _TEXTURE_DIMENSION_Y = 2*1280;

  namespace v_0_2_3{

  	uint32_t const _MAJOR_VERSION = 0U;
  	uint32_t const _MINOR_VERSION = 2U;
  	uint32_t const _MICRO_VERSION = 3U;

  struct _uint16_pos2d_t {
    uint16_t u = std::numeric_limits<uint16_t>::max();
    uint16_t v = std::numeric_limits<uint16_t>::max();

    uint16_t& operator[](int32_t const idx) { return ((idx == 0) ? u : v ); }
  };

  struct _texture_bounding_box_t {
    _uint16_pos2d_t min;
    _uint16_pos2d_t max;

    _uint16_pos2d_t& operator[](int32_t const idx) { return ((idx == 0) ? min : max ); }
  };

	struct _header_data_t {
    bool        is_calibration_data             = false;

	  // data filled in case is_calibration_data = false
    bool        is_data_compressed        = true;
    bool        is_fully_encoded_vertex_data = false;
    float                    global_bb_min[3]                = {0, 0, 0}; 
	  float                    global_bb_max[3]                = {0, 0, 0};  
	  float                    timestamp                       = -1.0f;
	  float                    geometry_creation_time_in_ms    = -1.0f;
	  uint32_t                 num_textured_triangles       = 0;
    uint32_t                 geometry_payload_size        = 0;         
	  uint32_t                 texture_payload_size         = 0;         
	  uint32_t                 num_best_triangles_per_sensor[_MAX_NUM_SENSORS]   = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    _texture_bounding_box_t  tex_bounding_box[_MAX_NUM_SENSORS];
    uint32_t                 bounding_box_pixel_coverage[_MAX_NUM_SENSORS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint32_t                 jpeg_bytes_per_sensor[_MAX_NUM_SENSORS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	  float                    lod_scaling = 1.0;
    float                    texture_lod_scaling = 1.0;
    int32_t                  package_reply_id = -1;
    int64_t                  passed_microseconds_since_request;


	  // data filled in case (is_calibration_data == true)
	  uint32_t    inv_xyz_volume_res[3]           = {0, 0, 0};
	  uint32_t    uv_volume_res[3]                = {0, 0, 0};
	  uint32_t    num_sensors		      = 0;
	  float       inv_vol_to_world_mat[16]        = {1.0, 0.0, 0.0, 0.0,
	  						                                   0.0, 1.0, 0.0, 0.0,
	  						                                   0.0, 0.0, 1.0, 0.0,
	  						                                   0.0, 0.0, 0.0, 1.0};

	  // contains total payload of either avatar or calibration data
	  uint32_t    total_payload                = 0;

    void fill_texture_byte_offsets_to_bounding_boxes() {

      for(int kinect_layer_idx = 0; kinect_layer_idx < 4; ++kinect_layer_idx) { 

        uint32_t current_bb_width  = (1 + tex_bounding_box[kinect_layer_idx].max.u
                                        - tex_bounding_box[kinect_layer_idx].min.u);
        uint32_t current_bb_height = (1 + tex_bounding_box[kinect_layer_idx].max.v 
                                        - tex_bounding_box[kinect_layer_idx].min.v);
        bounding_box_pixel_coverage[kinect_layer_idx] = current_bb_height * current_bb_width;
      }

    }
	};

    struct _vertex_data_t {
      // offset:
      // 14 bit qz pos x
      // 13 bit qz pos y
      // 13 bit qz pos z
      // 3x8 bit plain rgb OR 2x 12 bit integer uv-coords
      uint64_t data;
    };

  std::size_t const _MAX_MESSAGE_SIZE = 500 * 1024 * 1024 ; // 500 MB

  //textured triangle formats
  std::size_t const _VERTEX_XYZ_3x32F_UV_2x32F_UNCOMPRESSED_SIZE  = 3 * sizeof(float) + 2 * sizeof(float);
  std::size_t const _VERTEX_XYZ_3x16UI_QUANTIZED_SIZE     = 3 * sizeof(uint16_t); //16 bit per vertex positions
 

  } // namespace v0.2.3

  using namespace v_0_2_3;
} //namespace SGTP

#endif //SGTP_PROTOCOL_V_0_2_3
