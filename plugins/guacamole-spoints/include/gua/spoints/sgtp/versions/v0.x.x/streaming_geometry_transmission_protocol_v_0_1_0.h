#ifndef SGTP_PROTOCOL_V_0_1_0
#define SGTP_PROTOCOL_V_0_1_0
// streaming geometry transmission protocol v0.1

#include <cstdint>

namespace SGTP {
  namespace v_0_1_0{

  	uint32_t const _MAJOR_VERSION = 0U;
  	uint32_t const _MINOR_VERSION = 1U;
  	uint32_t const _MICRO_VERSION = 0U;

	struct _header_data_t {
	  float       global_bb_min[3]                = {0, 0, 0}; 
	  float       global_bb_max[3]                = {0, 0, 0}; 
	  float       timestamp                       = -1.0f;
	  float       geometry_creation_time_in_ms    = -1.0f;

	  uint64_t    num_points                   = 0;
	  uint64_t    num_vertex_col_points        = 0;         
	  uint64_t    num_vertex_col_triangles     = 0;         
	  uint64_t    num_textured_triangles       = 0;         
	  uint64_t    texture_payload_size         = 0;         
	  uint64_t    texture_space_triangle_size  = 0;
	  uint64_t    total_payload                = 0; 

         
	};

    struct _vertex_data_t {
      // offset:
      // 14 bit qz pos x
      // 13 bit qz pos y
      // 13 bit qz pos z
      // 3x8 bit plain rgb OR 2x 12 bit integer uv-coords
      uint64_t data;
    };

  std::size_t const _VERTEX_COL_POINT_SIZE    = sizeof(_vertex_data_t);
  std::size_t const _VERTEX_COL_TRIANGLE_SIZE = 3 * sizeof(_vertex_data_t);
  std::size_t const _TEXTURED_TRIANGLE_SIZE   = 3 * sizeof(_vertex_data_t); 

  std::size_t const _MAX_MESSAGE_SIZE = 50 * 1024 * 1024 ; // 50 MB

  } // namespace v0.1.0

  using namespace v_0_1_0;
} //namespace SGTP

#endif //SGTP_PROTOCOL_V_0_1_0
