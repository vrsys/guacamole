#ifndef SGTP_PROTOCOL_HEADER
#define SGTP_PROTOCOL_HEADER
// compatability header of the streaming geometry transmission protocol

#include "./versions/v0.x.x/streaming_geometry_transmission_protocol_v_0_2_3.h"

namespace SGTP {

  /* version */
  uint64_t const MAJOR_VERSION = _MAJOR_VERSION;
  uint64_t const MINOR_VERSION = _MINOR_VERSION;
  uint64_t const MICRO_VERSION = _MICRO_VERSION;

  uint64_t const VERSION = (MAJOR_VERSION * 1000U + MINOR_VERSION) * 1000U + MICRO_VERSION;

  using header_data_t = _header_data_t;
  using vertex_data_t = _vertex_data_t;
  using texture_bounding_box_t = _texture_bounding_box_t;

  /* helper function declarations */
  /* ============================ */
  // returns byte offset to geometry payload in entire message
  std::size_t get_geometry_read_offset(header_data_t const& message_header);
  // returns num bytes of geometry payload in entire message
  std::size_t get_num_geometry_bytes(header_data_t const& message_header);
  // returns byte offset to texture payload in entire message 
  std::size_t get_total_texture_read_offset(header_data_t const& message_header);
  // returns num bytes of entire texture payload in entire message
  std::size_t get_total_num_texture_bytes(header_data_t const& message_header);
  // returns num bytes of entire texture payload in entire message
  std::size_t get_texture_read_offset_for_sensor(header_data_t const& message_header, int sensor_idx);
  // returns num bytes of entire texture payload in entire message
  std::size_t get_num_texture_bytes_for_sensor(header_data_t const& message_header, int sensor_idx);
  
  // returns the integer coordinates for texture space bounding boxes in the texture atlas, 
  // s.t. it can be used for subtexture updates in from [min, max]
  texture_bounding_box_t 
  get_texture_bounding_box_for_sensor(header_data_t const& message_header, int sensor_idx);

  /* size related constants */
  std::size_t const MAX_MESSAGE_SIZE = _MAX_MESSAGE_SIZE;

  std::size_t const HEADER_BYTE_SIZE = sizeof(_header_data_t);
  std::size_t const MAX_PAYLOAD_SIZE = MAX_MESSAGE_SIZE - HEADER_BYTE_SIZE;

  std::size_t const TEXTURE_DIMENSION_X = _TEXTURE_DIMENSION_X;
  std::size_t const TEXTURE_DIMENSION_Y = _TEXTURE_DIMENSION_Y;
  
  //GEOMETRY SIZES
  //textured triangle formats
  std::size_t const TEXTURED_VERTEX_UNCOMPRESSED_SIZE    = _VERTEX_XYZ_3x32F_UV_2x32F_UNCOMPRESSED_SIZE;
  std::size_t const TEXTURED_VERTEX_QZ_POS_ONLY_SIZE     = _VERTEX_XYZ_3x16UI_QUANTIZED_SIZE; 

  /* helper functions */
  inline std::size_t get_geometry_read_offset() {
    return HEADER_BYTE_SIZE;
  }

  inline std::size_t get_num_geometry_bytes(header_data_t const& message_header) {
    return message_header.geometry_payload_size;
  }

  inline std::size_t get_total_texture_read_offset(header_data_t const& message_header) {
    return HEADER_BYTE_SIZE + message_header.geometry_payload_size;
  }

  inline std::size_t get_total_num_texture_bytes(header_data_t const& message_header) {
    return message_header.texture_payload_size;
  }

  inline std::size_t get_texture_read_offset_for_sensor(header_data_t const& message_header, int sensor_idx) {
    std::size_t local_texture_offset_per_sensor = get_total_texture_read_offset(message_header);

    for(int sensor_it_idx = 0; sensor_it_idx < (sensor_idx - 1); ++sensor_it_idx) {
      if(!message_header.is_data_compressed) {
        // BGR8 format, 3 byte per pixel
        local_texture_offset_per_sensor += 3 * message_header.bounding_box_pixel_coverage[sensor_it_idx];
      } else {
        local_texture_offset_per_sensor += message_header.jpeg_bytes_per_sensor[sensor_it_idx];
      }
    }

    return local_texture_offset_per_sensor;
  }

  inline std::size_t get_num_texture_bytes_for_sensor(header_data_t const& message_header, int sensor_idx) {
    if(!message_header.is_data_compressed) {
      return 3 * message_header.bounding_box_pixel_coverage[sensor_idx];
    } else {
      return message_header.jpeg_bytes_per_sensor[sensor_idx];
    }
  }

  inline static void print_version_string() {
    printf("SGTP Version: %ld.%ld.%ld\n", MAJOR_VERSION, MINOR_VERSION, MICRO_VERSION);
  }


  struct send_package_t {
    header_data_t header;
    uint8_t* message;
  };
  
}

#endif //SGTP_PROTOCOL_HEADER
