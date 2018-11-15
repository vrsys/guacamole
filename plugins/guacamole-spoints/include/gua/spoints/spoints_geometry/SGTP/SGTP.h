#ifndef SGTP_PROTOCOL_HEADER
#define SGTP_PROTOCOL_HEADER
// compatability header of the streaming geometry transmission protocol

#include "streaming_geometry_transmission_protocol_v_0_1_0.h"

namespace SGTP {
  uint64_t const MAJOR_VERSION = _MAJOR_VERSION;
  uint64_t const MINOR_VERSION = _MINOR_VERSION;
  uint64_t const MICRO_VERSION = _MICRO_VERSION;


  uint64_t const VERSION = (MAJOR_VERSION * 1000U + MINOR_VERSION) * 1000U + MICRO_VERSION;

  using header_data_t = _header_data_t;
  using vertex_data_t = _vertex_data_t;

  std::size_t const MAX_MESSAGE_SIZE = _MAX_MESSAGE_SIZE;

  std::size_t const HEADER_BYTE_SIZE = sizeof(_header_data_t);
  std::size_t const MAX_PAYLOAD_SIZE = MAX_MESSAGE_SIZE - HEADER_BYTE_SIZE;


  //GEOMETRY SIZES
  std::size_t const VERTEX_COL_POINT_SIZE    = _VERTEX_COL_POINT_SIZE   ;
  std::size_t const VERTEX_COL_TRIANGLE_SIZE = _VERTEX_COL_TRIANGLE_SIZE;
  std::size_t const TEXTURED_TRIANGLE_SIZE   = _TEXTURED_TRIANGLE_SIZE  ; 

  static void print_version_string() {
  	printf("SGTP Version: %ld.%ld.%ld\n", MAJOR_VERSION, MINOR_VERSION, MICRO_VERSION);
  }

  struct send_package_t {
    header_data_t header;
    uint8_t* message;
  };
  
}

#endif //SGTP_PROTOCOL_HEADER