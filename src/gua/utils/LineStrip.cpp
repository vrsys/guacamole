// class header
#include <gua/utils/LineStrip.hpp>

// guacamole headers
#include <gua/utils/Logger.hpp>
#include <gua/utils/ToGua.hpp>
// #include <gua/utils/Timer.hpp>

//external headers
#include <iostream>


namespace gua {


LineStrip::
LineStrip(unsigned int intitial_line_buffer_size) : 
  vertex_reservoir_size(intitial_line_buffer_size),
  num_occupied_vertex_slots(0) {
  positions.resize(vertex_reservoir_size);
  colors.resize(vertex_reservoir_size);
  thicknesses.resize(vertex_reservoir_size);
}

void LineStrip::
enlarge_reservoirs() {
  vertex_reservoir_size *= 2;
  unsigned int padded_reservoir_size = vertex_reservoir_size + 2;
  positions.resize(padded_reservoir_size);
  colors.resize(padded_reservoir_size);
  thicknesses.resize(padded_reservoir_size);
}

void LineStrip::push_vertex(Vertex const& v_to_push) {
  if(num_occupied_vertex_slots == vertex_reservoir_size) {
    enlarge_reservoirs();
  }

  positions[num_occupied_vertex_slots] = v_to_push.pos;
  colors[num_occupied_vertex_slots] = v_to_push.col;
  thicknesses[num_occupied_vertex_slots] = v_to_push.thick;

  ++num_occupied_vertex_slots;
}

void LineStrip::pop_vertex() {
  if(!num_occupied_vertex_slots == 0) {
    std::cout << "No Vertex left to pop!\n";
    return;
  }
  --num_occupied_vertex_slots;
}


void LineStrip::copy_to_buffer(Vertex* vertex_buffer)  const {
  for (int vertex_id(0); vertex_id < num_occupied_vertex_slots; ++vertex_id) {

    vertex_buffer[vertex_id].pos = positions[vertex_id];

    vertex_buffer[vertex_id].col = colors[vertex_id];

    vertex_buffer[vertex_id].thick = thicknesses[vertex_id];


  }
}

scm::gl::vertex_format LineStrip::get_vertex_format() const {
  return scm::gl::vertex_format(
    0, 0, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
    0, 1, scm::gl::TYPE_VEC4F, sizeof(Vertex))(
    0, 2, scm::gl::TYPE_FLOAT, sizeof(Vertex));
}

} // namespace gua