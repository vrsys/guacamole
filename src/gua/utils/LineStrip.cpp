// class header
#include <gua/utils/Mesh.hpp>

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
  positions[num_occupied_vertex_slots] = v_to_push.col;
  positions[num_occupied_vertex_slots] = v_to_push.thick;

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
  for (unsigned v(0); v < num_vertices; ++v) {

    vertex_buffer[v].pos = positions[v];

    vertex_buffer[v].col = colors[v];

    vertex_buffer[v].thick = thicknesses[v];


  }
}

scm::gl::vertex_format LineStrip::get_vertex_format() const {
  return scm::gl::vertex_format(
    0, 0, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
    0, 1, scm::gl::TYPE_VEC4F, sizeof(Vertex))(
    0, 2, scm::gl::TYPE_FLOAT, sizeof(Vertex));
}

} // namespace gua