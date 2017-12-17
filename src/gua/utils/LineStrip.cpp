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

  std::cout << "INITIAL LINE BUFFER SIZE: " << intitial_line_buffer_size << "\n";
  positions.resize(vertex_reservoir_size);
  colors.resize(vertex_reservoir_size);
  thicknesses.resize(vertex_reservoir_size);

}


LineStrip::
LineStrip(LineObject const& line_object) {
  //create line strip vbos from parsed line object

  if(line_object.vertex_attribute_ids.empty()){
    //consider all vertex attributes in order of their appearance
    if(    (line_object.vertex_position_database.size() != line_object.vertex_color_database.size()) 
        || (line_object.vertex_position_database.size() != line_object.vertex_thickness_database.size()) ) {
      Logger::LOG_WARNING << "Unequal size of line strip vertex attributes!" << std::endl;
      Logger::LOG_WARNING << line_object.vertex_position_database.size() << ", "
                          << line_object.vertex_color_database.size() << ", "
                          << line_object.vertex_thickness_database.size() << "\n";
    } else {
      vertex_reservoir_size = num_occupied_vertex_slots = line_object.vertex_position_database.size();
      positions   = line_object.vertex_position_database;
      colors      = line_object.vertex_color_database;
      thicknesses = line_object.vertex_thickness_database;

    }
  } else {
    //indexed construction not implemented yet
  }
}

void LineStrip::
enlarge_reservoirs() {
  if(vertex_reservoir_size > 0) {
    vertex_reservoir_size *= 2;
  } else {
    vertex_reservoir_size = 2;
  }
  unsigned int padded_reservoir_size = vertex_reservoir_size + 2;
  positions.resize(padded_reservoir_size);
  colors.resize(padded_reservoir_size);
  thicknesses.resize(padded_reservoir_size);
}

bool LineStrip::push_vertex(Vertex const& v_to_push) {
  if(num_occupied_vertex_slots >= vertex_reservoir_size) {
    enlarge_reservoirs();
  }

  positions[num_occupied_vertex_slots] = v_to_push.pos;
  colors[num_occupied_vertex_slots] = v_to_push.col;
  thicknesses[num_occupied_vertex_slots] = v_to_push.thick;

  ++num_occupied_vertex_slots;
 return true;
}

bool LineStrip::pop_front_vertex() {
  if(num_occupied_vertex_slots < 2) {
    Logger::LOG_WARNING << "No LineStrip Vertex left to pop!" << std::endl;
    return false;
  }
  positions.erase(positions.begin());
  colors.erase(colors.begin());
  thicknesses.erase(thicknesses.begin());

  --num_occupied_vertex_slots;
  return true;
}

bool LineStrip::pop_back_vertex() {
  if(num_occupied_vertex_slots < 2) {
    Logger::LOG_WARNING << "No LineStrip Vertex left to pop!" << std::endl;

    return false;
  }
  positions.pop_back();
  colors.pop_back();
  thicknesses.pop_back();

  --num_occupied_vertex_slots;

  return true;
}

bool LineStrip::clear_vertices() {
  if(!num_occupied_vertex_slots == 0) {
    positions.clear();
    colors.clear();
    thicknesses.clear();

    num_occupied_vertex_slots = 0;

    return true;
  }
  return false;
}

void LineStrip::copy_to_buffer(Vertex* vertex_buffer)  const {
 
  std::cout << "NUM OCCUPIED VERTEX SLOTS: " << num_occupied_vertex_slots << "\n";

  vertex_buffer[0].pos = positions[0];
  vertex_buffer[0].col = colors[0];
  vertex_buffer[0].thick = thicknesses[0];

  for (int vertex_id(0); vertex_id < num_occupied_vertex_slots; ++vertex_id) {
    vertex_buffer[vertex_id+1].pos = positions[vertex_id];
    vertex_buffer[vertex_id+1].col = colors[vertex_id];
    vertex_buffer[vertex_id+1].thick = thicknesses[vertex_id];
  }

  vertex_buffer[num_occupied_vertex_slots + 1].pos = positions[num_occupied_vertex_slots-1];
  vertex_buffer[num_occupied_vertex_slots + 1].col = colors[num_occupied_vertex_slots-1];
  vertex_buffer[num_occupied_vertex_slots + 1].thick = thicknesses[num_occupied_vertex_slots-1];


}

scm::gl::vertex_format LineStrip::get_vertex_format() const {
  return scm::gl::vertex_format(
    0, 0, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
    0, 1, scm::gl::TYPE_VEC4F, sizeof(Vertex))(
    0, 2, scm::gl::TYPE_FLOAT, sizeof(Vertex));
}

} // namespace gua