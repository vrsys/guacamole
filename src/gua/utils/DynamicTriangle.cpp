// class header
#include <gua/utils/DynamicTriangle.hpp>

// guacamole headers
#include <gua/utils/Logger.hpp>
#include <gua/utils/ToGua.hpp>
// #include <gua/utils/Timer.hpp>

// external headers
#include <iostream>

#include <mutex>

namespace gua
{
DynamicTriangle::DynamicTriangle(unsigned int intitial_geometry_buffer_size) : DynamicGeometry::DynamicGeometry(intitial_geometry_buffer_size) { uvs.resize(vertex_reservoir_size); }

DynamicTriangle::DynamicTriangle(DynamicGeometryObject const& dynamic_geometry_object)
{
    // create dynamic geometry vbos from parsed dynamic geometry object

    if(dynamic_geometry_object.vertex_attribute_ids.empty())
    {
        // consider all vertex attributes in order of their appearance
        if((dynamic_geometry_object.vertex_position_database.size() != dynamic_geometry_object.vertex_color_database.size()) ||
           (dynamic_geometry_object.vertex_position_database.size() != dynamic_geometry_object.vertex_thickness_database.size()) ||
           (dynamic_geometry_object.vertex_position_database.size() != dynamic_geometry_object.vertex_uv_database.size()))
        {
            Logger::LOG_WARNING << "Unequal size of dynamic geometry vertex attributes!" << std::endl;
            Logger::LOG_WARNING << dynamic_geometry_object.vertex_position_database.size() << ", " << dynamic_geometry_object.vertex_color_database.size() << ", "
                                << dynamic_geometry_object.vertex_thickness_database.size() << ", " << dynamic_geometry_object.vertex_uv_database.size() << "\n";
        }
        else
        {
            vertex_reservoir_size = num_occupied_vertex_slots = dynamic_geometry_object.vertex_position_database.size();
            positions = dynamic_geometry_object.vertex_position_database;
            colors = dynamic_geometry_object.vertex_color_database;
            thicknesses = dynamic_geometry_object.vertex_thickness_database;
            uvs = dynamic_geometry_object.vertex_uv_database;
        }
    }
    else
    {
        // indexed construction not implemented yet
    }
}

void DynamicTriangle::enlarge_reservoirs()
{
    if(vertex_reservoir_size > 0)
    {
        vertex_reservoir_size *= 2;
    }
    else
    {
        vertex_reservoir_size = 2;
    }
    unsigned int padded_reservoir_size = vertex_reservoir_size + 2;
    positions.resize(padded_reservoir_size);
    colors.resize(padded_reservoir_size);
    thicknesses.resize(padded_reservoir_size);
    uvs.resize(padded_reservoir_size);
}

void DynamicTriangle::compute_consistent_normals() const
{
    // bool last_plane_normal_exists = false;
    // scm::math::vec3f last_plane_normal = scm::math::vec3f(0.0f, 0.0f, 0.0f);

    // int32_t positions_size_minus_one = int32_t(positions.size()) - 1;

    // for(int32_t normal_idx = 0; normal_idx < num_occupied_vertex_slots; ++normal_idx) {
    //   if(0 == normal_idx) {
    //     if(positions_size_minus_one == normal_idx) {
    //       normals[0] = scm::math::vec3f(0.0, 1.0, 0.0);
    //     }
    //     else {
    //       scm::math::vec3 to_normalize = positions[1] - positions[0];
    //       if(scm::math::length(to_normalize) > 1e-6f) {
    //         normals[0] = scm::math::normalize(to_normalize);
    //       } else {
    //         normals[0] = scm::math::vec3f(1.0f, 0.0f, 0.0);
    //       }

    //     }
    //   } else if(positions_size_minus_one == normal_idx) {
    //     if(0 == normal_idx) {
    //       normals[positions_size_minus_one] = scm::math::vec3f(0.0, 1.0, 0.0);
    //     } else {

    //       scm::math::vec3 to_normalize = positions[positions_size_minus_one] - positions[positions_size_minus_one - 1];
    //       if(scm::math::length(to_normalize) > 1e-6f) {
    //         normals[positions_size_minus_one] = scm::math::normalize(to_normalize);
    //       } else {
    //         normals[positions_size_minus_one] = scm::math::vec3f(0.0, 1.0, 0.0);
    //       }
    //     }
    //   } else { //actual computation with consistency check

    //     scm::math::vec3 p0_to_pC = positions[normal_idx] - positions[normal_idx-1];
    //     scm::math::vec3 pC_to_p1 = positions[normal_idx+1] - positions[normal_idx];

    //     scm::math::vec3 plane_normal = scm::math::cross(p0_to_pC, pC_to_p1);

    //     if(last_plane_normal_exists) {
    //       if( scm::math::dot(last_plane_normal, plane_normal) < 0.0f ) {
    //         plane_normal *= -1.0f;
    //       }
    //     }

    //     last_plane_normal_exists = true;
    //     last_plane_normal = plane_normal;

    //     scm::math::mat4f rot_mat = scm::math::make_rotation(90.0f, plane_normal);

    //     scm::math::vec4 p0_to_p1 = scm::math::vec3(p0_to_pC + pC_to_p1, 0.0f);

    //     scm::math::vec3 to_normalize = scm::math::vec3f(rot_mat * p0_to_p1);
    //     if(scm::math::length(to_normalize) > 1e-6f) {
    //       scm::math::vec3f final_normal = scm::math::normalize(to_normalize);
    //       normals[normal_idx] = final_normal;
    //     } else {
    //       normals[positions_size_minus_one] = scm::math::vec3f(0.0, 0.0, 1.0);
    //     }

    //   }
    // }
}

void DynamicTriangle::compile_buffer_string(std::string& buffer_string)
{
    uint64_t num_vertices_to_write = num_occupied_vertex_slots;

    uint64_t size_of_byte_count = sizeof(uint64_t);
    uint64_t size_of_positions = num_vertices_to_write * sizeof(Vertex::pos);
    uint64_t size_of_colors = num_vertices_to_write * sizeof(Vertex::col);
    uint64_t size_of_thicknesses = num_vertices_to_write * sizeof(Vertex::thick);
    uint64_t size_of_uvs = num_vertices_to_write * sizeof(TriVertex::uv);

    uint64_t num_string_bytes = size_of_byte_count + size_of_positions + size_of_colors + size_of_thicknesses + size_of_uvs;

    std::string tmp_string(num_string_bytes, '0');

    uint64_t write_offset = 0;
    memcpy(&tmp_string[write_offset], &num_vertices_to_write, size_of_byte_count);
    write_offset += size_of_byte_count;
    memcpy(&tmp_string[write_offset], &positions[0], size_of_positions);
    write_offset += size_of_positions;
    memcpy(&tmp_string[write_offset], &colors[0], size_of_colors);
    write_offset += size_of_colors;
    memcpy(&tmp_string[write_offset], &thicknesses[0], size_of_thicknesses);
    write_offset += size_of_thicknesses;
    memcpy(&tmp_string[write_offset], &uvs[0], size_of_uvs);
    write_offset += size_of_uvs;

    buffer_string = tmp_string;
}

void DynamicTriangle::uncompile_buffer_string(std::string const& buffer_string)
{
    uint64_t num_vertices_written = 0;
    uint64_t read_offset = 0;


    if(buffer_string.size() >= sizeof(uint64_t))
    {
        memcpy(&num_vertices_written, &buffer_string[0], sizeof(num_vertices_written));
    }

    if(buffer_string.size() != num_vertices_written * sizeof(TriVertex) + sizeof(uint64_t))
    {
        Logger::LOG_WARNING << "Buffer String size and expected size are not consistent! Ignoring dynamic triangle update." << std::endl;
        return;
    }

    uint64_t currently_occupied_vertex_slots = 0;
    int64_t newly_occupied_vertex_slots = currently_occupied_vertex_slots + num_vertices_written;

    if(newly_occupied_vertex_slots > vertex_reservoir_size)
    {
        positions.resize(newly_occupied_vertex_slots);
        colors.resize(newly_occupied_vertex_slots);
        thicknesses.resize(newly_occupied_vertex_slots);
        uvs.resize(newly_occupied_vertex_slots);

        vertex_reservoir_size = newly_occupied_vertex_slots;
    }

    read_offset += sizeof(num_vertices_written);
    memcpy(&positions[currently_occupied_vertex_slots], &buffer_string[read_offset], num_vertices_written * sizeof(Vertex::pos));
    read_offset += num_vertices_written * sizeof(Vertex::pos);
    memcpy(&colors[currently_occupied_vertex_slots], &buffer_string[read_offset], num_vertices_written * sizeof(Vertex::col));
    read_offset += num_vertices_written * sizeof(Vertex::col);
    memcpy(&thicknesses[currently_occupied_vertex_slots], &buffer_string[read_offset], num_vertices_written * sizeof(Vertex::thick));
    read_offset += num_vertices_written * sizeof(Vertex::thick);
    memcpy(&uvs[currently_occupied_vertex_slots], &buffer_string[read_offset], num_vertices_written * sizeof(TriVertex::uv));

    num_occupied_vertex_slots = num_vertices_written;

}

bool DynamicTriangle::push_vertex(TriVertex const& v_to_push)
{
    if(num_occupied_vertex_slots >= vertex_reservoir_size)
    {
        enlarge_reservoirs();
    }

    positions[num_occupied_vertex_slots] = v_to_push.pos;
    colors[num_occupied_vertex_slots] = v_to_push.col;
    thicknesses[num_occupied_vertex_slots] = v_to_push.thick;
    uvs[num_occupied_vertex_slots] = v_to_push.uv;
    ++num_occupied_vertex_slots;
    return true;
}

bool DynamicTriangle::update_vertex(int vertex_idx, TriVertex const& v_to_update)
{
    if(vertex_idx < positions.size())
    {
        positions[vertex_idx] = v_to_update.pos;
        colors[vertex_idx] = v_to_update.col;
        thicknesses[vertex_idx] = v_to_update.thick;
        uvs[vertex_idx] = v_to_update.uv;

        return true;
    }
    else
    {
        return false;
    }
}

bool DynamicTriangle::pop_front_vertex()
{
    if(num_occupied_vertex_slots < 2)
    {
        Logger::LOG_WARNING << "No DynamicTriangle Vertex left to pop!" << std::endl;
        return false;
    }
    positions.erase(positions.begin());
    colors.erase(colors.begin());
    thicknesses.erase(thicknesses.begin());
    uvs.erase(uvs.begin());

    --num_occupied_vertex_slots;
    return true;
}

bool DynamicTriangle::pop_back_vertex()
{
    if(num_occupied_vertex_slots < 2)
    {
        Logger::LOG_WARNING << "No DynamicGeometry Vertex left to pop!" << std::endl;

        return false;
    }
    positions.pop_back();
    colors.pop_back();
    thicknesses.pop_back();
    uvs.pop_back();

    --num_occupied_vertex_slots;

    return true;
}

bool DynamicTriangle::clear_vertices()
{
    if(num_occupied_vertex_slots != 0)
    {
        uvs.clear();
    }

    return DynamicGeometry::clear_vertices();
}

void DynamicTriangle::forward_queued_vertices(std::vector<scm::math::vec3f> const& queued_positions,
                                              std::vector<scm::math::vec4f> const& queued_colors,
                                              std::vector<float> const& queued_thicknesses,
                                              std::vector<scm::math::vec2f> const& queued_uvs)
{
    uvs = queued_uvs;
    DynamicGeometry::forward_queued_vertices(queued_positions, queued_colors, queued_thicknesses);
}

void DynamicTriangle::copy_to_buffer(TriVertex* vertex_buffer) const
{
    for(int vertex_id(0); vertex_id < num_occupied_vertex_slots; ++vertex_id)
    {
        vertex_buffer[vertex_id].pos = positions[vertex_id];
        vertex_buffer[vertex_id].col = colors[vertex_id];
        vertex_buffer[vertex_id].thick = thicknesses[vertex_id];
        vertex_buffer[vertex_id].uv = uvs[vertex_id];
    }
}

scm::gl::vertex_format DynamicTriangle::get_vertex_format() const
{
    return scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC3F, sizeof(TriVertex))(0, 1, scm::gl::TYPE_VEC4F, sizeof(TriVertex))(0, 2, scm::gl::TYPE_FLOAT, sizeof(TriVertex))(
        0, 3, scm::gl::TYPE_VEC2F, sizeof(TriVertex));
}

} // namespace gua