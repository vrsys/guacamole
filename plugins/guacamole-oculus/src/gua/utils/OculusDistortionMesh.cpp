/******************************************************************************
 * guacamole - delicious VR                                                   *
 *                                                                            *
 * Copyright: (c) 2011-2013 Bauhaus-Universität Weimar                        *
 * Contact:   felix.lauer@uni-weimar.de / simon.schneegans@uni-weimar.de      *
 *                                                                            *
 * This program is free software: you can redistribute it and/or modify it    *
 * under the terms of the GNU General Public License as published by the Free *
 * Software Foundation, either version 3 of the License, or (at your option)  *
 * any later version.                                                         *
 *                                                                            *
 * This program is distributed in the hope that it will be useful, but        *
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY *
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License   *
 * for more details.                                                          *
 *                                                                            *
 * You should have received a copy of the GNU General Public License along    *
 * with this program. If not, see <http://www.gnu.org/licenses/>.             *
 *                                                                            *
 ******************************************************************************/

// guacamole headers
#include <gua/utils/OculusDistortionMesh.hpp>

#ifndef _WIN32

namespace gua
{
OculusDistortionMesh::OculusDistortionMesh()
    : ndc_2d_positions(), tex_coords_r(), tex_coords_g(), tex_coords_b(), vig_factors(), indices(), num_vertices(0), num_indices(0), index_buffer_component_offset(0)
{
}

OculusDistortionMesh::~OculusDistortionMesh() {}

void OculusDistortionMesh::initialize_distortion_mesh(ovrDistortionMesh const& mesh_component, ovrVector2f* UVScaleOffset, bool isLeftEye)
{
    num_vertices = mesh_component.VertexCount;
    num_indices = mesh_component.IndexCount;

    ndc_2d_positions.reserve(num_vertices);
    tex_coords_r.reserve(num_vertices);
    tex_coords_g.reserve(num_vertices);
    tex_coords_b.reserve(num_vertices);
    vig_factors.reserve(num_vertices);
    indices.reserve(num_indices);

    // populate vertex buffer components
    for(unsigned int vertex_idx = 0; vertex_idx < mesh_component.VertexCount; ++vertex_idx)
    {
        // get vertex in oculus sdk format
        ovrDistortionVertex const& distortion_vertex = mesh_component.pVertexData[vertex_idx];

        float corrected_x_pos = distortion_vertex.ScreenPosNDC.x;

        /*
              if(isLeftEye) {
                corrected_x_pos = (distortion_vertex.ScreenPosNDC.x + 1.0) * 2.0 - 1.0;
              } else {
                corrected_x_pos = (distortion_vertex.ScreenPosNDC.x - 1.0) * 2.0 + 1.0;
              }
        */
        scm::math::vec2f vPos(corrected_x_pos, distortion_vertex.ScreenPosNDC.y);

        float tmp_red_x_tex_coord = distortion_vertex.TanEyeAnglesR.x * UVScaleOffset[0].x + UVScaleOffset[1].x;
        float tmp_green_x_tex_coord = distortion_vertex.TanEyeAnglesG.x * UVScaleOffset[0].x + UVScaleOffset[1].x;
        float tmp_blue_x_tex_coord = distortion_vertex.TanEyeAnglesB.x * UVScaleOffset[0].x + UVScaleOffset[1].x;

        float tmp_red_y_tex_coord = distortion_vertex.TanEyeAnglesR.y * UVScaleOffset[0].y + UVScaleOffset[1].y;
        float tmp_green_y_tex_coord = distortion_vertex.TanEyeAnglesG.y * UVScaleOffset[0].y + UVScaleOffset[1].y;
        float tmp_blue_y_tex_coord = distortion_vertex.TanEyeAnglesB.y * UVScaleOffset[0].y + UVScaleOffset[1].y;

        if(isLeftEye)
        {
            tmp_red_x_tex_coord -= 1.0;
            tmp_blue_x_tex_coord -= 1.0;
            tmp_green_x_tex_coord -= 1.0;
        }
        tmp_red_x_tex_coord = 1.0 - tmp_red_x_tex_coord;
        tmp_blue_x_tex_coord = 1.0 - tmp_blue_x_tex_coord;
        tmp_green_x_tex_coord = 1.0 - tmp_green_x_tex_coord;

        scm::math::vec2f vTexCoordsRed(tmp_red_x_tex_coord, 1.0 - tmp_red_y_tex_coord);

        scm::math::vec2f vTexCoordsGreen(tmp_green_x_tex_coord, 1.0 - tmp_green_y_tex_coord);

        scm::math::vec2f vTexCoordsBlue(tmp_blue_x_tex_coord, 1.0 - tmp_blue_y_tex_coord);

        ndc_2d_positions.push_back(vPos);
        tex_coords_r.push_back(vTexCoordsRed);
        tex_coords_g.push_back(vTexCoordsGreen);
        tex_coords_b.push_back(vTexCoordsBlue);
        vig_factors.push_back(distortion_vertex.VignetteFactor);
    }

    // populate index buffer
    /*
      NOTE: since all of the distortion mesh components start
            at index 0, we have to keep track of the index
            offsets on our own
    */
    unsigned highest_index = index_buffer_component_offset;
    for(unsigned int i = 0; i < mesh_component.IndexCount; ++i)
    {
        // manual->index( mesh_component.pIndexData[i] );
        unsigned current_index = index_buffer_component_offset + mesh_component.pIndexData[i];
        if(current_index + 1 > highest_index)
        {
            highest_index = current_index + 1;
        }

        indices.push_back(current_index);
    }

    index_buffer_component_offset = highest_index;
}

void OculusDistortionMesh::copy_to_buffer(DistortionVertex* d_vertex_buffer) const
{
    for(unsigned v(0); v < num_vertices; ++v)
    {
        d_vertex_buffer[v].ndc_2d_pos = ndc_2d_positions[v];

        d_vertex_buffer[v].tex_r = tex_coords_r[v];

        d_vertex_buffer[v].tex_g = tex_coords_g[v];

        d_vertex_buffer[v].tex_b = tex_coords_b[v];

        d_vertex_buffer[v].vig_factor = vig_factors[v];
    }
}

scm::gl::vertex_format OculusDistortionMesh::get_vertex_format() const
{
    return scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC2F, sizeof(DistortionVertex))(0, 1, scm::gl::TYPE_VEC2F, sizeof(DistortionVertex))(0, 2, scm::gl::TYPE_VEC2F, sizeof(DistortionVertex))(
        0, 3, scm::gl::TYPE_VEC2F, sizeof(DistortionVertex))(0, 4, scm::gl::TYPE_FLOAT, sizeof(DistortionVertex));
}

}; // namespace gua

#endif