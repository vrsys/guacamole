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

#ifndef GUA_OCULUS_DISTORTION_MESH_HPP
#define GUA_OCULUS_DISTORTION_MESH_HPP

#if defined(_MSC_VER)
#if defined(GUA_OCULUS_LIBRARY)
#define GUA_OCULUS_DLL __declspec(dllexport)
#else
#define GUA_OCULUS_DLL __declspec(dllimport)
#endif
#else
#define GUA_OCULUS_DLL
#endif // #if defined(_MSC_VER)

#ifndef _WIN32

#include <gua/math/math.hpp>
#include <OVR.h>

#include <scm/gl_core.h>

namespace gua
{
struct OculusDistortionMesh
{
    /* the vertex contains three different texture coordinates
       since it has to correct chromatic abberations for each
       wavelength (~color channel) independently
    */
    struct DistortionVertex
    {
        scm::math::vec2f ndc_2d_pos;
        scm::math::vec2f tex_r;
        scm::math::vec2f tex_g;
        scm::math::vec2f tex_b;
        float vig_factor;
    };

    OculusDistortionMesh();

    ~OculusDistortionMesh();

    void initialize_distortion_mesh(ovrDistortionMesh const& mesh_component, ovrVector2f* UVScaleOffset, bool isLeftEye);

    void copy_to_buffer(DistortionVertex* d_vertex_buffer) const;

    virtual scm::gl::vertex_format get_vertex_format() const;

    // member
    std::vector<scm::math::vec2f> ndc_2d_positions;
    std::vector<scm::math::vec2f> tex_coords_r;
    std::vector<scm::math::vec2f> tex_coords_g;
    std::vector<scm::math::vec2f> tex_coords_b;
    std::vector<float> vig_factors;
    std::vector<unsigned> indices;

    unsigned int num_vertices;
    unsigned int num_indices;

    unsigned int index_buffer_component_offset;
};

} // namespace gua

#endif

#endif // GUA_OCULUS_DISTORTION_MESH_HPP
