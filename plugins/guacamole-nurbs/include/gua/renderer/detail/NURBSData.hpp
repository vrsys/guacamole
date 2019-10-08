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

#ifndef GUA_NURBSDATA_HPP_INCLUDED
#define GUA_NURBSDATA_HPP_INCLUDED

#include <gpucast/core/import/igs.hpp>
#include <gpucast/core/beziersurfaceobject.hpp>

#include <scm/gl_core.h>
#include <gua/math/BoundingBox.hpp>

namespace gua
{
struct NURBSData
{
  public:
    struct per_patch_data
    {
        unsigned surface_offset;
        unsigned char order_u;
        unsigned char order_v;
        unsigned short trim_type;
        unsigned trim_id;
        unsigned obb_id;

        scm::math::vec4f nurbs_domain;
        scm::math::vec4f bbox_min;
        scm::math::vec4f bbox_max;

        float ratio_uv;
        float edge_length_u;
        float edge_length_v;
        float curvature;
    };

  public:
    // Constructor and Destructor
    NURBSData(std::shared_ptr<gpucast::beziersurfaceobject> const& o, unsigned pre_subdivision_u, unsigned pre_subdivision_v, unsigned trim_texture);

    std::shared_ptr<gpucast::beziersurfaceobject> object;

    // adaptive_tesselation data
    std::vector<scm::math::vec4f> tess_patch_data;      // Domain Points
    std::vector<unsigned> tess_index_data;              // Index Data
    std::vector<scm::math::vec4f> tess_parametric_data; // Control Points of all the surfaces
    std::vector<per_patch_data> tess_attribute_data;
};

} // namespace gua

#endif // GUA_NURBSDATA_HPP_INCLUDED
