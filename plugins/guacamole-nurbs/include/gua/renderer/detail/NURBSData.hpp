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

namespace gua {

struct NURBSData 
{
 private:

  struct page {
   public:
    page() {}
    ~page() {}

    unsigned surface_offset;
    unsigned order_u;
    unsigned order_v;
    unsigned trim_id;

    scm::math::vec4f nurbs_domain;
    scm::math::vec4f bbox_min;
    scm::math::vec4f bbox_max;

    scm::math::vec4f dist;
  };

 public:

  //Constructor and Destructor
   NURBSData(std::shared_ptr<gpucast::beziersurfaceobject> const& o, unsigned pre_subdivision_u, unsigned pre_subdivision_v);

  virtual ~NURBSData();

  std::shared_ptr<gpucast::beziersurfaceobject> object;

  // adaptive_tesselation data
  std::vector<scm::math::vec4f> tess_patch_data;       // Domain Points
  std::vector<unsigned>         tess_index_data;       // Index Data
  std::vector<scm::math::vec4f> tess_parametric_data;  // Control Points of all the surfaces
  std::vector<page>             tess_attribute_data;   

  //Data for Trimming
  std::vector<scm::math::vec4f> trim_partition;
  std::vector<scm::math::vec2f> trim_contourlist;
  std::vector<scm::math::vec4f> trim_curvelist;
  std::vector<float>            trim_curvedata;
  std::vector<scm::math::vec3f> trim_pointdata;

  //gua::math::BoundingBox<scm::math::vec3f> bbox;
};

}  // namespace gua

#endif  // GUA_NURBSDATA_HPP_INCLUDED
