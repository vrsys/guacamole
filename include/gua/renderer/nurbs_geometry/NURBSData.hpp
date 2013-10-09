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

#include <gua/renderer/nurbs_geometry/import/igs/igs_loader.hpp>
#include <gua/renderer/nurbs_geometry/TrimmedNurbsSurfaceObject.hpp>
#include <gua/renderer/nurbs_geometry/TrimmedBezierSurfaceObject.hpp>
#include <gua/renderer/nurbs_geometry/TrimmedSurfaceConverter.hpp>
#include <gua/renderer/nurbs_geometry/TrimmedBezierSurface.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/point.hpp>

#include <scm/gl_core.h>

namespace gua {

class NURBSData {
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

  typedef scm::math::vec4f point;

  template <typename T> struct buffer_data {
    buffer_data() : data() {}
    std::vector<T> data;
    unsigned size() { return data.size(); }
    void resize(int size) { data.resize(size); }
    unsigned data_size() { return data.size() * sizeof(T); }
    T* get() { return &data[0]; }
    void push(T p) { data.push_back(p); }
    std::vector<T>& original() { return data; }
    T& operator[](int index) { return data[index]; }
  };

  struct bbox {
    scm::math::vec3d _min;
    scm::math::vec3d _max;
  };

 public:
  //Constructor and Destructor
  NURBSData(std::shared_ptr<TrimmedBezierSurfaceObject> const& object,
            unsigned _InnerTessellationLevel = 5,
            unsigned _OuterTessellationLevel = 5);
  virtual ~NURBSData();

  //Tessellation Factors
  unsigned InnerTessellationLevel;
  unsigned OuterTessellationLevel;

  //Patch Data
  buffer_data<point> patch_data;  // Domain Points

  //Index Data
  buffer_data<unsigned> index_data;  // Index Data

  //Parametric Data
  buffer_data<point> parametric_data;  // Control Points of all the surfaces

  //Attribute Data
  buffer_data<page> attribute_data;

  //Data for Trimming
  buffer_data<scm::math::vec4f> trim_partition;
  buffer_data<scm::math::vec2f> trim_contourlist;
  buffer_data<scm::math::vec4f> trim_curvelist;
  buffer_data<float> trim_curvedata;
  buffer_data<scm::math::vec3f> trim_pointdata;

  //Object Bounding Box
  bbox object_bbox;

 private:

};

}  // namespace gua

#endif  // GUA_NURBSDATA_HPP_INCLUDED
