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
#include <gua/renderer/nurbs_geometry/NURBSData.hpp>

// guacamole headers
#include <gua/renderer/nurbs_geometry/TrimDomainSerializer_BinaryContourMap.hpp>

// external headers
#include <algorithm>
#include <iterator>
#include <limits>

namespace gua {

NURBSData::NURBSData(std::shared_ptr<TrimmedBezierSurfaceObject> const& object,
                     unsigned _InnerTessellationLevel,
                     unsigned _OuterTessellationLevel)
    : InnerTessellationLevel(_InnerTessellationLevel),
      OuterTessellationLevel(_OuterTessellationLevel),
      patch_data(),
      parametric_data(),
      attribute_data(),
      trim_partition(),
      trim_contourlist(),
      trim_curvelist(),
      trim_curvedata(),
      trim_pointdata() {
  using namespace scm::math;

  unsigned currentIndex = 0;
  int surfaceOffset = 0;

  // serialize trim domain
  boost::unordered_map<TrimDomainSerializer::curve_ptr, unsigned int>
      referenced_curves;
  boost::unordered_map<TrimDomainSerializer::trimdomain_ptr, unsigned int>
      referenced_domains;
  boost::unordered_map<
      TrimDomainSerializer_BinaryContourMap::contour_segment_ptr,
      unsigned int> referenced_segments;
  gua::TrimDomainSerializer_BinaryContourMap serializer;

  auto uint_to_float = [](unsigned const & i) { return *((float*)(&i)); }
  ;

  //  serialize patch data
  for (auto it = object->begin(); it != object->end(); ++it, ++currentIndex) {
    auto _p0 = (*it)->points().begin();
    patch_data.push(
        point((*_p0)[0], (*_p0)[1], (*_p0)[2], uint_to_float(currentIndex)));
    patch_data.push(point(0.0f, 0.0f, 0.0f, 0.0f));

    auto _p1 = _p0 + (*it)->points().width() - 1;
    patch_data.push(
        point((*_p1)[0], (*_p1)[1], (*_p1)[2], uint_to_float(currentIndex)));
    patch_data.push(point(1.0f, 0.0f, 0.0f, 0.0f));

    auto _p2 = (*it)->points().end() - (*it)->points().width();
    patch_data.push(
        point((*_p2)[0], (*_p2)[1], (*_p2)[2], uint_to_float(currentIndex)));
    patch_data.push(point(0.0f, 1.0f, 0.0f, 0.0f));

    auto _p3 = (*it)->points().end() - 1;
    patch_data.push(
        point((*_p3)[0], (*_p3)[1], (*_p3)[2], uint_to_float(currentIndex)));
    patch_data.push(point(1.0f, 1.0f, 0.0f, 0.0f));

    auto _v01 = (_p1->as_euclidian()) - (_p0->as_euclidian());
    auto _v13 = (_p3->as_euclidian()) - (_p1->as_euclidian());
    auto _v23 = (_p3->as_euclidian()) - (_p2->as_euclidian());
    auto _v02 = (_p2->as_euclidian()) - (_p0->as_euclidian());

    scm::math::vec4f edge_dist(0.0, 0.0, 0.0, 0.0);

    index_data.push(currentIndex * 4 + 0);
    index_data.push(currentIndex * 4 + 1);
    index_data.push(currentIndex * 4 + 3);
    index_data.push(currentIndex * 4 + 2);

    std::size_t trim_id = serializer.serialize((*it)->domain(),
                                               referenced_domains,
                                               referenced_curves,
                                               referenced_segments,
                                               trim_partition.data,
                                               trim_contourlist.data,
                                               trim_curvelist.data,
                                               trim_curvedata.data,
                                               trim_pointdata.data);

    //Attributes per patch
    page p;
    p.surface_offset = surfaceOffset;
    p.order_u = (*it)->order_u();
    p.order_v = (*it)->order_v();
    p.trim_id = trim_id;

    // not originally johari: check!
    p.nurbs_domain = point((*it)->bezierdomain().min[0],
                           (*it)->bezierdomain().min[1],
                           (*it)->bezierdomain().max[0],
                           (*it)->bezierdomain().max[1]);
    p.bbox_min = point((*it)->bbox().min[0],
                       (*it)->bbox().min[1],
                       (*it)->bbox().min[2],
                       (float) _InnerTessellationLevel);
    p.bbox_max = point((*it)->bbox().max[0],
                       (*it)->bbox().max[1],
                       (*it)->bbox().max[2],
                       (float) _OuterTessellationLevel);
    p.dist = scm::math::abs(edge_dist);

    attribute_data.push(p);

    int current_size = parametric_data.size();
    parametric_data.resize(current_size + std::distance((*it)->points().begin(),
                                                        (*it)->points().end()));

    int point_index = 0;

    for (auto it1 = (*it)->points().begin(); it1 != (*it)->points().end();
         ++it1, ++point_index) {
      parametric_data[current_size + point_index] =
          point(it1->as_homogenous()[0],
                it1->as_homogenous()[1],
                it1->as_homogenous()[2],
                it1->as_homogenous()[3]);
    }

    surfaceOffset = parametric_data.size();
  }
}

NURBSData::~NURBSData() {}

}  //namespace scm
