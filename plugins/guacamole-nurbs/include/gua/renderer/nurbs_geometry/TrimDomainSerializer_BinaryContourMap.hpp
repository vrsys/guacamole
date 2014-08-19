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

#ifndef GUA_TRIMDOMAIN_SERIALIZER_CONTOUR_MAP_BINARY_HPP
#define GUA_TRIMDOMAIN_SERIALIZER_CONTOUR_MAP_BINARY_HPP

// header, system
#include <boost/shared_ptr.hpp>
#include <boost/unordered_map.hpp>

#include <gua/renderer/nurbs_geometry/tml/parametric/domain/contour_map_binary.hpp>

#include <gua/renderer/nurbs_geometry/TrimDomain.hpp>
#include <gua/renderer/nurbs_geometry/TrimDomainSerializer.hpp>

//#define NO_EXTRA_CURVEINFO_BUFFER

namespace gua {

class TrimDomainSerializer_BinaryContourMap : public TrimDomainSerializer {
 public:  // enums/typedefs

  typedef TrimDomain::value_type value_type;
  typedef tml::contour_map_binary<value_type>::contour_segment_ptr
      contour_segment_ptr;

 public:  // c'tor/d'tor

  TrimDomainSerializer_BinaryContourMap();
  ~TrimDomainSerializer_BinaryContourMap();

 public:  // methods

  template <typename float4_type, typename float3_type, typename float2_type>
  address_type serialize(
      trimdomain_ptr const& input_domain,
      boost::unordered_map<trimdomain_ptr, address_type>&
          referenced_trimdomains,
      boost::unordered_map<curve_ptr, address_type>& referenced_curves,
      boost::unordered_map<contour_segment_ptr, address_type>&
          referenced_contour_segments,
      std::vector<float4_type>& output_partition,
      std::vector<float2_type>& output_contourlist,
      std::vector<float4_type>& output_curvelist,
      std::vector<float>& output_curvedata,
      std::vector<float3_type>& output_pointdata) const;

  template <typename float4_type, typename float3_type>
  address_type serialize_contour_segment(
      contour_segment_ptr const& contour_segment,
      boost::unordered_map<contour_segment_ptr, address_type>&
          referenced_contour_segments,
      boost::unordered_map<curve_ptr, address_type>& referenced_curves,
      std::vector<float4_type>& output_curvelist,
      std::vector<float>& output_curvedata,
      std::vector<float3_type>& output_pointdata) const;

 private:  // member

};

/////////////////////////////////////////////////////////////////////////////
template <typename float4_type, typename float3_type, typename float2_type>
TrimDomainSerializer::address_type
TrimDomainSerializer_BinaryContourMap::serialize(
    trimdomain_ptr const& input_domain,
    boost::unordered_map<trimdomain_ptr, address_type>& referenced_TrimDomains,
    boost::unordered_map<curve_ptr, address_type>& referenced_curves,
    boost::unordered_map<contour_segment_ptr, address_type>&
        referenced_contour_segments,
    std::vector<float4_type>& output_partition,
    std::vector<float2_type>& output_contourlist,
    std::vector<float4_type>& output_curvelist,
    std::vector<float>& output_curvedata,
    std::vector<float3_type>& output_pointdata) const {
  typedef tml::contour_map_binary<
      TrimmedBezierSurface::curve_point_type::value_type> contour_map_type;
  assert(output_partition.size() < std::numeric_limits<address_type>::max());

  // if already in buffer -> return index
  if (referenced_TrimDomains.count(input_domain)) {
    return referenced_TrimDomains.find(input_domain)->second;
  }

  address_type partition_index = output_partition.size();
  contour_map_type map;

  // fill loops into contour map
  std::for_each(input_domain->loops().begin(),
                input_domain->loops().end(),
                boost::bind(&contour_map_type::add, &map, _1));
  map.initialize();

  float_type umin = map.bounds().min[point_type::u];
  float_type umax = map.bounds().max[point_type::u];
  float_type vmin = map.bounds().min[point_type::v];
  float_type vmax = map.bounds().max[point_type::v];

  assert(map.partition().size() < std::numeric_limits<address_type>::max());
  address_type vintervals = map.partition().size();

  output_partition.resize(partition_index + 2 + map.partition().size());
  output_partition[partition_index] =
      float4_type(unsigned_bits_as_float(vintervals), 0, 0, 0);
  output_partition[partition_index + 1] = float4_type(umin, umax, vmin, vmax);
  std::size_t vindex = partition_index + 2;

  BOOST_FOREACH(auto const & vinterval, map.partition()) {
    assert(vinterval.cells.size() < std::numeric_limits<address_type>::max());

    address_type uid = output_partition.size();
    address_type ucells = vinterval.cells.size();

    output_partition[vindex++] = float4_type(vinterval.interval_v.minimum(),
                                             vinterval.interval_v.maximum(),
                                             unsigned_bits_as_float(uid),
                                             unsigned_bits_as_float(ucells));
    output_partition.push_back(float4_type(
        vinterval.interval_u.minimum(), vinterval.interval_u.maximum(), 0, 0));

    BOOST_FOREACH(auto const & cell, vinterval.cells) {
      address_type contourlist_id = output_contourlist.size();
      address_type type_and_contours =
          uint4ToUInt(0, cell.inside, cell.overlapping_segments.size(), 0);

      output_partition.push_back(
          float4_type(cell.interval_u.minimum(),
                      cell.interval_u.maximum(),
                      unsigned_bits_as_float(type_and_contours),
                      unsigned_bits_as_float(contourlist_id)));

      BOOST_FOREACH(auto const & contour_segment, cell.overlapping_segments) {
        address_type ncurves_uincreasing =
            uint4ToUInt(contour_segment->size(),
                        contour_segment->increasing(point_type::u),
                        0,
                        0);

        address_type curvelist_id =
            serialize_contour_segment(contour_segment,
                                      referenced_contour_segments,
                                      referenced_curves,
                                      output_curvelist,
                                      output_curvedata,
                                      output_pointdata);

        output_contourlist.push_back(
            float2_type(unsigned_bits_as_float(ncurves_uincreasing),
                        unsigned_bits_as_float(curvelist_id)));
      }
    }
  }

  // store domain_ptr to index mapping for later reference
  referenced_TrimDomains.insert(std::make_pair(input_domain, partition_index));

  // make sure buffers are still in range of address_type
  if (output_partition.size() >= std::numeric_limits<address_type>::max()) {
    throw std::runtime_error("Address exceeds maximum of addressable memory");
  }

  return partition_index;
}

/////////////////////////////////////////////////////////////////////////////
template <typename float4_type, typename float3_type>
TrimDomainSerializer::address_type
TrimDomainSerializer_BinaryContourMap::serialize_contour_segment(
    contour_segment_ptr const& contour_segment,
    boost::unordered_map<contour_segment_ptr, address_type>&
        referenced_contour_segments,
    boost::unordered_map<curve_ptr, address_type>& referenced_curves,
    std::vector<float4_type>& output_curvelist,
    std::vector<float>& output_curvedata,
    std::vector<float3_type>& output_pointdata) const {
  if (referenced_contour_segments.count(contour_segment)) {
    return referenced_contour_segments[contour_segment];
  }

  address_type contour_segment_index = output_curvelist.size();

  for (auto i_cptr = contour_segment->begin(); i_cptr != contour_segment->end();
       ++i_cptr) {
    curve_ptr curve = *i_cptr;

    address_type curveid = 0;
    if (!curve->is_constant(point_type::u) &&
        !curve->is_constant(
            point_type::v))  // linear curves do not have to be tested
        {
      curveid = TrimDomainSerializer::serialize(
          curve, referenced_curves, output_pointdata);
    }

    bbox_type bbox;
    curve->bbox_simple(bbox);

#ifdef NO_EXTRA_CURVEINFO_BUFFER
    output_curvelist.push_back(float4_type(
        bbox.min[point_type::v],
        bbox.max[point_type::v],
        unsigned_bits_as_float(float2_to_unsigned(bbox.min[point_type::u],
                                                  bbox.max[point_type::u])),
        unsigned_bits_as_float(uint8_24ToUInt(curve->order(), curveid))));
#else
    output_curvelist.push_back(float4_type(bbox.min[point_type::v],
                                           bbox.max[point_type::v],
                                           bbox.min[point_type::u],
                                           bbox.max[point_type::u]));

    output_curvedata.push_back(float_type(
        unsigned_bits_as_float(uint8_24ToUInt(curve->order(), curveid))));
#endif
  }

  return contour_segment_index;
}

}  // namespace gua

#endif  // GUA_TRIMDOMAIN_SERIALIZER_CONTOUR_MAP_BINARY_HPP
