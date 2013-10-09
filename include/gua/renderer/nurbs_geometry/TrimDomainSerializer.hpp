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

#ifndef GUA_TRIMDOMAIN_SERIALIZER_HPP
#define GUA_TRIMDOMAIN_SERIALIZER_HPP

// header, system
#include <gua/renderer/nurbs_geometry/tml/halffloat.hpp>

#include <boost/unordered_map.hpp>

// header, project
#include <gua/renderer/nurbs_geometry/TrimmedBezierSurface.hpp>

namespace gua {

class TrimDomainSerializer {
 public:  // enums/typedefs

  typedef unsigned address_type;
  typedef float float_type;

  typedef TrimmedBezierSurface::curve_point_type point_type;
  typedef tml::axis_aligned_boundingbox<point_type> bbox_type;
  typedef TrimmedBezierSurface::curve_type curve_type;
  typedef boost::shared_ptr<curve_type> curve_ptr;

  typedef TrimmedBezierSurface::trimdomain_ptr trimdomain_ptr;

 public:  // c'tor/d'tor

  TrimDomainSerializer();
  virtual ~TrimDomainSerializer();

 public:  // methods

  template <typename float3_type>
  address_type serialize(
      curve_ptr const& input_curve,
      boost::unordered_map<curve_ptr, address_type>& referenced_curves,
      std::vector<float3_type>& output_container) const;

  float_type unsigned_bits_as_float(address_type i) const;

  address_type float_bits_as_unsigned(float_type f) const;

  address_type uint4ToUInt(unsigned char a,
                           unsigned char b,
                           unsigned char c,
                           unsigned char d) const {
    assert(sizeof(address_type) == 4);

    address_type result = 0U;
    result |= (d & 0x000000FF) << 24U;
    result |= (c & 0x000000FF) << 16U;
    result |= (b & 0x000000FF) << 8U;
    result |= (a & 0x000000FF);

    return result;
  }

  address_type uint8_24ToUInt(unsigned char a, unsigned int b) const {
    assert(sizeof(address_type) == 4);

    address_type result = 0U;
    result |= (b & 0x00FFFFFF) << 8U;
    result |= (a & 0x000000FF);

    return result;
  }

  void intToUint8_24(address_type input,
                     unsigned char& a,
                     unsigned int& b) const {
    b = (input & 0xFFFFFF00) >> 8U;
    a = (input & 0x000000FF);
  }

  address_type float2_to_unsigned(float a, float b) const {
    tml::halffloat_t ah = tml::floatToHalf(a);
    tml::halffloat_t bh = tml::floatToHalf(b);

    address_type result = 0U;
    result |= (bh & 0x0000FFFF) << 16U;
    result |= (ah & 0x0000FFFF);

    return result;
  }

 private:  // member

};

/////////////////////////////////////////////////////////////////////////////
template <typename float3_type>
TrimDomainSerializer::address_type TrimDomainSerializer::serialize(
    curve_ptr const& input_curve,
    boost::unordered_map<curve_ptr, address_type>& referenced_curves,
    std::vector<float3_type>& output_container) const {
  // find curve index, if already referenced
  boost::unordered_map<curve_ptr, address_type>::const_iterator curve_index =
      referenced_curves.find(input_curve);

  if (curve_index != referenced_curves.end()) {
    return curve_index->second;
  } else {
    // save current index
    address_type index = address_type(output_container.size());

      // copy curve data into buffer and transform to hyperspace
    std::transform(input_curve->begin(),
                   input_curve->end(),
                   std::back_inserter(output_container),
                   [&](point_type const & p) {
      return float3_type(p[0] * p.weight(), p[1] * p.weight(), p.weight());
    });
    // insert curve pointer and according index into map
    referenced_curves.insert(std::make_pair(input_curve, index));

    if (output_container.size() >= std::numeric_limits<address_type>::max()) {
      throw std::runtime_error("Address exceeds maximum of addressable memory");
    }

    // return index the curve was written to
    return index;
  }
}

}  // namespace gua

#endif  // GUA_TRIMDOMAIN_SERIALIZER_HPP
