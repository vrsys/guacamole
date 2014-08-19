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

// class header
#include <gua/renderer/nurbs_geometry/TrimDomainSerializer.hpp>

// header, system

// header, project

namespace gua {

/////////////////////////////////////////////////////////////////////////////
TrimDomainSerializer::TrimDomainSerializer() {}

/////////////////////////////////////////////////////////////////////////////
/* virtual */ TrimDomainSerializer::~TrimDomainSerializer() {}

/////////////////////////////////////////////////////////////////////////////
TrimDomainSerializer::float_type TrimDomainSerializer::unsigned_bits_as_float(
    address_type i) const {
  assert(sizeof(float_type) == sizeof(address_type));

  float_type as_float = *reinterpret_cast<float_type*>(&i);
  return as_float;
}

/////////////////////////////////////////////////////////////////////////////
TrimDomainSerializer::address_type TrimDomainSerializer::float_bits_as_unsigned(
    float_type f) const {
  assert(sizeof(float_type) == sizeof(address_type));

  address_type as_unsigned = *reinterpret_cast<address_type*>(&f);
  return as_unsigned;
}

}  // namespace gua
