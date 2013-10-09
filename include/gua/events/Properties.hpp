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

#ifndef PROPERTIES_HPP_
#define PROPERTIES_HPP_

#include <gua/events/properties/NumericProperty.hpp>
#include <gua/events/properties/LogicalProperty.hpp>

namespace gua {
namespace events {

typedef NumericProperty<double> Double;
typedef NumericProperty<float> Float;
typedef NumericProperty<int> Int;
typedef NumericProperty<char> Char;
typedef NumericProperty<unsigned> UInt;
typedef NumericProperty<unsigned char> UChar;

typedef LogicalProperty<bool> Bool;

}
}

#endif /* PROPERTIES_HPP_ */
