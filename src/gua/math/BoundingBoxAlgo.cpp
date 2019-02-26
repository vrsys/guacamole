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

// header
#include <gua/math/BoundingBoxAlgo.hpp>

namespace gua
{
namespace math
{
math::BoundingBox<math::vec3> transform(math::BoundingBox<math::vec3> const& bbox, math::mat4 const& mat)
{
    auto tmp = math::BoundingBox<math::vec3>();
    tmp.expandBy(mat * bbox.min);
    tmp.expandBy(mat * bbox.max);
    tmp.expandBy(mat * math::vec3(bbox.min.x, bbox.min.y, bbox.max.z));
    tmp.expandBy(mat * math::vec3(bbox.min.x, bbox.max.y, bbox.min.z));
    tmp.expandBy(mat * math::vec3(bbox.min.x, bbox.max.y, bbox.max.z));
    tmp.expandBy(mat * math::vec3(bbox.max.x, bbox.min.y, bbox.max.z));
    tmp.expandBy(mat * math::vec3(bbox.max.x, bbox.max.y, bbox.min.z));
    tmp.expandBy(mat * math::vec3(bbox.max.x, bbox.min.y, bbox.min.z));
    return tmp;
}

} // namespace math

} // namespace gua
