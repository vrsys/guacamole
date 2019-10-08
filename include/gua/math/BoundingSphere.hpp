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

#ifndef GUA_BOUNDING_SPHERE_HPP
#define GUA_BOUNDING_SPHERE_HPP

#include <gua/platform.hpp>
#include <gua/math/traits.hpp>
#include <utility>
#include <cmath>
#include <limits>
#include <vector>

namespace gua
{
namespace math
{
/**
 * A math representation of a bounding sphere.
 */
template <typename V>
struct BoundingSphere
{
    using scalar_type = typename traits::scalar<V>::type;
    using point_type = V;

    point_type center;
    scalar_type radius;

    /** Create empty bounding sphere */
    BoundingSphere() : center(0), radius(0) {}

    /** Create a degenerate bounding sphere containing only a single point */
    explicit BoundingSphere(point_type const& p) : center(p), radius(0) {}

    explicit BoundingSphere(point_type const& c, scalar_type r) : center(c), radius(r) {}

#if 0
  /** Create the smallest bounding sphere containing all the given points */
  explicit BoundingSphere(std::vector<point_type> const& ps)
    // implement me
  }
#endif

    bool contains(point_type const& p) const
    {
        scalar_type sqDistance(0);
        for(unsigned int i = 0; i < traits::dimension<V>::value; ++i)
            sqDistance += (p[i] - center[i]) * (p[i] - center[i]);

        return sqDistance <= radius * radius;
    }

    bool intersects(BoundingSphere<V> const& rhs) const
    {
        scalar_type sqDistance(0);
        for(unsigned int i = 0; i < traits::dimension<V>::value; ++i)
            sqDistance += (rhs.center[i] - center[i]) * (rhs.center[i] - center[i]);

        return sqDistance <= (radius + rhs.radius) * (radius + rhs.radius);
    }

#if 0
  void expandBy(point_type const& p) {
    // implement me
  }
#endif

    void expandBy(BoundingSphere<V> const& rhs)
    {
        auto ab = rhs.center - center;

        scalar_type sqDistance(0);
        for(unsigned int i = 0; i < traits::dimension<V>::value; ++i)
            sqDistance += ab[i] * ab[i];

        auto dist = std::sqrt(sqDistance);

        radius = 0.5 * (radius + dist + rhs.radius);
        center = center + 0.5 * ab;
    }

    inline bool isEmpty() const { return point_type(0) == center; }

    bool operator==(BoundingSphere<V> const& rhs) const { return rhs.center == center && rhs.radius == radius; }

    bool operator!=(BoundingSphere<V> const& rhs) const { return !(*this == rhs); }
};

/** union of two bounding spheres */
template <typename V>
BoundingSphere<V> combine(BoundingSphere<V> const& lhs, BoundingSphere<V> const& rhs)
{
    BoundingSphere<V> tmp(lhs);
    tmp.expandBy(rhs);
    return tmp;
}

} // namespace math

} // namespace gua

#endif // GUA_BOUNDING_SPHERE_HPP
