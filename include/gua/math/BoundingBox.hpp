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

#ifndef GUA_BOUNDING_BOX_HPP
#define GUA_BOUNDING_BOX_HPP

#include <gua/platform.hpp>
#include <gua/math/traits.hpp>
#include <utility>
#include <limits>
#include <vector>

namespace gua
{
namespace math
{
/**
 * A math representation of a bounding box.
 */
template <typename V>
struct BoundingBox
{
    using scalar_type = typename traits::scalar<V>::type;
    using point_type = V;

    point_type min;
    point_type max;

    /** Create empty bounding box */
    BoundingBox() : min(std::numeric_limits<scalar_type>::max()), max(std::numeric_limits<scalar_type>::lowest()) {}

    /** Create a degenerate bounding box containing only a single point */
    explicit BoundingBox(point_type const& p) : min(p), max(p) {}

    explicit BoundingBox(point_type const& pmin, point_type const& pmax) : min(pmin), max(pmax) {}

    /** Create the smallest bounding box containing all the given points */
    explicit BoundingBox(std::vector<point_type> const& ps) : min(std::numeric_limits<scalar_type>::max()), max(std::numeric_limits<scalar_type>::lowest())
    {
        if(!ps.empty())
        {
            BoundingBox tmp(ps.front());

            for(point_type p : ps)
                tmp.expandBy(p);

            min = tmp.min;
            max = tmp.max;
        }
    }

    scalar_type size(unsigned dimension) const { return std::abs(max[dimension] - min[dimension]); }

    scalar_type surface_area() const {
        scalar_type bb_dims[3] = {0.0f, 0.0f, 0.0f};

        for(unsigned int dim_idx = 0; dim_idx < 3; ++dim_idx) {
            bb_dims[dim_idx] = max[dim_idx] - min[dim_idx];
        }

        return 2 * ((bb_dims[0] * bb_dims[1]) + 
                    (bb_dims[0] * bb_dims[2]) + 
                    (bb_dims[1] * bb_dims[2]) );
    };

    scalar_type volume() const {
        scalar_type bb_dims[3] = {0.0f, 0.0f, 0.0f};

        for(unsigned int dim_idx = 0; dim_idx < 3; ++dim_idx) {
            bb_dims[dim_idx] = max[dim_idx] - min[dim_idx];
        }

        return     ((bb_dims[0] * bb_dims[1]) *
                    (bb_dims[0] * bb_dims[2]) * 
                    (bb_dims[1] * bb_dims[2]) );
    };

    /** Lower and upper corners */
    std::pair<point_type, point_type> corners() const { return std::make_pair(min, max); }

    bool contains(point_type const& p) const
    {
        for(unsigned int i = 0; i < traits::dimension<V>::value; ++i)
            if(p[i] < min[i] || p[i] > max[i])
                return false;

        return true;
    }

    bool intersects(BoundingBox<V> const& rhs) const
    {
        for(unsigned int i = 0; i < traits::dimension<V>::value; ++i)
            if(rhs.max[i] < min[i] || rhs.min[i] > max[i])
                return false;

        return true;
    }

    void expandBy(point_type const& p)
    {
        for(unsigned int i = 0; i < traits::dimension<V>::value; ++i)
        {
            if(p[i] < min[i])
                min[i] = p[i];

            if(p[i] > max[i])
                max[i] = p[i];
        }
    }

    void expandBy(BoundingBox<V> const& rhs)
    {
        for(unsigned int i = 0; i < traits::dimension<V>::value; ++i)
        {
            if(rhs.min[i] < min[i])
                min[i] = rhs.min[i];

            if(rhs.max[i] > max[i])
                max[i] = rhs.max[i];
        }
    }

    bool isEmpty() const
    {
        for(unsigned int i = 0; i < traits::dimension<V>::value; ++i)
        {
            if(max[i] < min[i])
                return true;
        }
        return false;
    }

    point_type center() const { return (max + min) / 2; }

    bool operator==(BoundingBox<V> const& rhs) const { return rhs.min == min && rhs.max == max; }

    bool operator!=(BoundingBox<V> const& rhs) const { return rhs.min != min && rhs.max != max; }
};

/** union of two bounding boxes */
template <typename V>
BoundingBox<V> combine(BoundingBox<V> const& lhs, BoundingBox<V> const& rhs)
{
    BoundingBox<V> tmp(lhs);
    tmp.expandBy(rhs);
    return tmp;
}

/** intersection of two bounding boxes */
template <typename V>
BoundingBox<V> intersection(BoundingBox<V> const& lhs, BoundingBox<V> const& rhs)
{
    BoundingBox<V> tmp(lhs);

    for(unsigned int i = 0; i < traits::dimension<V>::value; ++i)
    {
        if(rhs.min[i] > tmp.min[i])
            tmp.min[i] = rhs.min[i];

        if(rhs.max[i] < tmp.max[i])
            tmp.max[i] = rhs.max[i];
    }

    return tmp;
}

} // namespace math

} // namespace gua

#endif // GUA_BOUNDING_BOX_HPP
