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

#ifndef GUA_PHYSICS_UTILS_HPP
#define GUA_PHYSICS_UTILS_HPP

// guacamole headers
#include <gua/math/math.hpp>

// external headers
#include <LinearMath/btVector3.h>
#include <LinearMath/btTransform.h>

namespace gua
{
namespace math
{
vec3 const btVector3_to_vec3(const btVector3& vec);

btVector3 const vec3_to_btVector3(const vec3& vec);

btTransform const mat4_to_btTransform(const mat4& m);

mat4 const btTransform_to_mat4(const btTransform& t);

btQuaternion const extract_quaternion(const mat4f& m);
btQuaternion const extract_quaternion(const mat4d& m);
} // namespace math

inline math::vec3 const math::btVector3_to_vec3(const btVector3& vec) { return math::vec3(vec.x(), vec.y(), vec.z()); }

inline btVector3 const math::vec3_to_btVector3(const math::vec3& vec) { return btVector3(vec.x, vec.y, vec.z); }

inline btTransform const math::mat4_to_btTransform(const math::mat4& m)
{
    return btTransform(btMatrix3x3(m.m00, m.m04, m.m08, m.m01, m.m05, m.m09, m.m02, m.m06, m.m10), btVector3(m.m12, m.m13, m.m14));
}

inline math::mat4 const math::btTransform_to_mat4(const btTransform& t)
{
    math::mat4 m;
    btMatrix3x3 b(t.getBasis());
    m.m00 = b[0].x();
    m.m01 = b[1].x();
    m.m02 = b[2].x();
    m.m03 = 0.f;
    m.m04 = b[0].y();
    m.m05 = b[1].y();
    m.m06 = b[2].y();
    m.m07 = 0.f;
    m.m08 = b[0].z();
    m.m09 = b[1].z();
    m.m10 = b[2].z();
    m.m11 = 0.f;
    m.m12 = t.getOrigin().x();
    m.m13 = t.getOrigin().y();
    m.m14 = t.getOrigin().z();
    m.m15 = 1.f;
    return m;
}

inline btQuaternion const math::extract_quaternion(const math::mat4d& m)
{
    math::mat4f tmp(m);
    return math::extract_quaternion(tmp);
}
// Based on OpenEXR's quaternion extraction implementation
// https://github.com/openexr/openexr/blob/master/IlmBase/Imath/ImathMatrixAlgo.h
// Copyright (c) 2002-2012, Industrial Light & Magic, a division of Lucas
// Digital Ltd. LLC
inline btQuaternion const math::extract_quaternion(const math::mat4f& m)
{
    float tr, s;
    float q[4];
    int i, j, k;
    float mat[4][4] = {{static_cast<float>(m.m00), static_cast<float>(m.m01), static_cast<float>(m.m02), static_cast<float>(m.m03)},
                       {static_cast<float>(m.m04), static_cast<float>(m.m05), static_cast<float>(m.m06), static_cast<float>(m.m07)},
                       {static_cast<float>(m.m08), static_cast<float>(m.m09), static_cast<float>(m.m10), static_cast<float>(m.m11)},
                       {static_cast<float>(m.m12), static_cast<float>(m.m13), static_cast<float>(m.m14), static_cast<float>(m.m15)}};

    int nxt[3] = {1, 2, 0};
    tr = mat[0][0] + mat[1][1] + mat[2][2];

    // check the diagonal
    if(tr > 0.0)
    {
        s = sqrt(tr + 1.0f);
        q[3] = s / 2.0f;
        s = 0.5f / s;

        q[0] = (mat[1][2] - mat[2][1]) * s;
        q[1] = (mat[2][0] - mat[0][2]) * s;
        q[2] = (mat[0][1] - mat[1][0]) * s;
    }
    else
    {
        // diagonal is negative
        i = 0;
        if(mat[1][1] > mat[0][0])
            i = 1;
        if(mat[2][2] > mat[i][i])
            i = 2;

        j = nxt[i];
        k = nxt[j];
        s = sqrt((mat[i][i] - (mat[j][j] + mat[k][k])) + 1.0f);

        q[i] = s * 0.5f;
        if(s != 0.0f)
            s = 0.5f / s;

        q[3] = (mat[j][k] - mat[k][j]) * s;
        q[j] = (mat[i][j] + mat[j][i]) * s;
        q[k] = (mat[i][k] + mat[k][i]) * s;
    }
    return btQuaternion(btVector3(q[0], q[1], q[2]), q[3]);
}

} // namespace gua

#endif // GUA_PHYSICS_UTILS_HPP
