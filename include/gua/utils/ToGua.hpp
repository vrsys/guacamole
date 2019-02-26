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

#ifndef GUA_TOGUA_HPP
#define GUA_TOGUA_HPP

// guacamole headers
#include <gua/config.hpp>
#include <gua/platform.hpp>
#include <gua/utils/fbxfwd.hpp>

// external headers
#include <scm/gl_core.h>
#include <scm/core/math/quat.h>
#include <assimp/scene.h> // for ainodeanim

namespace to_gua
{
GUA_DLL scm::math::mat4f mat4f(aiMatrix4x4 const& m);
GUA_DLL scm::math::quatf quatf(aiQuaternion const& q);

template <typename T>
scm::math::vec3f vec3f(T const& v)
{
    scm::math::vec3f res(v[0], v[1], v[2]);
    return res;
}

template <typename T>
scm::math::vec2f vec2f(T const& v)
{
    scm::math::vec2f res(v[0], v[1]);
    return res;
}

template <typename T>
scm::math::vec4f vec4f(T const& v)
{
    scm::math::vec4 res(v[0], v[1], v[2], v[3]);
    return res;
}

#ifdef GUACAMOLE_FBX
GUA_DLL scm::math::mat4f mat4f(FbxAMatrix const& m);
GUA_DLL scm::math::mat4d mat4d(FbxAMatrix const& m);
GUA_DLL scm::math::quatf quatf(FbxQuaternion const& q);
#endif
} // namespace to_gua

#endif // GUA_TOGUA_HPP