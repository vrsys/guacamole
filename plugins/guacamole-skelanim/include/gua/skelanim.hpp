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

#ifndef GUA_INCLUDE_SKELANIM_HPP
#define GUA_INCLUDE_SKELANIM_HPP

#if defined (_MSC_VER)
#if defined (GUA_SKELANIM_LIBRARY)
#define GUA_SKELANIM_DLL __declspec( dllexport )
#else
#define GUA_SKELANIM_DLL __declspec( dllimport )
#endif
#else
#define GUA_SKELANIM_DLL
#endif // #if defined(_MSC_VER)

#include <gua/skelanim/node/SkeletalAnimationNode.hpp>
#include <gua/skelanim/renderer/SkeletalAnimationPass.hpp>
#include <gua/skelanim/renderer/SkeletalAnimationRenderer.hpp>
#include <gua/skelanim/renderer/SkeletalAnimationLoader.hpp>

#endif  // GUA_INCLUDE_SKELANIM_HPP
