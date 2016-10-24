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

#ifndef GUA_BONE_HPP
#define GUA_BONE_HPP

// guacamole headers
#include <gua/config.hpp>
#include <gua/utils/fbxfwd.hpp>
// #include <gua/Skelanim.hpp>
 #if defined (_MSC_VER)
#if defined (GUA_SKELANIM_LIBRARY)
#define GUA_SKELANIM_DLL __declspec( dllexport )
#else
#define GUA_SKELANIM_DLL __declspec( dllimport )
#endif
#else
#define GUA_SKELANIM_DLL
#endif // #if defined(_MSC_VER)

// external headers
#include <scm/gl_core.h>
#include <vector>
#include <map>

struct aiNode;

namespace gua {

class SkeletalPose;
class Skeleton;
/**
 * @brief represents one node in skeletal hierarchy
 * @details has methods to traverse skeleton hierarchy
 */
class GUA_SKELANIM_DLL Bone {
 public:
  Bone();
  Bone(aiNode const& node);

#ifdef GUACAMOLE_FBX
  Bone(FbxNode& node);
#endif

  std::string name;
  std::vector<unsigned> children;
  int index;

 private:

  std::string parentName;
  unsigned numChildren;
  scm::math::mat4f transformation;
  //transforms to bone space
  scm::math::mat4f offsetMatrix;

  friend class Skeleton;
};

}

#endif  //GUA_BONE_HPP
