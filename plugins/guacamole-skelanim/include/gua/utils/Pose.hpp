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

#ifndef GUA_POSE_HPP
#define GUA_POSE_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/utils/Mesh.hpp>
#include <gua/utils/Transformation.hpp>

#include <vector>
#include <map>

namespace gua {
class Bone;

class Pose {
 public:
  Pose();

  ~Pose();

  bool contains(std::string const& name ) const;

  Transformation const& get_transform(std::string const& name) const;

  void set_transform(std::string const& name, Transformation const& value);

  void blend(Pose const& pose2, float blendFactor);

  Pose& operator+=(Pose const& pose2);
  Pose operator+(Pose const& p2) const;

  Pose& operator*=(float const factor);
  Pose operator*(float const factor) const;

  void partial_replace(Pose const& pose2, std::shared_ptr<Bone> const& pNode);

 private: 
  std::map<std::string, Transformation> transforms;
};

}

#endif //GUA_POSE_HPP
