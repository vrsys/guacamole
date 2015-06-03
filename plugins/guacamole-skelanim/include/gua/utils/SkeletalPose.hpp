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

//external headers
#include <map>
#include <memory>

namespace gua {
class Bone;
class BonePose;

class SkeletalPose {
 public:
  SkeletalPose();

  ~SkeletalPose();

  bool contains(std::string const& name) const;

  BonePose const& get_transform(std::string const& name) const;

  void set_transform(std::string const& name, BonePose const& value);

  void blend(SkeletalPose const& pose2, float blendFactor);

  SkeletalPose& operator+=(SkeletalPose const& pose2);
  SkeletalPose operator+(SkeletalPose const& p2) const;

  SkeletalPose& operator*=(float const factor);
  SkeletalPose operator*(float const factor) const;

  void partial_replace(SkeletalPose const& pose2,
                       std::shared_ptr<Bone> const& pNode);

 private:
  std::map<std::string, BonePose> transforms;
};

}

#endif  //GUA_POSE_HPP
