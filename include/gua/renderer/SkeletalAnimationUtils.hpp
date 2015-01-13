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

#ifndef GUA_SKELETAL_ANIMATION_UTILS_HPP
#define GUA_SKELETAL_ANIMATION_UTILS_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/BoneTransformUniformBlock.hpp>
#include <gua/utils/Logger.hpp>

// external headers
#include <scm/gl_core.h>
#include <scm/core/math/quat.h>

#include <vector>
#include <map>
#include <assimp/scene.h>       // Output data structure

namespace {

scm::math::mat4f ai_to_gua(aiMatrix4x4 const& m) {
  scm::math::mat4f res(m.a1,m.b1,m.c1,m.d1
                      ,m.a2,m.b2,m.c2,m.d2
                      ,m.a3,m.b3,m.c3,m.d3
                      ,m.a4,m.b4,m.c4,m.d4);
  return res;
}

scm::math::vec3 ai_to_gua(aiVector3D const& v) {
  scm::math::vec3 res(v.x, v.y, v.z);
  return res;
}

scm::math::quatf ai_to_gua(aiQuaternion const& q) {
  scm::math::quatf res(q.w, q.x, q.y, q.z);
  return res;
}

}

namespace gua {
class Pose;

struct Transformation {
 public:
  Transformation();

  Transformation(scm::math::vec3 const& scale, scm::math::quatf const& rotate, scm::math::vec3 const& translate);

  ~Transformation();

  scm::math::mat4f to_matrix() const;

  Transformation blend(Transformation const& t, float const factor) const;

  Transformation operator+(Transformation const& t) const;
  Transformation& operator+=(Transformation const& t);

  Transformation operator*(float const factor) const;
  Transformation& operator*=(float const f);

 private:
  scm::math::vec3 scaling;
  scm::math::quatf rotation;
  scm::math::vec3 translation;
};

template<class T>
struct Key {

  Key(double time, T const& value):
    time{time},
    value{value}
  {}

  ~Key(){};

  double time;
  T value;
};


class Node {
 public:
  Node(aiNode const* node);

  ~Node();

  void collect_indices(std::map<std::string, int>& ids) const;

  void set_properties(std::map<std::string, std::pair<uint, scm::math::mat4f>> const& infos);

  std::shared_ptr<Node> find(std::string const& name) const;

  void accumulate_matrices(std::vector<scm::math::mat4f>& transformMat4s, Pose const& pose, scm::math::mat4f const& parentTransform) const;

  std::string name;
  std::vector<std::shared_ptr<Node>> children;
  
 private:
  int index;
  std::string parentName;
  unsigned numChildren;
  scm::math::mat4f transformation;
  //transforms to bone space
  scm::math::mat4f offsetMatrix;
};

class BoneAnimation {
 public:
  BoneAnimation();

  ~BoneAnimation();

  BoneAnimation(aiNodeAnim* anim);

  Transformation calculate_transform(float time) const;

  std::string const& get_name() const;

 private:

  scm::math::vec3 interpolate(scm::math::vec3 val1, scm::math::vec3 val2, float factor) const;

  scm::math::quatf interpolate(scm::math::quatf val1, scm::math::quatf val2, float factor) const;

  template<class T> 
  uint find_key(float animationTime, std::vector<Key<T>> keys) const;

  template<class T> 
  T calculate_value(float time, std::vector<Key<T>> keys) const;

  std::string name;

  std::vector<Key<scm::math::vec3>> scalingKeys;
  std::vector<Key<scm::math::quatf>> rotationKeys;
  std::vector<Key<scm::math::vec3>> translationKeys;
};

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

  void partial_replace(Pose const& pose2, std::shared_ptr<Node> const& pNode);

 private: 
  std::map<std::string, Transformation> transforms;
};

class SkeletalAnimation {
 public:
  SkeletalAnimation();

  SkeletalAnimation(aiAnimation* anim);

  ~SkeletalAnimation();

  Pose calculate_pose(float time) const;

  double get_duration() const;

 private:
  std::string name;
  unsigned numFrames;
  double numFPS;
  double duration;
  unsigned numBoneAnims;

  std::vector<BoneAnimation> boneAnims;
};

namespace blend {
  float cos(float x); 
  float swap(float x); 
  float linear(float x);
  float smoothstep(float x);
};

class SkeletalAnimationUtils {
 public:

  static std::vector<std::shared_ptr<SkeletalAnimation>> load_animations(aiScene const*);
  static std::shared_ptr<Node> load_hierarchy(aiScene const* scene);

  static void calculate_matrices(float TimeInSeconds, std::shared_ptr<Node> const& root, std::shared_ptr<SkeletalAnimation> const& pAnim, std::vector<scm::math::mat4f>& Transforms);

 private:
  SkeletalAnimationUtils() = delete;
};

}

#endif //GUA_SKELETAL_ANIMATION_UTILS_HPP
