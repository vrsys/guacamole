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

struct Vec3Key {

  Vec3Key(aiVectorKey key):
    time{key.mTime},
    value{ai_to_gua(key.mValue)}
  {}

  ~Vec3Key(){};

  double time;
  scm::math::vec3 value;
};

struct QuatKey {

  QuatKey(aiQuatKey key):
    time{key.mTime},
    value{ai_to_gua(key.mValue)}
  {}

  ~QuatKey(){};

  double time;
  scm::math::quatf value;
};

struct BoneAnimation {


  BoneAnimation():
    name{"default"},
    numScalingKeys{0},
    numRotationKeys{0},
    numTranslationKeys{0},
    scalingKeys{},
    rotationKeys{},
    translationKeys{}
  {}

  BoneAnimation(aiNodeAnim* anim):
    name{anim->mNodeName.C_Str()}
   {

    for(unsigned i = 0; i < anim->mNumScalingKeys; ++i) {
      scalingKeys.push_back(Vec3Key{anim->mScalingKeys[i]});
    }
    for(unsigned i = 0; i < anim->mNumRotationKeys; ++i) {
      rotationKeys.push_back(QuatKey{anim->mRotationKeys[i]});
    }
    for(unsigned i = 0; i < anim->mNumPositionKeys; ++i) {
      translationKeys.push_back(Vec3Key{anim->mPositionKeys[i]});
    }

    numScalingKeys = scalingKeys.size();
    numRotationKeys = rotationKeys.size();
    numTranslationKeys = translationKeys.size();
  }

  ~BoneAnimation(){};

  std::string name;
  unsigned numScalingKeys;
  unsigned numRotationKeys;
  unsigned numTranslationKeys;

  std::vector<Vec3Key> scalingKeys;
  std::vector<QuatKey> rotationKeys;
  std::vector<Vec3Key> translationKeys;
};

struct SkeletalAnimation {

  SkeletalAnimation():
    name{"default"},
    numFrames{0},
    numFPS{0},
    duration{0},
    numBoneAnims{0},
    boneAnims{}
  {}

  SkeletalAnimation(aiAnimation* anim):
    name{anim->mName.C_Str()},
    numFrames{unsigned(anim->mDuration)},
    numFPS{anim->mTicksPerSecond > 0 ? anim->mTicksPerSecond : 25},
    duration{double(numFrames) / numFPS},
    numBoneAnims{anim->mNumChannels},
    boneAnims{}
  {
    for(unsigned i = 0; i < numBoneAnims; ++i) {
      boneAnims.push_back(BoneAnimation{anim->mChannels[i]});
    }
  }

  ~SkeletalAnimation(){};

  std::string name;
  unsigned numFrames;
  double numFPS;
  double duration;
  unsigned numBoneAnims;
  std::vector<BoneAnimation> boneAnims;
};

struct Node {
  Node(aiNode const* node):
    index{-1},
    name{node->mName.C_Str()},
    parentName{node->mParent != NULL ? node->mParent->mName.C_Str() : "none"},
    numChildren{node->mNumChildren},
    transformation{ai_to_gua(node->mTransformation)},
    offsetMatrix{scm::math::mat4f::identity()}
  {
    for(unsigned i = 0; i < node->mNumChildren; ++i) {
      std::shared_ptr<Node> child = std::make_shared<Node>(node->mChildren[i]);
      children.push_back(child);
    }
  }

  ~Node(){};

  std::string name;
  std::string parentName;
  unsigned numChildren;
  std::vector<std::shared_ptr<Node>> children;
  scm::math::mat4f transformation;
  //transforms to bone space
  scm::math::mat4f offsetMatrix;
  int index;
};

struct Transformation {
  Transformation():
    scaling{1.0f},
    rotation{scm::math::quatf::identity()},
    translation{0.0f}
  {}

  Transformation(scm::math::vec3 const& scale, scm::math::quatf const& rotate, scm::math::vec3 const& translate):
    scaling{scale},
    rotation{rotate},
    translation{translate}
  {}

  scm::math::mat4f to_matrix() const {
    return scm::math::make_translation(translation) * rotation.to_matrix() * scm::math::make_scale(scaling);
  }

  Transformation blend(Transformation const& t, float const factor   ) const {
    return Transformation{scaling * (1 - factor) + t.scaling * factor, slerp(rotation, t.rotation, factor), translation * (1 - factor) + t.translation * factor};
  }

  ~Transformation(){};

  scm::math::vec3 scaling;
  scm::math::quatf rotation;
  scm::math::vec3 translation;
};

class SkeletalAnimationUtils {
 public:

  static std::vector<std::shared_ptr<SkeletalAnimation>> load_animations(aiScene const*);
  static std::shared_ptr<Node> load_hierarchy(aiScene const* scene);

  static void set_bone_properties(std::map<std::string, std::pair<uint, scm::math::mat4f>> const& info, std::shared_ptr<Node>& currNode);
  static void collect_bone_indices(std::map<std::string, int>& ids, std::shared_ptr<Node> const& pNode);
  
  static void calculate_pose(float TimeInSeconds, std::shared_ptr<Node> const& root, std::shared_ptr<SkeletalAnimation> const& pAnim, std::vector<scm::math::mat4f>& Transforms);

  static void accumulate_transforms(std::vector<scm::math::mat4f>& transformMat4s, std::shared_ptr<Node> const& pNode, std::map<std::string, Transformation> const& transforms, scm::math::mat4f const& ParentTransform);
  static std::map<std::string, Transformation> calculate_transforms(float animationTime, std::shared_ptr<SkeletalAnimation> const& pAnim);

  static void blend(std::map<std::string, Transformation>& transforms1, std::map<std::string, Transformation> const& transforms2, float blendFactor);
  static void partial_blend(std::map<std::string, Transformation>& transforms1, std::map<std::string, Transformation> const& transforms2, std::shared_ptr<Node> const& start);
  static std::shared_ptr<Node> find_node(std::string const& name, std::shared_ptr<Node> const& root);
 private:

  static scm::math::vec3 interpolate_scaling(float AnimationTime, BoneAnimation const& nodeAnim);
  static scm::math::quatf interpolate_rotation(float AnimationTime, BoneAnimation const& nodeAnim);
  static scm::math::vec3 interpolate_position(float AnimationTime, BoneAnimation const& nodeAnim);    
  
  static uint find_scaling(float AnimationTime, BoneAnimation const& nodeAnim);
  static uint find_rotation(float AnimationTime, BoneAnimation const& nodeAnim);
  static uint find_position(float AnimationTime, BoneAnimation const& nodeAnim);
  
  static BoneAnimation const* find_node_anim(std::shared_ptr<SkeletalAnimation> const& pAnimation, std::string const& nodeName);  

  inline SkeletalAnimationUtils(){};
  inline ~SkeletalAnimationUtils(){};
};

}

#endif //GUA_SKELETAL_ANIMATION_UTILS_HPP
