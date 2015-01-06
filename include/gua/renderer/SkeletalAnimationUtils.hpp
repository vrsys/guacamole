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

  Transformation blend(Transformation const& t, float const factor) const {
    return Transformation{scaling * (1 - factor) + t.scaling * factor, slerp(rotation, t.rotation, factor), translation * (1 - factor) + t.translation * factor};
  }

  Transformation operator+(Transformation const& t) const {
    return Transformation{scaling + t.scaling, scm::math::normalize(t.rotation * rotation), translation + t.translation};
  }
  Transformation& operator+=(Transformation const& t) {
    *this = *this + t;
    return *this;
  }

  Transformation operator*(float const factor) const {
    return Transformation{scaling * factor, slerp(scm::math::quatf::identity(), rotation, factor), translation * factor};
  }
  Transformation& operator*=(float const f) {
    *this = *this * f;
    return *this;
  }

  ~Transformation(){};

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

class BoneAnimation {
 public:
  BoneAnimation():
    name{"default"},
    scalingKeys{},
    rotationKeys{},
    translationKeys{}
  {}

  ~BoneAnimation(){};

  BoneAnimation(aiNodeAnim* anim):
    name{anim->mNodeName.C_Str()}
   {

    for(unsigned i = 0; i < anim->mNumScalingKeys; ++i) {
      scalingKeys.push_back(Key<scm::math::vec3>{anim->mScalingKeys[i].mTime, ai_to_gua(anim->mScalingKeys[i].mValue)});
    }
    for(unsigned i = 0; i < anim->mNumRotationKeys; ++i) {
      rotationKeys.push_back(Key<scm::math::quatf>{anim->mRotationKeys[i].mTime, ai_to_gua(anim->mRotationKeys[i].mValue)});
    }
    for(unsigned i = 0; i < anim->mNumPositionKeys; ++i) {
      translationKeys.push_back(Key<scm::math::vec3>{anim->mPositionKeys[i].mTime, ai_to_gua(anim->mPositionKeys[i].mValue)});
    }
  }

  Transformation calculate_transform(float time) const {
    return Transformation{calculate_value(time, scalingKeys), calculate_value(time, rotationKeys), calculate_value(time, translationKeys)};
  }

  std::string const& get_name() const {
    return name;
  }

 private:

  scm::math::vec3 interpolate(scm::math::vec3 val1, scm::math::vec3 val2, float factor) const {
    return val1 * (1 - factor) + val2 * factor;
  }

  scm::math::quatf interpolate(scm::math::quatf val1, scm::math::quatf val2, float factor) const {
    return normalize(slerp(val1, val2, factor));
  }

  template<class T> 
  uint find_key(float animationTime, std::vector<T> keys) const {    
    if(keys.size() < 1) {
      Logger::LOG_ERROR << "no keys" << std::endl;
      assert(false);
    } 

    for(uint i = 0 ; i < keys.size() - 1 ; i++) {
      if(animationTime < (float)keys[i + 1].time) {
        return i;
      }
    }

    Logger::LOG_ERROR << "no key found" << std::endl;
    assert(false);

    return 0;
  }

  template<class T> 
  T calculate_value(float time, std::vector<Key<T>> keys) const {
    if(keys.size() == 1) {
       return keys[0].value;
    }

    uint lastIndex = find_key(time, keys);
    uint nextIndex = (lastIndex + 1);

    if(nextIndex > keys.size()) {
      Logger::LOG_ERROR << "frame out of range" << std::endl;
      assert(false);
    }

    float deltaTime = (float)(keys[nextIndex].time - keys[lastIndex].time);
    float factor = (time - (float)keys[lastIndex].time) / deltaTime;
    //assert(factor >= 0.0f && factor <= 1.0f);
    T const& key1 = keys[lastIndex].value;
    T const& key2 = keys[nextIndex].value;

    return interpolate(key1, key2, factor);
  }

  std::string name;

  std::vector<Key<scm::math::vec3>> scalingKeys;
  std::vector<Key<scm::math::quatf>> rotationKeys;
  std::vector<Key<scm::math::vec3>> translationKeys;
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


class Pose {
 public:
  Pose():
    transforms{}
  {}

  ~Pose(){};

  bool contains(std::string const& name ) const {
    return transforms.find(name) != transforms.end();
  }

  Transformation const& get_transform(std::string const& name) const{
    try {
      return transforms.at(name);
    }
    catch(std::exception const& e) {
      Logger::LOG_ERROR << "bone '" << name << "' not contained in pose" << std::endl;
      return transforms.begin()->second;
    }
  }

  void set_transform(std::string const& name, Transformation const& value) {
    transforms[name] = value;
  }

  void blend(Pose const& pose2, float blendFactor) {
    for_each(pose2.cbegin(), pose2.cend(), [this, &blendFactor](std::pair<std::string, Transformation> const& p) {
      if(contains(p.first)) {
        set_transform(p.first, get_transform(p.first).blend(p.second, blendFactor));
      }
      else {
        set_transform(p.first, p.second);
      }
    });
    // *this = *this * (1 - blendFactor) + pose2 * blendFactor;
  }

  Pose& operator+=(Pose const& pose2) {
    for_each(pose2.cbegin(), pose2.cend(), [this](std::pair<std::string, Transformation> const& p) {
      if(contains(p.first)) {
        set_transform(p.first, get_transform(p.first) + p.second);
      }
      else {
        set_transform(p.first, p.second);
      }
    });
    return *this;
  }
  Pose operator+(Pose const& p2) const {
    Pose temp{*this};
    temp += p2;
    return temp;
  }

  Pose& operator*=(float const factor) {
    for(auto& p : transforms)
    {
      p.second *=factor;
    }
    return *this;
  }
  Pose operator*(float const factor) const {
    Pose temp{*this};
    temp *= factor;
    return temp;
  }


  void partial_replace(Pose const& pose2, std::shared_ptr<Node> const& pNode) {
    if(pose2.contains(pNode->name)) {
      set_transform(pNode->name, pose2.get_transform(pNode->name));
    }

    for(std::shared_ptr<Node>& child : pNode->children) {
      partial_replace(pose2, child);
    }
  }

  inline std::map<std::string, Transformation>::iterator begin() {
    return transforms.begin();
  }   

  inline std::map<std::string, Transformation>::const_iterator cbegin() const {
    return transforms.cbegin();
  } 

  inline std::map<std::string, Transformation>::iterator end() {
    return transforms.end();
  } 
  inline std::map<std::string, Transformation>::const_iterator cend() const {
    return transforms.cend();
  } 
 
 private:
  std::map<std::string, Transformation> transforms;
};

class Blend {
 public:
  static float cos(float x); 
  static float swap(float x); 
  static float linear(float x);
  static float smoothstep(float x);

 private:
  Blend() = delete;
};

class SkeletalAnimationUtils {
 public:

  static std::vector<std::shared_ptr<SkeletalAnimation>> load_animations(aiScene const*);
  static std::shared_ptr<Node> load_hierarchy(aiScene const* scene);

  static void set_bone_properties(std::map<std::string, std::pair<uint, scm::math::mat4f>> const& info, std::shared_ptr<Node>& currNode);
  static void collect_bone_indices(std::map<std::string, int>& ids, std::shared_ptr<Node> const& pNode);
  
  static void calculate_matrices(float TimeInSeconds, std::shared_ptr<Node> const& root, std::shared_ptr<SkeletalAnimation> const& pAnim, std::vector<scm::math::mat4f>& Transforms);

  static void accumulate_matrices(std::vector<scm::math::mat4f>& transformMat4s, std::shared_ptr<Node> const& pNode, Pose const& pose, scm::math::mat4f const& ParentTransform);
  static Pose calculate_pose(float animationTime, std::shared_ptr<SkeletalAnimation> const& pAnim);

  static std::shared_ptr<Node> find_node(std::string const& name, std::shared_ptr<Node> const& root);

 private:

  static BoneAnimation const* find_node_anim(std::shared_ptr<SkeletalAnimation> const& pAnimation, std::string const& nodeName);  

  inline SkeletalAnimationUtils(){};
  inline ~SkeletalAnimationUtils(){};
};

}

#endif //GUA_SKELETAL_ANIMATION_UTILS_HPP
