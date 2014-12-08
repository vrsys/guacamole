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

#ifndef GUA_SKELETAL_ANIMATION_DIRECTOR_HPP
#define GUA_SKELETAL_ANIMATION_DIRECTOR_HPP

// guacamole headers
#include <gua/platform.hpp>
 #include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/BoneTransformUniformBlock.hpp>
#include <gua/utils/Timer.hpp>

// external headers
#include <scm/gl_core.h>
#include <scm/core/math/quat.h>

#include <vector>
#include <map>
#include <assimp/scene.h>       // Output data structure

namespace {

scm::math::mat4f ai_to_gua(aiMatrix4x4 const& m){
  scm::math::mat4f res(m.a1,m.b1,m.c1,m.d1
                      ,m.a2,m.b2,m.c2,m.d2
                      ,m.a3,m.b3,m.c3,m.d3
                      ,m.a4,m.b4,m.c4,m.d4);
  return res;
}

scm::math::vec3 ai_to_gua(aiVector3D const& v){
  scm::math::vec3 res(v.x, v.y, v.z);
  return res;
}

scm::math::quatf ai_to_gua(aiQuaternion const& q){
  scm::math::quatf res(q.w, q.x, q.y, q.z);
  return res;
}

}

namespace gua {

class SkeletalAnimationDirector{
 public:

  SkeletalAnimationDirector(aiScene const*);
  inline ~SkeletalAnimationDirector(){};

 uint getBoneID(std::string const& name);
 void updateBoneTransforms(RenderContext const& ctx);
 
 void LoadAnimations(aiScene const*);
 
 private:

  struct BoneInfo
  {
    scm::math::mat4f BoneOffset;
    scm::math::mat4f FinalTransformation;        

    BoneInfo()
    {
      //BoneOffset.SetZero();
      //FinalTransformation.SetZero();

      BoneOffset = scm::math::mat4f(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
      FinalTransformation = scm::math::mat4f(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);              
    }
  
  };

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
      duration{0},
      keysPerSecond{0},
      numBoneAnims{0},
      boneAnims{}
    {}

    SkeletalAnimation(aiAnimation* anim):
      name{anim->mName.C_Str()},
      duration{anim->mDuration},
      keysPerSecond{anim->mTicksPerSecond},
      numBoneAnims{anim->mNumChannels},
      boneAnims{}
    {
      for(unsigned i = 0; i < numBoneAnims; ++i) {
        boneAnims.push_back(BoneAnimation{anim->mChannels[i]});
      }
    }

    ~SkeletalAnimation(){};

    std::string name;
    double duration;
    double keysPerSecond;
    unsigned numBoneAnims;
    std::vector<BoneAnimation> boneAnims;
  };

  struct Node {
    Node(aiNode const* node):
      name{node->mName.C_Str()},
      numChildren{node->mNumChildren},
      transformation{ai_to_gua(node->mTransformation)}
    {
      for(unsigned i = 0; i < node->mNumChildren; ++i) {
        std::shared_ptr<Node> child = std::make_shared<Node>(node->mChildren[i]);
        children.push_back(child);
      }
    }

    ~Node(){};

    std::string name;
    unsigned numChildren;
    std::vector<std::shared_ptr<Node>> children;
    scm::math::mat4f transformation;
  };

  //void LoadBones(uint MeshIndex, const aiMesh* pMesh, std::vector<VertexBoneData>& Bones);

  void LoadBones();

  void BoneTransform(float TimeInSeconds, std::vector<scm::math::mat4f>& Transforms);

  void ReadNodeHierarchy(float AnimationTime, const aiNode* pNode, const scm::math::mat4f& ParentTransform);
  void ReadNodeHierarchyStatic(const aiNode* pNode, const scm::math::mat4f& ParentTransform);
  void CalcInterpolatedScaling(scm::math::vec3& Out, float AnimationTime, BoneAnimation const& nodeAnim);
  void CalcInterpolatedRotation(scm::math::quatf& Out, float AnimationTime, BoneAnimation const& nodeAnim);
  void CalcInterpolatedPosition(scm::math::vec3& Out, float AnimationTime, BoneAnimation const& nodeAnim);    
  uint FindScaling(float AnimationTime, BoneAnimation const& nodeAnim);
  uint FindRotation(float AnimationTime, BoneAnimation const& nodeAnim);
  uint FindPosition(float AnimationTime, BoneAnimation const& nodeAnim);
  BoneAnimation const* FindNodeAnim(std::shared_ptr<SkeletalAnimation> const& pAnimation, std::string const& nodeName);

  std::map<std::string,uint> bone_mapping_; // maps a bone name to its index
  uint num_bones_;
  std::vector<BoneInfo> bone_info_;

  std::shared_ptr<BoneTransformUniformBlock> bone_transforms_block_;

  std::vector<std::shared_ptr<SkeletalAnimation>> animations_;
  std::shared_ptr<SkeletalAnimation> currAnimation_;

  aiScene const* scene_;


  bool firstRun_;
  bool hasAnims_;

  Timer timer_;
};

}

#endif  // GUA_SKELETAL_ANIMATION_DIRECTOR_HPP
