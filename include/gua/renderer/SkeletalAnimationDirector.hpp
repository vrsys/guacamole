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
  
  //void LoadBones(uint MeshIndex, const aiMesh* pMesh, std::vector<VertexBoneData>& Bones);

  void LoadBones();

  void BoneTransform(float TimeInSeconds, std::vector<scm::math::mat4f>& Transforms);

  void ReadNodeHierarchy(float AnimationTime, const aiNode* pNode, const scm::math::mat4f& ParentTransform);
  void ReadNodeHierarchyStatic(const aiNode* pNode, const scm::math::mat4f& ParentTransform);
  void CalcInterpolatedScaling(scm::math::vec3& Out, float AnimationTime, const aiNodeAnim* pNodeAnim);
  void CalcInterpolatedRotation(scm::math::quatf& Out, float AnimationTime, const aiNodeAnim* pNodeAnim);
  void CalcInterpolatedPosition(scm::math::vec3& Out, float AnimationTime, const aiNodeAnim* pNodeAnim);    
  uint FindScaling(float AnimationTime, const aiNodeAnim* pNodeAnim);
  uint FindRotation(float AnimationTime, const aiNodeAnim* pNodeAnim);
  uint FindPosition(float AnimationTime, const aiNodeAnim* pNodeAnim);
  const aiNodeAnim* FindNodeAnim(const aiAnimation* pAnimation, const std::string NodeName);

  std::map<std::string,uint> bone_mapping_; // maps a bone name to its index
  uint num_bones_;
  std::vector<BoneInfo> bone_info_;

  std::shared_ptr<BoneTransformUniformBlock> bone_transforms_block_;

  aiScene const* scene_;

  std::vector<std::shared_ptr<aiAnimation>> animations_;

  bool firstRun_;
  bool hasAnims_;

  Timer timer_;
};

}

#endif  // GUA_SKELETAL_ANIMATION_DIRECTOR_HPP
