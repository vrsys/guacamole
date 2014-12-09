// class header
#include <gua/renderer/SkeletalAnimationUtils.hpp>

// guacamole headers
#include <gua/utils/Logger.hpp>

//external headers
#include <iostream>

namespace gua
{

void SkeletalAnimationUtils::collect_bone_indices(std::map<std::string, uint>& ids, std::shared_ptr<Node> const& pNode) {
  ids[pNode->name] = pNode->index;

  for(std::shared_ptr<Node>& child : pNode->children) {
    collect_bone_indices(ids, child);
  }
}

std::shared_ptr<Node> SkeletalAnimationUtils::load_hierarchy(aiScene const* scene) {
  //construct hierarchy
  std::shared_ptr<Node> root = std::make_shared<Node>(scene->mRootNode);

  std::map<std::string, std::pair<uint, scm::math::mat4f>> bone_info{};
  unsigned num_bones = 0;
  for (uint i = 0 ; i < scene->mNumMeshes ; i++) {
    for (uint b = 0; b < scene->mMeshes[i]->mNumBones; ++b){

      std::string BoneName(scene->mMeshes[i]->mBones[b]->mName.data);  
      if (bone_info.find(BoneName) == bone_info.end()) {
         
        bone_info[BoneName] = std::make_pair(num_bones, ai_to_gua(scene->mMeshes[i]->mBones[b]->mOffsetMatrix));   
        ++num_bones;
      }
    }
  }
 set_bone_properties(bone_info, root);
  return root;
}

void SkeletalAnimationUtils::set_bone_properties(std::map<std::string, std::pair<uint, scm::math::mat4f>> const& infos, std::shared_ptr<Node>& currNode) {
  try {
    currNode->offsetMatrix = infos.at(currNode->name).second;
    currNode->index = infos.at(currNode->name).first;
  }
  catch(std:: exception& e) {
    Logger::LOG_WARNING <<  currNode->name << " has no vertices mapped to it" << std::endl;
  }

  for(std::shared_ptr<Node>& child : currNode->children) {
    set_bone_properties(infos, child);
  }
}


std::vector<std::shared_ptr<SkeletalAnimation>> SkeletalAnimationUtils::load_animations(aiScene const* scene) {
  std::vector<std::shared_ptr<SkeletalAnimation>> animations{};
  if(!scene->HasAnimations()) Logger::LOG_WARNING << "scene contains no animations!" << std::endl;
 
  for(uint i = 0; i < scene->mNumAnimations; ++i) {
    animations.push_back(std::make_shared<SkeletalAnimation>(scene->mAnimations[i]));
  }

  return animations;
}

void SkeletalAnimationUtils::accumulate_transforms(std::vector<scm::math::mat4f>& transforms, float AnimationTime, std::shared_ptr<Node> const& pNode, std::shared_ptr<SkeletalAnimation> const& pAnim, scm::math::mat4f& ParentTransform)
{
      
  scm::math::mat4f NodeTransformation{pNode->transformation};
   
  BoneAnimation const* pNodeAnim = SkeletalAnimationUtils::find_node_anim(pAnim, pNode->name);

  if(pNodeAnim) {
    BoneAnimation const& nodeAnim = *pNodeAnim;

    // Interpolate scaling and generate scaling transformation matrix
    scm::math::vec3 Scaling;
    SkeletalAnimationUtils::interpolate_scaling(Scaling, AnimationTime, nodeAnim);
    scm::math::mat4f ScalingM = scm::math::make_scale(Scaling);

    // Interpolate rotation and generate rotation transformation matrix
    scm::math::quatf RotationQ;
    SkeletalAnimationUtils::interpolate_rotation(RotationQ, AnimationTime, nodeAnim); 
    scm::math::mat4f RotationM = RotationQ.to_matrix();

    // Interpolate translation and generate translation transformation matrix
    scm::math::vec3 Translation;
    SkeletalAnimationUtils::interpolate_position(Translation, AnimationTime, nodeAnim);
    scm::math::mat4f TranslationM = scm::math::make_translation(Translation);
    
    // Combine the above transformations
    NodeTransformation = TranslationM * RotationM * ScalingM;  
  }
  
  scm::math::mat4f GlobalTransformation = ParentTransform * NodeTransformation;
  
  //update transform if bone is mapped
  if (pNode->index >= 0) {
    transforms[pNode->index] = GlobalTransformation * pNode->offsetMatrix;
  }
  
  for (uint i = 0 ; i < pNode->numChildren ; i++) {
    accumulate_transforms(transforms, AnimationTime, pNode->children[i], pAnim, GlobalTransformation);
  }
}

BoneAnimation const* SkeletalAnimationUtils::find_node_anim(std::shared_ptr<SkeletalAnimation> const& pAnimation, std::string const& nodeName)
{
  for (uint i = 0 ; i < pAnimation->numBoneAnims ; i++) {
    BoneAnimation const& nodeAnim = pAnimation->boneAnims[i];
    
    //std::cout << nodeName << ", with " << nodeAnim.name << std::endl;
    if (nodeAnim.name == nodeName) {
        return &nodeAnim;
    }
  }

  //Logger::LOG_ERROR << "No matching bone in animation hierarchy found" << std::endl;
  //assert(false);
  return NULL;
}

////////////////////////////////////////////////////////////////////////////////

uint SkeletalAnimationUtils::find_position(float AnimationTime, BoneAnimation const& nodeAnim)
{    
  if(nodeAnim.numTranslationKeys < 1) {
    Logger::LOG_ERROR << "no keys" << std::endl;
    assert(false);
  } 

  for (uint i = 0 ; i < nodeAnim.numTranslationKeys - 1 ; i++) {
    if (AnimationTime < (float)nodeAnim.translationKeys[i + 1].time) {
        return i;
    }
  }

  Logger::LOG_ERROR << "no key found" << std::endl;
  assert(false);

  return 0;
}


uint SkeletalAnimationUtils::find_rotation(float AnimationTime, BoneAnimation const& nodeAnim)
{
  if(nodeAnim.numRotationKeys < 1) {
    Logger::LOG_ERROR << "no keys" << std::endl;
    assert(false);
  }

  for (uint i = 0 ; i < nodeAnim.numRotationKeys - 1 ; i++) {
    if (AnimationTime < (float)nodeAnim.rotationKeys[i + 1].time) {
        return i;
    }
  }
  
  Logger::LOG_ERROR << "no key found" << std::endl;
  assert(false);

  return 0;
}


uint SkeletalAnimationUtils::find_scaling(float AnimationTime, BoneAnimation const& nodeAnim)
{
  if(nodeAnim.numScalingKeys < 1) {
    Logger::LOG_ERROR << "no keys" << std::endl;
    assert(false);
  }
  
  for (uint i = 0 ; i < nodeAnim.numScalingKeys - 1 ; i++) {
    if (AnimationTime < (float)nodeAnim.scalingKeys[i + 1].time) {
        return i;
    }
  }
  
  Logger::LOG_ERROR << "no key found" << std::endl;
  assert(false);

  return 0;
}


void SkeletalAnimationUtils::interpolate_position(scm::math::vec3& Out, float AnimationTime, BoneAnimation const& nodeAnim)
{
  if (nodeAnim.numTranslationKeys == 1) {
      Out = nodeAnim.translationKeys[0].value;
      return;
  }
          
  uint PositionIndex = find_position(AnimationTime, nodeAnim);
  uint NextPositionIndex = (PositionIndex + 1);

  if(NextPositionIndex > nodeAnim.numTranslationKeys) {
    Logger::LOG_ERROR << "frame out of range" << std::endl;
    assert(false);
  }

  float DeltaTime = (float)(nodeAnim.translationKeys[NextPositionIndex].time - nodeAnim.translationKeys[PositionIndex].time);
  float Factor = (AnimationTime - (float)nodeAnim.translationKeys[PositionIndex].time) / DeltaTime;
  //assert(Factor >= 0.0f && Factor <= 1.0f);
  scm::math::vec3 const& Start = nodeAnim.translationKeys[PositionIndex].value;
  scm::math::vec3 const& End = nodeAnim.translationKeys[NextPositionIndex].value;
  scm::math::vec3 Delta = End - Start;
  Out = Start + Factor * Delta;
}


void SkeletalAnimationUtils::interpolate_rotation(scm::math::quatf& Out, float AnimationTime, BoneAnimation const& nodeAnim)
{
  // we need at least two values to interpolate...
  if (nodeAnim.numRotationKeys == 1) {
      Out = nodeAnim.rotationKeys[0].value;
      return;
  }
  
  uint RotationIndex = find_rotation(AnimationTime, nodeAnim);
  uint NextRotationIndex = (RotationIndex + 1);

  if(NextRotationIndex > nodeAnim.numRotationKeys) {
    Logger::LOG_ERROR << "frame out of range" << std::endl;
    assert(false);
  }

  float DeltaTime = (float)(nodeAnim.rotationKeys[NextRotationIndex].time - nodeAnim.rotationKeys[RotationIndex].time);
  float Factor = (AnimationTime - (float)nodeAnim.rotationKeys[RotationIndex].time) / DeltaTime;
  //assert(Factor >= 0.0f && Factor <= 1.0f);
  scm::math::quatf const& StartRotationQ = nodeAnim.rotationKeys[RotationIndex].value;
  scm::math::quatf const& EndRotationQ   = nodeAnim.rotationKeys[NextRotationIndex].value;  
  scm::math::quatf temp;  
  temp = slerp(StartRotationQ, EndRotationQ, Factor);
  Out = normalize(temp);
}


void SkeletalAnimationUtils::interpolate_scaling(scm::math::vec3& Out, float AnimationTime, BoneAnimation const& nodeAnim)
{
  if (nodeAnim.numScalingKeys == 1) {
      Out = nodeAnim.scalingKeys[0].value;
      return;
  }

  uint ScalingIndex = find_scaling(AnimationTime, nodeAnim);
  uint NextScalingIndex = (ScalingIndex + 1);

  if(NextScalingIndex > nodeAnim.numScalingKeys) {
    Logger::LOG_ERROR << "frame out of range" << std::endl;
    assert(false);
  }

  float DeltaTime = (float)(nodeAnim.scalingKeys[NextScalingIndex].time - nodeAnim.scalingKeys[ScalingIndex].time);
  float Factor = (AnimationTime - (float)nodeAnim.scalingKeys[ScalingIndex].time) / DeltaTime;
  //assert(Factor >= 0.0f && Factor <= 1.0f);
  scm::math::vec3 const& Start = nodeAnim.scalingKeys[ScalingIndex].value;
  scm::math::vec3 const& End   = nodeAnim.scalingKeys[NextScalingIndex].value;
  scm::math::vec3 Delta = End - Start;
  Out = Start + Factor * Delta;
}


} // namespace gua