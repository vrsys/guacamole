// class header
#include <gua/renderer/SkeletalAnimationUtils.hpp>

// guacamole headers
#include <gua/utils/Logger.hpp>

//external headers
#include <iostream>

namespace gua {

void SkeletalAnimationUtils::collect_bone_indices(std::map<std::string, int>& ids, std::shared_ptr<Node> const& pNode) {
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

void SkeletalAnimationUtils::set_bone_properties(std::map<std::string, std::pair<uint, scm::math::mat4f>> const& infos, std::shared_ptr<Node>& pNode) {
  try {
    pNode->offsetMatrix = infos.at(pNode->name).second;
    pNode->index = infos.at(pNode->name).first;
  }
  catch(std:: exception& e) {
    Logger::LOG_WARNING <<  pNode->name << " has no vertices mapped to it" << std::endl;
  }

  for(std::shared_ptr<Node>& child : pNode->children) {
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

void SkeletalAnimationUtils::blend(std::map<std::string, Transformation>& transforms1, std::map<std::string, Transformation> const& transforms2, float blendFactor) {
  auto iter = transforms1.begin();
  for_each(transforms2.begin(), transforms2.end(), [&transforms1, &blendFactor, &iter](std::pair<std::string, Transformation> p) {
    iter->second = p.second.blend(iter->second, blendFactor);
    ++iter;
  });
}
void SkeletalAnimationUtils::partial_blend(std::map<std::string, Transformation>& transforms1, std::map<std::string, Transformation> const& transforms2, std::shared_ptr<Node> const& pNode) {
  transforms1.at(pNode->name) = transforms2.at(pNode->name);

  for(std::shared_ptr<Node>& child : pNode->children) {
    partial_blend(transforms1, transforms2, child);
  }
}

void SkeletalAnimationUtils::calculate_pose(float timeInSeconds, std::shared_ptr<Node> const& root, std::shared_ptr<SkeletalAnimation> const& pAnim, std::vector<scm::math::mat4f>& transforms) {
 
  float timeNormalized = 0;
  std::map<std::string, Transformation> transformStructs{};

  if(pAnim) {
    timeNormalized = timeInSeconds / pAnim->duration;
    timeNormalized = scm::math::fract(timeNormalized);

    transformStructs = calculate_transforms(timeNormalized, pAnim);
  }

  scm::math::mat4f identity = scm::math::mat4f::identity();
  accumulate_transforms(transforms, root, transformStructs, identity);
}

void SkeletalAnimationUtils::accumulate_transforms(std::vector<scm::math::mat4f>& transformMat4s, std::shared_ptr<Node> const& pNode, std::map<std::string, Transformation> const& transformStructs, scm::math::mat4f const& parentTransform) {
  scm::math::mat4f nodeTransformation{pNode->transformation};

  if(transformStructs.find(pNode->name) != transformStructs.end()) { 
    nodeTransformation = transformStructs.at(pNode->name).to_matrix();  
  }
  
  scm::math::mat4f finalTransformation = parentTransform * nodeTransformation;

  //update transform if bone is mapped
  if (pNode->index >= 0) {
    transformMat4s[pNode->index] = finalTransformation * pNode->offsetMatrix;
  }
  
  for (uint i = 0 ; i < pNode->numChildren ; i++) {
    accumulate_transforms(transformMat4s, pNode->children[i], transformStructs, finalTransformation);
  }
}

std::map<std::string, Transformation> SkeletalAnimationUtils::calculate_transforms(float animationTime, std::shared_ptr<SkeletalAnimation> const& pAnim) {
  
  std::map<std::string, Transformation> transforms{};

  float currFrame = animationTime * float(pAnim->numFrames);
   
  for(BoneAnimation const& boneAnim : pAnim->boneAnims) {
    Transformation boneTransform{};

    // Interpolate scaling and generate scaling transformation matrix
    boneTransform.scaling = interpolate_scaling(currFrame, boneAnim);

    // Interpolate rotation and generate rotation transformation matrix
    boneTransform.rotation = interpolate_rotation(currFrame, boneAnim); 

    // Interpolate translation and generate translation transformation matrix
    boneTransform.translation = interpolate_position(currFrame, boneAnim);

    transforms[boneAnim.name] = boneTransform;
  }  

  return transforms;
}

BoneAnimation const* SkeletalAnimationUtils::find_node_anim(std::shared_ptr<SkeletalAnimation> const& pAnimation, std::string const& nodeName) {
  for (uint i = 0 ; i < pAnimation->numBoneAnims ; i++) {
    BoneAnimation const& nodeAnim = pAnimation->boneAnims[i];
    
    if (nodeAnim.name == nodeName) {
        return &nodeAnim;
    }
  }

  return NULL;
}

std::shared_ptr<Node> SkeletalAnimationUtils::find_node(std::string const& name, std::shared_ptr<Node> const& root) {
  if(root->name == name) return root;

  for(std::shared_ptr<Node> const& child : root->children) {

    std::shared_ptr<Node> found = find_node(name, child);
    if (found) return found;
  }

  return nullptr;
}
////////////////////////////////////////////////////////////////////////////////

uint SkeletalAnimationUtils::find_position(float animationTime, BoneAnimation const& nodeAnim) {    
  if(nodeAnim.numTranslationKeys < 1) {
    Logger::LOG_ERROR << "no keys" << std::endl;
    assert(false);
  } 

  for (uint i = 0 ; i < nodeAnim.numTranslationKeys - 1 ; i++) {
    if (animationTime < (float)nodeAnim.translationKeys[i + 1].time) {
        return i;
    }
  }

  Logger::LOG_ERROR << "no key found" << std::endl;
  assert(false);

  return 0;
}

uint SkeletalAnimationUtils::find_rotation(float animationTime, BoneAnimation const& nodeAnim) {
  if(nodeAnim.numRotationKeys < 1) {
    Logger::LOG_ERROR << "no keys" << std::endl;
    assert(false);
  }

  for (uint i = 0 ; i < nodeAnim.numRotationKeys - 1 ; i++) {
    if (animationTime < (float)nodeAnim.rotationKeys[i + 1].time) {
        return i;
    }
  }
  
  Logger::LOG_ERROR << "no key found" << std::endl;
  assert(false);

  return 0;
}

uint SkeletalAnimationUtils::find_scaling(float animationTime, BoneAnimation const& nodeAnim) {
  if(nodeAnim.numScalingKeys < 1) {
    Logger::LOG_ERROR << "no keys" << std::endl;
    assert(false);
  }
  
  for (uint i = 0 ; i < nodeAnim.numScalingKeys - 1 ; i++) {
    if (animationTime < (float)nodeAnim.scalingKeys[i + 1].time) {
        return i;
    }
  }
  
  Logger::LOG_ERROR << "no key found" << std::endl;
  assert(false);

  return 0;
}

scm::math::vec3 SkeletalAnimationUtils::interpolate_position(float animationTime, BoneAnimation const& nodeAnim) {
  if (nodeAnim.numTranslationKeys == 1) {
      return nodeAnim.translationKeys[0].value;
  }
          
  uint PositionIndex = find_position(animationTime, nodeAnim);
  uint NextPositionIndex = (PositionIndex + 1);

  if(NextPositionIndex > nodeAnim.numTranslationKeys) {
    Logger::LOG_ERROR << "frame out of range" << std::endl;
    assert(false);
  }

  float DeltaTime = (float)(nodeAnim.translationKeys[NextPositionIndex].time - nodeAnim.translationKeys[PositionIndex].time);
  float Factor = (animationTime - (float)nodeAnim.translationKeys[PositionIndex].time) / DeltaTime;
  //assert(Factor >= 0.0f && Factor <= 1.0f);
  scm::math::vec3 const& Start = nodeAnim.translationKeys[PositionIndex].value;
  scm::math::vec3 const& End = nodeAnim.translationKeys[NextPositionIndex].value;
  scm::math::vec3 Delta = End - Start;

  return Start + Factor * Delta;
}

scm::math::quatf SkeletalAnimationUtils::interpolate_rotation(float animationTime, BoneAnimation const& nodeAnim) {
  // we need at least two values to interpolate...
  if (nodeAnim.numRotationKeys == 1) {
      return nodeAnim.rotationKeys[0].value;

  }
  
  uint RotationIndex = find_rotation(animationTime, nodeAnim);
  uint NextRotationIndex = (RotationIndex + 1);

  if(NextRotationIndex > nodeAnim.numRotationKeys) {
    Logger::LOG_ERROR << "frame out of range" << std::endl;
    assert(false);
  }

  float DeltaTime = (float)(nodeAnim.rotationKeys[NextRotationIndex].time - nodeAnim.rotationKeys[RotationIndex].time);
  float Factor = (animationTime - (float)nodeAnim.rotationKeys[RotationIndex].time) / DeltaTime;
  //assert(Factor >= 0.0f && Factor <= 1.0f);
  scm::math::quatf const& StartRotationQ = nodeAnim.rotationKeys[RotationIndex].value;
  scm::math::quatf const& EndRotationQ   = nodeAnim.rotationKeys[NextRotationIndex].value;  
  scm::math::quatf temp;  
  temp = slerp(StartRotationQ, EndRotationQ, Factor);

  return normalize(temp);
}

scm::math::vec3 SkeletalAnimationUtils::interpolate_scaling(float animationTime, BoneAnimation const& nodeAnim) {
  if (nodeAnim.numScalingKeys == 1) {
     return nodeAnim.scalingKeys[0].value;
  }

  uint ScalingIndex = find_scaling(animationTime, nodeAnim);
  uint NextScalingIndex = (ScalingIndex + 1);

  if(NextScalingIndex > nodeAnim.numScalingKeys) {
    Logger::LOG_ERROR << "frame out of range" << std::endl;
    assert(false);
  }

  float DeltaTime = (float)(nodeAnim.scalingKeys[NextScalingIndex].time - nodeAnim.scalingKeys[ScalingIndex].time);
  float Factor = (animationTime - (float)nodeAnim.scalingKeys[ScalingIndex].time) / DeltaTime;
  //assert(Factor >= 0.0f && Factor <= 1.0f);
  scm::math::vec3 const& Start = nodeAnim.scalingKeys[ScalingIndex].value;
  scm::math::vec3 const& End   = nodeAnim.scalingKeys[NextScalingIndex].value;
  scm::math::vec3 Delta = End - Start;

  return Start + Factor * Delta;
}

} // namespace gua