// class header
#include <gua/renderer/SkeletalAnimationDirector.hpp>

// guacamole headers
#include <gua/utils/Logger.hpp>

//external headers
#include <iostream>

namespace gua
{
SkeletalAnimationDirector::SkeletalAnimationDirector(aiScene const* scene)
    :num_bones_(0),
    bone_transforms_block_(nullptr),
    scene_{scene},
    hasAnims_{scene_->HasAnimations()},
    firstRun_{true},
    animations_{},
    timer_(){
  timer_.start();
  LoadBones();
}

void SkeletalAnimationDirector::LoadBones(){

  for (uint i = 0 ; i < scene_->mNumMeshes ; i++) {
    for (uint b = 0; b < scene_->mMeshes[i]->mNumBones; ++b){

      std::string BoneName(scene_->mMeshes[i]->mBones[b]->mName.data);  
      if (bone_mapping_.find(BoneName) == bone_mapping_.end()) {

        // Allocate an index for a new bone
        uint BoneIndex = num_bones_;
        num_bones_++;            
        BoneInfo bi;      

        bone_info_.push_back(bi);
        bone_info_[BoneIndex].BoneOffset = ai_to_gua(scene_->mMeshes[i]->mBones[b]->mOffsetMatrix);   
        bone_mapping_[BoneName] = BoneIndex;
      }

    }

  }

}

void SkeletalAnimationDirector::LoadAnimations(aiScene const* scene) {
  if(!scene->HasAnimations()) Logger::LOG_WARNING << "scene contains no animations!" << std::endl;
  std::cout << scene->mNumAnimations << std::endl;
  std::cout << scene->mAnimations[0] << std::endl;
 
  for(uint i = 0; i < scene->mNumAnimations; ++i) {
    animations_.push_back(std::make_shared<SkeletalAnimation>(scene->mAnimations[i]));

    std::cout << animations_.size() << std::endl;
  }

  if(animations_.size() > 0) hasAnims_ = true;

  std::cout << scene->mRootNode->mName.data << std::endl;
  if(scene->mRootNode->mName.data != scene_->mRootNode->mName.data) {

    aiNode const* currNode = scene_->mRootNode;

    for(unsigned i = 0; i < scene_->mRootNode->mNumChildren; ++i) {

      if(scene_->mRootNode->mChildren[i]->mName == scene->mRootNode->mName) {
        rootNode_ = scene_->mRootNode->mChildren[i]->mChildren[0];
        return;
      }
      else {
        std::cout << scene_->mRootNode->mChildren[i]->mName.data << " does not match "<<std::endl;
      }
    }
    Logger::LOG_ERROR << "No matching root node in mesh hierarchy found" << std::endl;
    assert(false);
  }
  else {
    rootNode_ = scene_->mRootNode;
  }
}

uint SkeletalAnimationDirector::getBoneID(std::string const& name) {
  return bone_mapping_.at(name);
}

void SkeletalAnimationDirector::BoneTransform(float TimeInSeconds, std::vector<scm::math::mat4f>& Transforms) {
  scm::math::mat4f Identity = scm::math::mat4f::identity();
  
  //if no frame frequency is given, set to 25
  float TicksPerSecond = 25.0f;
  if(animations_[0]->keysPerSecond != 0)
  {
    TicksPerSecond = animations_[0]->keysPerSecond;
  } 

  float TimeInTicks = TimeInSeconds * TicksPerSecond;
  float AnimationTime = fmod(TimeInTicks, (float)animations_[0]->duration);

  ReadNodeHierarchy(AnimationTime, rootNode_, Identity);

  for (uint i = 0 ; i < num_bones_ ; i++) {
      Transforms[i] = bone_info_[i].FinalTransformation;
  }
}

void SkeletalAnimationDirector::ReadNodeHierarchy(float AnimationTime, const aiNode* pNode, const scm::math::mat4f& ParentTransform)
{    
  std::string NodeName(pNode->mName.data);
  
  std::shared_ptr<SkeletalAnimation> const& pAnimation = animations_[0];
      
  scm::math::mat4f NodeTransformation{ai_to_gua(pNode->mTransformation)};
   
  BoneAnimation const& nodeAnim = FindNodeAnim(pAnimation, NodeName);
  
  // Interpolate scaling and generate scaling transformation matrix
  scm::math::vec3 Scaling;
  CalcInterpolatedScaling(Scaling, AnimationTime, nodeAnim);
  scm::math::mat4f ScalingM = scm::math::make_scale(Scaling);

  // Interpolate rotation and generate rotation transformation matrix
  scm::math::quatf RotationQ;
  CalcInterpolatedRotation(RotationQ, AnimationTime, nodeAnim); 
  scm::math::mat4f RotationM = RotationQ.to_matrix();

  // Interpolate translation and generate translation transformation matrix
  scm::math::vec3 Translation;
  CalcInterpolatedPosition(Translation, AnimationTime, nodeAnim);
  scm::math::mat4f TranslationM = scm::math::make_translation(Translation);
  
  // Combine the above transformations
  NodeTransformation = TranslationM * RotationM * ScalingM;
     
  scm::math::mat4f GlobalTransformation = ParentTransform * NodeTransformation;
  
  if (bone_mapping_.find(NodeName) != bone_mapping_.end()) {
    uint BoneIndex = bone_mapping_[NodeName];
    //bone_info_[BoneIndex].FinalTransformation = m_GlobalInverseTransform * GlobalTransformation * bone_info_[BoneIndex].BoneOffset;
    bone_info_[BoneIndex].FinalTransformation = GlobalTransformation * bone_info_[BoneIndex].BoneOffset;
  }
  
  for (uint i = 0 ; i < pNode->mNumChildren ; i++) {
    ReadNodeHierarchy(AnimationTime, pNode->mChildren[i], GlobalTransformation);
  }
}

void SkeletalAnimationDirector::ReadNodeHierarchyStatic(const aiNode* pNode, const scm::math::mat4f& ParentTransform)
{    
  std::string NodeName(pNode->mName.data);
     
  scm::math::mat4f GlobalTransformation = ParentTransform * ai_to_gua(pNode->mTransformation);
  
  if (bone_mapping_.find(NodeName) != bone_mapping_.end()) {
    uint BoneIndex = bone_mapping_[NodeName];
    //bone_info_[BoneIndex].FinalTransformation = m_GlobalInverseTransform * GlobalTransformation * bone_info_[BoneIndex].BoneOffset;
    bone_info_[BoneIndex].FinalTransformation = GlobalTransformation * bone_info_[BoneIndex].BoneOffset;
  }
  
  for (uint i = 0 ; i < pNode->mNumChildren ; i++) {
    ReadNodeHierarchyStatic(pNode->mChildren[i], GlobalTransformation);
  }
}
////////////////////////////////////////////////////////////////////////////////

void SkeletalAnimationDirector::updateBoneTransforms(RenderContext const& ctx)
{
  if(!hasAnims_ && !firstRun_) return;

  if(!bone_transforms_block_) {
    //TODO one transform block per context
    bone_transforms_block_ = std::make_shared<BoneTransformUniformBlock>(ctx.render_device);
  }
  //reserve vector for transforms
  std::vector<scm::math::mat4f> Transforms{num_bones_, scm::math::mat4f::identity()};

  if(hasAnims_) {
    BoneTransform(timer_.get_elapsed(), Transforms);
  }
  else {        //this will be only called once, transformations dont need to be updated without anims
    ReadNodeHierarchyStatic(rootNode_, scm::math::mat4f::identity());

    for (uint i = 0 ; i < num_bones_ ; i++) {
        Transforms[i] = bone_info_[i].FinalTransformation;
    }

    firstRun_ = false;
  }

  bone_transforms_block_->update(ctx.render_context, Transforms);
  ctx.render_context->bind_uniform_buffer( bone_transforms_block_->block().block_buffer(), 1 );

}

////////////////////////////////////////////////////////////////////////////////
SkeletalAnimationDirector::BoneAnimation const& SkeletalAnimationDirector::FindNodeAnim(std::shared_ptr<SkeletalAnimation> const& pAnimation, std::string const& nodeName)
{
  for (uint i = 0 ; i < pAnimation->numBoneAnims ; i++) {
    BoneAnimation const& nodeAnim = pAnimation->boneAnims[i];
    
    //std::cout << nodeName << ", with " << nodeAnim.name << std::endl;
    if (nodeAnim.name == nodeName) {
        return nodeAnim;
    }
  }

  Logger::LOG_ERROR << "No matching bone in animation hierarchy found" << std::endl;
  assert(false);
  return BoneAnimation{};
}

uint SkeletalAnimationDirector::FindPosition(float AnimationTime, BoneAnimation const& nodeAnim)
{    
  if(nodeAnim.numTranslationKeys < 1) {
    Logger::LOG_ERROR << "FindPosition - no translation keys" << std::endl;
    assert(false);
  } 

  for (uint i = 0 ; i < nodeAnim.numTranslationKeys - 1 ; i++) {
    if (AnimationTime < (float)nodeAnim.translationKeys[i + 1].time) {
        return i;
    }
  }

  Logger::LOG_ERROR << "FindPosition failed" << std::endl;
  assert(false);

  return 0;
}


uint SkeletalAnimationDirector::FindRotation(float AnimationTime, BoneAnimation const& nodeAnim)
{
  if(nodeAnim.numRotationKeys < 1) {
    Logger::LOG_ERROR << "FindRotation - no rotation keys" << std::endl;
    assert(false);
  }

  for (uint i = 0 ; i < nodeAnim.numRotationKeys - 1 ; i++) {
    if (AnimationTime < (float)nodeAnim.rotationKeys[i + 1].time) {
        return i;
    }
  }
  
  Logger::LOG_ERROR << "FindRotation failed" << std::endl;
  assert(false);

  return 0;
}


uint SkeletalAnimationDirector::FindScaling(float AnimationTime, BoneAnimation const& nodeAnim)
{
  if(nodeAnim.numScalingKeys < 1) {
    Logger::LOG_ERROR << "FindScaling - no scaling keys" << std::endl;
    assert(false);
  }
  
  for (uint i = 0 ; i < nodeAnim.numScalingKeys - 1 ; i++) {
    if (AnimationTime < (float)nodeAnim.scalingKeys[i + 1].time) {
        return i;
    }
  }
  
  Logger::LOG_ERROR << "FindScaling failed" << std::endl;

  return 0;
}


void SkeletalAnimationDirector::CalcInterpolatedPosition(scm::math::vec3& Out, float AnimationTime, BoneAnimation const& nodeAnim)
{
  if (nodeAnim.numTranslationKeys == 1) {
      Out = nodeAnim.translationKeys[0].value;
      return;
  }
          
  uint PositionIndex = FindPosition(AnimationTime, nodeAnim);
  uint NextPositionIndex = (PositionIndex + 1);

  if(NextPositionIndex > nodeAnim.numTranslationKeys) {
    Logger::LOG_ERROR << "CalcInterpolatedPosition - frame out of range" << std::endl;
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


void SkeletalAnimationDirector::CalcInterpolatedRotation(scm::math::quatf& Out, float AnimationTime, BoneAnimation const& nodeAnim)
{
  // we need at least two values to interpolate...
  if (nodeAnim.numRotationKeys == 1) {
      Out = nodeAnim.rotationKeys[0].value;
      return;
  }
  
  uint RotationIndex = FindRotation(AnimationTime, nodeAnim);
  uint NextRotationIndex = (RotationIndex + 1);

  if(NextRotationIndex > nodeAnim.numRotationKeys) {
    Logger::LOG_ERROR << "CalcInterpolatedRotation - frame out of range" << std::endl;
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


void SkeletalAnimationDirector::CalcInterpolatedScaling(scm::math::vec3& Out, float AnimationTime, BoneAnimation const& nodeAnim)
{
  if (nodeAnim.numScalingKeys == 1) {
      Out = nodeAnim.scalingKeys[0].value;
      return;
  }

  uint ScalingIndex = FindScaling(AnimationTime, nodeAnim);
  uint NextScalingIndex = (ScalingIndex + 1);

  if(NextScalingIndex > nodeAnim.numScalingKeys) {
    Logger::LOG_ERROR << "CalcInterpolatedScaling - frame out of range" << std::endl;
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