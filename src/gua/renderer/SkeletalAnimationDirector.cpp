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
    hasAnims_{scene->HasAnimations()},
    firstRun_{true},
    animations_{},
    currAnimation_{},
    timer_(){
  timer_.start();
  LoadBones();
  LoadHierarchy();
}

void SkeletalAnimationDirector::LoadBones() {

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
void SkeletalAnimationDirector::LoadHierarchy() {
  root_ = std::make_shared<Node>(scene_->mRootNode);
}

void SkeletalAnimationDirector::LoadAnimations(aiScene const* scene) {
  if(!scene->HasAnimations()) Logger::LOG_WARNING << "scene contains no animations!" << std::endl;
 
  for(uint i = 0; i < scene->mNumAnimations; ++i) {
    animations_.push_back(std::make_shared<SkeletalAnimation>(scene->mAnimations[i]));
  }

  if(animations_.size() > 0) hasAnims_ = true;
  currAnimation_ = animations_[animations_.size()-1];
}

uint SkeletalAnimationDirector::getBoneID(std::string const& name) {
  return bone_mapping_.at(name);
}

void SkeletalAnimationDirector::BoneTransform(float TimeInSeconds, std::vector<scm::math::mat4f>& Transforms) {
  scm::math::mat4f Identity = scm::math::mat4f::identity();
  
  //if no frame frequency is given, set to 25
  float TicksPerSecond = 25.0f;
  float AnimationTime = 0;

  if(currAnimation_) {
    if(currAnimation_->keysPerSecond != 0) {
      TicksPerSecond = currAnimation_->keysPerSecond;
    } 
    float TimeInTicks = TimeInSeconds * TicksPerSecond;
    AnimationTime = fmod(TimeInTicks, (float)currAnimation_->duration);
  }

  ReadNodeHierarchy(AnimationTime, root_, Identity);

  for (uint i = 0 ; i < num_bones_ ; i++) {
      Transforms[i] = bone_info_[i].FinalTransformation;
  }
}

void SkeletalAnimationDirector::ReadNodeHierarchy(float AnimationTime, std::shared_ptr<Node> const& pNode, const scm::math::mat4f& ParentTransform)
{    
  std::string NodeName(pNode->name);
  
  std::shared_ptr<SkeletalAnimation> const& pAnimation = currAnimation_;
      
  scm::math::mat4f NodeTransformation{pNode->transformation};
   
  BoneAnimation const* pNodeAnim = FindNodeAnim(pAnimation, NodeName);

  if(pNodeAnim) {
    BoneAnimation const& nodeAnim = *pNodeAnim;

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
       
  }
  
  scm::math::mat4f GlobalTransformation = ParentTransform * NodeTransformation;
  
  if (bone_mapping_.find(NodeName) != bone_mapping_.end()) {
    uint BoneIndex = bone_mapping_[NodeName];
    //bone_info_[BoneIndex].FinalTransformation = m_GlobalInverseTransform * GlobalTransformation * bone_info_[BoneIndex].BoneOffset;
    bone_info_[BoneIndex].FinalTransformation = GlobalTransformation * bone_info_[BoneIndex].BoneOffset;
  }
  
  for (uint i = 0 ; i < pNode->numChildren ; i++) {
    ReadNodeHierarchy(AnimationTime, pNode->children[i], GlobalTransformation);
  }
}

SkeletalAnimationDirector::BoneAnimation const* SkeletalAnimationDirector::FindNodeAnim(std::shared_ptr<SkeletalAnimation> const& pAnimation, std::string const& nodeName)
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

void SkeletalAnimationDirector::updateBoneTransforms(RenderContext const& ctx)
{
  if(!hasAnims_ && !firstRun_) return;
  if(!hasAnims_) firstRun_ = false;

  if(!bone_transforms_block_) {
    //TODO one transform block per context
    bone_transforms_block_ = std::make_shared<BoneTransformUniformBlock>(ctx.render_device);
  }
  //reserve vector for transforms
  std::vector<scm::math::mat4f> Transforms{num_bones_, scm::math::mat4f::identity()};

  BoneTransform(timer_.get_elapsed(), Transforms);

  bone_transforms_block_->update(ctx.render_context, Transforms);
  ctx.render_context->bind_uniform_buffer( bone_transforms_block_->block().block_buffer(), 1 );

}

////////////////////////////////////////////////////////////////////////////////

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
          
  uint PositionIndex = SkeletalAnimationDirector::FindPosition(AnimationTime, nodeAnim);
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
  
  uint RotationIndex = SkeletalAnimationDirector::FindRotation(AnimationTime, nodeAnim);
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

  uint ScalingIndex = SkeletalAnimationDirector::FindScaling(AnimationTime, nodeAnim);
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