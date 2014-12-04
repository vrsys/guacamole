// class header
#include <gua/renderer/SkeletalAnimationDirector.hpp>

// guacamole headers
#include <gua/utils/Logger.hpp>

//external headers
#include <iostream>

namespace {

scm::math::mat4f to_gua(aiMatrix4x4 const& m){
  scm::math::mat4f res(m.a1,m.b1,m.c1,m.d1
                      ,m.a2,m.b2,m.c2,m.d2
                      ,m.a3,m.b3,m.c3,m.d3
                      ,m.a4,m.b4,m.c4,m.d4);
  return res;
}

scm::math::vec3 to_gua(aiVector3D const& v){
  scm::math::vec3 res(v.x, v.y, v.z);
  return res;
}

scm::math::quatf to_gua(aiQuaternion const& q){
  scm::math::quatf res(q.w, q.x, q.y, q.z);
  return res;
}

}


namespace gua
{
SkeletalAnimationDirector::SkeletalAnimationDirector(aiScene const* scene)
    :num_bones_(0),
    bone_transforms_block_(nullptr),
    scene_{scene},
    hasAnims_{scene_->HasAnimations()},
    firstRun_{true},
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
        bone_info_[BoneIndex].BoneOffset = to_gua(scene_->mMeshes[i]->mBones[b]->mOffsetMatrix);    
        bone_mapping_[BoneName] = BoneIndex;
      }

    }

  }

}

uint SkeletalAnimationDirector::getBoneID(std::string const& name)
{
  return bone_mapping_.at(name);
}

void SkeletalAnimationDirector::BoneTransform(float TimeInSeconds, std::vector<scm::math::mat4f>& Transforms)
{
  scm::math::mat4f Identity = scm::math::mat4f::identity();
  
  //if no frame frequency is given, set to 25
  float TicksPerSecond = 25.0f;
  if(scene_->mAnimations[0]->mTicksPerSecond != 0)
  {
    TicksPerSecond = scene_->mAnimations[0]->mTicksPerSecond;
  } 

  float TimeInTicks = TimeInSeconds * TicksPerSecond;
  float AnimationTime = fmod(TimeInTicks, (float)scene_->mAnimations[0]->mDuration);

  ReadNodeHierarchy(AnimationTime, scene_->mRootNode, Identity);

  for (uint i = 0 ; i < num_bones_ ; i++) {
      Transforms[i] = bone_info_[i].FinalTransformation;
  }
}

void SkeletalAnimationDirector::ReadNodeHierarchy(float AnimationTime, const aiNode* pNode, const scm::math::mat4f& ParentTransform)
{    
  std::string NodeName(pNode->mName.data);
  
  const aiAnimation* pAnimation = scene_->mAnimations[0];
      
  scm::math::mat4f NodeTransformation(to_gua(pNode->mTransformation));
   
  const aiNodeAnim* pNodeAnim = FindNodeAnim(pAnimation, NodeName);
  
  if (pNodeAnim) {
    // Interpolate scaling and generate scaling transformation matrix
    scm::math::vec3 Scaling;
    CalcInterpolatedScaling(Scaling, AnimationTime, pNodeAnim);
    scm::math::mat4f ScalingM = scm::math::make_scale(Scaling);

    // Interpolate rotation and generate rotation transformation matrix
    scm::math::quatf RotationQ;
    CalcInterpolatedRotation(RotationQ, AnimationTime, pNodeAnim); 
    scm::math::mat4f RotationM = RotationQ.to_matrix();

    // Interpolate translation and generate translation transformation matrix
    scm::math::vec3 Translation;
    CalcInterpolatedPosition(Translation, AnimationTime, pNodeAnim);
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
  
  for (uint i = 0 ; i < pNode->mNumChildren ; i++) {
    ReadNodeHierarchy(AnimationTime, pNode->mChildren[i], GlobalTransformation);
  }
}

void SkeletalAnimationDirector::ReadNodeHierarchyStatic(const aiNode* pNode, const scm::math::mat4f& ParentTransform)
{    
  std::string NodeName(pNode->mName.data);
     
  scm::math::mat4f GlobalTransformation = ParentTransform * to_gua(pNode->mTransformation);
  
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
    ReadNodeHierarchyStatic(scene_->mRootNode, scm::math::mat4f::identity());

    for (uint i = 0 ; i < num_bones_ ; i++) {
        Transforms[i] = bone_info_[i].FinalTransformation;
    }

    firstRun_ = false;
  }

  bone_transforms_block_->update(ctx.render_context, Transforms);
  ctx.render_context->bind_uniform_buffer( bone_transforms_block_->block().block_buffer(), 1 );

}

////////////////////////////////////////////////////////////////////////////////



const aiNodeAnim* SkeletalAnimationDirector::FindNodeAnim(const aiAnimation* pAnimation, const std::string NodeName)
{
  for (uint i = 0 ; i < pAnimation->mNumChannels ; i++) {
    const aiNodeAnim* pNodeAnim = pAnimation->mChannels[i];
    
    if (std::string(pNodeAnim->mNodeName.data) == NodeName) {
        return pNodeAnim;
    }
  }
  
  return NULL;
}

uint SkeletalAnimationDirector::FindPosition(float AnimationTime, const aiNodeAnim* pNodeAnim)
{    
  if(pNodeAnim->mNumPositionKeys < 1) {
    Logger::LOG_ERROR << "FindPosition - no translation keys" << std::endl;
    assert(false);
  } 

  for (uint i = 0 ; i < pNodeAnim->mNumPositionKeys - 1 ; i++) {
    if (AnimationTime < (float)pNodeAnim->mPositionKeys[i + 1].mTime) {
        return i;
    }
  }

  Logger::LOG_ERROR << "FindPosition failed" << std::endl;
  assert(false);

  return 0;
}


uint SkeletalAnimationDirector::FindRotation(float AnimationTime, const aiNodeAnim* pNodeAnim)
{
  if(pNodeAnim->mNumRotationKeys < 1) {
    Logger::LOG_ERROR << "FindRotation - no rotation keys" << std::endl;
    assert(false);
  }

  for (uint i = 0 ; i < pNodeAnim->mNumRotationKeys - 1 ; i++) {
    if (AnimationTime < (float)pNodeAnim->mRotationKeys[i + 1].mTime) {
        return i;
    }
  }
  
  Logger::LOG_ERROR << "FindRotation failed" << std::endl;
  assert(false);

  return 0;
}


uint SkeletalAnimationDirector::FindScaling(float AnimationTime, const aiNodeAnim* pNodeAnim)
{
  if(pNodeAnim->mNumScalingKeys < 1) {
    Logger::LOG_ERROR << "FindScaling - no scaling keys" << std::endl;
    assert(false);
  }
  
  for (uint i = 0 ; i < pNodeAnim->mNumScalingKeys - 1 ; i++) {
    if (AnimationTime < (float)pNodeAnim->mScalingKeys[i + 1].mTime) {
        return i;
    }
  }
  
  Logger::LOG_ERROR << "FindScaling failed" << std::endl;

  return 0;
}


void SkeletalAnimationDirector::CalcInterpolatedPosition(scm::math::vec3& Out, float AnimationTime, const aiNodeAnim* pNodeAnim)
{
  if (pNodeAnim->mNumPositionKeys == 1) {
      Out = to_gua(pNodeAnim->mPositionKeys[0].mValue);
      return;
  }
          
  uint PositionIndex = FindPosition(AnimationTime, pNodeAnim);
  uint NextPositionIndex = (PositionIndex + 1);

  if(NextPositionIndex > pNodeAnim->mNumPositionKeys) {
    Logger::LOG_ERROR << "CalcInterpolatedPosition - frame out of range" << std::endl;
    assert(false);
  }

  float DeltaTime = (float)(pNodeAnim->mPositionKeys[NextPositionIndex].mTime - pNodeAnim->mPositionKeys[PositionIndex].mTime);
  float Factor = (AnimationTime - (float)pNodeAnim->mPositionKeys[PositionIndex].mTime) / DeltaTime;
  //assert(Factor >= 0.0f && Factor <= 1.0f);
  const aiVector3D& Start = pNodeAnim->mPositionKeys[PositionIndex].mValue;
  const aiVector3D& End = pNodeAnim->mPositionKeys[NextPositionIndex].mValue;
  aiVector3D Delta = End - Start;
  Out = to_gua(Start + Factor * Delta);
}


void SkeletalAnimationDirector::CalcInterpolatedRotation(scm::math::quatf& Out, float AnimationTime, const aiNodeAnim* pNodeAnim)
{
  // we need at least two values to interpolate...
  if (pNodeAnim->mNumRotationKeys == 1) {
      Out = to_gua(pNodeAnim->mRotationKeys[0].mValue);
      return;
  }
  
  uint RotationIndex = FindRotation(AnimationTime, pNodeAnim);
  uint NextRotationIndex = (RotationIndex + 1);

  if(NextRotationIndex > pNodeAnim->mNumRotationKeys) {
    Logger::LOG_ERROR << "CalcInterpolatedRotation - frame out of range" << std::endl;
    assert(false);
  }

  float DeltaTime = (float)(pNodeAnim->mRotationKeys[NextRotationIndex].mTime - pNodeAnim->mRotationKeys[RotationIndex].mTime);
  float Factor = (AnimationTime - (float)pNodeAnim->mRotationKeys[RotationIndex].mTime) / DeltaTime;
  //assert(Factor >= 0.0f && Factor <= 1.0f);
  const aiQuaternion& StartRotationQ = pNodeAnim->mRotationKeys[RotationIndex].mValue;
  const aiQuaternion& EndRotationQ   = pNodeAnim->mRotationKeys[NextRotationIndex].mValue;  
  aiQuaternion temp;  
  aiQuaternion::Interpolate(temp, StartRotationQ, EndRotationQ, Factor);
  Out = to_gua(temp.Normalize());
}


void SkeletalAnimationDirector::CalcInterpolatedScaling(scm::math::vec3& Out, float AnimationTime, const aiNodeAnim* pNodeAnim)
{
  if (pNodeAnim->mNumScalingKeys == 1) {
      Out = to_gua(pNodeAnim->mScalingKeys[0].mValue);
      return;
  }

  uint ScalingIndex = FindScaling(AnimationTime, pNodeAnim);
  uint NextScalingIndex = (ScalingIndex + 1);

  if(NextScalingIndex > pNodeAnim->mNumScalingKeys) {
    Logger::LOG_ERROR << "CalcInterpolatedScaling - frame out of range" << std::endl;
    assert(false);
  }

  float DeltaTime = (float)(pNodeAnim->mScalingKeys[NextScalingIndex].mTime - pNodeAnim->mScalingKeys[ScalingIndex].mTime);
  float Factor = (AnimationTime - (float)pNodeAnim->mScalingKeys[ScalingIndex].mTime) / DeltaTime;
  //assert(Factor >= 0.0f && Factor <= 1.0f);
  const aiVector3D& Start = pNodeAnim->mScalingKeys[ScalingIndex].mValue;
  const aiVector3D& End   = pNodeAnim->mScalingKeys[NextScalingIndex].mValue;
  aiVector3D Delta = End - Start;
  Out = to_gua(Start + Factor * Delta);
}


} // namespace gua