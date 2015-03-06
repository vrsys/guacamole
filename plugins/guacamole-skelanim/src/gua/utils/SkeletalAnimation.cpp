// class header
#include <gua/utils/SkeletalAnimation.hpp>

//external headers
#include <iostream>
#include <queue>

namespace gua {

SkeletalAnimation::SkeletalAnimation():
  name{"default"},
  numFrames{0},
  numFPS{0},
  duration{0},
  numBoneAnims{0},
  boneAnims{}
{}

SkeletalAnimation::SkeletalAnimation(aiAnimation const& anim, std::string const& nm):
  name{nm == "" ? anim.mName.C_Str() : nm},
  numFrames{unsigned(anim.mDuration)},
  numFPS{anim.mTicksPerSecond > 0 ? anim.mTicksPerSecond : 25},
  duration{double(numFrames) / numFPS},
  numBoneAnims{anim.mNumChannels},
  boneAnims{}
{
  for(unsigned i = 0; i < numBoneAnims; ++i) {
    boneAnims.push_back(BoneAnimation{anim.mChannels[i]});
  }
}
SkeletalAnimation::SkeletalAnimation(FbxAnimStack* anim, std::vector<FbxNode*> const& bones, std::string const& nm):
  name{nm == "" ? anim->GetName() : nm},
  numFrames{0},
  numFPS{0},
  duration{0},
  numBoneAnims{0},
  boneAnims{}
{
  //set animation for which the bones will be evaluated 
  FbxScene* scene = anim->GetScene();
  scene->SetCurrentAnimationStack(anim);

  FbxTakeInfo* take = scene->GetTakeInfo(anim->GetName());
  numFPS = 30;
  numFrames = take->mLocalTimeSpan.GetDuration().GetFrameCount(FbxTime::eFrames30);
  duration = double(numFrames) / numFPS;

  for(FbxNode* const bone : bones) {
    boneAnims.push_back(BoneAnimation{*take, *bone});
  }
}

SkeletalAnimation::~SkeletalAnimation()
{}

Pose SkeletalAnimation::calculate_pose(float time) const { 
  Pose pose{};

  float currFrame = time * float(numFrames);
   
  for(BoneAnimation const& boneAnim : boneAnims) {
    Transformation boneTransform = boneAnim.calculate_transform(currFrame);

    pose.set_transform(boneAnim.get_name(), boneTransform);
  }  

  return pose;
}

double SkeletalAnimation::get_duration() const {
  return duration;
}

std::string const& SkeletalAnimation::get_name() const {
  return name;  
}

} // namespace gua