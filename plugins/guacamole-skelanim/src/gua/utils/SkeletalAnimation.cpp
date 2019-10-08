// class header
#include <gua/skelanim/utils/SkeletalAnimation.hpp>

// guacamole headers
#include <gua/skelanim/utils/SkeletalPose.hpp>
#include <gua/skelanim/utils/BonePose.hpp>
#include <gua/skelanim/utils/BoneAnimation.hpp>

// external headers
#ifdef GUACAMOLE_FBX
#include <fbxsdk.h>
#endif
#include <assimp/scene.h>

namespace gua
{
SkeletalAnimation::SkeletalAnimation() : name("default"), numFrames(0), numFPS(0), duration(0), numBoneAnims(0), boneAnims() {}

SkeletalAnimation::SkeletalAnimation(aiAnimation const& anim, std::string const& nm)
    : name(nm == "" ? anim.mName.C_Str() : nm), numFrames(unsigned(anim.mDuration)), numFPS(anim.mTicksPerSecond > 0 ? anim.mTicksPerSecond : 25), duration(double(numFrames) / numFPS),
      numBoneAnims(anim.mNumChannels), boneAnims()
{
    for(unsigned i = 0; i < numBoneAnims; ++i)
    {
        boneAnims.push_back(BoneAnimation{anim.mChannels[i]});
    }
}

#ifdef GUACAMOLE_FBX
SkeletalAnimation::SkeletalAnimation(FbxAnimStack* anim, std::vector<FbxNode*> const& bones, std::string const& nm)
    : name(nm == "" ? anim->GetName() : nm), numFrames(0), numFPS(0), duration(0), numBoneAnims(0), boneAnims()
{
    // set animation for which the bones will be evaluated
    FbxScene* scene = anim->GetScene();
    scene->SetCurrentAnimationStack(anim);

    FbxTakeInfo* take = scene->GetTakeInfo(anim->GetName());
    numFPS = 30;
    numFrames = take->mLocalTimeSpan.GetDuration().GetFrameCount(FbxTime::eFrames30);
    duration = double(numFrames) / numFPS;
    for(FbxNode* const bone : bones)
    {
        boneAnims.push_back(BoneAnimation{*take, *bone});
    }
}
#endif

SkeletalAnimation::~SkeletalAnimation() {}

SkeletalPose SkeletalAnimation::calculate_pose(float time) const
{
    SkeletalPose pose{};

    float currFrame = time * numFrames;
    for(BoneAnimation const& boneAnim : boneAnims)
    {
        BonePose boneTransform = boneAnim.calculate_pose(currFrame);

        pose.set_transform(boneAnim.get_name(), boneTransform);
    }

    return pose;
}

double SkeletalAnimation::get_duration() const { return duration; }

std::string const& SkeletalAnimation::get_name() const { return name; }

} // namespace gua