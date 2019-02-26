// class header
#include <gua/skelanim/utils/BoneAnimation.hpp>

// guacamole headers
#include <gua/utils/Logger.hpp>
#include <gua/skelanim/utils/BonePose.hpp>
#include <gua/utils/ToGua.hpp>

// external headers
#ifdef GUACAMOLE_FBX
#include <fbxsdk.h>
#endif
#include <assimp/scene.h> // for ainodeanim

namespace gua
{
BoneAnimation::BoneAnimation() : name("default"), scalingKeys(), rotationKeys(), translationKeys() {}

#ifdef GUACAMOLE_FBX
BoneAnimation::BoneAnimation(FbxTakeInfo const& take, FbxNode& Bone) : name(Bone.GetName()), scalingKeys(), rotationKeys(), translationKeys()
{
    FbxTime start = take.mLocalTimeSpan.GetStart();
    FbxTime end = take.mLocalTimeSpan.GetStop();

    FbxTime time;
    scm::math::mat4f transform_matrix;
    unsigned start_frame = start.GetFrameCount(FbxTime::eFrames30);
    for(unsigned i = start_frame; i <= end.GetFrameCount(FbxTime::eFrames30); ++i)
    {
        time.SetFrame(i, FbxTime::eFrames30);
        // get complete transformation at current time
        transform_matrix = to_gua::mat4f(Bone.EvaluateLocalTransform(time));

        scalingKeys.push_back(Keyframe<scm::math::vec3f>{double(i - start_frame), to_gua::vec3f(Bone.EvaluateLocalScaling(time))});
        // get rotation from transformation matrix
        rotationKeys.push_back(Keyframe<scm::math::quatf>{double(i - start_frame), scm::math::quatf::from_matrix(transform_matrix)});
        // and translation
        translationKeys.push_back(Keyframe<scm::math::vec3f>{double(i - start_frame), scm::math::vec3f{transform_matrix[12], transform_matrix[13], transform_matrix[14]}});
    }
}
#endif

BoneAnimation::~BoneAnimation() {}

BoneAnimation::BoneAnimation(aiNodeAnim* anim) : name(anim->mNodeName.C_Str()), scalingKeys(), rotationKeys(), translationKeys()
{
    // mTime is double but stored in frames
    for(unsigned i = 0; i < anim->mNumScalingKeys; ++i)
    {
        scalingKeys.push_back(Keyframe<scm::math::vec3f>{anim->mScalingKeys[i].mTime, to_gua::vec3f(anim->mScalingKeys[i].mValue)});
    }
    for(unsigned i = 0; i < anim->mNumRotationKeys; ++i)
    {
        rotationKeys.push_back(Keyframe<scm::math::quatf>{anim->mRotationKeys[i].mTime, to_gua::quatf(anim->mRotationKeys[i].mValue)});
    }
    for(unsigned i = 0; i < anim->mNumPositionKeys; ++i)
    {
        translationKeys.push_back(Keyframe<scm::math::vec3f>{anim->mPositionKeys[i].mTime, to_gua::vec3f(anim->mPositionKeys[i].mValue)});
    }
}

BonePose BoneAnimation::calculate_pose(float time) const { return BonePose{calculate_value(time, scalingKeys), calculate_value(time, rotationKeys), calculate_value(time, translationKeys)}; }

std::string const& BoneAnimation::get_name() const { return name; }

scm::math::vec3f BoneAnimation::interpolate(scm::math::vec3f val1, scm::math::vec3f val2, float factor) const { return val1 * (1 - factor) + val2 * factor; }

scm::math::quatf BoneAnimation::interpolate(scm::math::quatf val1, scm::math::quatf val2, float factor) const { return normalize(slerp(val1, val2, factor)); }

template <class T>
int BoneAnimation::find_key(float animationTime, std::vector<Keyframe<T>> keys) const
{
    if(keys.size() < 1)
    {
        Logger::LOG_ERROR << "no keys" << std::endl;
        assert(false);
    }

    for(int i = -1; i < int(keys.size() - 1); ++i)
    {
        if(animationTime < (float)keys[i + 1].time)
        {
            return i;
        }
    }

    return keys.size() - 1;
}

template <class T>
T BoneAnimation::calculate_value(float time, std::vector<Keyframe<T>> keys) const
{
    if(keys.size() == 1)
    {
        return keys[0].value;
    }

    int last_index = find_key(time, keys);
    unsigned next_index = (last_index + 1);

    if(last_index == -1)
    {
        return keys[0].value;
    }
    else if(last_index == int(keys.size()) - 1)
    {
        return keys.back().value;
    }

    float deltaTime = (float)(keys[next_index].time - keys[last_index].time);
    float factor = (time - (float)keys[last_index].time) / deltaTime;
    // assert(factor >= 0.0f && factor <= 1.0f);
    T const& key1 = keys[last_index].value;
    T const& key2 = keys[next_index].value;

    return interpolate(key1, key2, factor);
}

} // namespace gua