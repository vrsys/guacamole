// class header
#include <gua/utils/BoneAnimation.hpp>
//external headers
#include <iostream>
#include <queue>

namespace gua {

BoneAnimation::BoneAnimation():
  name{"default"},
  scalingKeys{},
  rotationKeys{},
  translationKeys{}
{}
BoneAnimation::BoneAnimation(FbxTakeInfo const& take, FbxNode& Bone):
  name{Bone.GetName()},
  scalingKeys{},
  rotationKeys{},
  translationKeys{}
{
  FbxTime start = take.mLocalTimeSpan.GetStart();
  FbxTime end = take.mLocalTimeSpan.GetStop();

  FbxTime time;
  scm::math::mat4f transform_matrix;
  unsigned start_frame = start.GetFrameCount(FbxTime::eFrames30);
  for(unsigned i = start_frame; i <= end.GetFrameCount(FbxTime::eFrames30); ++i) {
    time.SetFrame(i, FbxTime::eFrames30);
    //get complete transformation at current time
    transform_matrix = to_gua::mat4(Bone.EvaluateLocalTransform(time));
    
    scalingKeys.push_back(Keyframe<scm::math::vec3f>{double(i - start_frame), to_gua::vec3(Bone.EvaluateLocalScaling(time))});
    //get rotation from transformation matrix
    rotationKeys.push_back(Keyframe<scm::math::quatf>{double(i - start_frame), scm::math::quatf::from_matrix(transform_matrix)});
    //and translation
    translationKeys.push_back(Keyframe<scm::math::vec3f>{double(i - start_frame), scm::math::vec3f{transform_matrix[12], transform_matrix[13], transform_matrix[14]}});
  }
}

BoneAnimation::~BoneAnimation()
{}

BoneAnimation::BoneAnimation(aiNodeAnim* anim):
  name{anim->mNodeName.C_Str()},
  scalingKeys{},
  rotationKeys{},
  translationKeys{}
 {
  //mTime is double but stored in frames
  for(unsigned i = 0; i < anim->mNumScalingKeys; ++i) {
    scalingKeys.push_back(Keyframe<scm::math::vec3f>{anim->mScalingKeys[i].mTime, to_gua::vec3(anim->mScalingKeys[i].mValue)});
  }
  for(unsigned i = 0; i < anim->mNumRotationKeys; ++i) {
    rotationKeys.push_back(Keyframe<scm::math::quatf>{anim->mRotationKeys[i].mTime, to_gua::quat(anim->mRotationKeys[i].mValue)});
  }
  for(unsigned i = 0; i < anim->mNumPositionKeys; ++i) {
    translationKeys.push_back(Keyframe<scm::math::vec3f>{anim->mPositionKeys[i].mTime, to_gua::vec3(anim->mPositionKeys[i].mValue)});
  }
}

Transformation BoneAnimation::calculate_transform(float time) const {
  return Transformation{calculate_value(time, scalingKeys), calculate_value(time, rotationKeys), calculate_value(time, translationKeys)};
}

std::string const& BoneAnimation::get_name() const {
  return name;
}

scm::math::vec3f BoneAnimation::interpolate(scm::math::vec3f val1, scm::math::vec3f val2, float factor) const {
  return val1 * (1 - factor) + val2 * factor;
}

scm::math::quatf BoneAnimation::interpolate(scm::math::quatf val1, scm::math::quatf val2, float factor) const {
  return normalize(slerp(val1, val2, factor));
}

template<class T> 
uint BoneAnimation::find_key(float animationTime, std::vector<Keyframe<T>> keys) const {    
  if(keys.size() < 1) {
    Logger::LOG_ERROR << "no keys" << std::endl;
    assert(false);
  } 

  for(uint i = 0 ; i < keys.size() - 1 ; i++) {
    if(animationTime < (float)keys[i + 1].time) {
      return i;
    }
  }

  Logger::LOG_ERROR << "no key found at frame " << animationTime << std::endl;
  assert(false);

  return 0;
}

template<class T> 
T BoneAnimation::calculate_value(float time, std::vector<Keyframe<T>> keys) const {
  if(keys.size() == 1) {
     return keys[0].value;
  }

  uint lastIndex = find_key(time, keys);
  uint nextIndex = (lastIndex + 1);

  if(nextIndex > keys.size()) {
    Logger::LOG_ERROR << "frame out of range" << std::endl;
    assert(false);
  }

  float deltaTime = (float)(keys[nextIndex].time - keys[lastIndex].time);
  float factor = (time - (float)keys[lastIndex].time) / deltaTime;
  //assert(factor >= 0.0f && factor <= 1.0f);
  T const& key1 = keys[lastIndex].value;
  T const& key2 = keys[nextIndex].value;

  return interpolate(key1, key2, factor);
}

} // namespace gua