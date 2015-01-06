// class header
#include <gua/renderer/SkeletalAnimationUtils.hpp>

//external headers
#include <iostream>

namespace gua {

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
  root->set_properties(bone_info);
  return root;
}

std::vector<std::shared_ptr<SkeletalAnimation>> SkeletalAnimationUtils::load_animations(aiScene const* scene) {
  std::vector<std::shared_ptr<SkeletalAnimation>> animations{};
  if(!scene->HasAnimations()) Logger::LOG_WARNING << "scene contains no animations!" << std::endl;
 
  for(uint i = 0; i < scene->mNumAnimations; ++i) {
    animations.push_back(std::make_shared<SkeletalAnimation>(scene->mAnimations[i]));
  }

  return animations;
}

void SkeletalAnimationUtils::calculate_matrices(float timeInSeconds, std::shared_ptr<Node> const& root, std::shared_ptr<SkeletalAnimation> const& pAnim, std::vector<scm::math::mat4f>& transforms) {
 
  float timeNormalized = 0;
  Pose pose{};

  if(pAnim) {
    timeNormalized = timeInSeconds / pAnim->get_duration();
    timeNormalized = scm::math::fract(timeNormalized);

    pose = pAnim->calculate_pose(timeNormalized);
  }

  scm::math::mat4f identity = scm::math::mat4f::identity();
  root->accumulate_matrices(transforms, pose, identity);
}

////////////////////////////////////////////////////////////////////////////////
float Blend::cos(float x) {
  x *= scm::math::pi_f;
  return 0.5f * (1 - scm::math::cos(x));
}

float Blend::linear(float x) {
  //values from 0 to 2 accepted
  x = fmod(x, 2.0f);
  x = 1 - scm::math::abs(x - 1);
  return x;
}

float Blend::smoothstep(float x) {
  x = fmod(x, 2.0f);
  x = 1 - scm::math::abs(x - 1);
  
  return 3 * x * x - 2 * x * x * x;
}

float Blend::swap(float x)
{
  x = fmod(x, 2.0f);
  return (x > 0.5) ? 1 : 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Node::Node(aiNode const* node):
  index{-1},
  name{node->mName.C_Str()},
  parentName{node->mParent != NULL ? node->mParent->mName.C_Str() : "none"},
  numChildren{node->mNumChildren},
  transformation{ai_to_gua(node->mTransformation)},
  offsetMatrix{scm::math::mat4f::identity()}
{
  for(unsigned i = 0; i < node->mNumChildren; ++i) {
    std::shared_ptr<Node> child = std::make_shared<Node>(node->mChildren[i]);
    children.push_back(child);
  }
}

Node::~Node()
{}

std::shared_ptr<Node> Node::find(std::string const& name) const {

  for(std::shared_ptr<Node> const& child : children) {
    if(child->name == name) return child;

    std::shared_ptr<Node> found = child->find(name);
    if(found) return found;
  }

  return nullptr;
}

void Node::collect_indices(std::map<std::string, int>& ids) const {
  ids[name] = index;

  for(std::shared_ptr<Node> const& child : children) {
    child->collect_indices(ids);
  }
}

void Node::set_properties(std::map<std::string, std::pair<uint, scm::math::mat4f>> const& infos) {
  if(infos.find(name) != infos.end()) {
    offsetMatrix = infos.at(name).second;
    index = infos.at(name).first;
  }

  for(std::shared_ptr<Node>& child : children) {
    child->set_properties(infos);
  }
}

void Node::accumulate_matrices(std::vector<scm::math::mat4f>& transformMat4s, Pose const& pose, scm::math::mat4f const& parentTransform) const {
  scm::math::mat4f nodeTransformation{transformation};

  if(pose.contains(name)) { 
    nodeTransformation = pose.get_transform(name).to_matrix();  
  }
  
  scm::math::mat4f finalTransformation = parentTransform * nodeTransformation;

  //update transform if bone is mapped
  if (index >= 0) {
    transformMat4s[index] = finalTransformation * offsetMatrix;
  }
  
  for (std::shared_ptr<Node> const& child : children) {
     child->accumulate_matrices(transformMat4s, pose, finalTransformation);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Transformation::Transformation():
  scaling{1.0f},
  rotation{scm::math::quatf::identity()},
  translation{0.0f}
{}

Transformation::Transformation(scm::math::vec3 const& scale, scm::math::quatf const& rotate, scm::math::vec3 const& translate):
  scaling{scale},
  rotation{rotate},
  translation{translate}
{}

Transformation::~Transformation()
{}

scm::math::mat4f Transformation::to_matrix() const {
  return scm::math::make_translation(translation) * rotation.to_matrix() * scm::math::make_scale(scaling);
}

Transformation Transformation::blend(Transformation const& t, float const factor) const {
  return Transformation{scaling * (1 - factor) + t.scaling * factor, slerp(rotation, t.rotation, factor), translation * (1 - factor) + t.translation * factor};
}

Transformation Transformation::operator+(Transformation const& t) const {
  return Transformation{scaling + t.scaling, scm::math::normalize(t.rotation * rotation), translation + t.translation};
}
Transformation& Transformation::operator+=(Transformation const& t) {
  *this = *this + t;
  return *this;
}

Transformation Transformation::operator*(float const factor) const {
  return Transformation{scaling * factor, slerp(scm::math::quatf::identity(), rotation, factor), translation * factor};
}
Transformation& Transformation::operator*=(float const f) {
  *this = *this * f;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
BoneAnimation::BoneAnimation():
  name{"default"},
  scalingKeys{},
  rotationKeys{},
  translationKeys{}
{}

BoneAnimation::~BoneAnimation()
{}

BoneAnimation::BoneAnimation(aiNodeAnim* anim):
  name{anim->mNodeName.C_Str()}
 {

  for(unsigned i = 0; i < anim->mNumScalingKeys; ++i) {
    scalingKeys.push_back(Key<scm::math::vec3>{anim->mScalingKeys[i].mTime, ai_to_gua(anim->mScalingKeys[i].mValue)});
  }
  for(unsigned i = 0; i < anim->mNumRotationKeys; ++i) {
    rotationKeys.push_back(Key<scm::math::quatf>{anim->mRotationKeys[i].mTime, ai_to_gua(anim->mRotationKeys[i].mValue)});
  }
  for(unsigned i = 0; i < anim->mNumPositionKeys; ++i) {
    translationKeys.push_back(Key<scm::math::vec3>{anim->mPositionKeys[i].mTime, ai_to_gua(anim->mPositionKeys[i].mValue)});
  }
}

Transformation BoneAnimation::calculate_transform(float time) const {
  return Transformation{calculate_value(time, scalingKeys), calculate_value(time, rotationKeys), calculate_value(time, translationKeys)};
}

std::string const& BoneAnimation::get_name() const {
  return name;
}

scm::math::vec3 BoneAnimation::interpolate(scm::math::vec3 val1, scm::math::vec3 val2, float factor) const {
  return val1 * (1 - factor) + val2 * factor;
}

scm::math::quatf BoneAnimation::interpolate(scm::math::quatf val1, scm::math::quatf val2, float factor) const {
  return normalize(slerp(val1, val2, factor));
}

template<class T> 
uint BoneAnimation::find_key(float animationTime, std::vector<Key<T>> keys) const {    
  if(keys.size() < 1) {
    Logger::LOG_ERROR << "no keys" << std::endl;
    assert(false);
  } 

  for(uint i = 0 ; i < keys.size() - 1 ; i++) {
    if(animationTime < (float)keys[i + 1].time) {
      return i;
    }
  }

  Logger::LOG_ERROR << "no key found" << std::endl;
  assert(false);

  return 0;
}

template<class T> 
T BoneAnimation::calculate_value(float time, std::vector<Key<T>> keys) const {
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
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
SkeletalAnimation::SkeletalAnimation():
  name{"default"},
  numFrames{0},
  numFPS{0},
  duration{0},
  numBoneAnims{0},
  boneAnims{}
{}

SkeletalAnimation::SkeletalAnimation(aiAnimation* anim):
  name{anim->mName.C_Str()},
  numFrames{unsigned(anim->mDuration)},
  numFPS{anim->mTicksPerSecond > 0 ? anim->mTicksPerSecond : 25},
  duration{double(numFrames) / numFPS},
  numBoneAnims{anim->mNumChannels},
  boneAnims{}
{
  for(unsigned i = 0; i < numBoneAnims; ++i) {
    boneAnims.push_back(BoneAnimation{anim->mChannels[i]});
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
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Pose::Pose():
  transforms{}
{}

Pose::~Pose()
{}

bool Pose::contains(std::string const& name ) const {
  return transforms.find(name) != transforms.end();
}

Transformation const& Pose::get_transform(std::string const& name) const{
  try {
    return transforms.at(name);
  }
  catch(std::exception const& e) {
    Logger::LOG_ERROR << "bone '" << name << "' not contained in pose" << std::endl;
    return transforms.begin()->second;
  }
}

void Pose::set_transform(std::string const& name, Transformation const& value) {
  transforms[name] = value;
}

void Pose::blend(Pose const& pose2, float blendFactor) {
  for_each(pose2.cbegin(), pose2.cend(), [this, &blendFactor](std::pair<std::string, Transformation> const& p) {
    if(contains(p.first)) {
      set_transform(p.first, get_transform(p.first).blend(p.second, blendFactor));
    }
    else {
      set_transform(p.first, p.second);
    }
  });
  // *this = *this * (1 - blendFactor) + pose2 * blendFactor;
}

Pose& Pose::operator+=(Pose const& pose2) {
  for_each(pose2.cbegin(), pose2.cend(), [this](std::pair<std::string, Transformation> const& p) {
    if(contains(p.first)) {
      set_transform(p.first, get_transform(p.first) + p.second);
    }
    else {
      set_transform(p.first, p.second);
    }
  });
  return *this;
}
Pose Pose::operator+(Pose const& p2) const {
  Pose temp{*this};
  temp += p2;
  return temp;
}

Pose& Pose::operator*=(float const factor) {
  for(auto& p : transforms)
  {
    p.second *=factor;
  }
  return *this;
}
Pose Pose::operator*(float const factor) const {
  Pose temp{*this};
  temp *= factor;
  return temp;
}

void Pose::partial_replace(Pose const& pose2, std::shared_ptr<Node> const& pNode) {
  if(pose2.contains(pNode->name)) {
    set_transform(pNode->name, pose2.get_transform(pNode->name));
  }

  for(std::shared_ptr<Node>& child : pNode->children) {
    partial_replace(pose2, child);
  }
}

inline std::map<std::string, Transformation>::iterator Pose::begin() {
  return transforms.begin();
}   

inline std::map<std::string, Transformation>::const_iterator Pose::cbegin() const {
  return transforms.cbegin();
} 

inline std::map<std::string, Transformation>::iterator Pose::end() {
  return transforms.end();
} 
inline std::map<std::string, Transformation>::const_iterator Pose::cend() const {
  return transforms.cend();
} 

} // namespace gua