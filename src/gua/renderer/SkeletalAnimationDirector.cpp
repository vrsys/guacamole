// class header
#include <gua/renderer/SkeletalAnimationDirector.hpp>


// guacamole headers
#include <gua/utils/Logger.hpp>

//external headers
#include <iostream>

namespace gua 
{
SkeletalAnimationDirector::SkeletalAnimationDirector(aiScene const* scene):
    num_bones_{0},
    has_anims_{scene->HasAnimations()},
    firstRun_{true},
    animations_{},
    currAnimation_{},
    animNum{0},
    timer_{} {
  timer_.start();
  add_hierarchy(scene);
}

void SkeletalAnimationDirector::add_hierarchy(aiScene const* scene) {
  root_ = SkeletalAnimationUtils::load_hierarchy(scene);
  SkeletalAnimationUtils::collect_bone_indices(bone_mapping_, root_);
  num_bones_ = bone_mapping_.size();
}

void SkeletalAnimationDirector::add_animations(aiScene const* scene) {
  std::vector<std::shared_ptr<SkeletalAnimation>> newAnims{SkeletalAnimationUtils::load_animations(scene)};

  animations_.reserve(animations_.size() + newAnims.size());
  std::move(newAnims.begin(), newAnims.end(), std::inserter(animations_, animations_.end()));
  newAnims.clear();

  has_anims_ = animations_.size() > 0;
  currAnimation_ = animations_[animations_.size()-1];
}

void SkeletalAnimationDirector::blend_pose(float timeInSeconds, std::shared_ptr<SkeletalAnimation> const& pAnim1, std::shared_ptr<SkeletalAnimation> const& pAnim2, std::vector<scm::math::mat4f>& transforms) {
 
  float animationTime1 = 0;
  float animationTime2 = 0;

  float timeInFrames = timeInSeconds * pAnim1->numFPS;
  animationTime1 = fmod(timeInFrames, (float)pAnim1->numFrames);

  timeInFrames = timeInSeconds * pAnim2->numFPS;
  animationTime2 = fmod(timeInFrames, (float)pAnim2->numFrames);

  scm::math::mat4f identity = scm::math::mat4f::identity();
  float blendFactor = 0.5;

  std::map<std::string, Transformation> transformStructs1{SkeletalAnimationUtils::calculate_transforms(animationTime1, pAnim1)};
  std::map<std::string, Transformation> transformStructs2{SkeletalAnimationUtils::calculate_transforms(animationTime2, pAnim2)};
  
  SkeletalAnimationUtils::blend(transformStructs1, transformStructs2, blendFactor);

  SkeletalAnimationUtils::accumulate_transforms(transforms, root_, transformStructs1, identity);
}

void SkeletalAnimationDirector::partial_blend(float timeInSeconds, std::shared_ptr<SkeletalAnimation> const& pAnim1, std::shared_ptr<SkeletalAnimation> const& pAnim2, std::string const& nodeName, std::vector<scm::math::mat4f>& transforms) {
  
  std::shared_ptr<Node> start{};
  start = SkeletalAnimationUtils::find_node(nodeName, root_->children[1]->children[0]->children[0]);
  
  if(!start) {
    Logger::LOG_WARNING << "node '"<< nodeName << "' not found" << std::endl; 
    return;
  }

  float animationTime1 = 0;
  float timeInFrames = timeInSeconds * pAnim1->numFPS;
  animationTime1 = fmod(timeInFrames, (float)pAnim1->numFrames);

  float animationTime2 = 0;
  timeInFrames = timeInSeconds * pAnim2->numFPS;
  animationTime2 = fmod(timeInFrames, (float)pAnim2->numFrames);

  // float animationTime3 = 0;
  // timeInFrames = timeInSeconds * animations_[2]->numFPS;
  // animationTime3 = fmod(timeInFrames, (float)animations_[2]->numFrames);

  scm::math::mat4f identity = scm::math::mat4f::identity();

  std::map<std::string, Transformation> full_body{SkeletalAnimationUtils::calculate_transforms(animationTime1, pAnim1)};
  std::map<std::string, Transformation> upper_body{SkeletalAnimationUtils::calculate_transforms(animationTime2, pAnim2)};
  // std::map<std::string, Transformation> arms{SkeletalAnimationUtils::calculate_transforms(animationTime3, animations_[2])};

  SkeletalAnimationUtils::partial_blend(full_body, upper_body, start);
  // SkeletalAnimationUtils::partial_blend(full_body, arms, SkeletalAnimationUtils::find_node("Shoulders", root_->children[1]->children[0]->children[0]));

  SkeletalAnimationUtils::accumulate_transforms(transforms, root_->children[1]->children[0]->children[0], full_body, identity);
}

std::vector<scm::math::mat4f> SkeletalAnimationDirector::get_bone_transforms()
{
  if(has_anims_ && timer_.get_elapsed() > currAnimation_->duration) {
    timer_.reset();
    currAnimation_ = animations_[animNum % animations_.size()];
    ++animNum;
  }

  //reserve vector for transforms
  std::vector<scm::math::mat4f> transforms{num_bones_, scm::math::mat4f::identity()};

  //blend two anims
  partial_blend(timer_.get_elapsed(), animations_[0], animations_[1], "Waist", transforms);
  //play all anims 
  // SkeletalAnimationUtils::calculate_pose(timer_.get_elapsed(), root_->children[1]->children[0]->children[0], currAnimation_, transforms);
  //blend two anims
  // blend_pose(timer_.get_elapsed(), animations_[0], animations_[1], transforms);
  
  return transforms;
}

int SkeletalAnimationDirector::getBoneID(std::string const& name) {
  return bone_mapping_.at(name);
}

bool SkeletalAnimationDirector::has_anims() const {
  return has_anims_;
}

} // namespace gua