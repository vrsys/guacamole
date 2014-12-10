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
    bone_transforms_block_{nullptr},
    hasAnims_{scene->HasAnimations()},
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

  if(animations_.size() > 0) hasAnims_ = true;
  currAnimation_ = animations_[animations_.size()-1];
}

int SkeletalAnimationDirector::getBoneID(std::string const& name) {
  return bone_mapping_.at(name);
}

void SkeletalAnimationDirector::calculate_pose(float timeInSeconds, std::shared_ptr<SkeletalAnimation> const& pAnim1, std::shared_ptr<SkeletalAnimation> const& pAnim2, std::vector<scm::math::mat4f>& transforms) {
 
  float animationTime1 = 0;
  float animationTime2 = 0;

  float timeInFrames = timeInSeconds * pAnim1->numFPS;
  animationTime1 = fmod(timeInFrames, (float)pAnim1->numFrames);

  timeInFrames = timeInSeconds * pAnim2->numFPS;
  animationTime2 = fmod(timeInFrames, (float)pAnim2->numFrames);

  scm::math::mat4f identity = scm::math::mat4f::identity();
  float blendFactor = 0.3;

  std::map<std::string, Transformation> transformStructs1{SkeletalAnimationUtils::calculate_transforms(animationTime1, pAnim1)};
  std::map<std::string, Transformation> transformStructs2{SkeletalAnimationUtils::calculate_transforms(animationTime2, pAnim2)};
  
  SkeletalAnimationUtils::blend(transformStructs1, transformStructs2, blendFactor);

  SkeletalAnimationUtils::accumulate_transforms(transforms, root_, transformStructs1, identity);
}

void SkeletalAnimationDirector::partial_blend(float timeInSeconds, std::shared_ptr<SkeletalAnimation> const& pAnim1, std::shared_ptr<SkeletalAnimation> const& pAnim2, std::shared_ptr<Node> const& start, std::vector<scm::math::mat4f>& transforms) {
 
  float animationTime1 = 0;
  float animationTime2 = 0;

  float timeInFrames = timeInSeconds * pAnim1->numFPS;
  animationTime1 = fmod(timeInFrames, (float)pAnim1->numFrames);

  timeInFrames = timeInSeconds * pAnim2->numFPS;
  animationTime2 = fmod(timeInFrames, (float)pAnim2->numFrames);

  scm::math::mat4f identity = scm::math::mat4f::identity();

  std::map<std::string, Transformation> transformStructs1{SkeletalAnimationUtils::calculate_transforms(animationTime1, pAnim1)};
  std::map<std::string, Transformation> transformStructs2{SkeletalAnimationUtils::calculate_transforms(animationTime2, pAnim2)};

  SkeletalAnimationUtils::accumulate_transforms(transforms, root_, transformStructs1, identity);

  std::shared_ptr<Node> parentNode = SkeletalAnimationUtils::find_node(start->parentName, root_);
  scm::math::mat4f parentTransform = parentNode->currentTransformation;
  
  SkeletalAnimationUtils::accumulate_transforms(transforms, start, transformStructs2, parentTransform);
}

void SkeletalAnimationDirector::updateBoneTransforms(RenderContext const& ctx)
{
  if(!hasAnims_ && !firstRun_) return;
  if(!hasAnims_) firstRun_ = false;

  if(!bone_transforms_block_) {
    //TODO one transform block per context
    bone_transforms_block_ = std::make_shared<BoneTransformUniformBlock>(ctx.render_device);
  }

  if(hasAnims_ && timer_.get_elapsed() > currAnimation_->duration) {
    timer_.reset();
    currAnimation_ = animations_[animNum % animations_.size()];
    ++animNum;
  }

  //reserve vector for transforms
  std::vector<scm::math::mat4f> transforms{num_bones_, scm::math::mat4f::identity()};

  std::shared_ptr<Node> start{};
  start = SkeletalAnimationUtils::find_node("Waist", root_->children[1]->children[0]->children[0]);
  if(start) partial_blend(timer_.get_elapsed(), animations_[0], animations_[1], start, transforms);
  else {
    Logger::LOG_WARNING << "node not found" << std::endl;
    SkeletalAnimationUtils::calculate_pose(timer_.get_elapsed(), root_->children[1]->children[0]->children[0], currAnimation_, transforms);
  } 
  // calculate_pose(timer_.get_elapsed(), animations_[0], animations_[1], transforms);

  bone_transforms_block_->update(ctx.render_context, transforms);
  ctx.render_context->bind_uniform_buffer( bone_transforms_block_->block().block_buffer(), 1 );
}

} // namespace gua