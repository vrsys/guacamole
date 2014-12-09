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

uint SkeletalAnimationDirector::getBoneID(std::string const& name) {
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
  
  auto iter = transformStructs2.begin();
  for_each(transformStructs1.begin(), transformStructs1.end(), [&transformStructs2, &blendFactor, &iter](std::pair<std::string, Transformation> p) {
    iter->second = p.second.blend(iter->second, blendFactor);
    ++iter;
  });
  SkeletalAnimationUtils::accumulate_transforms(transforms, root_, transformStructs2, identity);
}

void SkeletalAnimationDirector::updateBoneTransforms(RenderContext const& ctx)
{
  if(!hasAnims_ && !firstRun_) return;
  if(!hasAnims_) firstRun_ = false;

  if(!bone_transforms_block_) {
    //TODO one transform block per context
    bone_transforms_block_ = std::make_shared<BoneTransformUniformBlock>(ctx.render_device);
  }

  if(timer_.get_elapsed() > currAnimation_->duration) {
    timer_.reset();
    currAnimation_ = animations_[animNum % animations_.size()];
    ++animNum;
  }

  //reserve vector for transforms
  std::vector<scm::math::mat4f> transforms{num_bones_, scm::math::mat4f::identity()};

  //SkeletalAnimationUtils::calculate_pose(timer_.get_elapsed(), root_->children[1]->children[0]->children[0], currAnimation_, transforms);
  calculate_pose(timer_.get_elapsed(), animations_[0], animations_[1], transforms);

  bone_transforms_block_->update(ctx.render_context, transforms);
  ctx.render_context->bind_uniform_buffer( bone_transforms_block_->block().block_buffer(), 1 );
}

} // namespace gua