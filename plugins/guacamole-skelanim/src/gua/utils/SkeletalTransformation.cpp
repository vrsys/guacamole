// class header
#include <gua/utils/SkeletalTransformation.hpp>
#include <gua/utils/SkeletalPose.hpp>


// guacamole headers
#include <gua/utils/Logger.hpp>
#include <gua/utils/Blending.hpp>

//external headers
#include <iostream>
#include <queue>
#include <fbxsdk.h>


namespace gua 
{

namespace SkeletalTransformation {

void blend_anims(std::shared_ptr<Bone> const& anim_start_node_, float blend_factor, float timeInSeconds1, float timeInSeconds2, SkeletalAnimation const& pAnim1, SkeletalAnimation const& pAnim2, std::vector<scm::math::mat4f>& transforms) {
  float time_normalized1 = timeInSeconds1 / pAnim1.get_duration();
  time_normalized1 = scm::math::fract(time_normalized1);

  float time_normalized2 = timeInSeconds2 / pAnim2.get_duration();
  time_normalized2 = scm::math::fract(time_normalized2);

  scm::math::mat4f identity = scm::math::mat4f::identity();

  SkeletalPose pose1{pAnim1.calculate_pose(time_normalized1)};
  SkeletalPose pose2{pAnim2.calculate_pose(time_normalized2)};
  
  pose1.blend(pose2, blend_factor);

  anim_start_node_->accumulate_matrices(transforms, pose1, identity);
}

void partial_blend(std::shared_ptr<Bone> const& anim_start_node_, float timeInSeconds, SkeletalAnimation const& pAnim1, SkeletalAnimation const& pAnim2, std::string const& nodeName, std::vector<scm::math::mat4f>& transforms) {

  float time_normalized1 = timeInSeconds / pAnim1.get_duration();
  time_normalized1 = scm::math::fract(time_normalized1);

  float time_normalized2 = timeInSeconds / pAnim2.get_duration();
  time_normalized2 = scm::math::fract(time_normalized2);

  scm::math::mat4f identity = scm::math::mat4f::identity();

  SkeletalPose full_body{pAnim1.calculate_pose(time_normalized1)};
  SkeletalPose upper_body{pAnim2.calculate_pose(time_normalized2)};

  full_body.partial_replace(upper_body, anim_start_node_);

  anim_start_node_->accumulate_matrices(transforms, full_body, identity);
}

void from_anim(std::shared_ptr<Bone> const& anim_start_node_, float timeInSeconds, SkeletalAnimation const& pAnim, std::vector<scm::math::mat4f>& transforms) {
 
  float time_normalized = 0;
  SkeletalPose pose{};

  time_normalized = timeInSeconds / pAnim.get_duration();
  time_normalized = scm::math::fract(time_normalized);

  pose = pAnim.calculate_pose(time_normalized);

  scm::math::mat4f identity = scm::math::mat4f::identity();
  anim_start_node_->accumulate_matrices(transforms, pose, identity);
}

void from_hierarchy(std::shared_ptr<Bone> const& anim_start_node_, std::vector<scm::math::mat4f>& transforms) {

  SkeletalPose pose{};

  scm::math::mat4f identity = scm::math::mat4f::identity();
  anim_start_node_->accumulate_matrices(transforms, pose, identity);
}

}
} // namespace gua