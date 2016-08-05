// class header
#include <gua/utils/SkeletalTransformation.hpp>

// guacamole headers
#include <gua/utils/Logger.hpp>
#include <gua/utils/SkeletalPose.hpp>
#include <gua/utils/BonePose.hpp>
#include <gua/utils/SkeletalAnimation.hpp>
#include <gua/utils/Bone.hpp>
#include <gua/utils/Skeleton.hpp>

namespace gua {

namespace SkeletalTransformation {

void blend_anims(std::shared_ptr<Bone> const& start_node,
                 float blend_factor,
                 float time_normalized1,
                 float time_normalized2,
                 SkeletalAnimation const& anim_1,
                 SkeletalAnimation const& anim_2,
                 std::vector<scm::math::mat4f>& transforms) {

  SkeletalPose pose1 { anim_1.calculate_pose(time_normalized1) }
  ;
  SkeletalPose pose2 { anim_2.calculate_pose(time_normalized2) }
  ;

  pose1.blend(pose2, blend_factor);

  start_node->accumulate_matrices(transforms, pose1);
}

void partial_blend(std::shared_ptr<Bone> const& start_node,
                   float time_normalized1,
                   float time_normalized2,
                   SkeletalAnimation const& anim_1,
                   SkeletalAnimation const& anim_2,
                   std::string const& split_node_name,
                   std::vector<scm::math::mat4f>& transforms) {

  SkeletalPose full_body { anim_1.calculate_pose(time_normalized1) }
  ;
  SkeletalPose upper_body { anim_2.calculate_pose(time_normalized2) }
  ;

  std::shared_ptr<Bone> const& split_node {
    start_node->find(split_node_name)
  }
  ;
  if (split_node) {
    full_body.partial_replace(upper_body, split_node.get());
  } else {
    Logger::LOG_ERROR << "Bone '" << split_node_name << "' not found"
                      << std::endl;
  }

  start_node->accumulate_matrices(transforms, full_body);
}

void from_anim(std::shared_ptr<Bone> const& start_node,
               float time_normalized,
               SkeletalAnimation const& anim,
               std::vector<scm::math::mat4f>& transforms) {

  start_node->accumulate_matrices(transforms,
                                        anim.calculate_pose(time_normalized));
}

void from_hierarchy(std::shared_ptr<Bone> const& start_node,
                    std::vector<scm::math::mat4f>& transforms) {

  start_node->accumulate_matrices(transforms,
                                        SkeletalPose {
  });
}
/////////////////////////////////////////////////////////////////////////////////////////
void blend_anims(Skeleton const& skeleton, unsigned start_node,
                 float blend_factor,
                 float time_normalized1,
                 float time_normalized2,
                 SkeletalAnimation const& anim_1,
                 SkeletalAnimation const& anim_2,
                 std::vector<scm::math::mat4f>& transforms) {

  SkeletalPose pose1 { anim_1.calculate_pose(time_normalized1) }
  ;
  SkeletalPose pose2 { anim_2.calculate_pose(time_normalized2) }
  ;

  pose1.blend(pose2, blend_factor);

  skeleton.accumulate_matrices(start_node, transforms, pose1);
}

void partial_blend(Skeleton const& skeleton, unsigned start_node,
                   float time_normalized1,
                   float time_normalized2,
                   SkeletalAnimation const& anim_1,
                   SkeletalAnimation const& anim_2,
                   std::string const& split_node_name,
                   std::vector<scm::math::mat4f>& transforms) {

  SkeletalPose full_body { anim_1.calculate_pose(time_normalized1) }
  ;
  SkeletalPose upper_body { anim_2.calculate_pose(time_normalized2) }
  ;

  Bone const* split_node {
    skeleton.find(split_node_name)
  }
  ;
  if (split_node) {
    full_body.partial_replace(upper_body, skeleton, split_node->index);
  } else {
    Logger::LOG_ERROR << "Bone '" << split_node_name << "' not found"
                      << std::endl;
  }

  skeleton.accumulate_matrices(start_node, transforms, full_body);
}

void from_anim(Skeleton const& skeleton, unsigned start_node,
               float time_normalized,
               SkeletalAnimation const& anim,
               std::vector<scm::math::mat4f>& transforms) {

  skeleton.accumulate_matrices(start_node, transforms,
                                        anim.calculate_pose(time_normalized));
}

void from_hierarchy(Skeleton const& skeleton, unsigned start_node,
                    std::vector<scm::math::mat4f>& transforms) {

  skeleton.accumulate_matrices(start_node, transforms,
                                        SkeletalPose {
  });
}

}
}  // namespace gua