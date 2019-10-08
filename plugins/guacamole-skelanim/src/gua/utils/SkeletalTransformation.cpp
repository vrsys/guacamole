// class header
#include <gua/skelanim/utils/SkeletalTransformation.hpp>

// guacamole headers
#include <gua/utils/Logger.hpp>
#include <gua/skelanim/utils/SkeletalPose.hpp>
#include <gua/skelanim/utils/BonePose.hpp>
#include <gua/skelanim/utils/SkeletalAnimation.hpp>
#include <gua/skelanim/utils/Skeleton.hpp>

namespace gua
{
namespace SkeletalTransformation
{
std::vector<scm::math::mat4f>
blend_anims(Skeleton const& skeleton, unsigned start_node, float blend_factor, float time_normalized1, float time_normalized2, SkeletalAnimation const& anim_1, SkeletalAnimation const& anim_2)
{
    SkeletalPose pose1{anim_1.calculate_pose(time_normalized1)};
    SkeletalPose pose2{anim_2.calculate_pose(time_normalized2)};

    pose1.blend(pose2, blend_factor);

    return skeleton.accumulate_matrices(start_node, pose1);
}

std::vector<scm::math::mat4f> partial_blend(
    Skeleton const& skeleton, unsigned start_node, float time_normalized1, float time_normalized2, SkeletalAnimation const& anim_1, SkeletalAnimation const& anim_2, std::string const& split_node_name)
{
    SkeletalPose full_body{anim_1.calculate_pose(time_normalized1)};
    SkeletalPose upper_body{anim_2.calculate_pose(time_normalized2)};

    int split_index = skeleton.find(split_node_name);
    if(split_index >= 0)
    {
        full_body.partial_replace(upper_body, skeleton, split_index);
    }
    else
    {
        Logger::LOG_ERROR << "Bone '" << split_node_name << "' not found" << std::endl;
    }

    return skeleton.accumulate_matrices(start_node, full_body);
}

std::vector<scm::math::mat4f> from_anim(Skeleton const& skeleton, unsigned start_node, float time_normalized, SkeletalAnimation const& anim)
{
    return skeleton.accumulate_matrices(start_node, anim.calculate_pose(time_normalized));
}

std::vector<scm::math::mat4f> from_hierarchy(Skeleton const& skeleton, unsigned start_node) { return skeleton.accumulate_matrices(start_node, SkeletalPose{}); }

} // namespace SkeletalTransformation
} // namespace gua