// class header
#include <gua/skelanim/utils/SkeletalPose.hpp>

// guacamole headers
#include <gua/skelanim/utils/BonePose.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/skelanim/utils/Skeleton.hpp>

namespace gua
{
const BonePose SkeletalPose::blank_pose = BonePose{};

SkeletalPose::SkeletalPose() : transforms{} {}

bool SkeletalPose::contains(std::string const& name) const { return transforms.find(name) != transforms.end(); }

BonePose const& SkeletalPose::get_transform(std::string const& name) const
{
    if(contains(name))
    {
        return transforms.at(name);
    }
    else
    {
        Logger::LOG_ERROR << "bone '" << name << "' not contained in pose" << std::endl;
        return blank_pose;
    }
}

void SkeletalPose::set_transform(std::string const& name, BonePose const& value) { transforms[name] = value; }

void SkeletalPose::blend(SkeletalPose const& pose2, float blendFactor)
{
    for_each(pose2.transforms.cbegin(), pose2.transforms.cend(), [this, &blendFactor](std::pair<std::string, BonePose> const& p) {
        if(contains(p.first))
        {
            set_transform(p.first, get_transform(p.first).blend(p.second, blendFactor));
        }
        else
        {
            set_transform(p.first, p.second);
        }
    });
    // *this = *this * (1 - blendFactor) + pose2 * blendFactor;
}

SkeletalPose& SkeletalPose::operator+=(SkeletalPose const& pose2)
{
    for_each(pose2.transforms.cbegin(), pose2.transforms.cend(), [this](std::pair<std::string, BonePose> const& p) {
        if(contains(p.first))
        {
            set_transform(p.first, get_transform(p.first) + p.second);
        }
        else
        {
            set_transform(p.first, p.second);
        }
    });
    return *this;
}
SkeletalPose SkeletalPose::operator+(SkeletalPose const& p2) const
{
    SkeletalPose temp{*this};
    temp += p2;
    return temp;
}

SkeletalPose& SkeletalPose::operator*=(float const factor)
{
    for(auto& p : transforms)
    {
        p.second *= factor;
    }
    return *this;
}
SkeletalPose SkeletalPose::operator*(float const factor) const
{
    SkeletalPose temp{*this};
    temp *= factor;
    return temp;
}

void SkeletalPose::partial_replace(SkeletalPose const& pose2, Skeleton const& skeleton, unsigned bone)
{
    if(pose2.contains(skeleton.get(bone).name))
    {
        set_transform(skeleton.get(bone).name, pose2.get_transform(skeleton.get(bone).name));
    }

    for(auto const& child : skeleton.get(bone).children)
    {
        partial_replace(pose2, skeleton, child);
    }
}

} // namespace gua