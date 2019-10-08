// class header
#include <gua/skelanim/utils/BonePose.hpp>

namespace gua
{
BonePose::BonePose() : scaling{1.0f}, rotation{scm::math::quatf::identity()}, translation{0.0f} {}

BonePose::BonePose(scm::math::vec3f const& scale, scm::math::quatf const& rotate, scm::math::vec3f const& translate) : scaling{scale}, rotation{rotate}, translation{translate} {}

BonePose::~BonePose() {}

scm::math::mat4f BonePose::to_matrix() const { return scm::math::make_translation(translation) * rotation.to_matrix() * scm::math::make_scale(scaling); }

BonePose BonePose::blend(BonePose const& t, float const factor) const
{
    return BonePose{scaling * (1 - factor) + t.scaling * factor, slerp(rotation, t.rotation, factor), translation * (1 - factor) + t.translation * factor};
}

BonePose BonePose::operator+(BonePose const& t) const { return BonePose{scaling + t.scaling, scm::math::normalize(t.rotation * rotation), translation + t.translation}; }
BonePose& BonePose::operator+=(BonePose const& t)
{
    *this = *this + t;
    return *this;
}

BonePose BonePose::operator*(float const factor) const { return BonePose{scaling * factor, slerp(scm::math::quatf::identity(), rotation, factor), translation * factor}; }
BonePose& BonePose::operator*=(float const f)
{
    *this = *this * f;
    return *this;
}

} // namespace gua