// class header
#include <gua/utils/Transformation.hpp>
//external headers
#include <iostream>
#include <queue>

namespace gua {

Transformation::Transformation():
  scaling{1.0f},
  rotation{scm::math::quatf::identity()},
  translation{0.0f}
{}

Transformation::Transformation(scm::math::vec3f const& scale, scm::math::quatf const& rotate, scm::math::vec3f const& translate):
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

} // namespace gua