// class header
#include <gua/utils/Blending.hpp>

// external headers
#include <scm/gl_core.h>
#include <scm/core/math/quat.h>

#include <iostream>

namespace gua {

float blend::cos(float x) {
  x *= scm::math::pi_f;
  return 0.5f * (1 - scm::math::cos(x));
}

float blend::linear(float x) {
  //values from 0 to 2 accepted
  x = fmod(x, 2.0f);
  x = 1 - scm::math::abs(x - 1);
  return x;
}

float blend::smoothstep(float x) {
  x = fmod(x, 2.0f);
  x = 1 - scm::math::abs(x - 1);

  return 3 * x * x - 2 * x * x * x;
}

float blend::swap(float x) {
  x = fmod(x, 2.0f);
  return (x > 0.5) ? 1 : 0;
}

}  // namespace gua