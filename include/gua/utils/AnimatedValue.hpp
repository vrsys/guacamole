/******************************************************************************
 * guacamole - delicious VR                                                   *
 *                                                                            *
 * Copyright: (c) 2011-2013 Bauhaus-Universität Weimar                        *
 * Contact:   felix.lauer@uni-weimar.de / simon.schneegans@uni-weimar.de      *
 *                                                                            *
 * This program is free software: you can redistribute it and/or modify it    *
 * under the terms of the GNU General Public License as published by the Free *
 * Software Foundation, either version 3 of the License, or (at your option)  *
 * any later version.                                                         *
 *                                                                            *
 * This program is distributed in the hope that it will be useful, but        *
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY *
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License   *
 * for more details.                                                          *
 *                                                                            *
 * You should have received a copy of the GNU General Public License along    *
 * with this program. If not, see <http://www.gnu.org/licenses/>.             *
 *                                                                            *
 ******************************************************************************/

#ifndef GUA_ANIMATEDVALUE_HPP
#define GUA_ANIMATEDVALUE_HPP

#include <gua/platform.hpp>

namespace gua
{
namespace utils
{
/**
 * A class for smooth value interpolation.
 */
class GUA_DLL AnimatedValue
{
  public:
    enum Direction
    {
        In,
        Out,
        InOut,
        OutIn,
        Linear
    };

    AnimatedValue();
    AnimatedValue(Direction direction, float start, float end, float duration, float multiplier = 0);

    void resetTarget(float end, float duration);
    void update(float time);

    inline float val() const { return val_; }
    inline float start() const { return start_; }
    inline float end() const { return end_; }

  private:
    float updateLinear(float t, float s, float e);
    float updateEaseIn(float t, float s, float e);
    float updateEaseOut(float t, float s, float e);
    float updateEaseInOut(float t, float s, float e);
    float updateEaseOutIn(float t, float s, float e);

    Direction direction_;

    float val_, start_;
    float end_, state_;
    float duration_, multiplier_;
};

} // namespace utils
} // namespace gua

#endif // GUA_ANIMATEDVALUE_HPP
