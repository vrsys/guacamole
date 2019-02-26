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

#include <gua/utils/AnimatedValue.hpp>

namespace gua
{
namespace utils
{
AnimatedValue::AnimatedValue() : direction_(In), val_(0), start_(0), end_(0), state_(0.0), duration_(0), multiplier_(0.0) {}

AnimatedValue::AnimatedValue(Direction direction, float start, float end, float duration, float multiplier)
    : direction_(direction), val_(start), start_(start), end_(end), state_(0.0), duration_(duration), multiplier_(multiplier)
{
}

void AnimatedValue::resetTarget(float end, float duration)
{
    start_ = val_;
    end_ = end;
    duration_ = duration;
    state_ = 0.0;
}

void AnimatedValue::update(float time)
{
    state_ += time / duration_;

    if(state_ < 1)
    {
        switch(direction_)
        {
        case Linear:
            val_ = updateLinear(state_, start_, end_);
            break;
        case In:
            val_ = updateEaseIn(state_, start_, end_);
            return;
        case Out:
            val_ = updateEaseOut(state_, start_, end_);
            return;
        case InOut:
            val_ = updateEaseInOut(state_, start_, end_);
            return;
        case OutIn:
            val_ = updateEaseOutIn(state_, start_, end_);
            return;
        }
    }
    else if(val_ != end_)
    {
        val_ = end_;
    }
}

float AnimatedValue::updateLinear(float t, float s, float e) { return (s + t * (e - s)); }

float AnimatedValue::updateEaseIn(float t, float s, float e) { return (s + (t * t * ((multiplier_ + 1) * t - multiplier_)) * (e - s)); }

float AnimatedValue::updateEaseOut(float t, float s, float e) { return (s + ((t - 1) * (t - 1) * ((multiplier_ + 1) * (t - 1) + multiplier_) + 1) * (e - s)); }

float AnimatedValue::updateEaseInOut(float t, float s, float e)
{
    if(state_ < 0.5f)
        return updateEaseIn(t * 2, s, e - (e - s) * 0.5f);
    else
        return updateEaseOut(t * 2 - 1, s + (e - s) * 0.5f, e);
}

float AnimatedValue::updateEaseOutIn(float t, float s, float e)
{
    if(state_ < 0.5f)
        return updateEaseOut(t * 2, s, e - (e - s) * 0.5f);
    else
        return updateEaseIn(t * 2 - 1, s + (e - s) * 0.5f, e);
}

} // namespace utils
} // namespace gua
