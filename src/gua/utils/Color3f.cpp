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

// class header
#include <gua/utils/Color3f.hpp>

// guacamole headers
#include <gua/math/random.hpp>

// external headers
#include <cmath>
#include <algorithm>

namespace gua
{
namespace utils
{
Color3f::Color3f() : r_(0.0f), g_(0.0f), b_(0.0f) {}

Color3f::Color3f(float r, float g, float b) : r_(r), g_(g), b_(b) {}

Color3f::Color3f(math::vec3f const& rgb) : r_(rgb.x), g_(rgb.y), b_(rgb.z) {}

float Color3f::h() const
{
    if(s() > 0.0f)
    {
        float maxi = std::max(std::max(r_, g_), b_);
        float mini = std::min(std::min(r_, g_), b_);

        if(maxi == r_)
            return fmod(60.f * ((g_ - b_) / (maxi - mini)) + 360.f, 360.f);
        else if(maxi == g_)
            return fmod(60.f * (2 + (b_ - r_) / (maxi - mini)) + 360.f, 360.f);
        else
            return fmod(60.f * (4 + (r_ - g_) / (maxi - mini)) + 360.f, 360.f);
    }
    else
        return 0.0f;
}

float Color3f::s() const
{
    if(v() == 0)
        return 0;
    else
        return ((v() - std::min(std::min(r_, g_), b_)) / v());
}

void Color3f::r(float red) { r_ = gua::math::clamp(red, 0.0f, 1.0f); }

void Color3f::g(float green) { g_ = gua::math::clamp(green, 0.0f, 1.0f); }

void Color3f::b(float blue) { b_ = gua::math::clamp(blue, 0.0f, 1.0f); }

void Color3f::h(float hue) { set_hsv(hue, s(), v()); }

void Color3f::s(float saturation) { set_hsv(h(), gua::math::clamp(saturation, 0.0f, 1.0f), v()); }

void Color3f::v(float value) { set_hsv(h(), s(), gua::math::clamp(value, 0.0f, 1.0f)); }

void Color3f::set_hsv(float hue, float saturation, float value)
{
    if(saturation == 0)
    {
        r_ = value;
        g_ = value;
        b_ = value;
        return;
    }
    hue = fmod(hue, 360);
    hue /= 60;
    int i = int(floor(hue));
    float f = hue - i;

    switch(i)
    {
    case 0:
        r_ = value;
        g_ = value * (1 - saturation * (1 - f));
        b_ = value * (1 - saturation);
        break;
    case 1:
        r_ = value * (1 - saturation * f);
        g_ = value;
        b_ = value * (1 - saturation);
        break;
    case 2:
        r_ = value * (1 - saturation);
        g_ = value;
        b_ = value * (1 - saturation * (1 - f));
        break;
    case 3:
        r_ = value * (1 - saturation);
        g_ = value * (1 - saturation * f);
        b_ = value;
        break;
    case 4:
        r_ = value * (1 - saturation * (1 - f));
        g_ = value * (1 - saturation);
        b_ = value;
        break;
    default:
        r_ = value;
        g_ = value * (1 - saturation);
        b_ = value * (1 - saturation * f);
        break;
    }
}

Color3f const Color3f::inverted() const
{
    Color3f inverted(*this);
    inverted.h(inverted.h() + 180.f);
    if(v() < 0.5f)
        inverted.v(1.f - inverted.v());
    return inverted;
}

Color3f const Color3f::brightened() const
{
    Color3f brightened(*this);
    if(brightened.v() < 0.5f)
        brightened.v(0.5f);
    if(brightened.s() < 0.5f)
        brightened.s(0.5f);
    return brightened;
}

scm::math::vec3f const Color3f::vec3f() const { return scm::math::vec3f(r_, g_, b_); }

Color3f const Color3f::random()
{
    Color3f result(math::random::get(0.0f, 1.0f), math::random::get(0.0f, 1.0f), math::random::get(0.0f, 1.0f));
    result.s(result.s() + 0.5);
    result.v(result.v() + 0.5);
    return result;
}

Color3f operator*(float lhs, Color3f const& rhs) { return Color3f(rhs.r() * lhs, rhs.g() * lhs, rhs.b() * lhs); }

Color3f operator*(Color3f const& lhs, float rhs) { return rhs * lhs; }

Color3f operator/(Color3f const& lhs, float rhs) { return Color3f(lhs.r() / rhs, lhs.g() / rhs, lhs.b() / rhs); }

Color3f operator+(Color3f const& lhs, Color3f const& rhs)
{
    Color3f result;
    result.r(lhs.r() + rhs.r());
    result.g(lhs.g() + rhs.g());
    result.b(lhs.b() + rhs.b());
    return result;
}

Color3f operator-(Color3f const& lhs, Color3f const& rhs)
{
    Color3f result;
    result.r(lhs.r() - rhs.r());
    result.g(lhs.g() - rhs.g());
    result.b(lhs.b() - rhs.b());
    return result;
}

bool operator==(Color3f const& lhs, Color3f const& rhs) { return lhs.r() == rhs.r() && lhs.g() == rhs.g() && lhs.b() == rhs.b(); }

std::ostream& operator<<(std::ostream& os, Color3f const& color)
{
    os << color.r() << " " << color.g() << " " << color.b() << std::endl;
    return os;
}

std::istream& operator>>(std::istream& is, Color3f& color)
{
    float cur_val;
    is >> cur_val;
    color.r(cur_val);
    is >> cur_val;
    color.g(cur_val);
    is >> cur_val;
    color.b(cur_val);
    return is;
}

} // namespace utils
} // namespace gua
