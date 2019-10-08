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

#ifndef COLOR_3F_HPP
#define COLOR_3F_HPP

#include <gua/platform.hpp>
#include <gua/math/math.hpp>

namespace gua
{
namespace utils
{
/**
 * A struct for color handling.
 *
 * This class stores color values in RGB manner, but provides an HSV
 * interface as well.
 */

struct GUA_DLL Color3f
{
  public:
    /**
     * Constructor.
     *
     * This constructs a Color with all values set to 0 (black).
     */
    Color3f();

    /**
     * Constructor.
     *
     * This constructs a Color from given RGB values.
     *
     * \param red       The red value.
     * \param green     The green value.
     * \param blue      The blue value.
     */
    Color3f(float red, float green, float blue);

    Color3f(math::vec3f const& rgb);

    ///@{
    /**
     * Returns a single Color value.
     */
    inline float r() const { return r_; }
    inline float g() const { return g_; }
    inline float b() const { return b_; }
    float h() const;
    float s() const;
    inline float v() const { return std::max(std::max(r_, g_), b_); }
    ///@}

    ///@{
    /**
     * Sets a single Color value.
     *
     * \param value    The new value to be set.
     */
    void r(float red);
    void g(float green);
    void b(float blue);
    void h(float hue);
    void s(float saturation);
    void v(float value);
    ///@}

    /**
     * Returns an inverted copy of the Color.
     *
     * \param color      The inverted copy of the Color.
     */
    Color3f const inverted() const;

    /**
     * Returns an inverted copy of the Color.
     *
     * \param color      The inverted copy of the Color.
     */
    Color3f const brightened() const;

    scm::math::vec3f const vec3f() const;

    /**
     * Returns a randomly generated Color.
     *
     * \return color      A randomly generated color.
     */
    static const Color3f random();

    friend bool operator==(Color3f const& lhs, Color3f const& rhs);

  private:
    void set_hsv(float hue, float saturation, float value);

    float r_, g_, b_;
};

// Multiplication of a color with a float.
Color3f operator*(float lhs, Color3f const& rhs);
Color3f operator*(Color3f const& lhs, float rhs);

// Addition of two colors. Clamped.
Color3f operator+(Color3f const& lhs, Color3f const& rhs);

// Subtraction of two colors. Clamped.
Color3f operator-(Color3f const& lhs, Color3f const& rhs);

// Division of a color by a float.
Color3f operator/(Color3f const& lhs, float rhs);

std::ostream& operator<<(std::ostream& os, Color3f const& color);
std::istream& operator>>(std::istream& is, Color3f& color);

} // namespace utils
} // namespace gua

#endif // COLOR_3F_HPP
