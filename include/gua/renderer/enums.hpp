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

#ifndef GUA_ENUMS_HPP
#define GUA_ENUMS_HPP

#include <sstream>
#include <string>
#include <set>
#include <boost/optional.hpp>

namespace gua
{
/**
 * Eye of a view.
 *
 * An enumeration used for determining from which point of view a scene
 * has to be rendered. Either from the left eye of a view, from the
 * right or from the exact view position.
 */
enum class CameraMode
{
    CENTER = 0,
    LEFT,
    RIGHT,
    BOTH
};

/**
 * Stereo mode for a pipeline.
 *
 * Determines how two stereo images should be mapped to each other.
 */
enum class StereoMode
{
    MONO = 0,
    SIDE_BY_SIDE,
    SIDE_BY_SIDE_SOFTWARE_MULTI_VIEW_RENDERING,
    SIDE_BY_SIDE_HARDWARE_MULTI_VIEW_RENDERING,
    ANAGLYPH_RED_GREEN,
    ANAGLYPH_RED_CYAN,
    CHECKERBOARD,
    NVIDIA_3D_VISION,
    QUAD_BUFFERED,
    SEPARATE_WINDOWS
};

/**
 * Shadow type for a shadow-casting node.
 *
 * Determines the quality of shadows.
 */
enum class ShadowMode
{
    OFF = 0,
    LOW_QUALITY,
    HIGH_QUALITY
};

/**
 * All uniform types which are supported by guacamole.
 */
enum class UniformType
{
    INT = 0,
    FLOAT,
    BOOL,
    VEC2,
    VEC3,
    VEC4,
    VEC2I,
    VEC3I,
    VEC4I,
    VEC2UI,
    VEC3UI,
    VEC4UI,
    MAT3,
    MAT4,
    SAMPLER1D,
    SAMPLER2D,
    SAMPLER3D,
    SAMPLERCUBE,
    NONE
};

/**
 * Different types of gbuffer layers.
 *
 * Each enum value describes a different buffer configuration. Data type and
 * component count are encoded.
 */
enum class BufferComponent
{
    I1 = 0,
    I2,
    I3,
    I4,
    U1,
    U2,
    U3,
    U4,
    H1,
    H2,
    H3,
    H4,
    F1,
    F2,
    F3,
    F4,
    DEPTH_16,
    DEPTH_24,
    NONE
};

/**
 * Different component types of gbuffer layers.
 *
 * Each of these types coresponds to some of the above layer configurations.
 */
enum BufferComponentType
{
    TYPE_INTEGER = 0,
    TYPE_UNSIGNED,
    TYPE_HALF,
    TYPE_FLOAT,
    TYPE_DEPTH,
    TYPE_NONE
};

namespace enums
{
///@{
/**
 * Converts BufferComponent to their GLSL representation.
 *
 * For example: I4 <-> ivec4
 */
std::string output_type_to_string(BufferComponent type);
boost::optional<BufferComponent> parse_output_type(std::string const& type);
///@}

///@{
/**
 * Converts UniformType to their GLSL representation.
 *
 * For example: UniformType::VEC2 <-> vec2
 */
std::string uniform_type_to_string(UniformType type);
boost::optional<UniformType> parse_uniform_type(std::string const& type);
///@}

///@{
/**
 * Converts BufferComponentType to their GLSL representation.
 *
 * For example: TYPE_FLOAT <-> float
 */
std::string buffer_component_type_to_string(BufferComponentType type);
boost::optional<BufferComponentType> parse_buffer_component_type(std::string const& type);
///@}

/**
 * Returns a valid string representation of a default uniform value.
 */
std::string get_default_value(UniformType type);

/**
 * Returns whether a string contains a valid uniform value.
 */
bool is_valid_value(UniformType type, std::string& value);

/**
 * Returns the BufferComponentType of a BufferComponent.
 */
BufferComponentType get_type(BufferComponent component);

/**
 * Returns the BufferComponent from a given BufferComponentType
 * and a number of components.
 */
BufferComponent get_component(BufferComponentType type, unsigned components);

/**
 * Returns the number of components of a BufferComponent.
 */
unsigned get_number_of_components(BufferComponent component);

std::set<std::string> list_output_types();
std::set<std::string> list_uniform_types();

template <typename T>
bool test_value_string(std::string& value)
{
    std::istringstream in(value);
    T t;
    if(in >> t)
    {
        std::ostringstream out;
        out << t;
        value = out.str();

        return true;
    }

    return false;
}
} // namespace enums
} // namespace gua

#endif // GUA_ENUMS_HPP
