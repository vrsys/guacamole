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

#ifndef GUA_RANDOM_HPP
#define GUA_RANDOM_HPP

#include <gua/platform.hpp>

namespace gua
{
namespace math
{
namespace random
{
/**
 * Sets a new random seed.
 *
 * \param seed  The new seed.
 */
void GUA_DLL set_seed(unsigned int seed);

/**
 * Gets the current seed.
 *
 * \return      The current seed.
 */
unsigned int GUA_DLL get_seed();

/**
 * Returns a random floating point value.
 *
 * \param   begin Smallest possible result.
 * \param   end   Largets possible result.
 *
 * \return        A random floating point value.
 */
float GUA_DLL get(float begin, float end);

/**
 * Returns a random integer value.
 *
 * \param   begin Smallest possible result.
 * \param   end   Largets possible result.
 *
 * \return        A random integer value.
 */
int GUA_DLL get(int begin, int end);
} // namespace random
} // namespace math
} // namespace gua
#endif // GUA_RANDOM_HPP
