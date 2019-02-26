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

// header
#include <gua/math/random.hpp>

// external headers
#include <cstdlib>
#include <ctime>

namespace gua
{
namespace math
{
namespace random
{
namespace
{
////////////////////////////////////////////////////////////////////////////////

unsigned int init_seed()
{
    unsigned int seed = static_cast<unsigned int>(std::time(nullptr));
    std::srand(seed);
    return seed;
}

unsigned int global_seed = init_seed();

////////////////////////////////////////////////////////////////////////////////

} // namespace

////////////////////////////////////////////////////////////////////////////////

void set_seed(unsigned int seed)
{
    std::srand(seed);
    global_seed = seed;
}

////////////////////////////////////////////////////////////////////////////////

unsigned int get_seed() { return global_seed; }

////////////////////////////////////////////////////////////////////////////////

float get(float begin, float end) { return static_cast<float>(std::rand()) / RAND_MAX * (end - begin) + begin; }

////////////////////////////////////////////////////////////////////////////////

int get(int begin, int end) { return std::rand() % (end - begin + 1) + begin; }

////////////////////////////////////////////////////////////////////////////////

} // namespace random
} // namespace math
} // namespace gua
