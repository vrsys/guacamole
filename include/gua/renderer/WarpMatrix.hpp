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

#ifndef GUA_WARP_MATRIX_HPP
#define GUA_WARP_MATRIX_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/Texture2D.hpp>

namespace gua
{
/**
 * A class representing a warp matrix.
 *
 * Warp matrices are used to align colors of pixels of an projected image,
 * when the projecting beamers for the colors red, green and blue don't
 * overlap exactly.
 */
class GUA_DLL WarpMatrix : public Texture2D
{
  public:
    /**
     * Default constructor.
     *
     * Creates a new (invalid) WarpMatrix.
     */
    WarpMatrix();

    /**
     * Constructor.
     *
     * Creates a new WarpMatrix from a given path to a warp matrix file.
     *
     * \param file_name            Path to a warp matrix file.
     */
    WarpMatrix(std::string const& file_name);

  private:
    void upload_to(RenderContext const& context) const override;

    mutable std::vector<float> data_;
};

} // namespace gua
#endif // GUA_WARP_MATRIX_HPP
