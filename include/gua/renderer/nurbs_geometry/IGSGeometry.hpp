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

#ifndef IGS_MESH_HPP
#define IGS_MESH_HPP

#include <vector>
#include <thread>

namespace gua {

class RenderContext;

/**
 * Stores geometry data.
 *
 * A mesh can be loaded from a Assimp mesh and the draw onto multiple contexts.
 * Do not use this class directly, it is just used by the IGSGeometry class to
 * store the individual meshes of a file.
 */

class IGSGeometry {
 public:
  /**
   * Default constructor.
   *
   * Creates a new and empty IGSGeometry.
   */
  IGSGeometry();

  /**
   * Constructor from an Assimp mesh.
   *
   * Initializes the mesh from a given Assimp mesh.
   *
   * \param mesh The Assimp mesh to load the data from.
   */
  IGSGeometry(std::string const& file_name);

  /**
   * Draws the IGSGeometry.
   *
   * Draws the IGSGeometry to the given context.
   *
   * \param context The RenderContext to draw onto.
   */
  void draw(RenderContext const& context) const;

  static void set_fill_mode(scm::gl::fill_mode in_fill_mode);

 private:
  gua::nurbsobject_igs igs_object;

  std::string file_name_;

  static scm::gl::fill_mode fill_mode_;
};

}

#endif  // MESH_HPP
