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

#ifndef GUA_PBR_RESSOURCE_HPP
#define GUA_PBR_RESSOURCE_HPP



// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/GeometryRessource.hpp>
#include <gua/renderer/PBRUberShader.hpp>
#include <gua/utils/KDTree.hpp>

// external headers
#include <pbr/ren/raw_point_cloud.h>
#include <scm/gl_core.h>

#include <mutex>
#include <thread>

#include <vector>




namespace gua {

struct RenderContext;

/**
 * Stores geometry data.
 *
 * A mesh can be loaded from an Assimp mesh and the draw onto multiple
 * contexts.
 * Do not use this class directly, it is just used by the Geometry class to
 * store the individual meshes of a file.
 */
class PBRRessource : public GeometryRessource {
 public:

  /**
   * Default constructor.
   *
   * Creates a new and empty Mesh.
   */
   PBRRessource();

  /**
   * Constructor from an Assimp mesh.
   *
   * Initializes the mesh from a given Assimp mesh.
   *
   * \param mesh             The Assimp mesh to load the data from.
   */
   PBRRessource(std::shared_ptr<pbr::ren::RawPointCloud> point_cloud);

  /**
   * Draws the Mesh.
   *
   * Draws the Mesh to the given context.
   *
   * \param context          The RenderContext to draw onto.
   */
  void draw(RenderContext const& context) const;

  void ray_test(Ray const& ray, PickResult::Options options,
                Node* owner, std::set<PickResult>& hits);


  /*virtual*/ GeometryUberShader* get_ubershader() const;

 private:

  void upload_to(RenderContext const& context) const;

  mutable std::vector<scm::gl::buffer_ptr> buffers_;
  mutable std::vector<scm::gl::vertex_array_ptr> vertex_array_;
  mutable std::mutex upload_mutex_;

 public:

  //KDTree kd_tree_;

  std::shared_ptr<pbr::ren::RawPointCloud> point_cloud_;

};

}

#endif  // GUA_PBR_RESSOURCE_HPP
