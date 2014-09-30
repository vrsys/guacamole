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

#ifndef GUA_PLOD_RESSOURCE_HPP
#define GUA_PLOD_RESSOURCE_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/GeometryRessource.hpp>
#include <gua/renderer/PLODUberShader.hpp>
#include <gua/utils/KDTree.hpp>

// external headers
#include <pbr/types.h>
#include <pbr/ren/model_database.h>
#include <pbr/ren/cut_database.h>
#include <pbr/ren/cut.h>
#include <pbr/ren/lod_point_cloud.h>

namespace gua {

struct RenderContext;

/**
 * Stores a point cloud model with LOD.
 *
 * This class simply a wrapper for accessing models of PBR library
 */
class PLODRessource : public GeometryRessource {
 public:

  explicit PLODRessource(pbr::model_t model_id, bool is_pickable);

  void draw(RenderContext const& ctx) const {}

  /**
   * Draws the point cloud.
   *
   * Draws the point cloud to the given context.
   *
   * \param context  The RenderContext to draw onto.
   */
  void draw(RenderContext const& ctx,
            pbr::context_t context_id,
            pbr::view_t view_id,
            pbr::model_t model_id,
            scm::gl::vertex_array_ptr const& vertex_array,
            std::vector<bool> const& frustum_culling_results) const;

  void ray_test(Ray const& ray,
                PickResult::Options options,
                node::Node* owner,
                std::set<PickResult>& hits);

  std::shared_ptr<GeometryUberShader> create_ubershader() const override {
    return std::make_shared<PLODUberShader>();
  }

 private:

  bool is_pickable_;

  // TODO: do we need it here?
  pbr::model_t model_id_;

};

}

#endif  // GUA_PLOD_RESSOURCE_HPP
