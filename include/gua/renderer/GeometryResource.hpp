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

#ifndef GUA_GEOMETRY_RESSOURCE_HPP
#define GUA_GEOMETRY_RESSOURCE_HPP

// guacamole_headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/MaterialShaderMethod.hpp>
#include <gua/math/BoundingBox.hpp>
#include <gua/scenegraph/PickResult.hpp>

// external headers
#include <string>
#include <vector>
#include <scm/gl_util/primitives/box.h>

namespace gua {

class RessourceRenderer;

struct Ray;

namespace node {
  class GeometryNode;
};

/**
 * Base class for the render ressources (CPU/GPU) for different geometry types
 */
class GUA_DLL GeometryResource {
 public:

  /**
   * Draws the Geometry.
   *
   * Draws this Geometry object to the given context.
   *
   * \param context           The RenderContext to which this object
   *                          should be drawn.
   */
   virtual void draw(RenderContext const& context) const = 0;

  /**
   * Interface to implement pre-draw tasks (occlusion queries, LOD etc.)
   *
   * Predraw tasks that need to be done before drawing geometry
   *
   * \param context           The RenderContext to which this object
   *                          should be predrawn.
   */
   virtual void predraw(RenderContext const& context) const {}

  /**
  * Interface which provides the appropriate UberShader for the ressource
  */
   // virtual std::shared_ptr<GeometryUberShader> create_ubershader() const = 0;

  /**
   * Interface to intersect the geometry with a ray.
   *
   * \param ray               The Ray with whom the geometry is about to be
   *                          intersected.
   *
   * \return                  The intersection distance along the ray.
   */
  virtual void ray_test(Ray const& ray, PickResult::Options options,
                        node::Node* owner, std::set<PickResult>& hits) = 0;

  /**
   * Get the local bounding box of the geometry.
   *
   * \return                  The local bounding box of the geometry.
   */
  inline math::BoundingBox<math::vec3> const& get_bounding_box() const {
    return bounding_box_;
  }

  virtual MaterialShaderMethod const& get_vertex_material_pass() const {
    return vertex_material_pass_;
  }

  virtual MaterialShaderMethod const& get_fragment_material_pass() const {
    return fragment_material_pass_;
  }

  virtual std::shared_ptr<RessourceRenderer> create_renderer() const = 0;

 protected:

  math::BoundingBox<math::vec3> bounding_box_;

  MaterialShaderMethod vertex_material_pass_;
  MaterialShaderMethod fragment_material_pass_;
};

}

#endif  // GUA_GEOMETRY_RESSOURCE_HPP
