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

#ifndef GUA_SPOINTS_NODE_HPP
#define GUA_SPOINTS_NODE_HPP

// guacamole headers
#include <gua/spoints/platform.hpp>
#include <gua/node/GeometryNode.hpp>
#include <gua/databases/GeometryDescription.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/spoints/SPointsResource.hpp>

// external headers
#include <string>

/**
 * This class is used to represent kinect video in the SceneGraph.
 *
 */

namespace gua {

	class SPointsResource;

namespace node {

class GUA_SPOINTS_DLL SPointsNode : public GeometryNode {

 public :

  SPointsNode(std::string const& name,
              std::string const& spoints_description = "gua_default_video",
              std::shared_ptr<Material> const& material = nullptr,
              math::mat4  const& transform = math::mat4::identity());

 public :

  void ray_test_impl(Ray const& ray,
                     int options,
                     Mask const& mask,
                     std::set<PickResult>& hits) override;

  void update_cache() override;


  inline float get_screen_space_point_size() const { return screen_space_point_size_; }
  inline void set_screen_space_point_size(float point_size) { screen_space_point_size_ = point_size; }

  inline void set_global_grid_dimension_x(int grid_x) {
    auto spoints_resource = std::static_pointer_cast<SPointsResource>(
          GeometryDatabase::instance()->lookup(spoints_description_));
    if(!spoints_resource)
      return;
    spoints_resource->set_global_grid_dimension_x(grid_x);
  }

  inline int get_global_grid_dimension_x() const {
    auto spoints_resource = std::static_pointer_cast<SPointsResource>(
          GeometryDatabase::instance()->lookup(spoints_description_));
    if(!spoints_resource)
      return -1;
    return spoints_resource->get_global_grid_dimension_x();
  }

  inline void set_global_grid_dimension_y(int grid_y) {
    auto spoints_resource = std::static_pointer_cast<SPointsResource>(
          GeometryDatabase::instance()->lookup(spoints_description_));
    if(!spoints_resource)
      return;
    spoints_resource->set_global_grid_dimension_y(grid_y);
  }

  inline int get_global_grid_dimension_y() const {
    auto spoints_resource = std::static_pointer_cast<SPointsResource>(
          GeometryDatabase::instance()->lookup(spoints_description_));
    if(!spoints_resource)
      return -1;
    return spoints_resource->get_global_grid_dimension_y();
  }

  inline void set_global_grid_dimension_z(int grid_z) {
    auto spoints_resource = std::static_pointer_cast<SPointsResource>(
          GeometryDatabase::instance()->lookup(spoints_description_));
    if(!spoints_resource)
      return;
    spoints_resource->set_global_grid_dimension_z(grid_z);
  }

  inline int get_global_grid_dimension_z() const {
    auto spoints_resource = std::static_pointer_cast<SPointsResource>(
          GeometryDatabase::instance()->lookup(spoints_description_));
    if(!spoints_resource)
      return -1;
    return spoints_resource->get_global_grid_dimension_z();
  }

  inline void set_global_point_precision_x(int point_x) {
    auto spoints_resource = std::static_pointer_cast<SPointsResource>(
          GeometryDatabase::instance()->lookup(spoints_description_));
    if(!spoints_resource)
      return;
    spoints_resource->set_global_point_precision_x(point_x);
  }

  inline int get_global_point_precision_x() const {
    auto spoints_resource = std::static_pointer_cast<SPointsResource>(
          GeometryDatabase::instance()->lookup(spoints_description_));
    if(!spoints_resource)
      return -1;
    return spoints_resource->get_global_point_precision_x();
  }

  inline void set_global_point_precision_y(int point_y) {
    auto spoints_resource = std::static_pointer_cast<SPointsResource>(
          GeometryDatabase::instance()->lookup(spoints_description_));
    if(!spoints_resource)
      return;
    spoints_resource->set_global_point_precision_y(point_y);
  }

  inline int get_global_point_precision_y() const {
    auto spoints_resource = std::static_pointer_cast<SPointsResource>(
          GeometryDatabase::instance()->lookup(spoints_description_));
    if(!spoints_resource)
      return -1;
    return spoints_resource->get_global_point_precision_y();
  }

  inline void set_global_point_precision_z(int point_z) {
    auto spoints_resource = std::static_pointer_cast<SPointsResource>(
          GeometryDatabase::instance()->lookup(spoints_description_));
    if(!spoints_resource)
      return;
    spoints_resource->set_global_point_precision_z(point_z);
  }

  inline int get_global_point_precision_z() const {
    auto spoints_resource = std::static_pointer_cast<SPointsResource>(
          GeometryDatabase::instance()->lookup(spoints_description_));
    if(!spoints_resource)
      return -1;
    return spoints_resource->get_global_point_precision_z();
  }

  inline void set_global_color_precision_x(int color_x) {
    auto spoints_resource = std::static_pointer_cast<SPointsResource>(
          GeometryDatabase::instance()->lookup(spoints_description_));
    if(!spoints_resource)
      return;
    spoints_resource->set_global_color_precision_x(color_x);
  }

  inline int get_global_color_precision_x() const {
    auto spoints_resource = std::static_pointer_cast<SPointsResource>(
          GeometryDatabase::instance()->lookup(spoints_description_));
    if(!spoints_resource)
      return -1;
    return spoints_resource->get_global_color_precision_x();
  }

  inline void set_global_color_precision_y(int color_y) {
    auto spoints_resource = std::static_pointer_cast<SPointsResource>(
          GeometryDatabase::instance()->lookup(spoints_description_));
    if(!spoints_resource)
      return;
    spoints_resource->set_global_color_precision_y(color_y);
  }

  inline int get_global_color_precision_y() const {
    auto spoints_resource = std::static_pointer_cast<SPointsResource>(
          GeometryDatabase::instance()->lookup(spoints_description_));
    if(!spoints_resource)
      return -1;
    return spoints_resource->get_global_color_precision_y();
  }

  inline void set_global_color_precision_z(int color_z) {
    auto spoints_resource = std::static_pointer_cast<SPointsResource>(
          GeometryDatabase::instance()->lookup(spoints_description_));
    if(!spoints_resource)
      return;
    spoints_resource->set_global_color_precision_z(color_z);
  }

  inline int get_global_color_precision_z() const {
    auto spoints_resource = std::static_pointer_cast<SPointsResource>(
          GeometryDatabase::instance()->lookup(spoints_description_));
    if(!spoints_resource)
      return -1;
    return spoints_resource->get_global_color_precision_z();
  }

  /**
   * Accepts a visitor and calls concrete visit method.
   *
   * This method implements the visitor pattern for Nodes.
   *
   * \param visitor  A visitor to process the GeometryNode's data.
   */
  void accept(NodeVisitor& visitor) override;

  std::string const& get_spoints_description() const;
  void               set_spoints_description(std::string const& v);
  void               force_reload();

  std::shared_ptr<Material> const& get_material() const;
  void                      set_material(std::shared_ptr<Material> const& material);

 protected:
  std::shared_ptr<Node> copy() const override;

 private:

  float                            screen_space_point_size_;
  std::shared_ptr<SPointsResource> spoints_;
  std::string                      spoints_description_;
  bool                             spoints_changed_;

  std::shared_ptr<Material>        material_;
  bool                             material_changed_;
};

} // namespace node {
} // namespace gua {

#endif  // GUA_VIDEO3D_NODE_HPP
