// *****************************************************************************
//  * guacamole - delicious VR                                                   *
//  *                                                                            *
//  * Copyright: (c) 2011-2013 Bauhaus-Universität Weimar                        *
//  * Contact:   felix.lauer@uni-weimar.de / simon.schneegans@uni-weimar.de      *
//  *                                                                            *
//  * This program is free software: you can redistribute it and/or modify it    *
//  * under the terms of the GNU General Public License as published by the Free *
//  * Software Foundation, either version 3 of the License, or (at your option)  *
//  * any later version.                                                         *
//  *                                                                            *
//  * This program is distributed in the hope that it will be useful, but        *
//  * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY *
//  * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License   *
//  * for more details.                                                          *
//  *                                                                            *
//  * You should have received a copy of the GNU General Public License along    *
//  * with this program. If not, see <http://www.gnu.org/licenses/>.             *
//  *                                                                            *
//  *****************************************************************************

// #ifndef GUA_GUI_NODE_HPP
// #define GUA_GUI_NODE_HPP

// // guacamole headers
// #include <gua/node/GeometryNode.hpp>

// namespace gua {

// class GuiResource;

// /**
//  * This class is used to represent polygonal geometry in the SceneGraph.
//  *
//  * \ingroup gua_scenegraph
//  */
// class GUA_DLL GuiNode : public node::GeometryNode {
//  public:  // member

//   GuiNode(std::string const& name = "",
//           std::string const& resource_url = "",
//           math::mat4 const& transform = math::mat4::identity());


//   /**
//   * Implements ray picking for a gui node
//   */
//   void ray_test_impl(Ray const& ray,
//                      PickResult::Options options,
//                      Mask const& mask,
//                      std::set<PickResult>& hits) override;

//   /**
//   * Updates bounding box by accessing the ressource in the databse
//   */
//   void update_bounding_box() const override;
//   void update_cache() override;


//   void set_resource_url(std::string const& resource_url);
//   std::string const& get_resource_url() const;

//   std::shared_ptr<GuiResource> const& get_resource() const;

//   /**
//    * Accepts a visitor and calls concrete visit method.
//    *
//    * This method implements the visitor pattern for Nodes.
//    *
//    * \param visitor  A visitor to process the GeometryNode's data.
//    */
//   void accept(NodeVisitor& visitor) override;

//  protected:

//   std::shared_ptr<node::Node> copy() const override;

//  private:  // attributes e.g. special attributes for drawing

//   std::shared_ptr<GuiResource> resource_;

//   std::string resource_url_;
//   bool resource_url_changed_;

// };

// } // namespace gua {

// #endif  // GUA_GUI_NODE_HPP
