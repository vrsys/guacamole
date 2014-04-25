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
#include <gua/guacamole.hpp>
#include <gua/renderer/MeshLoader.hpp>
#include <gua/renderer/BuiltInTextures.hpp>
#include <gua/databases/Resources.hpp>

namespace gua {

void init(int argc, char** argv) {
  static scm::shared_ptr<scm::core> scm_core(new scm::core(argc, argv));

  create_resource_material("gua_bounding_box",
                            Resources::materials_gua_bounding_box_gsd,
                            Resources::materials_gua_bounding_box_gmd);

  create_resource_material("gua_textured_quad",
                            Resources::materials_gua_textured_quad_gsd,
                            Resources::materials_gua_textured_quad_gmd);

  gua::TextureDatabase::instance()->add("gua_default_texture", std::shared_ptr<Texture2D>(new DefaultTexture()));
  gua::TextureDatabase::instance()->add("gua_loading_texture", std::shared_ptr<Texture2D>(new LoadingTexture()));

  MeshLoader mesh_loader;

  GeometryDatabase::instance()->add(
      "gua_light_sphere_proxy",
      std::shared_ptr<GeometryRessource>(
      static_cast<GeometryRessource*>(mesh_loader.load_from_buffer(
              Resources::lookup_string(Resources::geometry_gua_light_sphere_obj).c_str(),
              Resources::geometry_gua_light_sphere_obj.size(), false)[0])));

  GeometryDatabase::instance()->add(
      "gua_light_cone_proxy",
      std::shared_ptr<GeometryRessource>(
      static_cast<GeometryRessource*>(mesh_loader.load_from_buffer(
              Resources::lookup_string(Resources::geometry_gua_light_cone_obj).c_str(),
              Resources::geometry_gua_light_cone_obj.size(), false)[0])));

  GeometryDatabase::instance()->add(
      "gua_ray_geometry",
      std::shared_ptr<GeometryRessource>(
      static_cast<GeometryRessource*>(mesh_loader.load_from_buffer(
              Resources::lookup_string(Resources::geometry_gua_ray_obj).c_str(),
              Resources::geometry_gua_ray_obj.size(), false)[0])));

  GeometryDatabase::instance()->add(
      "gua_plane_geometry",
      std::shared_ptr<GeometryRessource>(
      static_cast<GeometryRessource*>(mesh_loader.load_from_buffer(
              Resources::lookup_string(Resources::geometry_gua_plane_obj).c_str(),
              Resources::geometry_gua_plane_obj.size(), true)[0])));

  GeometryDatabase::instance()->add(
      "gua_bounding_box_geometry",
      std::shared_ptr<GeometryRessource>(
      static_cast<GeometryRessource*>(mesh_loader.load_from_buffer(
              Resources::lookup_string(Resources::geometry_gua_bounding_box_obj).c_str(),
              Resources::geometry_gua_bounding_box_obj.size(), false)[0])));
}

void create_resource_material(std::string const& material_name,
                              std::vector<unsigned char> const& shading_model_resource,
                              std::vector<unsigned char> const& material_resource) {
  std::shared_ptr<ShadingModel> shading_model(
      new ShadingModel(material_name,
                       Resources::lookup_string(shading_model_resource).c_str(),
                       shading_model_resource.size()));

  ShadingModelDatabase::instance()->add(material_name,
                                        shading_model);

  MaterialDescription material_description(
                       Resources::lookup_string(material_resource).c_str(),
                       material_resource.size());

  std::shared_ptr<Material> material(
      new Material(material_name, material_description));
  MaterialDatabase::instance()->add(material_name, material);
}


}
