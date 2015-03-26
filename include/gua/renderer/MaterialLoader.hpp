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

#ifndef GUA_MATERIAL_LOADER_HPP
#define GUA_MATERIAL_LOADER_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/Material.hpp>
#include <gua/renderer/TriMeshRessource.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>

// external headers
#include <string>
#include <list>
#include <memory>

#include <assimp/postprocess.h>
#include <assimp/scene.h>

namespace fbxsdk_2015_1{
  class FbxSurfaceMaterial;
}

namespace Assimp { class Importer; }

namespace gua {

class Node;
class GeometryNode;

/**
 * Loads and draws meshes.
 *
 * This class can load mesh data from files and display them in multiple
 * contexts. A MaterialLoader object is made of several Mesh objects.
 */
class GUA_DLL MaterialLoader {
 public:

  std::shared_ptr<Material> load_material(aiMaterial const* material,
                                          std::string const& assets_directory,
                                          bool optimize_material) const;

  std::shared_ptr<Material> load_json(std::string const& material,
                                std::string const& assets_directory,
                                std::shared_ptr<Material> const& base_mat = gua::MaterialShaderDatabase::instance()->lookup("gua_default_material")->make_new_material()) const;

#ifdef GUACAMOLE_FBX
  std::shared_ptr<Material> load_material(fbxsdk_2015_1::FbxSurfaceMaterial const& material,
                                std::string const& assets_directory) const;
#endif
	static std::shared_ptr<Material> load_unreal(std::string const& file_name,
															 	std::string const& assets_directory,
															  std::shared_ptr<Material> const& base_mat = gua::MaterialShaderDatabase::instance()->lookup("gua_default_material")->make_new_material());

  static std::set<std::string> parse_unreal_material(std::string const& file);
  static std::string get_file_name(std::string const& path);
  inline static bool file_exists(std::string const& path);
};

}

#endif  // GUA_MATERIAL_LOADER_HPP
