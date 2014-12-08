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

#ifndef GUA_SKELETAL_ANIMATION_LOADER_HPP
#define GUA_SKELETAL_ANIMATION_LOADER_HPP

// guacamole headers
#include <gua/renderer/SkeletalAnimationRessource.hpp>
#include <gua/node/SkeletalAnimationNode.hpp>
 
#include <gua/renderer/Material.hpp>

// external headers
#include <string>
#include <list>
#include <memory>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

namespace Assimp { class Importer; }

namespace gua {

namespace node {
class Node;
class InnerNode;
class GeometryNode;
}

/**
 * Loads and draws meshes.
 *
 * This class can load mesh data from files and display them in multiple
 * contexts. A MeshLoader object is made of several Mesh objects.
 */
class GUA_DLL SkeletalAnimationLoader {

 public: // typedefs, enums

   enum Flags {
     DEFAULTS = 0,
     LOAD_MATERIALS = 1 << 0,
     OPTIMIZE_GEOMETRY = 1 << 1,
     MAKE_PICKABLE = 1 << 2,
     NORMALIZE_POSITION = 1 << 3,
     NORMALIZE_SCALE = 1 << 4
   };

public:

  /**
   * Default constructor.
   *
   * Constructs a new and empty MeshLoader.
   */
   SkeletalAnimationLoader();

   /**
   *
   */
   std::shared_ptr<node::Node> load_geometry(std::string const& file_name, unsigned flags = DEFAULTS);

  /**
   *
   */
   void load_animation(std::shared_ptr<node::Node>& node, std::string const& file_name, unsigned flags);

   /**
   *
   */
   std::shared_ptr<node::Node> create_geometry_from_file(std::string const& node_name,
                                                   std::string const& file_name,
                                                   std::shared_ptr<Material> const& fallback_material,
                                                   unsigned flags = DEFAULTS);

  /**
   * Constructor from a file.
   *
   * Creates a new MeshLoader from a given file.
   *
   * \param file_name        The file to load the meshs data from.
   * \param material_name    The material name that was set to the parent node
   */
  std::shared_ptr<node::Node> load(std::string const& file_name,
                             unsigned flags);

  /**
   * Constructor from memory buffer.
   *
   * Creates a new MeshLoader from a existing memory buffer.
   *
   * \param buffer_name      The buffer to load the meh's data from.
   * \param buffer_size      The buffer's size.
   */
  // TODO
  /*std::vector<SkeletalAnimationRessource*> const load_from_buffer(char const* buffer_name,
                                                        unsigned buffer_size,
                                                        bool build_kd_tree);*/
  /**
  *
  */
  bool is_supported(std::string const& file_name) const;

 private: // methods

  /*std::shared_ptr<node::Node> get_tree(std::shared_ptr<Assimp::Importer> const& importer,
                aiScene const* ai_scene,
                aiNode* ai_root,
                std::string const& file_name,
                unsigned flags, unsigned& mesh_count);*/

  std::shared_ptr<node::Node> create_animation_node(std::shared_ptr<Assimp::Importer> const& importer,
                                              aiScene const* ai_scene,
                                              /*aiNode* ai_root,*/
                                              std::string const& file_name,
                                              unsigned flags);

  void apply_fallback_material(std::shared_ptr<node::Node> const& root, std::shared_ptr<Material> const& fallback_material) const;

private: // attributes

  std::string parent_material_name_;

  unsigned node_counter_;

  static std::unordered_map<std::string, std::shared_ptr<::gua::node::Node>> loaded_files_;
  static unsigned mesh_counter_;
};

}

#endif  // GUA_SKELETAL_ANIMATION_LOADER_HPP
