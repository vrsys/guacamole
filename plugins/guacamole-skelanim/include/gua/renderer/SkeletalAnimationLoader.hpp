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
#include <gua/platform.hpp>

// external headers
#include <string>
#include <memory>
#include <vector>

namespace fbxsdk_2015_1 {
class FbxScene;
class FbxManager;
}

namespace Assimp { class Importer; }
class aiScene;

namespace gua {
class SkeletalAnimation;
class Material;

namespace node {
class Node;
class InnerNode;
class GeometryNode;
class SkeletalAnimationNode;
}

/**
 * Loads skinned meshes and animations.
 *
 * This class can load mesh and animation data from files and returns
 * SkeletalanimationNodes.
 */
class GUA_DLL SkeletalAnimationLoader {

 public:  // typedefs, enums

  enum Flags {
    DEFAULTS = 0,
    LOAD_MATERIALS = 1 << 0,
    OPTIMIZE_GEOMETRY = 1 << 1,
    MAKE_PICKABLE = 1 << 2,
    NORMALIZE_POSITION = 1 << 3,
    NORMALIZE_SCALE = 1 << 4,
    NO_SHARED_MATERIALS = 1 << 5,
    OPTIMIZE_MATERIALS = 1 << 6
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
  std::shared_ptr<node::SkeletalAnimationNode> load_geometry(
      std::string const& file_name,
      std::string const& node_name,
      unsigned flags = DEFAULTS);

  /**
   *
   */
  std::vector<SkeletalAnimation> load_animation(
      std::string const& file_name,
      std::string const& animation_name);

  /**
  *
  */
  std::shared_ptr<node::SkeletalAnimationNode> create_geometry_from_file(
      std::string const& node_name,
      std::string const& file_name,
      std::shared_ptr<Material> const& fallback_material,
      unsigned flags = DEFAULTS);

  std::shared_ptr<node::SkeletalAnimationNode> create_geometry_from_file(
      std::string const& node_name,
      std::string const& file_name,
      unsigned flags = DEFAULTS);
  /**
   * Constructor from a file.
   *
   * Creates a new MeshLoader from a given file.
   *
   * \param file_name        The file to load the meshs data from.
   * \param material_name    The material name that was set to the parent node
   */
  std::shared_ptr<node::SkeletalAnimationNode> load(
      std::string const& file_name,
      std::string const& node_name,
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
  /*std::vector<SkeletalAnimationRessource*> const load_from_buffer(char const*
                                                        buffer_name,
                                                        unsigned buffer_size,
                                                        bool build_kd_tree);*/
  /**
  *
  */
  bool is_supported(std::string const& file_name) const;

 private:  // methods

  static std::shared_ptr<node::SkeletalAnimationNode> get_node(
      fbxsdk_2015_1::FbxScene* fbx_scene,
      std::string const& file_name,
      std::string const& node_name,
      unsigned flags);

  static std::shared_ptr<node::SkeletalAnimationNode> get_node(
      aiScene const* ai_scene,
      std::string const& file_name,
      std::string const& node_name,
      unsigned flags);

  static void apply_fallback_material(
      std::shared_ptr<node::SkeletalAnimationNode> const& root,
      std::shared_ptr<Material> const& fallback_material,
      bool no_shared_materials);

  static fbxsdk_2015_1::FbxScene* load_fbx_file(
      fbxsdk_2015_1::FbxManager* manager,
      std::string const& file_path);
};

}

#endif  // GUA_SKELETAL_ANIMATION_LOADER_HPP
