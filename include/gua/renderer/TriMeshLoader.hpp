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

#ifndef GUA_TRI_MESH_LOADER_HPP
#define GUA_TRI_MESH_LOADER_HPP

// guacamole headers
#include <gua/renderer/TriMeshRessource.hpp>
#include <gua/renderer/Material.hpp>
#include <gua/utils/Mesh.hpp>

// external headers
#include <string>
#include <list>
#include <memory>

namespace Assimp
{
class Importer;
}
struct aiScene;
struct aiNode;

namespace gua
{
namespace node
{
class Node;
class InnerNode;
class GeometryNode;
} // namespace node

/**
 * Loads and draws meshes.
 *
 * This class can load mesh data from files and display them in multiple
 * contexts. A MeshLoader object is made of several Mesh objects.
 */
class GUA_DLL TriMeshLoader
{
  public: // typedefs, enums
    enum Flags
    {
        DEFAULTS = 0,
        LOAD_MATERIALS = 1 << 0,
        OPTIMIZE_GEOMETRY = 1 << 1,
        MAKE_PICKABLE = 1 << 2,
        NORMALIZE_POSITION = 1 << 3,
        NORMALIZE_SCALE = 1 << 4,
        NO_SHARED_MATERIALS = 1 << 5,
        OPTIMIZE_MATERIALS = 1 << 6,
        PARSE_HIERARCHY = 1 << 7
    };

  public:
    /**
     * Default constructor.
     *
     * Constructs a new and empty MeshLoader.
     */
    TriMeshLoader();

    /**
     *
     */
    std::shared_ptr<node::Node> load_geometry(std::string const& file_name, unsigned flags = DEFAULTS);

    /**
     *
     */
    std::shared_ptr<node::Node> create_geometry_from_file(std::string const& node_name, std::string const& file_name, std::shared_ptr<Material> const& fallback_material, unsigned flags = DEFAULTS);

    std::shared_ptr<node::Node> create_geometry_from_file(std::string const& node_name, std::string const& file_name, unsigned flags = DEFAULTS);

    /**
     * Constructor from a file.
     *
     * Creates a new MeshLoader from a given file.
     *
     * \param file_name        The file to load the meshs data from.
     * \param material_name    The material name that was set to the parent node
     */
    std::shared_ptr<node::Node> load(std::string const& file_name, unsigned flags);

    /**
     * Constructor from memory buffer.
     *
     * Creates a new MeshLoader from a existing memory buffer.
     *
     * \param buffer_name      The buffer to load the meh's data from.
     * \param buffer_size      The buffer's size.
     */
    std::vector<TriMeshRessource*> const load_from_buffer(char const* buffer_name, unsigned buffer_size, bool build_kd_tree);
    /**
     *
     */
    bool is_supported(std::string const& file_name) const;

  private: // methods
    static std::shared_ptr<node::Node>
    get_tree(std::shared_ptr<Assimp::Importer> const& importer, aiScene const* ai_scene, aiNode* ai_root, std::string const& file_name, unsigned flags, unsigned& mesh_count, bool enforce_hierarchy);

    static void apply_fallback_material(std::shared_ptr<node::Node> const& root, std::shared_ptr<Material> const& fallback_material, bool no_shared_materials);

#ifdef GUACAMOLE_FBX
    static std::shared_ptr<node::Node> get_tree(FbxNode& node, std::string const& file_name, unsigned flags, unsigned& mesh_count);

    static FbxScene* load_fbx_file(FbxManager* manager, std::string const& file_path);
#endif

  private: // attributes
    static std::unordered_map<std::string, std::shared_ptr<::gua::node::Node>> loaded_files_;
    static gua::math::mat4 convert_transformation(aiMatrix4x4t<float> const& transform_mat);
    static void apply_transformation(std::shared_ptr<node::Node> node, aiMatrix4x4t<float> const& transform_mat);
};

} // namespace gua

#endif // GUA_TRI_MESH_LOADER_HPP
