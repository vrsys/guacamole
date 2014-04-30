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

// class header
#include <gua/renderer/TriMeshLoader.hpp>

// guacamole headers
#include <gua/utils/TextFile.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/utils/string_utils.hpp>
#include <gua/scenegraph/TriMeshNode.hpp>
#include <gua/scenegraph/TransformNode.hpp>
#include <gua/renderer/Material.hpp>
#include <gua/renderer/MaterialLoader.hpp>
#include <gua/renderer/GeometryLoader.hpp>
#include <gua/renderer/TriMeshRessource.hpp>
#include <gua/databases/MaterialDatabase.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/ShadingModelDatabase.hpp>

namespace gua {

  unsigned TriMeshLoader::mesh_counter_ = 0;

  /////////////////////////////////////////////////////////////////////////////

  TriMeshLoader::TriMeshLoader()
    : node_counter_(0) {}

  /////////////////////////////////////////////////////////////////////////////

  std::shared_ptr<Node> TriMeshLoader::load(std::string const& file_name,
                                       unsigned flags) {

  node_counter_ = 0;
  TextFile file(file_name);

  // MESSAGE("Loading mesh file %s", file_name.c_str());

  if (file.is_valid()) {
    auto importer = std::make_shared<Assimp::Importer>();

    importer->SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE,
                                  aiPrimitiveType_POINT | aiPrimitiveType_LINE);

    if ((flags & GeometryLoader::OPTIMIZE_GEOMETRY) &&
        (flags & GeometryLoader::LOAD_MATERIALS)) {

      importer->SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS, aiComponent_COLORS);
      importer->ReadFile(
          file_name,
          aiProcessPreset_TargetRealtime_MaxQuality | aiProcess_GenNormals |
              aiProcess_RemoveComponent | aiProcess_OptimizeGraph |
              aiProcess_PreTransformVertices);

    } else if (flags & GeometryLoader::OPTIMIZE_GEOMETRY) {

      importer->SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS,
                                    aiComponent_COLORS | aiComponent_MATERIALS);
      importer->ReadFile(
          file_name,
          aiProcessPreset_TargetRealtime_MaxQuality | aiProcess_GenNormals |
              aiProcess_RemoveComponent | aiProcess_OptimizeGraph |
              aiProcess_PreTransformVertices);
    } else {

      importer->ReadFile(
          file_name,
          aiProcessPreset_TargetRealtime_Quality | aiProcess_GenNormals);

    }

    aiScene const* scene(importer->GetScene());

    std::shared_ptr<Node> new_node;

    if (scene->mRootNode) {
      // new_node = std::make_shared(new GeometryNode("unnamed",
      //                             GeometryNode::Configuration("", ""),
      //                             math::mat4::identity()));
      unsigned count(0);
      new_node = get_tree(importer, scene, scene->mRootNode, file_name, flags, count);

    } else {
      Logger::LOG_WARNING << "Failed to load object \"" << file_name << "\": No valid root node contained!" << std::endl;
    }

    return new_node;

  }

  Logger::LOG_WARNING << "Failed to load object \"" << file_name << "\": File does not exist!" << std::endl;

  return nullptr;
}

  /////////////////////////////////////////////////////////////////////////////

std::vector<TriMeshRessource*> const TriMeshLoader::load_from_buffer(char const* buffer_name,
                                                                     unsigned buffer_size,
                                                                     bool build_kd_tree) {

  auto importer = std::make_shared<Assimp::Importer>();

  aiScene const* scene(importer->ReadFileFromMemory(
      buffer_name,
      buffer_size,
      aiProcessPreset_TargetRealtime_Quality | aiProcess_CalcTangentSpace));

  std::vector<TriMeshRessource*> meshes;

  for (unsigned int n = 0; n < scene->mNumMeshes; ++n) {
    meshes.push_back(new TriMeshRessource(scene->mMeshes[n], importer, build_kd_tree));
  }

  return meshes;

}

bool TriMeshLoader::is_supported(std::string const& file_name) const {
  auto point_pos(file_name.find_last_of("."));
  Assimp::Importer importer;

  if (file_name.substr(point_pos + 1) == "raw"){
	  return false;
  }

  return importer.IsExtensionSupported(file_name.substr(point_pos + 1));
}

std::shared_ptr<Node> TriMeshLoader::get_tree(std::shared_ptr<Assimp::Importer> const& importer,
                                              aiScene const* ai_scene,
                                              aiNode* ai_root,
                                              std::string const& file_name,
                                              unsigned flags, unsigned& mesh_count) {

  // creates a geometry node and returns it
  auto load_geometry = [&](int i) {
    // load geometry
    std::string mesh_name("type=file&file=" + file_name + "&id=" + string_utils::to_string(mesh_count++) + "&flags=" + string_utils::to_string(flags));
    GeometryDatabase::instance()->add(mesh_name, std::make_shared<TriMeshRessource>(ai_scene->mMeshes[ai_root->mMeshes[i]], importer, flags & GeometryLoader::MAKE_PICKABLE));

    // load material
    std::string material_name("");
    unsigned material_index(ai_scene->mMeshes[ai_root->mMeshes[i]]->mMaterialIndex);

    if (material_index != 0 && flags & GeometryLoader::LOAD_MATERIALS) {
      MaterialLoader material_loader;
      aiMaterial const* material(ai_scene->mMaterials[material_index]);
      material_name = material_loader.load_material(material, file_name);
    }

    auto result(std::make_shared<TriMeshNode>(mesh_name));
    result->set_filename(mesh_name);
    result->set_material(material_name);

    return result;
  };

  // there is only one child -- skip it!
  if (ai_root->mNumChildren == 1 && ai_root->mNumMeshes == 0) {
    return get_tree(
      importer, ai_scene, ai_root->mChildren[0],
      file_name, flags, mesh_count
    );
  }

  // there is only one geometry --- return it!
  if (ai_root->mNumChildren == 0 && ai_root->mNumMeshes == 1) {
    return load_geometry(0);
  }

  // else: there are multiple children and meshes
  auto group(std::make_shared<TransformNode>());

  for (unsigned i(0); i < ai_root->mNumMeshes; ++i) {
    group->add_child(load_geometry(i));
  }

  for (unsigned i(0); i < ai_root->mNumChildren; ++i) {
    group->add_child(
      get_tree(
        importer, ai_scene, ai_root->mChildren[i],
        file_name, flags, mesh_count
      )
    );
  }

  return group;
}

}
