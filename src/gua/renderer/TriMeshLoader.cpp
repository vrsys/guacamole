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
#include <gua/utils/ToGua.hpp>
#include <gua/node/TriMeshNode.hpp>
#include <gua/node/TransformNode.hpp>
#include <gua/renderer/MaterialLoader.hpp>
#include <gua/renderer/TriMeshRessource.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>
#include <gua/databases/GeometryDatabase.hpp>

// external headers
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#ifdef GUACAMOLE_FBX
  #include <fbxsdk.h>
#endif // GUACAMOLE_FBX

namespace gua {

/////////////////////////////////////////////////////////////////////////////
// static variables
/////////////////////////////////////////////////////////////////////////////
std::unordered_map<std::string, std::shared_ptr< ::gua::node::Node> >
    TriMeshLoader::loaded_files_ =
        std::unordered_map<std::string, std::shared_ptr< ::gua::node::Node> >();

/////////////////////////////////////////////////////////////////////////////

TriMeshLoader::TriMeshLoader(){}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<node::Node> TriMeshLoader::load_geometry(
    std::string const& file_name,
    unsigned flags) {
  std::shared_ptr<node::Node> cached_node;
  std::string key(file_name + "_" + string_utils::to_string(flags));
  auto searched(loaded_files_.find(key));

  if (searched != loaded_files_.end()) {

    cached_node = searched->second;

  } else {

    bool fileload_succeed = false;

    if (is_supported(file_name)) {
      cached_node = load(file_name, flags);
      cached_node->update_cache();

      loaded_files_.insert(std::make_pair(key, cached_node));

      // normalize mesh position and rotation
      if (flags & TriMeshLoader::NORMALIZE_POSITION ||
          flags & TriMeshLoader::NORMALIZE_SCALE) {
        auto bbox = cached_node->get_bounding_box();

        if (flags & TriMeshLoader::NORMALIZE_POSITION) {
          auto center((bbox.min + bbox.max) * 0.5f);
          cached_node->translate(-center);
        }

        if (flags & TriMeshLoader::NORMALIZE_SCALE) {
          auto size(bbox.max - bbox.min);
          auto max_size(std::max(std::max(size.x, size.y), size.z));
          cached_node->scale(1.f / max_size);
        }

      }

      fileload_succeed = true;
    }

    if (!fileload_succeed) {

      Logger::LOG_WARNING << "Unable to load " << file_name
                          << ": Type is not supported!" << std::endl;
    }
  }

  return cached_node;
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<node::Node> TriMeshLoader::create_geometry_from_file(
    std::string const& node_name,
    std::string const& file_name,
    std::shared_ptr<Material> const& fallback_material,
    unsigned flags) {
  auto cached_node(load_geometry(file_name, flags));

  if (cached_node) {
    auto copy(cached_node->deep_copy());

    apply_fallback_material(
        copy, fallback_material, flags & NO_SHARED_MATERIALS);

    copy->set_name(node_name);
    return copy;
  }

  return std::make_shared<node::TransformNode>(node_name);
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<node::Node> TriMeshLoader::create_geometry_from_file(
    std::string const& node_name,
    std::string const& file_name,
    unsigned flags) {
  auto cached_node(load_geometry(file_name, flags));

  if (cached_node) {
    auto copy(cached_node->deep_copy());

    auto shader(gua::MaterialShaderDatabase::instance()->lookup(
        "gua_default_material"));
    apply_fallback_material(
        copy, shader->make_new_material(), flags & NO_SHARED_MATERIALS);

    copy->set_name(node_name);
    return copy;
  }

  return std::make_shared<node::TransformNode>(node_name);
}

/////////////////////////////////////////////////////////////////////////////

std::shared_ptr<node::Node> TriMeshLoader::load(
    std::string const& file_name,
    unsigned flags) {
  TextFile file(file_name);

  // MESSAGE("Loading mesh file %s", file_name.c_str());

  if (file.is_valid()) {
#ifdef GUACAMOLE_FBX
    auto point_pos(file_name.find_last_of("."));
    if(file_name.substr(point_pos + 1) == "fbx" || file_name.substr(point_pos + 1) == "FBX" ) {

      //The first thing to do is to create the FBX Manager which is the object allocator for almost all the classes in the SDK
      FbxManager* sdk_manager = FbxManager::Create();
      if(!sdk_manager) {
          Logger::LOG_ERROR <<"Error: Unable to create FBX Manager!\n";
          assert(0);
      }

      //Create an IOSettings object. This object holds all import/export settings.
      FbxIOSettings* ios = FbxIOSettings::Create(sdk_manager, IOSROOT);
      if(flags & TriMeshLoader::LOAD_MATERIALS){
        ios->SetBoolProp(IMP_FBX_MATERIAL,        true);
        ios->SetBoolProp(IMP_FBX_TEXTURE,         true);
      } 
      else {
        ios->SetBoolProp(IMP_FBX_MATERIAL,        false);
        ios->SetBoolProp(IMP_FBX_TEXTURE,         false);        
      }
      ios->SetBoolProp(IMP_FBX_CHARACTER,        false);
      ios->SetBoolProp(IMP_FBX_CONSTRAINT,       false);
      ios->SetBoolProp(IMP_FBX_LINK,            false);
      ios->SetBoolProp(IMP_FBX_SHAPE,           false);
      ios->SetBoolProp(IMP_FBX_MODEL,           true);
      ios->SetBoolProp(IMP_FBX_GOBO,            false);
      ios->SetBoolProp(IMP_FBX_ANIMATION,       false);
      ios->SetBoolProp(IMP_FBX_GLOBAL_SETTINGS, false);
      sdk_manager->SetIOSettings(ios);
      FbxScene* scene = load_fbx_file(sdk_manager, file_name);

      unsigned count(0);
      std::shared_ptr<node::Node> tree{get_tree(*scene->GetRootNode(), file_name, flags, count)};
      sdk_manager->Destroy();

      return tree;
    } else
#endif
    {
      auto importer = std::make_shared<Assimp::Importer>();

      unsigned ai_process_flags = aiProcessPreset_TargetRealtime_Quality |
                            aiProcess_RemoveComponent;

      if(flags & TriMeshLoader::OPTIMIZE_GEOMETRY) {
        ai_process_flags |= aiProcessPreset_TargetRealtime_MaxQuality |
                            aiProcess_PreTransformVertices;
      }

      unsigned ai_ignore_flags = aiComponent_COLORS |
                            aiComponent_ANIMATIONS |
                            aiComponent_LIGHTS |
                            aiComponent_CAMERAS |
                            aiComponent_BONEWEIGHTS;

      if(!(flags & TriMeshLoader::LOAD_MATERIALS)) {
        ai_ignore_flags |= aiComponent_MATERIALS;
      } 

      importer->SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE, aiPrimitiveType_POINT | aiPrimitiveType_LINE);
      importer->SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS, ai_ignore_flags);

      importer->ReadFile(file_name, ai_process_flags);

      aiScene const* scene(importer->GetScene());

      std::shared_ptr<node::Node> new_node;

      std::string error = importer->GetErrorString();
      if (!error.empty())
      {
        Logger::LOG_WARNING << "TriMeshLoader::load(): Importing failed, " << error << std::endl;
      }

      if (scene->mRootNode) {
        unsigned count = 0;
        new_node = get_tree(
            importer, scene, scene->mRootNode, file_name, flags, count);

      } else {
        Logger::LOG_WARNING << "Failed to load object \"" << file_name << "\": No valid root node contained!" << std::endl;
      }

      return new_node;
    }
  }

  Logger::LOG_WARNING << "Failed to load object \"" << file_name
                      << "\": File does not exist!" << std::endl;

  return nullptr;
}

/////////////////////////////////////////////////////////////////////////////

std::vector<TriMeshRessource*> const TriMeshLoader::load_from_buffer(
    char const* buffer_name,
    unsigned buffer_size,
    bool build_kd_tree) {

  auto importer = std::make_shared<Assimp::Importer>();

  aiScene const* scene(importer->ReadFileFromMemory(
      buffer_name,
      buffer_size,
      aiProcessPreset_TargetRealtime_Quality | aiProcess_CalcTangentSpace));

  std::vector<TriMeshRessource*> meshes;

  for (unsigned int n = 0; n < scene->mNumMeshes; ++n) {
    meshes.push_back(
        new TriMeshRessource(Mesh{*scene->mMeshes[n]}, build_kd_tree));
  }

  return meshes;

}

////////////////////////////////////////////////////////////////////////////////

bool TriMeshLoader::is_supported(std::string const& file_name) const {
  auto point_pos(file_name.find_last_of("."));
  Assimp::Importer importer;

  if (file_name.substr(point_pos + 1) == "raw") {
    return false;
  }
#ifdef GUACAMOLE_FBX
  else if (file_name.substr(point_pos + 1) == "fbx" || file_name.substr(point_pos + 1) == "FBX"){
    return true;
  }
#endif
  return importer.IsExtensionSupported(file_name.substr(point_pos + 1));
}

////////////////////////////////////////////////////////////////////////////////
#ifdef GUACAMOLE_FBX
std::shared_ptr<node::Node> TriMeshLoader::get_tree(
    FbxNode& fbx_node,
    std::string const& file_name,
    unsigned flags, unsigned& mesh_count) {
  // creates a geometry node and returns it
  auto load_geometry = [&](FbxNode& fbx_node) 
  {
    FbxMesh* fbx_mesh = fbx_node.GetMesh();

    GeometryDescription desc ("TriMesh", file_name, mesh_count++, flags);
    GeometryDatabase::instance()->add(desc.unique_key(), std::make_shared<TriMeshRessource>(Mesh{*fbx_mesh}, flags & TriMeshLoader::MAKE_PICKABLE));

    // load material
    std::shared_ptr<Material> material;

    if(fbx_node.GetMaterialCount() > 0 && flags & TriMeshLoader::LOAD_MATERIALS) {
      MaterialLoader material_loader;
      if(fbx_node.GetMaterialCount() > 1) {
        Logger::LOG_WARNING << "Trimesh has more than one material, using only first one" << std::endl;
      }
      FbxSurfaceMaterial* mat = fbx_node.GetMaterial(0);
      material = material_loader.load_material(*mat, file_name, flags & TriMeshLoader::OPTIMIZE_MATERIALS);
    }

    auto node = std::shared_ptr<node::TriMeshNode>(
        new node::TriMeshNode("", desc.unique_key(), material));

    node->set_transform(to_gua::mat4d(fbx_node.EvaluateGlobalTransform()));
    return node;
  };

  auto group(std::make_shared<node::TransformNode>());

  if(fbx_node.GetGeometry() != nullptr) {
    
    if(fbx_node.GetGeometry()->GetAttributeType() == FbxNodeAttribute::eMesh) {

      // no children ->just return this
      if (fbx_node.GetChildCount() == 0) {
        return load_geometry(fbx_node);
      }

      group->add_child(load_geometry(fbx_node));
    }
  }

  // there is only one child -- skip it!
  if (fbx_node.GetChildCount() == 1 && fbx_node.GetChild(0)->GetGeometry() != nullptr) {
    if(fbx_node.GetChild(0)->GetGeometry()->GetAttributeType() == FbxNodeAttribute::eMesh) {
      return get_tree(*fbx_node.GetChild(0), file_name, flags, mesh_count);
    }
  }

  // else: there are multiple children and meshes
  for (int i = 0; i < fbx_node.GetChildCount(); ++i) {
    group->add_child(get_tree(*fbx_node.GetChild(i), file_name, flags, mesh_count));
  }

  return group;
}
#endif
////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<node::Node> TriMeshLoader::get_tree(
    std::shared_ptr<Assimp::Importer> const& importer,
    aiScene const* ai_scene,
    aiNode* ai_root,
    std::string const& file_name,
    unsigned flags,
    unsigned& mesh_count) {

  // creates a geometry node and returns it
  auto load_geometry = [&](int i) {
    GeometryDescription desc("TriMesh", file_name, mesh_count++, flags);
    GeometryDatabase::instance()->add(
        desc.unique_key(),
        std::make_shared<TriMeshRessource>(
            Mesh {*ai_scene->mMeshes[ai_root->mMeshes[i]]},
            flags & TriMeshLoader::MAKE_PICKABLE));

    // load material
    std::shared_ptr<Material> material;
    unsigned material_index(ai_scene->mMeshes[ai_root->mMeshes[i]]
                                ->mMaterialIndex);

    if (material_index != 0 && flags & TriMeshLoader::LOAD_MATERIALS) {
      MaterialLoader material_loader;
      aiMaterial const* ai_material(ai_scene->mMaterials[material_index]);
      material = material_loader.load_material(ai_material, file_name,
                                               flags & TriMeshLoader::OPTIMIZE_MATERIALS);
    }

    //return std::make_shared<node::TriMeshNode>("", desc.unique_key(),
    //material); // not allowed -> private c'tor
    return std::shared_ptr<node::TriMeshNode>(
        new node::TriMeshNode("", desc.unique_key(), material));
  };

  // there is only one child -- skip it!
  if (ai_root->mNumChildren == 1 && ai_root->mNumMeshes == 0) {
    return get_tree(importer,
                    ai_scene,
                    ai_root->mChildren[0],
                    file_name,
                    flags,
                    mesh_count);
  }

  // there is only one geometry --- return it!
  if (ai_root->mNumChildren == 0 && ai_root->mNumMeshes == 1) {
    return load_geometry(0);
  }

  // else: there are multiple children and meshes
  auto group(std::make_shared<node::TransformNode>());

  for (unsigned i(0); i < ai_root->mNumMeshes; ++i) {
    group->add_child(load_geometry(i));
  }

  for (unsigned i(0); i < ai_root->mNumChildren; ++i) {
    group->add_child(get_tree(importer,
                              ai_scene,
                              ai_root->mChildren[i],
                              file_name,
                              flags,
                              mesh_count));
  }

  return group;
}

////////////////////////////////////////////////////////////////////////////////

void TriMeshLoader::apply_fallback_material(
    std::shared_ptr<node::Node> const& root,
    std::shared_ptr<Material> const& fallback_material,
    bool no_shared_materials) {
  auto g_node(std::dynamic_pointer_cast<node::TriMeshNode>(root));

  if (g_node && !g_node->get_material()) {
    g_node->set_material(fallback_material);
    g_node->update_cache();
  } else if (g_node && no_shared_materials) {
    g_node->set_material(std::make_shared<Material>(*g_node->get_material()));
  }

  for (auto& child : root->get_children()) {
    apply_fallback_material(child, fallback_material, no_shared_materials);
  }
}

////////////////////////////////////////////////////////////////////////////////
#ifdef GUACAMOLE_FBX
FbxScene* TriMeshLoader::load_fbx_file(
    FbxManager* manager,
    std::string const& file_name) {
  // Create an importer.
  FbxImporter* lImporter = FbxImporter::Create(manager,"");

  int lFileMajor, lFileMinor, lFileRevision;
  int lSDKMajor,  lSDKMinor,  lSDKRevision;
  // Get the file version number generate by the FBX SDK.
  FbxManager::GetFileFormatVersion(lSDKMajor, lSDKMinor, lSDKRevision);
  lImporter->GetFileVersion(lFileMajor, lFileMinor, lFileRevision);

  // Initialize the importer by providing a filename.
  const bool lImportStatus = lImporter->Initialize(file_name.c_str(), -1, manager->GetIOSettings());
  if(!lImportStatus)
  {
    FbxString error = lImporter->GetStatus().GetErrorString();
    Logger::LOG_ERROR << "Call to FbxImporter::Initialize() failed." << std::endl;
    Logger::LOG_ERROR << "Error returned: " << error.Buffer() << std::endl;

    if (lImporter->GetStatus().GetCode() == FbxStatus::eInvalidFileVersion)
    {
        Logger::LOG_ERROR <<"FBX file format version for this FBX SDK is " << lSDKMajor << "." << lSDKMinor << "." << lSDKRevision << std::endl;
        Logger::LOG_ERROR <<"FBX file format version for file '" << file_name << "' is " << lFileMajor << "." << lFileMinor << "." << lFileRevision << " does not match" << std::endl;
    }
    assert(0);
  }

  if(!lImporter->IsFBX())
  {
    Logger::LOG_ERROR << "File \"" << file_name << "\" is no fbx" << std::endl;
    assert(0);
  }

  //Create an FBX scene. This object holds most objects imported/exported from/to files.
  FbxScene* scene = FbxScene::Create(manager, "My Scene");
  if(!scene) {
      Logger::LOG_ERROR <<"Error: Unable to create FBX scene!\n";
      assert(0);
  }

  bool result = lImporter->Import(scene);
  if(!result) {
    Logger::LOG_ERROR << "Failed to load object \"" << file_name << "\"" << std::endl;
    assert(0);
  } 

  return scene;
}
#endif
}
