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
#include <gua/renderer/SkeletalAnimationLoader.hpp>

// guacamole headers
#include <gua/utils/TextFile.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/utils/string_utils.hpp>
#include <gua/node/SkeletalAnimationNode.hpp>
#include <gua/node/TransformNode.hpp>
#include <gua/renderer/MaterialLoader.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>
#include <gua/databases/GeometryDatabase.hpp>

// external headers
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <fbxsdk.h>

namespace gua {

/////////////////////////////////////////////////////////////////////////////

SkeletalAnimationLoader::SkeletalAnimationLoader()
{}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<node::Node> SkeletalAnimationLoader::load_geometry(std::string const& file_name, std::string const& node_name, unsigned flags)
{
  std::shared_ptr<node::Node> new_node;

  bool fileload_succeed = false;

  if (is_supported(file_name))
  {
    new_node = load(file_name,node_name, flags);
    new_node->update_cache();

    // normalize mesh position and rotation
    if (flags & SkeletalAnimationLoader::NORMALIZE_POSITION || flags & SkeletalAnimationLoader::NORMALIZE_SCALE) {
      auto bbox = new_node->get_bounding_box();

      if (flags & SkeletalAnimationLoader::NORMALIZE_POSITION) {
        auto center((bbox.min + bbox.max)*0.5f);
        new_node->translate(-center);
      }

      if (flags & SkeletalAnimationLoader::NORMALIZE_SCALE) {
        auto size(bbox.max - bbox.min);
        auto max_size(std::max(std::max(size.x, size.y), size.z));
        new_node->scale(1.f / max_size);
      }

    }

    fileload_succeed = true;
  }

  if (!fileload_succeed) {

    Logger::LOG_WARNING << "Unable to load " << file_name << ": Type is not supported!" << std::endl;
  }

  return new_node;
}

void SkeletalAnimationLoader::load_animation(std::shared_ptr<node::Node>& node, std::string const& file_name, std::string const& animation_name)
{
  std::shared_ptr<node::SkeletalAnimationNode> skelNode = std::dynamic_pointer_cast<node::SkeletalAnimationNode, node::Node>(node);

  if(!skelNode) Logger::LOG_ERROR << "Node is no SkeletalAnimationNode" << std::endl;

  if (!is_supported(file_name)) {
    Logger::LOG_WARNING << "Unable to load " << file_name << ": Type is not supported!" << std::endl;
    return;
  }

  TextFile file(file_name);

  if (!file.is_valid()) {
    Logger::LOG_WARNING << "Failed to load object \"" << file_name << "\": File does not exist!" << std::endl;
    return;
  }

  auto point_pos(file_name.find_last_of("."));

  if(file_name.substr(point_pos + 1) == "fbx" || file_name.substr(point_pos + 1) == "FBX") {

    //The first thing to do is to create the FBX Manager which is the object allocator for almost all the classes in the SDK
    FbxManager* sdk_manager = FbxManager::Create();
    if(!sdk_manager) {
        Logger::LOG_ERROR <<"Error: Unable to create FBX Manager!\n";
        assert(0);
    }

    //Create an IOSettings object. This object holds all import/export settings.
    FbxIOSettings* ios = FbxIOSettings::Create(sdk_manager, IOSROOT);
    ios->SetBoolProp(IMP_FBX_MATERIAL,        false);
    ios->SetBoolProp(IMP_FBX_TEXTURE,         false);
    ios->SetBoolProp(IMP_FBX_CHARACTER,        false);
    ios->SetBoolProp(IMP_FBX_CONSTRAINT,       false);
    ios->SetBoolProp(IMP_FBX_LINK,            false);
    ios->SetBoolProp(IMP_FBX_SHAPE,           false);
    ios->SetBoolProp(IMP_FBX_MODEL,           false);
    ios->SetBoolProp(IMP_FBX_GOBO,            false);
    ios->SetBoolProp(IMP_FBX_ANIMATION,       true);
    ios->SetBoolProp(IMP_FBX_GLOBAL_SETTINGS, false);
    sdk_manager->SetIOSettings(ios);

    FbxScene* scene = load_fbx_file(sdk_manager, file_name);
    
    skelNode->add_animations(*scene,animation_name);

    sdk_manager->Destroy();
  }
  else {
    auto importer = std::make_shared<Assimp::Importer>();

    unsigned ai_ignore_flags = aiComponent_COLORS |
                        aiComponent_LIGHTS |
                        aiComponent_CAMERAS |
                        aiComponent_MATERIALS |
                        aiComponent_MESHES;

    importer->SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS, ai_ignore_flags);
    // removecomponent flag to enable removal of AI_CONFIG_PP_RVC_FLAGS
    importer->ReadFile(file_name, 0);

    aiScene const* scene(importer->GetScene());

    std::string error = importer->GetErrorString();
    if (!error.empty()) {
      Logger::LOG_WARNING << "SkeletalAnimationLoader::load_animation(): Importing failed, " << error << std::endl;
    }

    if(scene->HasAnimations()) {
      skelNode->add_animations(*scene,animation_name);
    }
    else {
      Logger::LOG_WARNING << "object \"" << file_name << "\" contains no animations!" << std::endl;
    }
  }
}
/////////////////////////////////////////////////////////////////////////////
std::shared_ptr<node::Node> SkeletalAnimationLoader::create_geometry_from_file(std::string const& node_name,
  std::string const& file_name,
  unsigned flags)
{
  auto new_node(load_geometry(file_name,node_name,flags));

  if (new_node) {

    auto shader(gua::MaterialShaderDatabase::instance()->lookup("gua_default_material"));
    apply_fallback_material(new_node, shader->make_new_material());
    new_node->set_name(node_name);

    return new_node;
  }

  return std::make_shared<node::TransformNode>(node_name);
}
////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<node::Node> SkeletalAnimationLoader::create_geometry_from_file(std::string const& node_name,
  std::string const& file_name,
  std::shared_ptr<Material> const& fallback_material,
  unsigned flags)
{
  auto new_node(load_geometry(file_name,node_name,flags));

  if (new_node) {

    apply_fallback_material(new_node, fallback_material);
    new_node->set_name(node_name);

    return new_node;
  }

  return std::make_shared<node::TransformNode>(node_name);
}

/////////////////////////////////////////////////////////////////////////////

std::shared_ptr<node::Node> SkeletalAnimationLoader::load(std::string const& file_name, std::string const& node_name,
 unsigned flags) {

  TextFile file(file_name);

  if (file.is_valid()) {
    auto point_pos(file_name.find_last_of("."));

    if(file_name.substr(point_pos + 1) == "fbx" || file_name.substr(point_pos + 1) == "FBX") {

      //Create manager to hold resources
      FbxManager* sdk_manager = FbxManager::Create();
      if(!sdk_manager) {
          Logger::LOG_ERROR <<"Error: Unable to create FBX Manager!\n";
          assert(0);
      }

      //set import settings
      FbxIOSettings* ios = FbxIOSettings::Create(sdk_manager, IOSROOT);
      if(flags & SkeletalAnimationLoader::LOAD_MATERIALS){
        ios->SetBoolProp(IMP_FBX_MATERIAL,        true);
        ios->SetBoolProp(IMP_FBX_TEXTURE,         true);
      } 
      else {
        ios->SetBoolProp(IMP_FBX_MATERIAL,        false);
        ios->SetBoolProp(IMP_FBX_TEXTURE,         false);        
      }
      ios->SetBoolProp(IMP_FBX_CHARACTER,        false);
      ios->SetBoolProp(IMP_FBX_CONSTRAINT,       false);
      ios->SetBoolProp(IMP_FBX_LINK,            true);
      ios->SetBoolProp(IMP_FBX_SHAPE,           false);
      ios->SetBoolProp(IMP_FBX_MODEL,           true);
      ios->SetBoolProp(IMP_FBX_GOBO,            false);
      ios->SetBoolProp(IMP_FBX_ANIMATION,       false);
      ios->SetBoolProp(IMP_FBX_GLOBAL_SETTINGS, false);
      sdk_manager->SetIOSettings(ios);
      FbxScene* scene = load_fbx_file(sdk_manager, file_name);

      std::shared_ptr<node::Node> new_node = get_node(scene, file_name, node_name, flags);
      sdk_manager->Destroy();

      return new_node;
    }
    else {  
      auto importer = std::make_shared<Assimp::Importer>();

      // equals aiProcessPreset_TargetRealtime_Quality, but with weight limiting removed
      unsigned ai_process_flags = aiProcess_CalcTangentSpace |
                          aiProcess_GenSmoothNormals |
                          aiProcess_JoinIdenticalVertices |
                          aiProcess_ImproveCacheLocality |
                          aiProcess_RemoveRedundantMaterials |
                          aiProcess_SplitLargeMeshes |
                          aiProcess_Triangulate |
                          aiProcess_GenUVCoords |
                          aiProcess_SortByPType |
                          aiProcess_FindDegenerates |
                          aiProcess_FindInvalidData |
                          aiProcess_RemoveComponent ; //extra, enable component ignoring while loading

      // equals aiProcessPreset_TargetRealtime_MaxQuality
      if (flags & SkeletalAnimationLoader::OPTIMIZE_GEOMETRY) {
      ai_process_flags |= aiProcess_FindInstances |
                          aiProcess_ValidateDataStructure |
                          aiProcess_OptimizeMeshes |
                          aiProcess_Debone |             //extra flags:
                          aiProcess_OptimizeGraph |
                          aiProcess_PreTransformVertices;
      } 

      unsigned ai_ignore_flags = aiComponent_COLORS |
                          aiComponent_ANIMATIONS |
                          aiComponent_LIGHTS |
                          aiComponent_CAMERAS;

      if(!(flags & SkeletalAnimationLoader::LOAD_MATERIALS)) {
        ai_ignore_flags |= aiComponent_MATERIALS;
      } 

      importer->SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE, aiPrimitiveType_POINT | aiPrimitiveType_LINE);
      importer->SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS, ai_ignore_flags);

      importer->ReadFile(file_name, ai_process_flags);

      aiScene const* scene(importer->GetScene());

      std::shared_ptr<node::Node> new_node;

      std::string error = importer->GetErrorString();
      if (!error.empty()) {
        Logger::LOG_WARNING << "SkeletalAnimationLoader::load(): Importing failed, " << error << std::endl;
      }

      if (scene->mRootNode) {

        new_node = get_node(scene, file_name, node_name, flags);

      } else {
        Logger::LOG_WARNING << "Failed to load object \"" << file_name << "\": No valid root node contained!" << std::endl;
      }

      return new_node;
    }
  }
  else {

    Logger::LOG_WARNING << "Failed to load object \"" << file_name << "\": File does not exist!" << std::endl;

    return nullptr;
  }
}

/////////////////////////////////////////////////////////////////////////////
std::shared_ptr<node::Node> SkeletalAnimationLoader::get_node(FbxScene* scene,
  std::string const& file_name,
  std::string const& node_name,
  unsigned flags) {

  std::shared_ptr<Bone> root = std::make_shared<Bone>(*scene);

  std::vector<std::string> geometry_descriptions{};
  std::vector<std::shared_ptr<Material>> materials{};
  unsigned mesh_count = 0;
  for(unsigned i = 0; i < scene->GetGeometryCount(); ++i) {
    FbxGeometry* geo = scene->GetGeometry(i);
    if(geo->GetAttributeType() == FbxNodeAttribute::eMesh) {  

      FbxNode* node = geo->GetNode();
      FbxMesh* mesh = dynamic_cast<FbxMesh*>(geo);

      for(unsigned j = 0; j < node->GetMaterialCount(); ++j)
      {
        GeometryDescription desc("SkeletalAnimation", file_name, mesh_count,flags);
        geometry_descriptions.push_back(desc.unique_key());

        GeometryDatabase::instance()->add(
          desc.unique_key() 
          ,std::make_shared<SkinnedMeshResource>(
            SkinnedMesh{*mesh, *root, j}
            , flags & SkeletalAnimationLoader::MAKE_PICKABLE
          )
        );

        //there need to be as many materials as there are geometries or the geometries without material wont be drawn
        std::shared_ptr<Material> material{};
        
        if(flags & SkeletalAnimationLoader::LOAD_MATERIALS) {
          MaterialLoader material_loader;
          FbxSurfaceMaterial* mat = node->GetMaterial(j);
          material = material_loader.load_material(*mat, file_name);
        }

        materials.push_back(material);
        ++mesh_count;
      }
    }
  }

  return std::make_shared<node::SkeletalAnimationNode>(file_name + "_" + node_name, geometry_descriptions, materials, root);
}
/////////////////////////////////////////////////////////////////////////////

std::shared_ptr<node::Node> SkeletalAnimationLoader::get_node(aiScene const* ai_scene,
  std::string const& file_name,
  std::string const& node_name,
  unsigned flags) {

  auto root = std::make_shared<Bone>(*ai_scene);

  std::vector<std::string> geometry_descriptions{};
  std::vector<std::shared_ptr<Material>> materials{};

  for(unsigned i = 0; i < ai_scene->mNumMeshes; ++i){

    GeometryDescription desc("SkeletalAnimation", file_name, i,flags);
    geometry_descriptions.push_back(desc.unique_key());

    GeometryDatabase::instance()->add(desc.unique_key() 
      ,std::make_shared<SkinnedMeshResource>(
        SkinnedMesh{*ai_scene->mMeshes[i], *root}
        , flags & SkeletalAnimationLoader::MAKE_PICKABLE));

    std::shared_ptr<Material> material;
    unsigned material_index(ai_scene->mMeshes[i]->mMaterialIndex);

    if (flags & SkeletalAnimationLoader::LOAD_MATERIALS) {
      MaterialLoader material_loader;
      aiMaterial const* ai_material(ai_scene->mMaterials[material_index]);
      material = material_loader.load_material(ai_material, file_name);
    }
    materials.push_back(material);
  }

  return std::make_shared<node::SkeletalAnimationNode>(file_name + "_" + node_name, geometry_descriptions, materials, root);
}

////////////////////////////////////////////////////////////////////////////////
// TODO
/*std::vector<SkinnedMeshResource*> const SkeletalAnimationLoader::load_from_buffer(char const* buffer_name,
 unsigned buffer_size,
 bool build_kd_tree) {

  auto importer = std::make_shared<Assimp::Importer>();

  aiScene const* scene(importer->ReadFileFromMemory(
    buffer_name,
    buffer_size,
    aiProcessPreset_TargetRealtime_Quality | aiProcess_CalcTangentSpace));

  std::vector<SkinnedMeshResource*> meshes;

  for (unsigned int n = 0; n < scene->mNumMeshes; ++n) {
    meshes.push_back(new SkinnedMeshResource(scene->mMeshes[n], importer, build_kd_tree));
  }

  return meshes;

}*/

////////////////////////////////////////////////////////////////////////////////

bool SkeletalAnimationLoader::is_supported(std::string const& file_name) const {
  auto point_pos(file_name.find_last_of("."));
  Assimp::Importer importer;

  if (file_name.substr(point_pos + 1) == "raw"){
    return false;
  }
  else if (file_name.substr(point_pos + 1) == "fbx" || file_name.substr(point_pos + 1) == "FBX"){
    return true;
  }

  return importer.IsExtensionSupported(file_name.substr(point_pos + 1));
}

////////////////////////////////////////////////////////////////////////////////

void SkeletalAnimationLoader::apply_fallback_material(std::shared_ptr<node::Node> const& root,
  std::shared_ptr<Material> const& fallback_material)
{
  auto g_node(std::dynamic_pointer_cast<node::SkeletalAnimationNode>(root));
  
  if(g_node) {
    auto& materials = g_node->get_materials();
    for(unsigned i = 0; i < materials.size(); ++i) {
      if(!materials[i]) g_node->set_material(fallback_material, i);
    }

    g_node->update_cache();
  }
}

////////////////////////////////////////////////////////////////////////////////

FbxScene* SkeletalAnimationLoader::load_fbx_file(FbxManager* manager, std::string const& file_name) {
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

}
