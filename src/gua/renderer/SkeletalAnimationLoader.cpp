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
#include <gua/renderer/SkeletalAnimationRessource.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>
#include <gua/databases/GeometryDatabase.hpp>

namespace gua {

  /////////////////////////////////////////////////////////////////////////////
  // static variables
  /////////////////////////////////////////////////////////////////////////////
  unsigned SkeletalAnimationLoader::mesh_counter_ = 0;

  std::unordered_map<std::string, std::shared_ptr<::gua::node::Node>>
    SkeletalAnimationLoader::loaded_files_ =
    std::unordered_map<std::string, std::shared_ptr<::gua::node::Node>>();
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////


  /////////////////////////////////////////////////////////////////////////////

  SkeletalAnimationLoader::SkeletalAnimationLoader()
    : node_counter_(0)
  {}

  ////////////////////////////////////////////////////////////////////////////////

  std::shared_ptr<node::Node> SkeletalAnimationLoader::load_geometry(std::string const& file_name, std::string const& node_name, unsigned flags)
  {
    std::shared_ptr<node::Node> cached_node;
    //std::string key(file_name + "_" + node_name + "_" + string_utils::to_string(flags));
    //auto searched(loaded_files_.find(key));

    
    // DO NOT CACHE NODE - can have different animations
    /*if (searched != loaded_files_.end()) {

      cached_node = searched->second;

    }
    else {*/

      bool fileload_succeed = false;

      if (is_supported(file_name))
      {
        cached_node = load(file_name,node_name, flags);
        cached_node->update_cache();

        //loaded_files_.insert(std::make_pair(key, cached_node));

        // normalize mesh position and rotation
        if (flags & SkeletalAnimationLoader::NORMALIZE_POSITION || flags & SkeletalAnimationLoader::NORMALIZE_SCALE) {
          auto bbox = cached_node->get_bounding_box();

          if (flags & SkeletalAnimationLoader::NORMALIZE_POSITION) {
            auto center((bbox.min + bbox.max)*0.5f);
            cached_node->translate(-center);
          }

          if (flags & SkeletalAnimationLoader::NORMALIZE_SCALE) {
            auto size(bbox.max - bbox.min);
            auto max_size(std::max(std::max(size.x, size.y), size.z));
            cached_node->scale(1.f / max_size);
          }

        }

        fileload_succeed = true;
      }

      if (!fileload_succeed) {

        Logger::LOG_WARNING << "Unable to load " << file_name << ": Type is not supported!" << std::endl;
      }
    //}

    return cached_node;
  }

  void SkeletalAnimationLoader::load_animation(std::shared_ptr<node::Node>& node, std::string const& file_name, unsigned flags)
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

    auto importer = std::make_shared<Assimp::Importer>();

    importer->SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE, aiPrimitiveType_POINT | aiPrimitiveType_LINE);

    importer->ReadFile(file_name, aiProcess_LimitBoneWeights);

    aiScene const* scene(importer->GetOrphanedScene());

    if(scene->HasAnimations()) {
      skelNode->get_director()->add_animations(scene);
    }
    else {
      Logger::LOG_WARNING << "object \"" << file_name << "\" contains no animations!" << std::endl;
    }
  }
  ////////////////////////////////////////////////////////////////////////////////

  std::shared_ptr<node::Node> SkeletalAnimationLoader::create_geometry_from_file(std::string const& node_name,
    std::string const& file_name,
    std::shared_ptr<Material> const& fallback_material,
    unsigned flags)
  {
      
    auto cached_node(load_geometry(file_name,node_name,flags));

    if (cached_node) {
      
      //auto copy(cached_node->deep_copy());

      //apply_fallback_material(copy, fallback_material);
      apply_fallback_material(cached_node, fallback_material);

      //copy->set_name(node_name);
      cached_node->set_name(node_name);
      //return copy;
      return cached_node;
    }

    return std::make_shared<node::TransformNode>(node_name);
  }

  /////////////////////////////////////////////////////////////////////////////

  std::shared_ptr<node::Node> SkeletalAnimationLoader::load(std::string const& file_name, std::string const& node_name,
                                       unsigned flags) {

  node_counter_ = 0;
  TextFile file(file_name);

  // MESSAGE("Loading mesh file %s", file_name.c_str());

  if (file.is_valid()) {
    auto importer = std::make_shared<Assimp::Importer>();

    importer->SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE,
                                  aiPrimitiveType_POINT | aiPrimitiveType_LINE);
    //prevent md5loader from loading animation with same name as mesh
    importer->SetPropertyBool(AI_CONFIG_IMPORT_MD5_NO_ANIM_AUTOLOAD, true);

    if ((flags & SkeletalAnimationLoader::OPTIMIZE_GEOMETRY) &&
        (flags & SkeletalAnimationLoader::LOAD_MATERIALS)) {

      importer->SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS, aiComponent_COLORS);
      importer->ReadFile(
          file_name,
          aiProcessPreset_TargetRealtime_MaxQuality |
              aiProcess_RemoveComponent | aiProcess_OptimizeGraph |
              aiProcess_PreTransformVertices | aiProcess_GenSmoothNormals | aiProcess_LimitBoneWeights);

    }
    else if (flags & SkeletalAnimationLoader::OPTIMIZE_GEOMETRY) {

      importer->SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS,
                                    aiComponent_COLORS | aiComponent_MATERIALS);
      importer->ReadFile(
          file_name,
          aiProcessPreset_TargetRealtime_MaxQuality |
              aiProcess_RemoveComponent | aiProcess_OptimizeGraph |
              aiProcess_PreTransformVertices | aiProcess_GenSmoothNormals | aiProcess_LimitBoneWeights);
    } else {

      importer->ReadFile(
          file_name,
          aiProcessPreset_TargetRealtime_Quality | aiProcess_GenSmoothNormals | aiProcess_LimitBoneWeights);

    }

    aiScene const* scene(importer->GetScene());

    std::shared_ptr<node::Node> new_node;

    if (scene->mRootNode) {

      // new_node = std::make_shared(new GeometryNode("unnamed",
      //                             GeometryNode::Configuration("", ""),
      //                             math::mat4::identity()));
      //unsigned count(0);
      //new_node = get_tree(importer, scene, scene->mRootNode, file_name, flags, count);
      new_node = create_animation_node(importer, scene, file_name,node_name, flags);


    } else {
      Logger::LOG_WARNING << "Failed to load object \"" << file_name << "\": No valid root node contained!" << std::endl;
    }

    return new_node;

  }

  Logger::LOG_WARNING << "Failed to load object \"" << file_name << "\": File does not exist!" << std::endl;

  return nullptr;
}

  /////////////////////////////////////////////////////////////////////////////
std::shared_ptr<node::Node> SkeletalAnimationLoader::create_animation_node(std::shared_ptr<Assimp::Importer> const& importer,
                                              aiScene const* ai_scene,
                                              /*aiNode* ai_root,*/
                                              std::string const& file_name,
                                              std::string const& node_name,
                                              unsigned flags) {


  auto animation_director = std::make_shared<SkeletalAnimationDirector>(ai_scene);

  std::vector<std::string> geometry_descriptions{};
  std::vector<std::shared_ptr<Material>> materials{};

  for(uint mesh_count(0);mesh_count<ai_scene->mNumMeshes;++mesh_count){


    GeometryDescription desc("SkeletalAnimation", file_name, mesh_count,flags);
    geometry_descriptions.push_back(desc.unique_key());

    GeometryDatabase::instance()->add(desc.unique_key() 
                                      ,std::make_shared<SkeletalAnimationRessource>(ai_scene->mMeshes[mesh_count]
                                                                                    , animation_director
                                                                                    , importer
                                                                                    , flags & SkeletalAnimationLoader::MAKE_PICKABLE));

    std::shared_ptr<Material> material;
    //TODO : texture atlas since multiple meshes in one resource !!!
    unsigned material_index(ai_scene->mMeshes[mesh_count]->mMaterialIndex);
    //if (material_index != 0 && flags & SkeletalAnimationLoader::LOAD_MATERIALS) {
    if (flags & SkeletalAnimationLoader::LOAD_MATERIALS) {
      MaterialLoader material_loader;
      aiMaterial const* ai_material(ai_scene->mMaterials[material_index]);
      material = material_loader.load_material(ai_material, file_name);
      material->set_uniform("Roughness", 0.6f);
    }
    materials.push_back(material);
  }

  return std::make_shared<node::SkeletalAnimationNode>(file_name + "_" + node_name, geometry_descriptions, materials, animation_director);
}


  /////////////////////////////////////////////////////////////////////////////

// TODO
/*std::vector<SkeletalAnimationRessource*> const SkeletalAnimationLoader::load_from_buffer(char const* buffer_name,
                                                                     unsigned buffer_size,
                                                                     bool build_kd_tree) {

  auto importer = std::make_shared<Assimp::Importer>();

  aiScene const* scene(importer->ReadFileFromMemory(
      buffer_name,
      buffer_size,
      aiProcessPreset_TargetRealtime_Quality | aiProcess_CalcTangentSpace));

  std::vector<SkeletalAnimationRessource*> meshes;

  for (unsigned int n = 0; n < scene->mNumMeshes; ++n) {
    meshes.push_back(new SkeletalAnimationRessource(scene->mMeshes[n], importer, build_kd_tree));
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

  return importer.IsExtensionSupported(file_name.substr(point_pos + 1));
}

////////////////////////////////////////////////////////////////////////////////

/*std::shared_ptr<node::Node> SkeletalAnimationLoader::get_tree(std::shared_ptr<Assimp::Importer> const& importer,
                                              aiScene const* ai_scene,
                                              aiNode* ai_root,
                                              std::string const& file_name,
                                              unsigned flags, unsigned& mesh_count) {

  // creates a geometry node and returns it
  auto load_geometry = [&](int i) {
    // load geometry
    std::string mesh_name("type=file&file=" + file_name + "&id=" + string_utils::to_string(mesh_count++) + "&flags=" + string_utils::to_string(flags));
    GeometryDatabase::instance()->add(mesh_name, std::make_shared<SkeletalAnimationRessource>(ai_scene->mMeshes[ai_root->mMeshes[i]], importer, flags & SkeletalAnimationLoader::MAKE_PICKABLE));

    // load material
    Material material;
    unsigned material_index(ai_scene->mMeshes[ai_root->mMeshes[i]]->mMaterialIndex);

    if (material_index != 0 && flags & SkeletalAnimationLoader::LOAD_MATERIALS) {
      MaterialLoader material_loader;
      aiMaterial const* ai_material(ai_scene->mMaterials[material_index]);
      material = material_loader.load_material(ai_material, file_name);
    }

    return std::make_shared<node::SkeletalAnimationNode>(mesh_name, mesh_name, material);
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
  auto group(std::make_shared<node::TransformNode>());

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
}*/


////////////////////////////////////////////////////////////////////////////////

void SkeletalAnimationLoader::apply_fallback_material(std::shared_ptr<node::Node> const& root,
                                            std::shared_ptr<Material> const& fallback_material) const
{
  auto g_node(std::dynamic_pointer_cast<node::SkeletalAnimationNode>(root));

    if (g_node) {
      g_node->set_fallback_materials(fallback_material);
      g_node->update_cache();
    }

    /*for (auto& child : root->get_children()) {
      apply_fallback_material(child, fallback_material);
    }*/

  }
}
