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
#include <gua/renderer/MaterialLoader.hpp>

// guacamole headers
#include <gua/utils/TextFile.hpp>
#include <fstream>
#include <gua/utils/Logger.hpp>
#include <gua/utils/string_utils.hpp>
#include <gua/node/Node.hpp>
#include <gua/renderer/MaterialShader.hpp>
#include <gua/renderer/TriMeshRessource.hpp>
#include <gua/renderer/PBSMaterialFactory.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>
#include <gua/databases/GeometryDatabase.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Material> MaterialLoader::load_material(
    aiMaterial const* ai_material,
    std::string const& assets_directory,
    bool optimize_material) const {

  // helper lambdas ------------------------------------------------------------
  auto get_color =
      [&](const char * pKey, unsigned int type, unsigned int idx)->std::string {
    aiColor3D value;
    if (AI_SUCCESS != ai_material->Get(pKey, type, idx, value))
      return "";
    math::vec3 color(value.r, value.g, value.b);
    return string_utils::to_string(color);
  };

  auto get_sampler =
      [&](const char * pKey, unsigned int type, unsigned int idx)->std::string {
    aiString value;
    if (AI_SUCCESS != ai_material->Get(pKey, type, idx, value))
      return "";
    return std::string(value.data);
  };

  auto get_float =
      [&](const char * pKey, unsigned int type, unsigned int idx)->std::string {
    float value;
    if (AI_SUCCESS != ai_material->Get(pKey, type, idx, value))
      return "";
    return string_utils::to_string(value);
  };

  PathParser path;
  path.parse(assets_directory);
  std::string assets(path.get_path(true));

  std::string uniform_color_map(      get_sampler(  AI_MATKEY_TEXTURE(aiTextureType_DIFFUSE, 0)));
  std::string uniform_color(          get_color(    AI_MATKEY_COLOR_DIFFUSE));
  std::string uniform_roughness_map(  get_sampler(  AI_MATKEY_TEXTURE(aiTextureType_SHININESS, 0)));
  std::string uniform_roughness(      get_float(    AI_MATKEY_SHININESS));
  std::string uniform_emit_map(       get_sampler(  AI_MATKEY_TEXTURE(aiTextureType_EMISSIVE, 0)));
  std::string uniform_emit(           get_color(    AI_MATKEY_COLOR_EMISSIVE));
  std::string uniform_reflection_map( get_sampler(  AI_MATKEY_TEXTURE(aiTextureType_REFLECTION, 0)));
  std::string ambient_map(            get_sampler(  AI_MATKEY_TEXTURE(aiTextureType_AMBIENT, 0)));
  std::string ambient_color(          get_color(    AI_MATKEY_COLOR_AMBIENT));
  std::string uniform_metalness_map(  get_sampler(  AI_MATKEY_TEXTURE(aiTextureType_SPECULAR, 0)));
  std::string uniform_metalness(      get_color(    AI_MATKEY_COLOR_SPECULAR));
  std::string uniform_opacity_map(    get_sampler(  AI_MATKEY_TEXTURE(aiTextureType_OPACITY, 0)));

  // obj bump and map_bump textures are imported as aiTextureType_HEIGHT
  std::string uniform_normal_map(     get_sampler(  AI_MATKEY_TEXTURE(aiTextureType_NORMALS, 0)));
  if (uniform_normal_map == "") {
    uniform_normal_map =              get_sampler(  AI_MATKEY_TEXTURE(aiTextureType_HEIGHT, 0));
  }

  unsigned capabilities;

  if (!optimize_material) {
    capabilities |= PBSMaterialFactory::ALL;
  } else {

    if (uniform_color_map != "" && uniform_color != "") {
      capabilities |= PBSMaterialFactory::COLOR_VALUE_AND_MAP;
    } else if (uniform_color_map != "") {
      capabilities |= PBSMaterialFactory::COLOR_MAP;
    } else if (uniform_color != "") {
      capabilities |= PBSMaterialFactory::COLOR_VALUE;
    }

  #if 0
    if (uniform_roughness_map != "") {
      capabilities |= PBSMaterialFactory::ROUGHNESS_MAP;
    } else if (uniform_roughness != "" && uniform_roughness != "0") {
      capabilities |= PBSMaterialFactory::ROUGHNESS_VALUE;
    }
  #endif

  #if 0
    if (uniform_metalness_map != "") {
      capabilities |= PBSMaterialFactory::METALNESS_MAP;
    } else if (uniform_metalness != "") {
      capabilities |= PBSMaterialFactory::METALNESS_VALUE;
    }
  #endif

    if (uniform_emit_map != "") {
      capabilities |= PBSMaterialFactory::EMISSIVITY_MAP;
    } else if (uniform_emit != "") {
      capabilities |= PBSMaterialFactory::EMISSIVITY_VALUE;
    }

    if (uniform_normal_map != "") {
      capabilities |= PBSMaterialFactory::NORMAL_MAP;
    }

  #if 0
    if (uniform_opacity_map != "") {
      Logger::LOG_WARNING << "Material not fully supported: guacamole does not support opacity maps. Please use the alpha channel of your diffuse map!" << std::endl;
    }

    if (uniform_reflection_map != "") {
      Logger::LOG_WARNING << "Material not fully supported: guacamole does not support reflection maps." << std::endl;
    }
  #endif

    if (ambient_map != "") {
      Logger::LOG_WARNING << "Material not fully supported: guacamole does not support ambient maps." << std::endl;
    } else if (ambient_color != "") {
      Logger::LOG_WARNING << "Material not fully supported: guacamole does not support ambient colors." << std::endl;
    }
  }

  auto new_mat(PBSMaterialFactory::create_material(static_cast<PBSMaterialFactory::Capabilities>(capabilities)));

  if (uniform_color_map != "") {
    new_mat->set_uniform("ColorMap", assets + uniform_color_map);
  }

  if (uniform_color != "") {
    auto c(string_utils::from_string<math::vec3>(uniform_color));
    new_mat->set_uniform("Color", scm::math::vec4f(gua::math::float_t(c.x), gua::math::float_t(c.y), gua::math::float_t(c.z), 1.f));
  }

#if 0
  if (uniform_roughness_map != "") {
    new_mat->set_uniform("RoughnessMap", assets + uniform_roughness_map);
  } else if (uniform_roughness != "" && uniform_roughness != "0") {
    // specular exponent is taken to the power of 0.02 in order to move it to the desired range
    new_mat->set_uniform("Roughness", std::min(1.f, std::pow(string_utils::from_string<float>(uniform_roughness), 0.02f)-1.f));
  }
#endif

#if 0
  if (uniform_metalness_map != "") {
    new_mat->set_uniform("MetalnessMap", assets + uniform_metalness_map);
  } else if (uniform_metalness != "") {
    // multiplying with 0.5, since metalness of 1.0 is seldomly wanted but specularity of 1.0 often given
    new_mat->set_uniform("Metalness", string_utils::from_string<math::vec3>(uniform_metalness)[0]*0.5f);
  }
#endif

  if (uniform_emit_map != "") {
    new_mat->set_uniform("EmissivityMap", assets + uniform_emit_map);
  } else if (uniform_emit != "") {
    new_mat->set_uniform("Emissivity", string_utils::from_string<scm::math::vec3f>(uniform_emit)[0]);
  }

  if (uniform_normal_map != "") {
    new_mat->set_uniform("NormalMap", assets + uniform_normal_map);
  }


  return new_mat;
}

////////////////////////////////////////////////////////////////////////////////
#ifdef GUACAMOLE_FBX
std::shared_ptr<Material> MaterialLoader::load_material(FbxSurfaceMaterial const& fbx_material, std::string const& assets_directory) const {
  PathParser path;
  path.parse(assets_directory);
  std::string assets(path.get_path(true));
  
  //method to check if texture is set for an attribute
  auto get_sampler = [&](const char* attribute)->std::string {
    FbxProperty property = fbx_material.FindProperty(attribute);
    unsigned texture_count = property.GetSrcObjectCount<FbxTexture>();
    if(texture_count > 0) {
      if(texture_count > 1) {
        Logger::LOG_WARNING << property.GetNameAsCStr() << " has more than one texture, only first one." << std::endl;
      }
      //texture could also be layered or procedural texture
      FbxFileTexture* texture = property.GetSrcObject<FbxFileTexture>(0);
      if(texture) {        
        std::string file_name = texture->GetFileName();
        //filter out possible path in front of filename
        auto path_end(file_name.find_last_of("/\\"));
        if(path_end != std::string::npos) {
          file_name = file_name.substr(path_end + 1);
        }
        return file_name;
      }
      else {
        Logger::LOG_WARNING << "Texturetype of "<< property.GetNameAsCStr() << " not supported." << std::endl;
      }
    }
    return "";
  };

  //check which shading model is used
  std::string shading{fbx_material.ShadingModel.Get().Buffer()};
  bool shading_supported = shading == "Phong" || shading == "Lambert" || shading == "lambert" || shading == "phong"; 
  if(!shading_supported) {
    Logger::LOG_DEBUG << "Shading Type '" << shading << "' not supported." << std::endl;
  }
  //cast to phong not necessary, only diffuse and emissive values needed
  FbxSurfaceLambert* lambert = (FbxSurfaceLambert*)&fbx_material;
  if(!lambert) {
    Logger::LOG_ERROR << "Casting Material to Lambert failed." << std::endl;
    assert(0);
  } 

  auto new_mat(gua::MaterialShaderDatabase::instance()->lookup("gua_default_material")->make_new_material());

  std::string color_map{get_sampler(FbxSurfaceMaterial::sDiffuse)};
  if(color_map != "") {
    new_mat->set_uniform("ColorMap", assets + color_map);
  }
  else {
    FbxDouble3 color = lambert->Diffuse.Get();
    new_mat->set_uniform("Color", math::vec4(color[0], color[1], color[2], 1.f));
  } 

  std::string normal_map{get_sampler(FbxSurfaceMaterial::sNormalMap)};
  //check bump slot if no normalmap found
  if(normal_map == "") {
    normal_map = get_sampler(FbxSurfaceMaterial::sBump);
  }
  if(normal_map != "") {
    new_mat->set_uniform("NormalMap", assets + normal_map);
  }

  std::string ambient_map{get_sampler(FbxSurfaceMaterial::sAmbient)};
  if (ambient_map != "") {
    Logger::LOG_WARNING << "Material not fully supported: guacamole does not support ambient maps." << std::endl;
  }

  std::string emit_map{get_sampler(FbxSurfaceMaterial::sEmissive)};
  if(emit_map != "") {
    new_mat->set_uniform("EmissivityMap", assets + emit_map);
  }
  else {
    FbxDouble3 color = lambert->Diffuse.Get();
    new_mat->set_uniform("Emissivity", math::vec4(color[0], color[1], color[2], 1.f));
  }

  return new_mat;
}
#endif
}
