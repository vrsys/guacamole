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
#include <gua/utils/Logger.hpp>
#include <gua/utils/string_utils.hpp>
#include <gua/node/Node.hpp>
#include <gua/renderer/MaterialShader.hpp>
#include <gua/renderer/TriMeshRessource.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>
#include <gua/databases/GeometryDatabase.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Material> MaterialLoader::load_material(
    aiMaterial const* ai_material,
    std::string const& assets_directory) const {

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


  auto new_mat(gua::MaterialShaderDatabase::instance()->lookup("gua_default_material")->get_new_material());

  if (uniform_color_map != "") {
    new_mat->set_uniform("ColorMap", assets + uniform_color_map);
  } else if (uniform_color != "") {
    new_mat->set_uniform("Color", string_utils::from_string<math::vec3>(uniform_color));
    // new_mat->set_uniform("ColorMap", std::string("0"));
  } else  {
    // new_mat->set_uniform("Color", math::vec3(0, 0, 0));
    // new_mat->set_uniform("ColorMap", std::string("0"));
  }

  if (uniform_roughness_map != "") {
    new_mat->set_uniform("RoughnessMap", assets + uniform_roughness_map);
  } else if (uniform_roughness != "" && uniform_roughness != "0") {
    // specular exponent is taken to the power of 0.02 in order to move it to the desired range
    new_mat->set_uniform("Roughness", std::min(1.f, std::pow(string_utils::from_string<float>(uniform_roughness), 0.02f)-1.f));
    // new_mat->set_uniform("RoughnessMap", std::string("0"));
  } else  {
    // new_mat->set_uniform("Roughness", 0.1f);
    // new_mat->set_uniform("RoughnessMap", std::string("0"));
  }

  if (uniform_metalness_map != "") {
    new_mat->set_uniform("MetalnessMap", assets + uniform_metalness_map);
  } else if (uniform_metalness != "") {
    // multiplying with 0.5, since metalness of 1.0 is seldomly wanted but specularity of 1.0 often given
    new_mat->set_uniform("Metalness", string_utils::from_string<math::vec3>(uniform_metalness)[0]*0.5f);
    // new_mat->set_uniform("MetalnessMap", std::string("0"));
  } else  {
    // new_mat->set_uniform("Metalness", 0.01f);
    // new_mat->set_uniform("MetalnessMap", std::string("0"));
  }

  if (uniform_emit_map != "") {
    new_mat->set_uniform("EmissivityMap", assets + uniform_emit_map);
  } else if (uniform_emit != "") {
    new_mat->set_uniform("Emissivity", string_utils::from_string<math::vec3>(uniform_emit)[0]);
    // new_mat->set_uniform("EmissivityMap", std::string("0"));
  } else  {
    // new_mat->set_uniform("Emissivity", 0.f);
    // new_mat->set_uniform("EmissivityMap", std::string("0"));
  }

  if (uniform_normal_map != "") {
    new_mat->set_uniform("NormalMap", assets + uniform_normal_map);
  } else {
    // new_mat->set_uniform("NormalMap", std::string("0"));
  }

  if (uniform_opacity_map != "") {
    new_mat->set_uniform("OpacityMap", assets + uniform_opacity_map);
  } else {
    // new_mat->set_uniform("OpacityMap", std::string("0"));
  }
  
  // if (uniform_reflection_map != "")
  // new_mat.set_uniform("uniform_reflection_map"),
  // assets + uniform_reflection_map;

  // if (ambient_map != "") {
  //   new_mat.set_uniform("ambient_map", assets + ambient_map);
  // } else if (ambient_color != "") {
  //   new_mat.set_uniform("ambient_color", string_utils::from_string<math::vec3>(ambient_color));
  // }

  return new_mat;
}

////////////////////////////////////////////////////////////////////////////////

}
