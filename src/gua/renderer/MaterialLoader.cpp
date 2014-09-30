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
#include <gua/databases/MaterialDatabase.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/ShadingModelDatabase.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

Material const& MaterialLoader::load_material(
    aiMaterial const* ai_material,
    std::string const& file_name) const {

  // helper lambdas ------------------------------------------------------------
  auto get_vec3 = [&](const char * pKey, unsigned int type, unsigned int idx, math::vec3& val) {
    aiColor3D value;
    if (AI_SUCCESS != ai_material->Get(pKey, type, idx, value)) {
      return false;
    }
    val = math::vec3(value.r, value.g, value.b);
    return true;
  };

  auto get_string = [&](const char * pKey, unsigned int type, unsigned int idx, std::string& val) {
    aiString value;
    if (AI_SUCCESS != ai_material->Get(pKey, type, idx, value)) {
      return false;
    }
    val = value.data;
    return true;
  };

  auto get_float = [&](const char * pKey, unsigned int type, unsigned int idx, float& val) {
    if (AI_SUCCESS != ai_material->Get(pKey, type, idx, val)) {
      return false;
    }
    return true;
  };

  // load material if it has not been loaded yet -------------------------------
  PathParser path;
  path.parse(file_name);
  std::string assets_directory(path.get_path(true));

  unsigned capabilities = 0;

  std::string diffuse_map;
  math::vec3  diffuse_color;
  if (get_string(AI_MATKEY_TEXTURE(aiTextureType_DIFFUSE, 0), diffuse_map)) {
    capabilities |= DIFFUSE_MAP;
  } else if (get_vec3(AI_MATKEY_COLOR_DIFFUSE, diffuse_color)) {
    capabilities |= DIFFUSE_COLOR;
  }

  std::string specular_map;
  math::vec3  specular_color;
  float       specularity;
  if (get_string(AI_MATKEY_TEXTURE(aiTextureType_SPECULAR, 0), specular_map)) {
    capabilities |= SPECULAR_MAP;
  } else if (get_vec3(AI_MATKEY_COLOR_SPECULAR, specular_color)) {
    capabilities |= SPECULAR_COLOR;
    specularity = specular_color.x;
  }

  std::string emit_map;
  math::vec3  emit_color;
  float       emissivity;
  if (get_string(AI_MATKEY_TEXTURE(aiTextureType_EMISSIVE, 0), emit_map)) {
    capabilities |= EMIT_MAP;
  } else if (get_vec3(AI_MATKEY_COLOR_EMISSIVE, emit_color)) {
    capabilities |= EMIT_COLOR;
    emissivity = emit_color.x;
  }

  std::string shinyness_map;
  math::vec3  shinyness_color;
  float       shinyness;
  if (get_string(AI_MATKEY_TEXTURE(aiTextureType_SHININESS, 0), shinyness_map)) {
    capabilities |= SHININESS_MAP;
  } else if (get_vec3(AI_MATKEY_SHININESS, shinyness_color)) {
    capabilities |= SHININESS_COLOR;
    shinyness = shinyness_color.x;
  }

  std::string normal_map;
  if (get_string(AI_MATKEY_TEXTURE(aiTextureType_NORMALS, 0), normal_map) || 
      get_string(AI_MATKEY_TEXTURE(aiTextureType_HEIGHT, 0), normal_map)) {
    capabilities |= NORMAL_MAP;
  }

  std::string reflection_map;
  if (get_string(AI_MATKEY_TEXTURE(aiTextureType_REFLECTION, 0), reflection_map)) {
    capabilities |= REFLECTION_MAP;
  }

  std::string opacity_map;
  if (get_string(AI_MATKEY_TEXTURE(aiTextureType_OPACITY, 0), opacity_map)) {
    capabilities |= OPACITY_MAP;
  }

  // std::string ambient_map(get_string(AI_MATKEY_TEXTURE(aiTextureType_AMBIENT, 0)));
  // math::vec3  ambient_color(get_vec3(AI_MATKEY_COLOR_AMBIENT));

  Material& instance(get_material_instance(capabilities));

  if (capabilities & DIFFUSE_MAP) {
    instance.set_uniform("diffuse_map", assets_directory + diffuse_map);
  }
  if (capabilities & DIFFUSE_COLOR) {
    instance.set_uniform("diffuse_color", diffuse_color);
  }
  if (capabilities & SPECULAR_MAP) {
    instance.set_uniform("specular_map", assets_directory + specular_map);
  }
  if (capabilities & SPECULAR_COLOR) {
    instance.set_uniform("specularity", specularity);
  }
  if (capabilities & EMIT_MAP) {
    instance.set_uniform("emit_map", assets_directory + emit_map);
  }
  if (capabilities & EMIT_COLOR) {
    instance.set_uniform("emit_color", emissivity);
  }
  if (capabilities & NORMAL_MAP) {
    instance.set_uniform("normal_map", assets_directory + normal_map);
  }
  // if (capabilities & REFLECTION_MAP) {
  // instance.set_uniform("reflection_map"),
  // assets_directory + reflection_map;
  // }
  // if (capabilities & AMBIENT_MAP) {
  //   instance.set_uniform("ambient_map", assets_directory + ambient_map);
  // }
  // if (capabilities & AMBIENT_COLOR) {
  //   instance.set_uniform("ambient_color", ambient_color);
  // }
  if (capabilities & SHININESS_MAP) {
    instance.set_uniform("shinyness_map", assets_directory + shinyness_map);
  }
  if (capabilities & SHININESS_COLOR) {
    instance.set_uniform("shinyness_color", shinyness);
  }
  if (capabilities & OPACITY_MAP) {
    instance.set_uniform("opacity_map", assets_directory + opacity_map);
  }

  return instance;
}

////////////////////////////////////////////////////////////////////////////////

Material& MaterialLoader::get_material_instance(unsigned capabilities) const {

  std::string material_name("gua_generated");

  if (capabilities & DIFFUSE_MAP) {
    material_name += "_dmap";
  }
  if (capabilities & DIFFUSE_COLOR) {
    material_name += "_dcol";
  }
  if (capabilities & SPECULAR_MAP) {
    material_name += "_smap";
  }
  if (capabilities & SPECULAR_COLOR) {
    material_name += "_scol";
  }
  if (capabilities & EMIT_MAP) {
    material_name += "_emap";
  }
  if (capabilities & EMIT_COLOR) {
    material_name += "_ecol";
  }
  if (capabilities & NORMAL_MAP) {
    material_name += "_nmap";
  }
  // if (capabilities & REFLECTION_MAP) {
  //   material_name += "_rmap";
  // }
  // if (capabilities & AMBIENT_MAP) {
  //   material_name += "_amap";
  // }
  // if (capabilities & AMBIENT_COLOR) {
  //   material_name += "_acol";
  // }
  if (capabilities & SHININESS_MAP) {
    material_name += "_shmap";
  }
  if (capabilities & SHININESS_COLOR) {
    material_name += "_shcol";
  }
  if (capabilities & OPACITY_MAP) {
    material_name += "_omap";
  }

  if (!MaterialDatabase::instance()->is_supported(material_name)) {
    MaterialDescription description;

    if (capabilities & OPACITY_MAP) {
      description.add_fragment_pass(MaterialPass("opacity_pass").set_source(R"(
        void opacity_pass() {
          if (texture2D(sampler2D(opacity_map), gua_texcoords).r < 0.5) {
            discard;
          } 
        }
      )").set_uniform("opacity_map", std::string("gua_default_texture")));
    }

    if (capabilities & NORMAL_MAP) {
      description.add_fragment_pass(MaterialPass("normal_mapping_pass").set_source(R"(
        void normal_mapping_pass() {
          vec3 ts_normal = normalize(texture2D(sampler2D(normal_map), gua_texcoords).rgb * 2.0 - 1.0);
          gua_normal     = normalize(gua_varying_tangent * ts_normal.x 
                                 + gua_varying_bitangent * ts_normal.y 
                                    + gua_varying_normal * ts_normal.z);
        }
      )").set_uniform("normal_map", std::string("gua_default_texture")));
    }

    if (capabilities & DIFFUSE_MAP) {
      description.add_fragment_pass(MaterialPass("diffuse_pass").set_source(R"(
        void diffuse_pass() {
          vec4 color = texture2D(sampler2D(diffuse_map), gua_texcoords);
          if (color.a < 0.5) {
            discard;
          } 

          gua_color = color.rgb; 
        }
      )").set_uniform("diffuse_map", std::string("gua_default_texture")));
    } else if (capabilities & DIFFUSE_COLOR) {
      description.add_fragment_pass(MaterialPass("diffuse_pass").set_source(R"(
        void diffuse_pass() {
          gua_color = diffuse_color; 
        }
      )").set_uniform("diffuse_color", math::vec3(1, 1, 1)));
    }


    auto add_property = [&](bool as_texture, bool as_color,
                            std::string const & name, 
                            float default_val) {
      if (as_texture) {
        description.add_fragment_pass(MaterialPass(name + "_pass").set_source(R"(
          void )" + name + R"(_pass() {
            gua_)" + name + R"( = texture2D(sampler2D()" + name + R"(_map), gua_texcoords).r;
          }
        )").set_uniform(name + "_map", std::string("gua_default_texture")));
      } else if (as_color) {
        description.add_fragment_pass(MaterialPass(name + "_pass").set_source(R"(
          void )" + name + R"(_pass() {
            gua_)" + name + R"( = )" + name + R"(;
          }
        )").set_uniform(name, default_val));
      } else {
         description.add_fragment_pass(MaterialPass(name + "_pass").set_source(R"(
          void )" + name + R"(_pass() {
            gua_)" + name + R"( = )" + std::to_string(default_val) + R"(;
          }
        )"));
      }
    };

    add_property(capabilities & SHININESS_MAP, capabilities & SHININESS_COLOR, "shinyness",   50.0f);
    add_property(capabilities & SPECULAR_MAP,  capabilities & SPECULAR_COLOR,  "specularity",  0.5f);
    add_property(capabilities & EMIT_MAP,      capabilities & EMIT_COLOR,      "emissivity",   0.0f);

    MaterialDatabase::instance()->add(std::make_shared<MaterialShader>(material_name, description));
  }

  return MaterialDatabase::instance()->lookup(material_name)->get_default_instance();
}

////////////////////////////////////////////////////////////////////////////////

}
