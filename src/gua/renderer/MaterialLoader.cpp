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
#include <gua/renderer/Material.hpp>
#include <gua/renderer/TriMeshRessource.hpp>
#include <gua/databases/MaterialDatabase.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/ShadingModelDatabase.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

MaterialInstance const& MaterialLoader::load_material(
    aiMaterial const* ai_material,
    std::string const& file_name) const {

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

  // load material if it has not been loaded yet -------------------------------
  PathParser path;
  path.parse(file_name);
  std::string assets_directory(path.get_path(true));

  unsigned capabilities = 0;

  std::string diffuse_map(get_sampler(AI_MATKEY_TEXTURE(aiTextureType_DIFFUSE, 0)));
  std::string diffuse_color(get_color(AI_MATKEY_COLOR_DIFFUSE));

  std::string specular_map(get_sampler(AI_MATKEY_TEXTURE(aiTextureType_SPECULAR, 0)));
  std::string specular_color(get_color(AI_MATKEY_COLOR_SPECULAR));

  std::string emit_map(get_sampler(AI_MATKEY_TEXTURE(aiTextureType_EMISSIVE, 0)));
  std::string emit_color(get_color(AI_MATKEY_COLOR_EMISSIVE));

  std::string normal_map(get_sampler(AI_MATKEY_TEXTURE(aiTextureType_NORMALS, 0)));

  // obj bump and map_bump textures are imported as aiTextureType_HEIGHT
  if (normal_map == "") {
    normal_map = get_sampler(AI_MATKEY_TEXTURE(aiTextureType_HEIGHT, 0));
  }

  std::string reflection_map(get_sampler(AI_MATKEY_TEXTURE(aiTextureType_REFLECTION, 0)));

  std::string ambient_map(get_sampler(AI_MATKEY_TEXTURE(aiTextureType_AMBIENT, 0)));
  std::string ambient_color(get_color(AI_MATKEY_COLOR_AMBIENT));

  std::string shinyness_map(get_sampler(AI_MATKEY_TEXTURE(aiTextureType_SHININESS, 0)));
  std::string shinyness_color(get_float(AI_MATKEY_SHININESS));

  std::string opacity_map(get_sampler(AI_MATKEY_TEXTURE(aiTextureType_OPACITY, 0)));

  if (diffuse_map != "") {
    capabilities |= DIFFUSE_MAP;
  } else if (diffuse_color != "") {
    capabilities |= DIFFUSE_COLOR;
  }

  if (specular_map != "") {
    capabilities |= SPECULAR_MAP;
  } else if (specular_color != "") {
    capabilities |= SPECULAR_COLOR;
  }

  if (emit_map != "") {
    capabilities |= EMIT_MAP;
  } else if (emit_color != "") {
    capabilities |= EMIT_COLOR;
  }

  if (normal_map != "") {
    capabilities |= NORMAL_MAP;
  }

  if (reflection_map != "")
    capabilities |= REFLECTION_MAP;

  if (ambient_map != "") {
    capabilities |= AMBIENT_MAP;
  } else if (ambient_color != "") {
    capabilities |= AMBIENT_COLOR;
  }

  if (shinyness_map != "") {
    capabilities |= SHININESS_MAP;
  } else if (shinyness_color != "" && shinyness_color != "0") {
    capabilities |= SHININESS_COLOR;
  }

  if (opacity_map != "") {
    capabilities |= OPACITY_MAP;
  }

  MaterialInstance instance(get_material_instance(capabilities));

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
    instance.set_uniform("specular_color", specular_color);
  }
  if (capabilities & EMIT_MAP) {
    instance.set_uniform("emit_map", assets_directory + emit_map);
  }
  if (capabilities & EMIT_COLOR) {
    instance.set_uniform("emit_color", emit_color);
  }
  if (capabilities & NORMAL_MAP) {
    instance.set_uniform("normal_map", assets_directory + normal_map);
  }
  // if (capabilities & REFLECTION_MAP) {
  // instance.set_uniform("reflection_map"),
  // assets_directory + reflection_map;
  // }
  if (capabilities & AMBIENT_MAP) {
    instance.set_uniform("ambient_map", assets_directory + ambient_map);
  }
  if (capabilities & AMBIENT_COLOR) {
    instance.set_uniform("ambient_color", ambient_color);
  }
  if (capabilities & SHININESS_MAP) {
    instance.set_uniform("shinyness_map", assets_directory + shinyness_map);
  }
  if (capabilities & SHININESS_COLOR) {
    instance.set_uniform("shinyness_color", shinyness_color);
  }
  if (capabilities & OPACITY_MAP) {
    instance.set_uniform("opacity_map", assets_directory + opacity_map);
  }

  return instance;
}

////////////////////////////////////////////////////////////////////////////////

MaterialInstance const& MaterialLoader::get_material_instance(unsigned capabilities) const {

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
  if (capabilities & REFLECTION_MAP) {
    material_name += "_rmap";
  }
  if (capabilities & AMBIENT_MAP) {
    material_name += "_amap";
  }
  if (capabilities & AMBIENT_COLOR) {
    material_name += "_acol";
  }
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
          if (texture2D(opacity_map, gua_texcoords).a < 0.5) {
            discard;
          } 
        }
      )").set_uniform("opacity_map", "gua_default_texture"));
    }

    if (capabilities & NORMAL_MAP) {
      description.add_fragment_pass(MaterialPass("normal_mapping_pass").set_source(R"(
        void normal_mapping_pass() {
          vec3 ts_normal = normalize(texture2D(normal_map, gua_texcoords).rgb * 2.0 - 1.0);
          gua_normal = normalize(varying_tangent * ts_normal.x + varying_bitangent * ts_normal.y + varying_normal * ts_normal.z);
        }
      )").set_uniform("normal_map", "gua_default_texture"));
    }

    if (capabilities & DIFFUSE_MAP) {
      description.add_fragment_pass(MaterialPass("diffuse_pass").set_source(R"(
        void diffuse_pass() {
          vec4 color = texture2D(diffuse_map, gua_texcoords);
          if (color.a < 0.5) {
            discard;
          } 

          gua_color = color.rgb; 
        }
      )").set_uniform("diffuse_map", "gua_default_texture"));
    } else if (capabilities & DIFFUSE_COLOR) {
      description.add_fragment_pass(MaterialPass("diffuse_pass").set_source(R"(
        void diffuse_pass() {
          vec4 color = diffuse_color;
          if (color.a < 0.5) {
            discard;
          } 

          gua_color = color.rgb; 
        }
      )").set_uniform("diffuse_color", math::vec4(1, 1, 1, 1)));
    }


    auto add_property = [&](bool as_texture, bool as_color,
                            std::string const & name, 
                            float default_val) {
      if (as_texture) {
        description.add_fragment_pass(MaterialPass(name + "_pass").set_source(R"(
          void )" + name + R"(_pass() {
            gua_)" + name + R"( = texture2D()" + name + R"(_map, gua_texcoords).r;
          }
        )").set_uniform(name + "_map", "gua_default_texture"));
      } else if (as_color) {
        description.add_fragment_pass(MaterialPass(name + "_pass").set_source(R"(
          void )" + name + R"(_pass() {
            gua_)" + name + R"( = )" + name + R"(;
          }
        )").set_uniform(name, default_val));
      }
    };

    add_property(capabilities & SHININESS_MAP, capabilities & SHININESS_COLOR, "shinyness",   50);
    add_property(capabilities & SPECULAR_MAP,  capabilities & SPECULAR_COLOR,  "specularity",  1);
    add_property(capabilities & EMIT_MAP,      capabilities & EMIT_COLOR,      "emissivity",   0);

    MaterialDatabase::instance()->add(std::make_shared<Material>(material_name, description));
  }

  return MaterialDatabase::instance()->lookup(material_name)->get_default_instance();
}

////////////////////////////////////////////////////////////////////////////////

}
