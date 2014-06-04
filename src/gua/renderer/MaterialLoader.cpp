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
#include <gua/scenegraph/Node.hpp>
#include <gua/renderer/Material.hpp>
#include <gua/renderer/TriMeshRessource.hpp>
#include <gua/databases/MaterialDatabase.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/ShadingModelDatabase.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

std::string const MaterialLoader::load_material(
    aiMaterial const* ai_material,
    std::string const& file_name) const {

  auto get_color =
      [&](const char * pKey, unsigned int type, unsigned int idx)->std::string {
    aiColor3D value;
    if (AI_SUCCESS != ai_material->Get(pKey, type, idx, value))
      return "";
    math::vec3 color(value.r, value.g, value.b);
    return string_utils::to_string(color);
  }
  ;

  auto get_sampler =
      [&](const char * pKey, unsigned int type, unsigned int idx)->std::string {
    aiString value;
    if (AI_SUCCESS != ai_material->Get(pKey, type, idx, value))
      return "";
    return std::string(value.data);
  }
  ;

  auto get_float =
      [&](const char * pKey, unsigned int type, unsigned int idx)->std::string {
    float value;
    if (AI_SUCCESS != ai_material->Get(pKey, type, idx, value))
      return "";
    return string_utils::to_string(value);
  }
  ;

  aiString ai_material_name;
  ai_material->Get(AI_MATKEY_NAME, ai_material_name);
  std::string material_name("type='generated'&source='" + file_name + "'&name='" + ai_material_name.data + "'");

  if (!MaterialDatabase::instance()->is_supported(material_name)) {

    PathParser path;
    path.parse(file_name);
    std::string assets_directory(path.get_path(true));

    unsigned capabilities = 0;

    std::string diffuse_map(
        get_sampler(AI_MATKEY_TEXTURE(aiTextureType_DIFFUSE, 0)));
    std::string diffuse_color(get_color(AI_MATKEY_COLOR_DIFFUSE));

    std::string specular_map(
        get_sampler(AI_MATKEY_TEXTURE(aiTextureType_SPECULAR, 0)));
    std::string specular_color(get_color(AI_MATKEY_COLOR_SPECULAR));

    std::string emit_map(
        get_sampler(AI_MATKEY_TEXTURE(aiTextureType_EMISSIVE, 0)));
    std::string emit_color(get_color(AI_MATKEY_COLOR_EMISSIVE));

    std::string normal_map(
        get_sampler(AI_MATKEY_TEXTURE(aiTextureType_NORMALS, 0)));

    // obj bump and map_bump textures are imported as aiTextureType_HEIGHT
    if (normal_map == "")
      normal_map = get_sampler(AI_MATKEY_TEXTURE(aiTextureType_HEIGHT, 0));

    std::string reflection_map(
        get_sampler(AI_MATKEY_TEXTURE(aiTextureType_REFLECTION, 0)));

    std::string ambient_map(
        get_sampler(AI_MATKEY_TEXTURE(aiTextureType_AMBIENT, 0)));
    std::string ambient_color(get_color(AI_MATKEY_COLOR_AMBIENT));

    std::string shinyness_map(
        get_sampler(AI_MATKEY_TEXTURE(aiTextureType_SHININESS, 0)));
    std::string shinyness_color(get_float(AI_MATKEY_SHININESS));

    std::string opacity_map(
        get_sampler(AI_MATKEY_TEXTURE(aiTextureType_OPACITY, 0)));

    if (diffuse_map != "")
      capabilities |= DIFFUSE_MAP;
    else if (diffuse_color != "")
      capabilities |= DIFFUSE_COLOR;

    if (specular_map != "")
      capabilities |= SPECULAR_MAP;
    else if (specular_color != "")
      capabilities |= SPECULAR_COLOR;

    if (emit_map != "")
      capabilities |= EMIT_MAP;
    else if (emit_color != "")
      capabilities |= EMIT_COLOR;

    if (normal_map != "")
      capabilities |= NORMAL_MAP;

    if (reflection_map != "")
      capabilities |= REFLECTION_MAP;

    if (ambient_map != "")
      capabilities |= AMBIENT_MAP;
    else if (ambient_color != "")
      capabilities |= AMBIENT_COLOR;

    if (shinyness_map != "")
      capabilities |= SHININESS_MAP;
    else if (shinyness_color != "" && shinyness_color != "0")
      capabilities |= SHININESS_COLOR;

    if (opacity_map != "")
      capabilities |= OPACITY_MAP;

    std::string shading_model(load_shading_model(capabilities));

    MaterialDescription material_description;
    material_description.set_shading_model(shading_model);

    if (capabilities & DIFFUSE_MAP)
      material_description.get_uniforms()["diffuse_map"] =
          assets_directory + diffuse_map;
    if (capabilities & DIFFUSE_COLOR)
      material_description.get_uniforms()["diffuse_color"] = diffuse_color;
    if (capabilities & SPECULAR_MAP)
      material_description.get_uniforms()["specular_map"] =
          assets_directory + specular_map;
    if (capabilities & SPECULAR_COLOR)
      material_description.get_uniforms()["specular_color"] = specular_color;
    if (capabilities & EMIT_MAP)
      material_description.get_uniforms()["emit_map"] =
          assets_directory + emit_map;
    if (capabilities & EMIT_COLOR)
      material_description.get_uniforms()["emit_color"] = emit_color;
    if (capabilities & NORMAL_MAP)
      material_description.get_uniforms()["normal_map"] =
          assets_directory + normal_map;
    //        if (capabilities & REFLECTION_MAP)
    // material_description.get_uniforms()["reflection_map"] =
    // assets_directory + reflection_map;
    if (capabilities & AMBIENT_MAP)
      material_description.get_uniforms()["ambient_map"] =
          assets_directory + ambient_map;
    if (capabilities & AMBIENT_COLOR)
      material_description.get_uniforms()["ambient_color"] = ambient_color;
    if (capabilities & SHININESS_MAP)
      material_description.get_uniforms()["shinyness_map"] =
          assets_directory + shinyness_map;
    if (capabilities & SHININESS_COLOR)
      material_description.get_uniforms()["shinyness_color"] = shinyness_color;
    if (capabilities & OPACITY_MAP)
      material_description.get_uniforms()["opacity_map"] =
          assets_directory + opacity_map;

    std::shared_ptr<Material> material =
        std::make_shared<Material>(material_name, material_description);
    MaterialDatabase::instance()->add(material_name, material);
  }

  return material_name;
}

////////////////////////////////////////////////////////////////////////////////

std::string const MaterialLoader::load_shading_model(
    unsigned capabilities) const {

  std::string shading_model_name("gua_generated");

  if (capabilities & DIFFUSE_MAP)
    shading_model_name += "_dmap";
  if (capabilities & DIFFUSE_COLOR)
    shading_model_name += "_dcol";
  if (capabilities & SPECULAR_MAP)
    shading_model_name += "_smap";
  if (capabilities & SPECULAR_COLOR)
    shading_model_name += "_scol";
  if (capabilities & EMIT_MAP)
    shading_model_name += "_emap";
  if (capabilities & EMIT_COLOR)
    shading_model_name += "_ecol";
  if (capabilities & NORMAL_MAP)
    shading_model_name += "_nmap";
  if (capabilities & REFLECTION_MAP)
    shading_model_name += "_rmap";
  if (capabilities & AMBIENT_MAP)
    shading_model_name += "_amap";
  if (capabilities & AMBIENT_COLOR)
    shading_model_name += "_acol";
  if (capabilities & SHININESS_MAP)
    shading_model_name += "_shmap";
  if (capabilities & SHININESS_COLOR)
    shading_model_name += "_shcol";
  if (capabilities & OPACITY_MAP)
    shading_model_name += "_omap";

  if (!ShadingModelDatabase::instance()->is_supported(shading_model_name)) {
    auto model = std::make_shared<ShadingModel>(shading_model_name);

    // GBuffer stage ///////////////////////////////////////////////////////
    std::string gbuffer_vertex_body;
    std::string gbuffer_fragment_body;

    model->get_gbuffer_vertex_stage().get_outputs()["gua_position"] = BufferComponent::F3;
    model->get_gbuffer_vertex_stage().get_outputs()["varying_normal"] =
        BufferComponent::F3;
    model->get_gbuffer_fragment_stage().get_outputs()["gua_normal"] = BufferComponent::F3;

    gbuffer_vertex_body += std::string(
        "   \n\
            gua_position = gua_world_position;\n\
            "
        "varying_normal = gua_world_normal;\n\
        ");

    if (capabilities & (DIFFUSE_MAP | SPECULAR_MAP | EMIT_MAP | NORMAL_MAP |
                        SHININESS_MAP | AMBIENT_MAP)) {

      model->get_gbuffer_vertex_stage().get_outputs()["varying_texcoords"] =
          BufferComponent::F2;

      gbuffer_vertex_body +=
          std::string("         \n\
                varying_texcoords = "
                      "gua_texcoords; \n\
            ");
    }

    if (capabilities & DIFFUSE_MAP) {

      model->get_gbuffer_fragment_stage().get_uniforms()["diffuse_map"] =
          UniformType::SAMPLER2D;

      gbuffer_fragment_body += std::string(
          "                     \n\
                if (texture2D(diffuse_map, "
          "varying_texcoords).a < 0.5) \n\
                    discard;        "
          "                                   \n\
            ");
    }

    if (capabilities & OPACITY_MAP) {

      model->get_gbuffer_fragment_stage().get_uniforms()["opacity_map"] =
          UniformType::SAMPLER2D;

      gbuffer_fragment_body += std::string(
          "                     \n\
                if (texture2D(opacity_map, "
          "varying_texcoords).r < 0.5) \n\
                    discard;        "
          "                                   \n\
            ");
    }

    if (capabilities & NORMAL_MAP) {

      model->get_gbuffer_vertex_stage().get_outputs()["varying_tangent"] =
          BufferComponent::F3;
      model->get_gbuffer_vertex_stage().get_outputs()["varying_bitangent"] =
          BufferComponent::F3;

      model->get_gbuffer_fragment_stage().get_uniforms()["normal_map"] =
          UniformType::SAMPLER2D;

      gbuffer_vertex_body += std::string(
          "          \n\
                varying_tangent = gua_world_tangent; "
          "    \n\
                varying_bitangent = gua_world_bitangent; "
          "\n\
            ");

      gbuffer_fragment_body += std::string(
          "                                                                    "
          "                   \n\
                vec3 ts_normal = "
          "normalize(texture2D(normal_map, varying_texcoords).rgb * 2.0 - "
          "1.0);                                    \n\
                "
          "gua_normal = normalize( varying_tangent * ts_normal.x + "
          "varying_bitangent * ts_normal.y + varying_normal * ts_normal.z); "
          "\n\
            ");

    } else {
      gbuffer_fragment_body +=
          std::string(" \n\
                gua_normal = varying_normal;       "
                      "\n\
            ");
    }

    auto add_output = [&](bool as_texture,
                          bool as_color,
                          std::string const & name,
                          std::string const & swizzle_components,
                          BufferComponent layer_type,
                          UniformType uniform_type) {
      if (as_texture) {

        std::string texture(name + "_map");
        std::string layer("gua_" + name);

        model->get_gbuffer_fragment_stage().get_uniforms()[texture] =
            UniformType::SAMPLER2D;
        model->get_gbuffer_fragment_stage().get_outputs()[layer] = layer_type;

        gbuffer_fragment_body +=
            layer + " = texture2D(" + texture + ", varying_texcoords)." +
            swizzle_components + ";";

      } else if (as_color) {

        std::string color(name + "_color");
        std::string layer("gua_" + name);

        model->get_gbuffer_fragment_stage().get_uniforms()[color] =
            uniform_type;
        model->get_gbuffer_fragment_stage().get_outputs()[layer] = layer_type;

        gbuffer_fragment_body +=
            layer + " = " + color + "." + swizzle_components + ";";
      }
    }
    ;

    add_output(capabilities & DIFFUSE_MAP,
               capabilities & DIFFUSE_COLOR,
               "diffuse",
               "rgb",
               BufferComponent::F3,
               UniformType::VEC3);
    add_output(capabilities & SPECULAR_MAP,
               capabilities & SPECULAR_COLOR,
               "specular",
               "r",
               BufferComponent::F1,
               UniformType::VEC3);
    add_output(capabilities & EMIT_MAP,
               capabilities & EMIT_COLOR,
               "emit",
               "rgb",
               BufferComponent::F3,
               UniformType::VEC3);
    add_output(capabilities & SHININESS_MAP,
               capabilities & SHININESS_COLOR,
               "shinyness",
               "r",
               BufferComponent::F1,
               UniformType::FLOAT);
    add_output(capabilities & AMBIENT_MAP,
               capabilities & AMBIENT_COLOR,
               "ambient",
               "r",
               BufferComponent::F1,
               UniformType::VEC3);

    model->get_gbuffer_vertex_stage().set_body(gbuffer_vertex_body);
    model->get_gbuffer_fragment_stage().set_body(gbuffer_fragment_body);

    // Lighting stage //////////////////////////////////////////////////////

    model->get_lbuffer_stage().get_outputs()["gua_light_diffuse"] = BufferComponent::F3;

    std::string lighting_body;

    lighting_body += "if (gua_light_diffuse_enable)";
    lighting_body +=
        "    gua_light_diffuse = dot(gua_normal, gua_light_direction) * "
        "gua_light_intensity * gua_light_color;";
    lighting_body += "else";
    lighting_body += "    gua_light_diffuse = vec3(0.0);";

    if (capabilities & (SPECULAR_COLOR | SPECULAR_MAP)) {

      model->get_lbuffer_stage().get_outputs()["gua_light_specular"] =
          BufferComponent::F3;

      lighting_body += "if (gua_light_specular_enable) {";

      if (capabilities & (SHININESS_MAP | SHININESS_COLOR))
        lighting_body += "float my_shinyness = gua_shinyness;";
      else
        lighting_body += "float my_shinyness = 50;";

      lighting_body +=
          "   gua_light_specular = pow(max(0, dot(reflect(gua_light_direction, "
          "gua_normal), normalize(gua_position - gua_camera_position))), "
          "my_shinyness) * gua_light_intensity * gua_specular * "
          "gua_light_color;";

      lighting_body += "} else";
      lighting_body += "    gua_light_specular = vec3(0.0);";
    }

    model->get_lbuffer_stage().set_body(lighting_body);

    // Final stage /////////////////////////////////////////////////////////

    model->get_final_shading_stage().get_outputs()["gua_color"] = BufferComponent::F3;

    std::string final_body;

    if (capabilities & (DIFFUSE_MAP | DIFFUSE_COLOR))
      final_body += "vec3 my_diffuse_color = gua_diffuse;";
    else
      final_body += "vec3 my_diffuse_color = vec3(0.0);";

    if (capabilities & (EMIT_MAP | EMIT_COLOR))
      final_body += "vec3 my_emit_color = gua_emit;";
    else
      final_body += "vec3 my_emit_color = vec3(0.0);";

    if (capabilities & (AMBIENT_MAP | AMBIENT_COLOR))
      final_body += "float my_ambient_color = gua_ambient;";
    else
      final_body += "float my_ambient_color = 0.5;";

    final_body += "gua_color = my_diffuse_color * gua_light_diffuse;";
    final_body += "gua_color += gua_ambient_color * my_diffuse_color * my_ambient_color * (1.0 - min(1, max(max(gua_light_diffuse.r, gua_light_diffuse.g), gua_light_diffuse.b)));";

    if (capabilities & (SPECULAR_COLOR | SPECULAR_MAP))
      final_body += "gua_color += gua_light_specular;";

    final_body += "    gua_color = gua_color * (vec3(1.0) - my_emit_color) + "
                  "my_emit_color;";

    model->get_final_shading_stage().set_body(final_body);

    ShadingModelDatabase::instance()->add(shading_model_name, model);
  }

  return shading_model_name;
}

////////////////////////////////////////////////////////////////////////////////

}
