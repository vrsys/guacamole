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
#include <gua/renderer/MaterialShader.hpp>
#include <gua/renderer/PBSMaterialFactory.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/utils/TextFile.hpp>
#include <gua/utils/string_utils.hpp>

// external headers
#include <assimp/scene.h>
#include <fstream>
#include <jsoncpp/json/json.h>

// Windows includes
#if defined(__WIN32__) || defined(_WIN32) || defined(_WIN64)
#include <io.h>
#define F_OK 0
#define X_OK 1
#define W_OK 2
#define R_OK 4
#endif

#ifdef GUACAMOLE_FBX
#include <fbxsdk.h>
#endif

namespace gua
{
////////////////////////////////////////////////////////////////////////////////

bool replace(std::string& str, const std::string& from, const std::string& to)
{
    size_t start_pos = str.find(from);
    if(start_pos == std::string::npos)
        return false;
    str.replace(start_pos, from.length(), to);
    return true;
}

std::shared_ptr<Material> MaterialLoader::load_material(aiMaterial const* ai_material, std::string const& assets_directory, bool optimize_material, bool nrp) const
{
    // helper lambdas ------------------------------------------------------------
    auto get_color = [&](const char* pKey, unsigned int type, unsigned int idx) -> std::string {
        aiColor3D value;
        if(AI_SUCCESS != ai_material->Get(pKey, type, idx, value))
            return "";
        math::vec3 color(value.r, value.g, value.b);
        return string_utils::to_string(color);
    };

    auto get_string = [&](const char* pKey, unsigned int type, unsigned int idx) -> std::string {
        aiString value;
        if(AI_SUCCESS != ai_material->Get(pKey, type, idx, value))
            return "";
        return std::string(value.data);
    };

    auto get_sampler = get_string;

    auto get_float = [&](const char* pKey, unsigned int type, unsigned int idx) -> std::string {
        float value;
        if(AI_SUCCESS != ai_material->Get(pKey, type, idx, value))
            return "";
        return string_utils::to_string(value);
    };

    PathParser path;
    path.parse(assets_directory);
    std::string assets(path.get_path(true));

    std::string uniform_color_map(get_sampler(AI_MATKEY_TEXTURE(aiTextureType_DIFFUSE, 0)));
    std::string uniform_color(get_color(AI_MATKEY_COLOR_DIFFUSE));
    std::string uniform_roughness_map(get_sampler(AI_MATKEY_TEXTURE(aiTextureType_REFLECTION, 0)));
    std::string uniform_roughness(get_float(AI_MATKEY_SHININESS));
    std::string uniform_emit_map(get_sampler(AI_MATKEY_TEXTURE(aiTextureType_EMISSIVE, 0)));
    std::string uniform_emit(get_color(AI_MATKEY_COLOR_EMISSIVE));
    std::string uniform_reflection_map(get_sampler(AI_MATKEY_TEXTURE(aiTextureType_SHININESS, 0)));
    std::string ambient_map(get_sampler(AI_MATKEY_TEXTURE(aiTextureType_AMBIENT, 0)));
    std::string ambient_color(get_color(AI_MATKEY_COLOR_AMBIENT));
    std::string uniform_metalness_map(get_sampler(AI_MATKEY_TEXTURE(aiTextureType_SPECULAR, 0)));
    std::string uniform_metalness(get_color(AI_MATKEY_COLOR_SPECULAR));
    std::string uniform_opacity_map(get_sampler(AI_MATKEY_TEXTURE(aiTextureType_OPACITY, 0)));
    std::string uniform_opacity(get_float(AI_MATKEY_OPACITY));

    std::string material_name(get_string(AI_MATKEY_NAME));

    // obj bump and map_bump textures are imported as aiTextureType_HEIGHT
    std::string uniform_normal_map(get_sampler(AI_MATKEY_TEXTURE(aiTextureType_NORMALS, 0)));
    if(uniform_normal_map.empty())
    {
        uniform_normal_map = get_sampler(AI_MATKEY_TEXTURE(aiTextureType_HEIGHT, 0));
    }

    if(nrp)
    {
        uniform_normal_map = std::string(uniform_color_map);
        uniform_roughness_map = std::string(uniform_color_map);
        uniform_metalness_map = std::string(uniform_color_map);

        std::string base_color_string = "Base_Color";

        replace(uniform_normal_map, base_color_string + ".jpg", "Normal.png");
        replace(uniform_roughness_map, base_color_string, "Roughness");
        replace(uniform_metalness_map, base_color_string, "Metallic");

        if(access((assets + uniform_color_map).c_str(), F_OK) == -1)
        {
            uniform_color_map = "";
        }

        if(access((assets + uniform_roughness_map).c_str(), F_OK) == -1)
        {
            uniform_roughness_map = "";
        }

        if(access((assets + uniform_metalness_map).c_str(), F_OK) == -1)
        {
            uniform_metalness_map = "";
        }

        if(access((assets + uniform_normal_map).c_str(), F_OK) == -1)
        {
            uniform_normal_map = "";
        }
    }

    unsigned capabilities = 0;

    if(!optimize_material)
    {
        capabilities |= PBSMaterialFactory::ALL;
    }
    else
    {
        if(!uniform_color_map.empty() && !uniform_color.empty())
        {
            capabilities |= PBSMaterialFactory::COLOR_VALUE_AND_MAP;
        }
        else if(!uniform_color_map.empty())
        {
            capabilities |= PBSMaterialFactory::COLOR_MAP;
        }
        else if(!uniform_color.empty())
        {
            capabilities |= PBSMaterialFactory::COLOR_VALUE;
        }

#if 1
        if(!uniform_roughness_map.empty())
        {
            capabilities |= PBSMaterialFactory::ROUGHNESS_MAP;
        }
        else if(!uniform_roughness.empty() && uniform_roughness != "0")
        {
            capabilities |= PBSMaterialFactory::ROUGHNESS_VALUE;
        }
#endif

#if 1
        if(!uniform_metalness_map.empty())
        {
            capabilities |= PBSMaterialFactory::METALNESS_MAP;
        }
        else if(!uniform_metalness.empty())
        {
            capabilities |= PBSMaterialFactory::METALNESS_VALUE;
        }
#endif

        if(!uniform_emit_map.empty())
        {
            capabilities |= PBSMaterialFactory::EMISSIVITY_MAP;
        }
        else if(!uniform_emit.empty())
        {
            capabilities |= PBSMaterialFactory::EMISSIVITY_VALUE;
        }

        if(!uniform_normal_map.empty())
        {
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

        if(!ambient_map.empty())
        {
            Logger::LOG_WARNING << "Material not fully supported: guacamole does not support ambient maps." << std::endl;
        }
        else if(!ambient_color.empty())
        {
            Logger::LOG_WARNING << "Material not fully supported: guacamole does not support ambient colors." << std::endl;
        }
    }

    auto new_mat(PBSMaterialFactory::create_material(static_cast<PBSMaterialFactory::Capabilities>(capabilities)));

    if(!uniform_color_map.empty())
    {
        new_mat->set_uniform("ColorMap", assets + uniform_color_map);
        TextureDatabase::instance()->load(assets + uniform_color_map);
    }

    if(!uniform_color.empty())
    {
        auto c(string_utils::from_string<math::vec3>(uniform_color));

        float opacity_to_set = 1.0f;
        if(!uniform_opacity.empty())
        {
            opacity_to_set = std::max(0.0f, std::min(1.0f, string_utils::from_string<float>(uniform_opacity)));
        }

        new_mat->set_uniform("Color", scm::math::vec4f(gua::math::float_t(c.x), gua::math::float_t(c.y), gua::math::float_t(c.z), opacity_to_set));
    }

#if 1
    if(!uniform_roughness_map.empty())
    {
        new_mat->set_uniform("RoughnessMap", assets + uniform_roughness_map);
    }
    else if(!uniform_roughness.empty() && uniform_roughness != "0")
    {
        // specular exponent is taken to the power of 0.02 in order to move it to the desired range
        new_mat->set_uniform("Roughness", float(std::min(1.f, std::pow(string_utils::from_string<float>(uniform_roughness), 0.02f) - 1.f)));
    }
#endif

#if 1
    if(!uniform_metalness_map.empty())
    {
        new_mat->set_uniform("MetalnessMap", assets + uniform_metalness_map);
    }
    else if(!uniform_metalness.empty())
    {
        // multiplying with 0.5, since metalness of 1.0 is seldomly wanted but specularity of 1.0 often given
        new_mat->set_uniform("Metalness", scm::math::vec3f(string_utils::from_string<math::vec3>(uniform_metalness)[0] * 0.5f));
    }
#endif

    if(!uniform_emit_map.empty())
    {
        new_mat->set_uniform("EmissivityMap", assets + uniform_emit_map);
    }
    else if(!uniform_emit.empty())
    {
        new_mat->set_uniform("Emissivity", string_utils::from_string<scm::math::vec3f>(uniform_emit)[0]);
    }

    if(!uniform_normal_map.empty())
    {
        new_mat->set_uniform("NormalMap", assets + uniform_normal_map);
    }

    new_mat->rename_existing_shader(material_name);

    return new_mat;
}

////////////////////////////////////////////////////////////////////////////////
#ifdef GUACAMOLE_FBX
std::shared_ptr<Material> MaterialLoader::load_material(FbxSurfaceMaterial const& fbx_material, std::string const& assets_directory, bool optimize_material, bool nrp) const
{
    PathParser path;
    path.parse(assets_directory);
    std::string assets(path.get_path(true));

    // see if there is an unreal engine material file avaible
    std::string mat_name = assets + fbx_material.GetName() + ".COPY";
    if(file_exists(mat_name))
    {
        return load_unreal(mat_name, assets_directory);
    }

    // see if there is a json material file avaible
    mat_name = assets + fbx_material.GetName() + ".json";
    if(file_exists(mat_name))
    {
        return load_material(mat_name, assets_directory);
    }

    // method to check if texture is set for an attribute
    auto get_sampler = [&](const char* attribute) -> std::string {
        FbxProperty property = fbx_material.FindProperty(attribute);
        unsigned texture_count = property.GetSrcObjectCount<FbxTexture>();
        if(texture_count > 0)
        {
            if(texture_count > 1)
            {
                Logger::LOG_WARNING << property.GetNameAsCStr() << " has more than one texture, only first one." << std::endl;
            }
            // texture could also be layered or procedural texture
            FbxFileTexture* texture = property.GetSrcObject<FbxFileTexture>(0);
            if(texture)
            {
                return get_file_name(texture->GetFileName());
            }
            else
            {
                Logger::LOG_WARNING << "Texturetype of " << property.GetNameAsCStr() << " not supported." << std::endl;
            }
        }
        return "";
    };

    // check which shading model is used
    std::string shading{fbx_material.ShadingModel.Get().Buffer()};
    bool shading_supported = shading == "Phong" || shading == "Lambert" || shading == "phong" || shading == "lambert";
    if(!shading_supported)
    {
        Logger::LOG_DEBUG << "Shading Type '" << shading << "' not supported." << std::endl;
    }
    // cast to phong not necessary, only diffuse and emissive values needed
    FbxSurfaceLambert* lambert = (FbxSurfaceLambert*)&fbx_material;
    if(!lambert)
    {
        Logger::LOG_ERROR << "Casting Material to Lambert failed." << std::endl;
        assert(0);
    }

    std::string uniform_ambient_map{get_sampler(FbxSurfaceMaterial::sAmbient)};
    std::string uniform_color_map{get_sampler(FbxSurfaceMaterial::sDiffuse)};
    std::string uniform_emit_map{get_sampler(FbxSurfaceMaterial::sEmissive)};
    std::string uniform_normal_map{get_sampler(FbxSurfaceMaterial::sNormalMap)};

    // check bump slot if no normalmap found
    if(uniform_normal_map == "")
    {
#if WIN32
        uniform_normal_map = get_sampler("Bump");
#else
        uniform_normal_map = get_sampler(FbxSurfaceMaterial::sBump);
#endif
    }

    unsigned capabilities = 0;

    if(!optimize_material)
    {
        capabilities |= PBSMaterialFactory::ALL;
    }
    else
    {
        if(uniform_color_map != "")
        {
            capabilities |= PBSMaterialFactory::COLOR_VALUE_AND_MAP;
        }
        else
        {
            capabilities |= PBSMaterialFactory::COLOR_VALUE;
        }

        if(uniform_emit_map != "")
        {
            capabilities |= PBSMaterialFactory::EMISSIVITY_MAP;
        }
        else
        {
            capabilities |= PBSMaterialFactory::EMISSIVITY_VALUE;
        }

        if(uniform_normal_map != "")
        {
            capabilities |= PBSMaterialFactory::NORMAL_MAP;
        }

        if(uniform_ambient_map != "")
        {
            Logger::LOG_WARNING << "Material not fully supported: guacamole does not support ambient maps." << std::endl;
        }
        // else if (ambient_color != "") {
        //   Logger::LOG_WARNING << "Material not fully supported: guacamole does not support ambient colors." << std::endl;
        // }
    }

    auto new_mat(PBSMaterialFactory::create_material(static_cast<PBSMaterialFactory::Capabilities>(capabilities)));

    if(capabilities & PBSMaterialFactory::COLOR_MAP || capabilities & PBSMaterialFactory::COLOR_VALUE_AND_MAP)
    {
        new_mat->set_uniform("ColorMap", assets + uniform_color_map);
        TextureDatabase::instance()->load(assets + uniform_color_map);
    }
    // fbx shader always contains a color value
    FbxDouble3 color = lambert->Diffuse.Get();
    new_mat->set_uniform("Color", math::vec4f(color[0], color[1], color[2], 1.f));

    if(capabilities & PBSMaterialFactory::NORMAL_MAP)
    {
        new_mat->set_uniform("NormalMap", assets + uniform_normal_map);
    }

    if(capabilities & PBSMaterialFactory::EMISSIVITY_MAP)
    {
        new_mat->set_uniform("EmissivityMap", assets + uniform_emit_map);
    }
    else
    {
        FbxDouble3 emit = lambert->Emissive.Get();
        new_mat->set_uniform("Emissivity", float((emit[0] + emit[1] + emit[2]) / 3.0f));
    }

    return new_mat;
}

std::shared_ptr<Material> MaterialLoader::load_unreal(std::string const& file_name, std::string const& assets_directory, bool optimize_material, bool nrp) const
{
    PathParser path;
    path.parse(assets_directory);
    std::string assets(path.get_path(true));

    std::set<std::string> textures{};

    std::string line;
    std::ifstream file{file_name};

    if(file.is_open())
    {
        while(getline(file, line))
        {
            if(line.find("Texture=") != std::string::npos)
            {
                std::string value = get_file_name(line);
                std::string name = value.substr(value.find(".") + 1, value.size() - value.find(".") - 3);

                if(textures.find(name) == textures.end())
                {
                    textures.insert(name);
                }
            }
        }

        file.close();
    }
    else
    {
        std::cout << "File '" << file_name << "' could not be opened" << std::endl;
        return PBSMaterialFactory::create_material(static_cast<PBSMaterialFactory::Capabilities>(PBSMaterialFactory::ALL));
    }

    auto ends_with = [](std::string const& name, std::string const& suffix) {
        for(unsigned i = 0; i < suffix.size(); ++i)
        {
            if(name.at(name.size() - suffix.size() + i) != suffix.at(i))
            {
                return false;
            }
        }
        return true;
    };

    std::string uniform_color_map{""};
    std::string uniform_emit_map{""};
    std::string uniform_normal_map{""};

    unsigned capabilities = 0;

    if(!optimize_material)
    {
        capabilities |= PBSMaterialFactory::ALL;
    }

    for(auto const& tex : textures)
    {
        if(ends_with(tex, "_D") || ends_with(tex, "diffuse") || ends_with(tex, "DF"))
        {
            std::string name{assets + tex};
            if(file_exists(name + ".TGA"))
            {
                capabilities |= PBSMaterialFactory::COLOR_MAP;
                uniform_color_map = name + ".TGA";
            }
            else if(file_exists(name + ".tga"))
            {
                capabilities |= PBSMaterialFactory::COLOR_MAP;
                uniform_color_map = name + ".tga";
            }
        }
        else if(ends_with(tex, "_N") || ends_with(tex, "normal") || ends_with(tex, "NRM"))
        {
            std::string name{assets + tex};
            if(file_exists(name + ".TGA"))
            {
                capabilities |= PBSMaterialFactory::NORMAL_MAP;
                uniform_normal_map = name + ".TGA";
            }
            else if(file_exists(name + ".tga"))
            {
                capabilities |= PBSMaterialFactory::NORMAL_MAP;
                uniform_normal_map = name + ".tga";
            }
        }
        else if(ends_with(tex, "Emissive") || ends_with(tex, "glow"))
        {
            std::string name{assets + tex};
            if(file_exists(name + ".TGA"))
            {
                capabilities |= PBSMaterialFactory::EMISSIVITY_MAP;
                uniform_emit_map = name + ".TGA";
            }
            else if(file_exists(name + ".tga"))
            {
                capabilities |= PBSMaterialFactory::EMISSIVITY_MAP;
                uniform_emit_map = name + ".tga";
            }
        }
    }

    auto new_mat(PBSMaterialFactory::create_material(static_cast<PBSMaterialFactory::Capabilities>(capabilities)));

    if(capabilities & PBSMaterialFactory::COLOR_MAP)
    {
        new_mat->set_uniform("ColorMap", uniform_color_map);
        TextureDatabase::instance()->load(uniform_color_map);
    }

    if(capabilities & PBSMaterialFactory::NORMAL_MAP)
    {
        new_mat->set_uniform("NormalMap", uniform_normal_map);
    }

    if(capabilities & PBSMaterialFactory::EMISSIVITY_MAP)
    {
        new_mat->set_uniform("EmissivityMap", uniform_emit_map);
    }

    return new_mat;
}
#endif

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Material> MaterialLoader::load_material(std::string const& file_name, std::string const& assets_directory, bool optimize_material, bool nrp) const
{
    PathParser path;
    path.parse(assets_directory);
    std::string assets(path.get_path(true));

    TextFile file{file_name};
    if(!file.is_valid())
    {
        Logger::LOG_WARNING << "Failed to load material description\"" << file_name
                            << "\": "
                               "File does not exist!"
                            << std::endl;
        return PBSMaterialFactory::create_material(static_cast<PBSMaterialFactory::Capabilities>(PBSMaterialFactory::ALL));
    }

    Json::Value properties;
    Json::Reader reader;
    if(!reader.parse(file.get_content(), properties))
    {
        Logger::LOG_WARNING << "Failed to parse json material description: " << file_name << std::endl;
        return PBSMaterialFactory::create_material(static_cast<PBSMaterialFactory::Capabilities>(PBSMaterialFactory::ALL));
    }

    // helper lambdas ------------------------------------------------------------
    auto get_sampler = [&properties](std::string const& name) -> std::string {
        if(properties[name] != Json::Value::null && properties[name].isString())
        {
            return get_file_name(properties[name].asString());
        }
        else
        {
            return "";
        }
    };

    auto get_float = [&properties](std::string const& name) -> float {
        if(properties[name] != Json::Value::null && (properties[name].isDouble() || properties[name].isInt()))
        {
            return float(properties[name].asDouble());
        }
        else
        {
            return NAN;
        }
    };
    auto get_bool = [&properties](std::string const& name) -> bool {
        if(properties[name] != Json::Value::null && properties[name].isBool())
        {
            return properties[name].asBool();
        }
        else
        {
            return true;
        }
    };

    auto get_color = [&properties](std::string const& name) -> scm::math::vec4f {
        if(properties[name] != Json::Value::null && properties[name].isArray())
        {
            size_t num_values = properties[name].size();
            if(num_values == 4)
            {
                return scm::math::vec4f{float(properties[name][0U].asDouble()), float(properties[name][1U].asDouble()), float(properties[name][2U].asDouble()), float(properties[name][3U].asDouble())};
            }
            else if(num_values == 3)
            {
                return scm::math::vec4f{float(properties[name][0U].asDouble()), float(properties[name][1U].asDouble()), float(properties[name][2U].asDouble()), 1.0f};
            }
            else
            {
                Logger::LOG_WARNING << "Color property '" << name << "' has unsupported value number of " << num_values << std::endl;
            }
        }
        return scm::math::vec4f{NAN, NAN, NAN};
    };

    unsigned capabilities;

    std::string uniform_color_map{get_sampler("color")};
    std::string uniform_normal_map{get_sampler("normal")};
    std::string uniform_roughness_map{get_sampler("roughness")};
    std::string uniform_metalness_map{get_sampler("metalness")};
    std::string uniform_emissivity_map{get_sampler("emissivity")};
    scm::math::vec4f uniform_color{get_color("color")};
    float uniform_roughness{get_float("roughness")};
    float uniform_metalness{get_float("metalness")};
    float uniform_emissivity{get_float("emissivity")};
    float uniform_opacity{get_float("opacity")};

    if(!optimize_material)
    {
        capabilities |= PBSMaterialFactory::ALL;
    }
    else
    {
        // color
        if(uniform_color_map != "")
        {
            if(!std::isnan(uniform_color[0]))
            {
                capabilities |= PBSMaterialFactory::COLOR_VALUE_AND_MAP;
            }
            else
            {
                capabilities |= PBSMaterialFactory::COLOR_MAP;
            }
        }
        else if(!std::isnan(uniform_color[0]))
        {
            capabilities |= PBSMaterialFactory::COLOR_VALUE;
        }
        // normals
        if(uniform_normal_map != "")
        {
            capabilities |= PBSMaterialFactory::NORMAL_MAP;
        }
        // roughness
        if(uniform_roughness_map != "")
        {
            capabilities |= PBSMaterialFactory::ROUGHNESS_MAP;
        }
        else if(!std::isnan(uniform_roughness))
        {
            capabilities |= PBSMaterialFactory::ROUGHNESS_VALUE;
        }
        // metalness
        if(uniform_metalness_map != "")
        {
            capabilities |= PBSMaterialFactory::METALNESS_MAP;
        }
        else if(!std::isnan(uniform_metalness))
        {
            capabilities |= PBSMaterialFactory::METALNESS_VALUE;
        }
        // emissivity
        if(uniform_emissivity_map != "")
        {
            capabilities |= PBSMaterialFactory::EMISSIVITY_MAP;
        }
        else if(!std::isnan(uniform_emissivity))
        {
            capabilities |= PBSMaterialFactory::EMISSIVITY_VALUE;
        }
    }

    auto new_mat(PBSMaterialFactory::create_material(static_cast<PBSMaterialFactory::Capabilities>(capabilities)));

    // color
    if(capabilities & PBSMaterialFactory::COLOR_MAP || capabilities & PBSMaterialFactory::COLOR_VALUE_AND_MAP)
    {
        new_mat->set_uniform("ColorMap", assets + uniform_color_map);
        TextureDatabase::instance()->load(assets + uniform_color_map);
        if(capabilities & PBSMaterialFactory::COLOR_VALUE_AND_MAP)
        {
            new_mat->set_uniform("Color", uniform_color);
        }
    }
    else if(capabilities & PBSMaterialFactory::COLOR_VALUE)
    {
        new_mat->set_uniform("Color", uniform_color);
    }

    // normals
    if(capabilities & PBSMaterialFactory::NORMAL_MAP)
    {
        new_mat->set_uniform("NormalMap", assets + uniform_normal_map);
    }
    // roughness
    if(capabilities & PBSMaterialFactory::ROUGHNESS_MAP)
    {
        new_mat->set_uniform("RoughtnessMap", assets + uniform_roughness_map);
    }
    else if(capabilities & PBSMaterialFactory::ROUGHNESS_VALUE)
    {
        new_mat->set_uniform("Roughness", uniform_roughness);
    }
    // metalness
    if(capabilities & PBSMaterialFactory::METALNESS_MAP)
    {
        new_mat->set_uniform("MetalnessMap", assets + uniform_metalness_map);
    }
    else if(capabilities & PBSMaterialFactory::METALNESS_VALUE)
    {
        new_mat->set_uniform("Metalness", uniform_metalness);
    }
    // emissivity
    if(capabilities & PBSMaterialFactory::EMISSIVITY_MAP)
    {
        new_mat->set_uniform("EmissivityMap", assets + uniform_emissivity_map);
    }
    else if(capabilities & PBSMaterialFactory::EMISSIVITY_VALUE)
    {
        new_mat->set_uniform("Emissivity", uniform_emissivity);
    }
    // opacity
    if(!std::isnan(uniform_opacity))
    {
        new_mat->set_uniform("Opacity", uniform_opacity);
    }
    // culling
    if(properties["backface_culling"] != Json::Value::null && properties["backface_culling"].isBool())
    {
        new_mat->set_show_back_faces(!properties["backface_culling"].asBool());
    }
    // wireframe rendering
    if(properties["wireframe_rendering"] != Json::Value::null && properties["wireframe_rendering"].isBool())
    {
        new_mat->set_render_wireframe(properties["wireframe_rendering"].asBool());
    }

    return new_mat;
}

////////////////////////////////////////////////////////////////////////////////

std::string MaterialLoader::get_file_name(std::string const& path)
{
    // filter out possible path in front of filename
    std::string file_name{path};
    auto path_end(path.find_last_of("/\\"));
    if(path_end != std::string::npos)
    {
        file_name = path.substr(path_end + 1);
    }
    return file_name;
}

inline bool MaterialLoader::file_exists(std::string const& path)
{
    std::ifstream file{path.c_str()};
    return !file.fail();
}
} // namespace gua
