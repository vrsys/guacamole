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
#include <gua/renderer/PBSMaterialFactory.hpp>

// guacamole headers
#include <gua/databases/MaterialShaderDatabase.hpp>
#include <gua/renderer/ResourceFactory.hpp>
#include <gua/databases/Resources.hpp>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Material> PBSMaterialFactory::create_material(Capabilities const& capabilities)
{
    auto mat_name(material_name_from_capabilites(capabilities));

    if(MaterialShaderDatabase::instance()->contains(mat_name))
    {
        return gua::MaterialShaderDatabase::instance()->lookup(mat_name)->make_new_material();
    }

    auto desc(std::make_shared<gua::MaterialShaderDescription>());
    ResourceFactory factory;

#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
    auto pbs_color_value_src(factory.read_plain_file("resources/materials/pbs_color_value.gmd"));
    auto pbs_color_map_src(factory.read_plain_file("resources/materials/pbs_color_map.gmd"));
    auto pbs_color_value_and_map_src(factory.read_plain_file("resources/materials/pbs_color_value_and_map.gmd"));

    auto pbs_roughness_value_src(factory.read_plain_file("resources/materials/pbs_roughness_value.gmd"));
    auto pbs_roughness_map_src(factory.read_plain_file("resources/materials/pbs_roughness_map.gmd"));

    auto pbs_metalness_value_src(factory.read_plain_file("resources/materials/pbs_metalness_value.gmd"));
    auto pbs_metalness_map_src(factory.read_plain_file("resources/materials/pbs_metalness_map.gmd"));

    auto pbs_emissivity_value_src(factory.read_plain_file("resources/materials/pbs_emissivity_value.gmd"));
    auto pbs_emissivity_map_src(factory.read_plain_file("resources/materials/pbs_emissivity_map.gmd"));

    auto pbs_normal_map_src(factory.read_plain_file("resources/materials/pbs_normal_map.gmd"));
#else
    auto pbs_color_value_src(Resources::lookup_string(Resources::materials_pbs_color_value_gmd).c_str());
    auto pbs_color_map_src(Resources::lookup_string(Resources::materials_pbs_color_map_gmd).c_str());
    auto pbs_color_value_and_map_src(Resources::lookup_string(Resources::materials_pbs_color_value_and_map_gmd).c_str());

    auto pbs_roughness_value_src(Resources::lookup_string(Resources::materials_pbs_roughness_value_gmd).c_str());
    auto pbs_roughness_map_src(Resources::lookup_string(Resources::materials_pbs_roughness_map_gmd).c_str());

    auto pbs_metalness_value_src(Resources::lookup_string(Resources::materials_pbs_metalness_value_gmd).c_str());
    auto pbs_metalness_map_src(Resources::lookup_string(Resources::materials_pbs_metalness_map_gmd).c_str());

    auto pbs_emissivity_value_src(Resources::lookup_string(Resources::materials_pbs_emissivity_value_gmd).c_str());
    auto pbs_emissivity_map_src(Resources::lookup_string(Resources::materials_pbs_emissivity_map_gmd).c_str());

    auto pbs_normal_map_src(Resources::lookup_string(Resources::materials_pbs_normal_map_gmd).c_str());
#endif

    auto pbs_color_value(std::make_shared<MaterialShaderMethod>());
    pbs_color_value->load_from_json(pbs_color_value_src);
    auto pbs_color_map(std::make_shared<MaterialShaderMethod>());
    pbs_color_map->load_from_json(pbs_color_map_src);
    auto pbs_color_value_and_map(std::make_shared<MaterialShaderMethod>());
    pbs_color_value_and_map->load_from_json(pbs_color_value_and_map_src);
    auto pbs_roughness_value(std::make_shared<MaterialShaderMethod>());
    pbs_roughness_value->load_from_json(pbs_roughness_value_src);
    auto pbs_roughness_map(std::make_shared<MaterialShaderMethod>());
    pbs_roughness_map->load_from_json(pbs_roughness_map_src);
    auto pbs_metalness_value(std::make_shared<MaterialShaderMethod>());
    pbs_metalness_value->load_from_json(pbs_metalness_value_src);
    auto pbs_metalness_map(std::make_shared<MaterialShaderMethod>());
    pbs_metalness_map->load_from_json(pbs_metalness_map_src);
    auto pbs_emissivity_value(std::make_shared<MaterialShaderMethod>());
    pbs_emissivity_value->load_from_json(pbs_emissivity_value_src);
    auto pbs_emissivity_map(std::make_shared<MaterialShaderMethod>());
    pbs_emissivity_map->load_from_json(pbs_emissivity_map_src);
    auto pbs_normal_map(std::make_shared<MaterialShaderMethod>());
    pbs_normal_map->load_from_json(pbs_normal_map_src);

    if(capabilities & Capabilities::ALL)
    {
        desc->add_fragment_method(pbs_color_value_and_map);
        desc->add_fragment_method(pbs_roughness_value);
        desc->add_fragment_method(pbs_roughness_map);
        desc->add_fragment_method(pbs_metalness_value);
        desc->add_fragment_method(pbs_metalness_map);
        desc->add_fragment_method(pbs_emissivity_value);
        desc->add_fragment_method(pbs_emissivity_map);
        desc->add_fragment_method(pbs_normal_map);
    }
    else
    {
        if(capabilities & Capabilities::COLOR_VALUE_AND_MAP)
        {
            desc->add_fragment_method(pbs_color_value_and_map);
        }
        else if(capabilities & Capabilities::COLOR_VALUE)
        {
            desc->add_fragment_method(pbs_color_value);
        }
        else if(capabilities & Capabilities::COLOR_MAP)
        {
            desc->add_fragment_method(pbs_color_map);
        }

        if(capabilities & Capabilities::ROUGHNESS_VALUE)
        {
            desc->add_fragment_method(pbs_roughness_value);
        }
        else if(capabilities & Capabilities::ROUGHNESS_MAP)
        {
            desc->add_fragment_method(pbs_roughness_map);
        }

        if(capabilities & Capabilities::METALNESS_VALUE)
        {
            desc->add_fragment_method(pbs_metalness_value);
        }
        else if(capabilities & Capabilities::METALNESS_MAP)
        {
            desc->add_fragment_method(pbs_metalness_map);
        }

        if(capabilities & Capabilities::EMISSIVITY_VALUE)
        {
            desc->add_fragment_method(pbs_emissivity_value);
        }
        else if(capabilities & Capabilities::EMISSIVITY_MAP)
        {
            desc->add_fragment_method(pbs_emissivity_value);
        }

        if(capabilities & Capabilities::NORMAL_MAP)
        {
            desc->add_fragment_method(pbs_normal_map);
        }
    }

    auto shader(std::make_shared<gua::MaterialShader>(mat_name, desc));
    gua::MaterialShaderDatabase::instance()->add(shader);

    return shader->make_new_material();
}

////////////////////////////////////////////////////////////////////////////////

std::string const PBSMaterialFactory::material_name_from_capabilites(Capabilities const& capabilities)
{
    std::string result("gua");

    if(capabilities & Capabilities::ALL)
    {
        result += "_default_material";
    }
    else
    {
        result += "_pbs";
        if(capabilities & Capabilities::COLOR_VALUE_AND_MAP)
        {
            result += "_color_value_and_map";
        }
        else if(capabilities & Capabilities::COLOR_VALUE)
        {
            result += "_color_value";
        }
        else if(capabilities & Capabilities::COLOR_MAP)
        {
            result += "_color_map";
        }

        if(capabilities & Capabilities::ROUGHNESS_VALUE)
        {
            result += "_roughness_value";
        }
        else if(capabilities & Capabilities::ROUGHNESS_MAP)
        {
            result += "_roughness_map";
        }

        if(capabilities & Capabilities::METALNESS_VALUE)
        {
            result += "_metalness_value";
        }
        else if(capabilities & Capabilities::METALNESS_MAP)
        {
            result += "_metalness_map";
        }

        if(capabilities & Capabilities::EMISSIVITY_VALUE)
        {
            result += "_emissivity_value";
        }
        else if(capabilities & Capabilities::EMISSIVITY_MAP)
        {
            result += "_emissivity_map";
        }

        if(capabilities & Capabilities::NORMAL_MAP)
        {
            result += "_normal_map";
        }
    }

    return result;
}

////////////////////////////////////////////////////////////////////////////////

} // namespace gua
