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
#include <gua/spoints/SPointsLoader.hpp>

// guacamole headers
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>
#include <gua/spoints/SPointsNode.hpp>
#include <gua/spoints/SPointsResource.hpp>
#include <gua/spoints/SPointsRenderer.hpp>

#include <algorithm>
#include <cctype>
#include <fstream>
#include <regex>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////

SPointsLoader::SPointsLoader() : _supported_file_extensions() { _supported_file_extensions.insert("sr"); }

////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////

bool SPointsLoader::is_supported(std::string const& file_name) const
{
    std::vector<std::string> filename_decomposition = gua::string_utils::split(file_name, '.');
    return filename_decomposition.empty() ? false : _supported_file_extensions.count(filename_decomposition.back()) > 0;
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<node::SPointsNode>
SPointsLoader::create_geometry_from_file(std::string const& node_name, std::string const& spoints_resource_filepath, std::shared_ptr<Material> material, unsigned flags)
{
    if(!is_supported(spoints_resource_filepath))
    {
        Logger::LOG_WARNING << "Unable to load " << spoints_resource_filepath << ": Type is not supported!" << std::endl;
    }

    try
    {
        GeometryDescription desc("SPoints", spoints_resource_filepath, 0, flags);


        auto resource = std::make_shared<SPointsResource>(spoints_resource_filepath, flags);
        GeometryDatabase::instance()->add(desc.unique_key(), resource);

        auto result = std::shared_ptr<node::SPointsNode>(new node::SPointsNode(node_name, desc.unique_key()));
        result->update_cache();

        // add a default spoints material if not already loaded
        if(!gua::MaterialShaderDatabase::instance()->contains("gua_default_spoints_material"))
        {
            ResourceFactory factory;
            auto material = factory.read_plain_file("resources/materials/spoints.gmd");
            auto desc(std::make_shared<gua::MaterialShaderDescription>());
            desc->load_from_json(material.c_str());
            auto shader(std::make_shared<gua::MaterialShader>("gua_default_spoints_material", desc));
            gua::MaterialShaderDatabase::instance()->add(shader);
        }

        if(!material)
        {
            result->set_material(gua::MaterialShaderDatabase::instance()->lookup("gua_default_spoints_material")->make_new_material());
        }
        else
        {
            result->set_material(material);
        }

        auto bbox = resource->get_bounding_box();

        // normalize position?
        auto normalize_position = flags & SPointsLoader::NORMALIZE_POSITION;
        if(normalize_position)
        {
            auto bbox_center_object_space = math::vec4(bbox.center().x, bbox.center().y, bbox.center().z, 1.0);
            result->translate(-bbox_center_object_space.x, -bbox_center_object_space.y, -bbox_center_object_space.z);
        }

        // normalize scale?
        auto normalize_scale = flags & SPointsLoader::NORMALIZE_SCALE;
        if(normalize_scale)
        {
            auto scale = 1.0f / scm::math::length(bbox.max - bbox.min);
            result->scale(scale, scale, scale);
        }

        return result;
    }
    catch(std::exception& e)
    {
        Logger::LOG_WARNING << "Warning: " << e.what() << " : Failed to load SPoints object " << spoints_resource_filepath.c_str() << std::endl;
        return nullptr;
    }
}

} // namespace gua
