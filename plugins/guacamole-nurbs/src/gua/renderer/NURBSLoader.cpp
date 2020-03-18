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
#include <gua/renderer/NURBSLoader.hpp>

// guacamole headers
#include <gua/utils.hpp>
#include <gua/utils/Logger.hpp>

#include <gua/node/NURBSNode.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/renderer/NURBSResource.hpp>

#include <gpucast/core/import/igs.hpp>
#include <gpucast/core/surface_converter.hpp>
#include <gpucast/core/nurbssurfaceobject.hpp>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////
NURBSLoader::NURBSLoader() : _supported_file_extensions()
{
    _supported_file_extensions.insert("igs");
    _supported_file_extensions.insert("iges");
    _supported_file_extensions.insert("IGS");
    _supported_file_extensions.insert("IGES");
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<node::NURBSNode> NURBSLoader::load_geometry(std::string const& nodename, std::string const& filename, std::shared_ptr<Material> const& fallback_material, unsigned flags)
{
    auto cached_node(load_geometry(filename, flags));

    if(cached_node)
    {
        auto copy = std::dynamic_pointer_cast<node::NURBSNode>(cached_node->deep_copy(false));

        if(copy)
        {
            apply_fallback_material(copy, fallback_material);
            copy->set_name(nodename);
            return copy;
        }
    }

    Logger::LOG_WARNING << "NURBSLoader::load_geometry() : unable to create NURBS Node" << std::endl;
    return std::shared_ptr<node::NURBSNode>(new node::NURBSNode(nodename));
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<node::NURBSNode> NURBSLoader::load_geometry(std::string const& filename, unsigned flags)
{
    try
    {
        if(!is_supported(filename))
        {
            throw std::runtime_error(std::string("Unsupported filetype: ") + filename);
        }
        else
        {
            // import model
            gpucast::igs_loader igsloader;
            gpucast::surface_converter surface_converter;

            auto bezier_object = std::make_shared<gpucast::beziersurfaceobject>();
            auto nurbsobjects = igsloader.load(filename);

            for(auto nurbsobject : nurbsobjects)
            {
                auto object = std::make_shared<gpucast::beziersurfaceobject>();
                surface_converter.convert(nurbsobject, object);
                bezier_object->merge(*object);
            }

            // check and set rendering mode and create render resources
            auto fill_mode = flags & WIREFRAME ? scm::gl::FILL_WIREFRAME : scm::gl::FILL_SOLID;
            auto pre_subdivision_u = flags & PRE_SUBDIVISION ? 1 : 0;
            auto pre_subdivision_v = flags & PRE_SUBDIVISION ? 1 : 0;

            auto trim_resolution = 0;

            if(flags & TRIM_TEXTURE_8)
                trim_resolution = 8;
            if(flags & TRIM_TEXTURE_16)
                trim_resolution = 16;
            if(flags & TRIM_TEXTURE_32)
                trim_resolution = 32;

            auto ressource = std::make_shared<NURBSResource>(bezier_object, pre_subdivision_u, pre_subdivision_v, trim_resolution, fill_mode);
            Logger::LOG_WARNING << "NURBS Object loaded with " << bezier_object->surfaces() << " surfaces.\n";

            // add resource to database
            GeometryDescription desc("NURBS", filename, 0, flags);
            GeometryDatabase::instance()->add(desc.unique_key(), ressource);

            auto node = std::shared_ptr<node::NURBSNode>(new node::NURBSNode(filename, desc.unique_key()));
            node->update_cache();

            auto bbox = ressource->get_bounding_box();

            // normalize position?
            auto normalize_position = flags & NORMALIZE_POSITION;
            if(normalize_position)
            {
                node->translate(-bbox.center());
            }

            // normalize scale?
            auto normalize_size = flags & NORMALIZE_SCALE;
            if(normalize_size)
            {
                node->scale(1.0f / scm::math::length(bbox.max - bbox.min));
            }

            return node;
        }
    }
    catch(std::exception& e)
    {
        Logger::LOG_WARNING << "Failed to load NURBS object \"" << filename << "\": " << e.what() << std::endl;
        return nullptr;
    }
}

////////////////////////////////////////////////////////////////////////////////

bool NURBSLoader::is_supported(std::string const& file_name) const
{
    std::vector<std::string> filename_decomposition = gua::string_utils::split(file_name, '.');
    return filename_decomposition.empty() ? false : _supported_file_extensions.count(filename_decomposition.back()) > 0;
}

////////////////////////////////////////////////////////////////////////////////

void NURBSLoader::apply_fallback_material(std::shared_ptr<node::Node> const& root, std::shared_ptr<Material> const& fallback_material) const
{
    auto g_node(std::dynamic_pointer_cast<node::NURBSNode>(root));

    if(g_node && !g_node->get_material())
    {
        g_node->set_material(fallback_material);
        g_node->update_cache();
    }

    for(auto& child : root->get_children())
    {
        apply_fallback_material(child, fallback_material);
    }
}

} // namespace gua
