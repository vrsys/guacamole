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
#include <gua/renderer/DynamicGeometryLoader.hpp>

// guacamole headers
#include <gua/utils/TextFile.hpp>
// #include <gua/utils/DynamicGeometryImporter.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/utils/string_utils.hpp>
#include <gua/utils/ToGua.hpp>
#include <gua/node/DynamicGeometryNode.hpp>
#include <gua/node/TransformNode.hpp>
#include <gua/renderer/MaterialLoader.hpp>
#include <gua/renderer/DynamicGeometryResource.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>
#include <gua/databases/GeometryDatabase.hpp>

namespace gua
{
/////////////////////////////////////////////////////////////////////////////
// static variables
/////////////////////////////////////////////////////////////////////////////
std::unordered_map<std::string, std::shared_ptr<::gua::node::Node>> DynamicGeometryLoader::loaded_files_ = std::unordered_map<std::string, std::shared_ptr<::gua::node::Node>>();

/////////////////////////////////////////////////////////////////////////////

DynamicGeometryLoader::DynamicGeometryLoader() {}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<node::Node> DynamicGeometryLoader::load_geometry(std::string const &file_name, unsigned flags, bool create_empty)
{
    std::shared_ptr<node::Node> cached_node;
    std::string key(file_name + "_" + string_utils::to_string(flags));
    auto searched(loaded_files_.find(key));
    if(searched != loaded_files_.end())
    {
        cached_node = searched->second;
    }
    else
    {
        bool fileload_succeed = false;
        int topology_type = is_supported(file_name);

        if(topology_type)
        {
            bool create_geometries = (topology_type == 1);
            cached_node = load(file_name, flags, create_geometries, create_empty);

            cached_node->update_cache();
            // std::cout << "" << static_cast<std::shared_ptr<DynamicGeometryNode>>(cached_node)->get_geometry() << std::endl;
            loaded_files_.insert(std::make_pair(key, cached_node));

            // normalize mesh position and rotation
            if(flags & DynamicGeometryLoader::NORMALIZE_POSITION || flags & DynamicGeometryLoader::NORMALIZE_SCALE)
            {
                auto bbox = cached_node->get_bounding_box();

                if(flags & DynamicGeometryLoader::NORMALIZE_POSITION)
                {
                    auto center((bbox.min + bbox.max) * 0.5f);
                    cached_node->translate(-center);
                }

                if(flags & DynamicGeometryLoader::NORMALIZE_SCALE)
                {
                    auto size(bbox.max - bbox.min);
                    auto max_size(std::max(std::max(size.x, size.y), size.z));
                    cached_node->scale(1.f / max_size);
                }
            }
            fileload_succeed = true;
        }

        if(!fileload_succeed)
        {
            Logger::LOG_WARNING << "Unable to load " << file_name << ": Type is not supported!" << std::endl;
        }
    }

    return cached_node;
}

/////////////////////////////////////////////////////////////////////////////

std::shared_ptr<node::Node> DynamicGeometryLoader::create_empty_geometry(std::string const &node_name, std::string const &empty_name, std::shared_ptr<Material> const &fallback_material,
                                                                         unsigned flags)
{
    auto cached_node(load_geometry(empty_name, flags, true));

    if(cached_node)
    {
        auto copy(cached_node->deep_copy(false));

        apply_fallback_material(copy, fallback_material, flags & NO_SHARED_MATERIALS);

        copy->set_name(node_name);
        return copy;
    }

    return std::make_shared<node::TransformNode>(node_name);
}

/////////////////////////////////////////////////////////////////////////////

std::shared_ptr<node::Node> DynamicGeometryLoader::create_empty_geometry(std::string const &node_name, std::string const &empty_name, unsigned flags)
{
    auto cached_node(load_geometry(empty_name, flags, true));

    if(cached_node)
    {
        auto copy(cached_node->deep_copy(false));

        auto shader(gua::MaterialShaderDatabase::instance()->lookup("gua_default_material"));
        apply_fallback_material(copy, shader->make_new_material(), flags & NO_SHARED_MATERIALS);

        copy->set_name(node_name);
        return copy;
    }

    return std::make_shared<node::TransformNode>(node_name);
}

/////////////////////////////////////////////////////////////////////////////

std::shared_ptr<node::Node> DynamicGeometryLoader::load(std::string const &file_name, unsigned flags, bool create_geometries, bool create_empty)
{
    TextFile file(file_name);

    // MESSAGE("Loading mesh file %s", file_name.c_str());

    if(file.is_valid() || create_empty)
    {
        auto importer = std::make_shared<DynamicGeometryImporter>();

        importer->create_empty_dynamic_geometry(file_name);

        if(importer->parsing_successful())
        {
            std::cout << "Successfully parsed lob object\n";
        }
        else
        {
            std::cout << "Did not parse lob object successfully\n";
        }

        uint32_t dynamic_geometry_count = 0;
        // creates a geometry node and returns it
        auto load_geometry = [&]() {
            GeometryDescription desc("DynamicGeometry", file_name, dynamic_geometry_count++, flags);

            std::shared_ptr<node::DynamicGeometryNode> node_to_return = create_geometry_instance(importer, desc, flags);

            if(create_geometries)
            {
                node_to_return->set_render_vertices_as_points(false);
                // node_to_return->set_render_volumetric(false);
                node_to_return->set_render_volumetric(false);
            }
            // std::cout<<"node to return"<<node_to_return->get_geometry()<<std::endl;
            return node_to_return;
        };

        // there is only one geometry --- return it!
        if(importer->num_parsed_dynamic_geometries() == 1)
        {
            // std::cout<<"show geo " << load_geometry() << " " << load_geometry()->get_geometry_description()<<std::endl;
            return load_geometry();
        }
    }

    Logger::LOG_WARNING << "Failed to load object \"" << file_name << "\": File does not exist!" << std::endl;

    return nullptr;
}

/////////////////////////////////////////////////////////////////////////////

std::vector<DynamicGeometryResource *> const DynamicGeometryLoader::load_from_buffer(char const *buffer_name, unsigned buffer_size, bool build_kd_tree)
{
    /*
      auto importer = std::make_shared<Assimp::Importer>();

      aiScene const* scene(importer->ReadFileFromMemory(
          buffer_name,
          buffer_size,
          aiProcessPreset_TargetRealtime_Quality | aiProcess_CalcTangentSpace));

      std::vector<DynamicGeometryResource*> meshes;

      for (unsigned int n = 0; n < scene->mNumMeshes; ++n) {
        meshes.push_back(
            new DynamicGeometryResource(Mesh{*scene->mMeshes[n]}, build_kd_tree));
      }

      return meshes;
    */
    return std::vector<DynamicGeometryResource *>();
}

////////////////////////////////////////////////////////////////////////////////

int DynamicGeometryLoader::is_supported(std::string const &file_name) const
{
    auto point_pos(file_name.find_last_of("."));
    // Assimp::Importer importer;

    if(file_name.substr(point_pos + 1) == "lob")
    {
        return 1;
    }

    if(file_name.substr(point_pos + 1) == "pob")
    {
        return 2;
    }

    return 0; // importer.IsExtensionSupported(file_name.substr(point_pos + 1));
}

////////////////////////////////////////////////////////////////////////////////

void DynamicGeometryLoader::apply_fallback_material(std::shared_ptr<node::Node> const &root, std::shared_ptr<Material> const &fallback_material, bool no_shared_materials)
{
    auto g_node(std::dynamic_pointer_cast<node::DynamicGeometryNode>(root));

    if(g_node && !g_node->get_material())
    {
        g_node->set_material(fallback_material);
        g_node->update_cache();
    }
    else if(g_node && no_shared_materials)
    {
        g_node->set_material(std::make_shared<Material>(*g_node->get_material()));
    }

    for(auto &child : root->get_children())
    {
        apply_fallback_material(child, fallback_material, no_shared_materials);
    }
}

} // namespace gua
