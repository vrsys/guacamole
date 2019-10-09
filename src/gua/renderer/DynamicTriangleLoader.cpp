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
#include <gua/renderer/DynamicTriangleLoader.hpp>

// guacamole headers
#include <gua/utils/TextFile.hpp>
// #include <gua/utils/DynamicGeometryImporter.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/utils/string_utils.hpp>
#include <gua/utils/ToGua.hpp>
#include <gua/node/DynamicTriangleNode.hpp>
#include <gua/node/TransformNode.hpp>
#include <gua/renderer/MaterialLoader.hpp>
#include <gua/renderer/DynamicGeometryResource.hpp>
#include <gua/renderer/DynamicTriangleResource.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>
#include <gua/databases/GeometryDatabase.hpp>

namespace gua
{
/////////////////////////////////////////////////////////////////////////////
// static variables
/////////////////////////////////////////////////////////////////////////////
std::unordered_map<std::string, std::shared_ptr<::gua::node::Node>> DynamicTriangleLoader::loaded_files_ = std::unordered_map<std::string, std::shared_ptr<::gua::node::Node>>();

/////////////////////////////////////////////////////////////////////////////

DynamicTriangleLoader::DynamicTriangleLoader() {}

////////////////////////////////////////////////////////////////////////////////

void DynamicTriangleLoader::apply_fallback_material(std::shared_ptr<node::Node> const &root, std::shared_ptr<Material> const &fallback_material, bool no_shared_materials)
{
    auto g_node(std::dynamic_pointer_cast<node::DynamicTriangleNode>(root));

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

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<node::DynamicGeometryNode> DynamicTriangleLoader::create_geometry_instance(std::shared_ptr<DynamicGeometryImporter> importer, GeometryDescription const &desc, unsigned flags)
{
    GeometryDatabase::instance()->add(
        desc.unique_key(),
        // std::make_shared<DynamicTriangleResource>(
        //     DynamicTriangle {*importer->get_dynamic_geometry_object_ptr()},
        //     flags & DynamicTriangleLoader::MAKE_PICKABLE)
        std::make_shared<DynamicTriangleResource>(std::make_shared<DynamicTriangle>(DynamicTriangle(*importer->get_dynamic_geometry_object_ptr())), flags & DynamicTriangleLoader::MAKE_PICKABLE));
    //     DynamicTriangle {*importer->get_dynamic_geometry_object_ptr()},
    //     flags & DynamicTriangleLoader::MAKE_PICKABLE)

    std::shared_ptr<node::DynamicGeometryNode> node_to_return = std::make_shared<node::DynamicTriangleNode>(node::DynamicTriangleNode("", desc.unique_key()));

    node_to_return->set_empty();

    // std::cout<<"LOAD TRIANGLE"<< node_to_return<<" "<<node_to_return->get_geometry_description()<<std::endl;

    return node_to_return;
}
} // namespace gua
