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
#include <gua/renderer/StreamingVoxelsLoader.hpp>

// guacamole headers
#include <gua/utils/TextFile.hpp>
#include <gua/utils/LineStripImporter.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/utils/string_utils.hpp>
#include <gua/utils/ToGua.hpp>
#include <gua/node/StreamingVoxelsNode.hpp>
#include <gua/node/TransformNode.hpp>
#include <gua/renderer/MaterialLoader.hpp>
#include <gua/renderer/StreamingVoxelsResource.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>
#include <gua/databases/GeometryDatabase.hpp>

namespace gua {

/////////////////////////////////////////////////////////////////////////////

std::shared_ptr<node::Node> StreamingVoxelsLoader::create_net_node_at_port(std::string const& node_name,
                                                                           uint16_t recv_socket_port,
                                                                           std::string const& feedback_ip, 
                                                                           uint16_t feedback_port ) {

  GeometryDescription desc("StreamingVoxels", node_name, 0, 0);
  GeometryDatabase::instance()->add(
    desc.unique_key(),
    std::make_shared<LineStripResource>(recv_socket_port, feedback_ip, feedback_port) );

  std::shared_ptr<node::StreamingVoxelsNode> node_to_return = 
      std::make_shared<node::StreamingVoxelsNode>(node::StreamingVoxelsNode("", desc.unique_key()) );

  auto shader(gua::MaterialShaderDatabase::instance()->lookup(
        "gua_default_material"));
  apply_fallback_material(
        node_to_return, shader->make_new_material(), 0);

  node_to_return->set_render_vertices_as_points(true);
  node_to_return->set_render_volumetric(true);
  
  return node_to_return;
}

}