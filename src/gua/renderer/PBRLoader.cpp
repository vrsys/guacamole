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
#include <gua/renderer/PBRLoader.hpp>

// guacamole headers
#include <gua/utils/TextFile.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/utils/string_utils.hpp>
#include <gua/scenegraph/PBRNode.hpp>
#include <gua/scenegraph/TransformNode.hpp>
#include <gua/renderer/Material.hpp>
#include <gua/renderer/MaterialLoader.hpp>
#include <gua/renderer/GeometryLoader.hpp>
#include <gua/renderer/PBRRessource.hpp>
#include <gua/databases/MaterialDatabase.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/ShadingModelDatabase.hpp>

namespace gua {

unsigned PBRLoader::mesh_counter_ = 0;

  /////////////////////////////////////////////////////////////////////////////

PBRLoader::PBRLoader()
    : node_counter_(0) {}

  /////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Node> PBRLoader::load(std::string const& file_name,
                                      unsigned flags) {

  node_counter_ = 0;
  TextFile file(file_name);

  // MESSAGE("Loading mesh file %s", file_name.c_str());

  if (file.is_valid()) {
  
    //return new_node;
    return nullptr;

  }

  Logger::LOG_WARNING << "Failed to load object \"" << file_name << "\": File does not exist!" << std::endl;

  return nullptr;
}

  /////////////////////////////////////////////////////////////////////////////

std::vector<PBRRessource*> const PBRLoader::load_from_buffer(char const* buffer_name,
                                                             unsigned buffer_size,
                                                             bool build_kd_tree) {

  return std::vector<PBRRessource*>();
}

bool PBRLoader::is_supported(std::string const& file_name) const {
  auto point_pos(file_name.find_last_of("."));

  return file_name.substr(point_pos + 1) == "kdn";
}

}
