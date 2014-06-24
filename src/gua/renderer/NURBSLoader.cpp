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

#include <gua/node/NURBSNode.hpp>
#include <gua/databases/GeometryDatabase.hpp>

#include <gua/renderer/NURBSRessource.hpp>
#include <gua/renderer/nurbs_geometry/import/igs/igs_loader.hpp>
#include <gua/renderer/nurbs_geometry/TrimmedSurfaceConverter.hpp>
#include <gua/renderer/nurbs_geometry/TrimmedNurbsSurfaceObject.hpp>
#include <gua/renderer/nurbs_geometry/TrimmedBezierSurfaceObject.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////
NURBSLoader::NURBSLoader() : GeometryLoader(), _supported_file_extensions() {
  _supported_file_extensions.insert("igs");
  _supported_file_extensions.insert("iges");
  _supported_file_extensions.insert("IGS");
  _supported_file_extensions.insert("IGES");
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<node::Node> NURBSLoader::create_geometry_from_file(std::string const& nodename, 
                                                             std::string const& filename,
                                                             std::string const& material,
                                                             unsigned flags)
{
  try {
    if (!is_supported(filename))
    {
      throw std::runtime_error(std::string("Unsupported filetype: ") + filename);
    }
    else {

      igs_loader igsloader;
      TrimmedSurfaceConverter surface_converter;

      auto nurbs_object = std::make_shared<TrimmedNurbsSurfaceObject>();
      auto bezier_object = std::make_shared<TrimmedBezierSurfaceObject>();

      igsloader.load(filename, nurbs_object);
      surface_converter.convert(nurbs_object, bezier_object);

      auto ressource = std::make_shared<NURBSRessource>(bezier_object);

      std::string mesh_name("type=file&file=" + filename + "&flags=" + string_utils::to_string(flags));
      GeometryDatabase::instance()->add(mesh_name, ressource);

      auto node = std::make_shared<node::NURBSNode>(nodename, mesh_name, material);
      node->update_cache();

      return node;
    }
  } catch (std::exception & e) {
    Logger::LOG_WARNING << "Failed to load NURBS object \"" << filename << "\": " << e.what() << std::endl;
    return nullptr;
  }
}


////////////////////////////////////////////////////////////////////////////////
/* virtual */
bool NURBSLoader::is_supported(std::string const& file_name) const 
{
  std::vector<std::string> filename_decomposition = gua::string_utils::split(file_name, '.');
  return filename_decomposition.empty()
             ? false
             : _supported_file_extensions.count(filename_decomposition.back()) >
                   0;
}

}  // namespace gua
