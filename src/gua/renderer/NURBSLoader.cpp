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

#include <gua/scenegraph/GeometryNode.hpp>
#include <gua/databases/GeometryDatabase.hpp>

#include <gua/renderer/NURBS.hpp>
#include <gua/renderer/nurbs_geometry/import/igs/igs_loader.hpp>
#include <gua/renderer/nurbs_geometry/TrimmedSurfaceConverter.hpp>
#include <gua/renderer/nurbs_geometry/TrimmedNurbsSurfaceObject.hpp>
#include <gua/renderer/nurbs_geometry/TrimmedBezierSurfaceObject.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////
NURBSLoader::NURBSLoader() : LoaderBase(), _supported_file_extensions() {
  _supported_file_extensions.insert("igs");
  _supported_file_extensions.insert("iges");
  _supported_file_extensions.insert("IGS");
  _supported_file_extensions.insert("IGES");
}

////////////////////////////////////////////////////////////////////////////////
/* virtual */
std::shared_ptr<Node> NURBSLoader::load(std::string const& file_name,
                                        unsigned flags) {
  try {
    igs_loader igsloader;
    TrimmedSurfaceConverter surface_converter;

    std::shared_ptr<TrimmedNurbsSurfaceObject> nurbs_object(
        new TrimmedNurbsSurfaceObject);
    std::shared_ptr<TrimmedBezierSurfaceObject> bezier_object(
        new TrimmedBezierSurfaceObject);

    igsloader.load(file_name, nurbs_object);
    surface_converter.convert(nurbs_object, bezier_object);

    GeometryDatabase::instance()->add(
        file_name, std::make_shared<NURBS>(bezier_object));

    auto result = std::make_shared<GeometryNode>("unnamed_nurbs");
    result->set_geometry(file_name);
    result->set_material("");

    return result;

  } catch (std::exception & e) {
    Logger::LOG_WARNING << "Failed to load NURBS object \"" << file_name << "\": " << e.what() << std::endl;
    return nullptr;
  }
}

////////////////////////////////////////////////////////////////////////////////
/* virtual */
bool NURBSLoader::is_supported(std::string const& file_name) const {
  std::vector<std::string> filename_decomposition =
      gua::string_utils::split(file_name, '.');
  return filename_decomposition.empty()
             ? false
             : _supported_file_extensions.count(filename_decomposition.back()) >
                   0;
}

}  // namespace gua
