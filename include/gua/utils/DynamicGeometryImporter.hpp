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

#ifndef GUA_DYNAMIC_GEOMETRY_IMPORTER_HPP
#define GUA_DYNAMIC_GEOMETRY_IMPORTER_HPP

// guacamole headers
#include <gua/config.hpp>
#include <gua/platform.hpp>

// external headers
#include <scm/gl_core.h>
#include <scm/core/math/quat.h>

#include <vector>

namespace gua
{
class TransformNode;

using IndexTriplet = std::tuple<int, int, int>;

struct DynamicGeometryObject
{
    DynamicGeometryObject(unsigned int max_geometry_vertices = 10000)
    {
        vertex_position_database.reserve(max_geometry_vertices);
        vertex_color_database.reserve(max_geometry_vertices);
        vertex_thickness_database.reserve(max_geometry_vertices);
        // vertex_normal_database.reserve(max_geometry_vertices);
        vertex_uv_database.reserve(max_geometry_vertices);
    }

    std::vector<scm::math::vec3f> vertex_position_database;
    std::vector<scm::math::vec4f> vertex_color_database;
    std::vector<float> vertex_thickness_database;
    // std::vector<scm::math::vec3f> vertex_normal_database;
    std::vector<scm::math::vec2f> vertex_uv_database;

    // not used in the first version of the importer
    std::vector<IndexTriplet> vertex_attribute_ids;
};

// possible to do ableiten

/**
 * @brief holds vertex information of one dynamic geometry
 */
class GUA_DLL DynamicGeometryImporter
{
    // todo change name to creator instead of importer

    friend class DynamicGeometry;

  public:
    void create_empty_dynamic_geometry(std::string const& empty_dynamic_geometry_name);

    bool parsing_successful() const;

    int num_parsed_dynamic_geometries() const;

    std::shared_ptr<DynamicGeometryObject> get_dynamic_geometry_object_ptr() const;

  private:
    bool parsing_successful_ = false;

    int num_parsed_dynamic_geometries_ = 0;

    std::shared_ptr<DynamicGeometryObject> dynamic_geometry_object_ptr_;
};

} // namespace gua

#endif // GUA_DYNAMIC_GEOMETRY_IMPORTER_HPP
