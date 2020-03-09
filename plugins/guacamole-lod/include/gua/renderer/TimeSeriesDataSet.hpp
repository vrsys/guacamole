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

#ifndef GUA_TIME_SERIES_DATA_HPP
#define GUA_TIME_SERIES_DATA_HPP

// guacamole_headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/MaterialShaderMethod.hpp>
#include <gua/math/BoundingBox.hpp>
#include <gua/scenegraph/PickResult.hpp>

// external headers
#include <string>
#include <vector>
#include <scm/gl_util/primitives/box.h>

namespace gua
{
class RessourceRenderer;


namespace node
{
class GeometryNode;
};


struct GUA_DLL TimeSeriesDataSet {
    std::string name = "";
    uint32_t num_attributes = 0;
    uint32_t num_timesteps = 0;
    float sequence_length = 0;

    float time_cursor_position = 0.0f;

    std::vector<std::pair<float, float> > extreme_values;    
    std::vector<float> data; //for now we only allow float attributes

    scm::math::mat4f time_series_transform_matrix;

    std::string simulation_positions_filename = "";
    uint32_t num_simulation_positions_per_timestep = 0;
    std::vector<std::vector<scm::math::vec3f>> simulation_positions;

    std::size_t uuid = boost::hash<boost::uuids::uuid>()(boost::uuids::random_generator()());

    void upload_time_range_to(RenderContext& ctx, bool deformation_enabled, bool coloring_enabled, int vis_attribut_id, int start_time_step_id = -1, int end_time_step_id = -1) const;

    void bind_to(RenderContext& ctx, int buffer_binding_point, std::shared_ptr<ShaderProgram>& shader_program, int attribute_to_render, float mix_in_factor);

    float calculate_active_cursor_position(float in_node_time_cursor) const;
};

/**
 * 
 *
class GUA_DLL TimeSeriesDataSetCollectionResource
{
  public:
    std::vector<TimeSeriesItem> data_sets;
};
*/

} // namespace gua

#endif // GUA_TIME_SERIES_DATA_HPP
