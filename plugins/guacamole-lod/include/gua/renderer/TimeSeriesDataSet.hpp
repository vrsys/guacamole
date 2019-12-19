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

#ifndef GUA_TIME_SERIES_DATA_RESOURCE_HPP
#define GUA_TIME_SERIES_DATA_RESOURCE_HPP

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

    std::vector<std::pair<float, float> > extreme_values;    
    std::vector<float> data; //for now we only allow float attributes

    std::size_t uuid = boost::hash<boost::uuids::uuid>()(boost::uuids::random_generator()());

    void upload_time_range_to(RenderContext& ctx, int start_time_step_id = -1, int end_time_step_id = -1) const;

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

#endif // GUA_TIME_SERIES_DATA_RESOURCE_HPP