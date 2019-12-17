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

#include <gua/renderer/TimeSeriesDataSet.hpp>

namespace gua
{

void TimeSeriesDataSet::upload_time_range_to(RenderContext& ctx, int start_time_step_id, int end_time_step_id) const {

	auto time_series_data_set_ssbo_iterator = ctx.shader_storage_buffer_objects.find(uuid);

	//inclusive ranges for uploading time steps
	int upload_time_step_id_start = start_time_step_id;
	int upload_time_step_id_end   = end_time_step_id;

	if(	   (start_time_step_id < 0)
	    || (start_time_step_id >  (num_timesteps - 1) )
		|| (start_time_step_id > end_time_step_id) 
		|| end_time_step_id > (num_timesteps - 1) ) {
		
		upload_time_step_id_start = 0;
		upload_time_step_id_end   = num_timesteps - 1;
	}


	int num_timesteps_to_upload = (upload_time_step_id_end - upload_time_step_id_start) + 1;

	std::size_t num_bytes_per_timestep = data.size() * sizeof(float) / (num_attributes * num_timesteps);

	std::cout << "NUM BYTES PER TIMESTEP: " << num_bytes_per_timestep << std::endl;

	std::cout << "Num timesteps to upload: " << num_timesteps_to_upload << std::endl;

	std::size_t read_offset_in_timesteps = num_bytes_per_timestep * start_time_step_id / sizeof(float); 


	if(time_series_data_set_ssbo_iterator == ctx.shader_storage_buffer_objects.end()) {

		std::cout << "GOING TO READ " << num_bytes_per_timestep * num_timesteps_to_upload << " BYTES" << std::endl;
		ctx.shader_storage_buffer_objects[uuid] = ctx.render_device->create_buffer(scm::gl::BIND_STORAGE_BUFFER, scm::gl::USAGE_STATIC_DRAW, num_bytes_per_timestep * num_timesteps_to_upload, &data[read_offset_in_timesteps]);
		//create new ssbo

		std::cout << "Created new SSBO!" << std::endl;

	}



	//RenderContext::Mesh cmesh{};

	//    mutable std::unordered_map<std::size_t, scm::gl::buffer_ptr> shader_storage_buffer_objects;
}


}