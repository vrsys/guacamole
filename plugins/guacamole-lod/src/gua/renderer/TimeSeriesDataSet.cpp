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

#include <gua/renderer/ShaderProgram.hpp>
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

	/*
	for(int i = 0; i < data.size(); ++i) {
		if(data[i] != 0.0) {
			std::cout << data[i] << "\t\ti: " << i << std::endl;
		}
	}*/



	//std::cout << "NUM BYTES PER TIMESTEP: " << num_bytes_per_timestep << std::endl;

	//std::cout << "Num timesteps to upload: " << num_timesteps_to_upload << std::endl;

	std::size_t read_offset_in_timesteps = num_bytes_per_timestep * start_time_step_id / sizeof(float); 

	size_t num_bytes_to_upload = num_timesteps_to_upload * num_bytes_per_timestep;


	if(time_series_data_set_ssbo_iterator == ctx.shader_storage_buffer_objects.end()) {

		//std::cout << "CREATING SSBO FOR BOUND BUFFER" << std::endl;

		//std::cout << "GOING TO READ " << num_bytes_per_timestep * num_timesteps_to_upload << " BYTES" << std::endl;

        //std::vector<float> dummy_data(600000, 5.2);

		//ctx.shader_storage_buffer_objects[uuid] = ctx.render_device->create_buffer(scm::gl::BIND_STORAGE_BUFFER, scm::gl::USAGE_DYNAMIC_DRAW, sizeof(float) * 600000, dummy_data.data());
		//create new ssbo

		//std::cout << "Num bytes to upload xxx: " << num_bytes_to_upload << std::endl;

		ctx.shader_storage_buffer_objects[uuid] = ctx.render_device->create_buffer(scm::gl::BIND_STORAGE_BUFFER, scm::gl::USAGE_STATIC_DRAW, data.size() * sizeof(float), data.data());
		//create new ssbo

		//std::cout << "Created new SSBO!" << std::endl;

	}



	//RenderContext::Mesh cmesh{};

	//    mutable std::unordered_map<std::size_t, scm::gl::buffer_ptr> shader_storage_buffer_objects;
}

void TimeSeriesDataSet::bind_to(RenderContext& ctx, int buffer_binding_point, std::shared_ptr<ShaderProgram>& shader_program, int attribute_to_visualize = 0) {
    int32_t floats_per_timestep    = data.size() / (num_attributes * num_timesteps);
    int32_t attribute_element_offset = data.size() / num_attributes;

    shader_program->set_uniform(ctx, attribute_to_visualize, "attribute_to_visualize");
    shader_program->set_uniform(ctx, floats_per_timestep, "floats_per_attribute_timestep");
    shader_program->set_uniform(ctx, attribute_element_offset, "attribute_offset");                
    shader_program->set_uniform(ctx, extreme_values[attribute_to_visualize].first, "min_ssbo_value");        
    shader_program->set_uniform(ctx, extreme_values[attribute_to_visualize].second, "max_ssbo_value"); 
    shader_program->set_uniform(ctx, int(buffer_binding_point), "time_series_data_ssbo");

    auto current_ssbo_ptr_it = ctx.shader_storage_buffer_objects.find(uuid);

    if(ctx.shader_storage_buffer_objects.end() == current_ssbo_ptr_it) {
        exit(-1);
    }

    ctx.render_context->bind_storage_buffer( current_ssbo_ptr_it->second, buffer_binding_point, 0, data.size() * sizeof(float) );// looked_up_time_series_data_item->data.size());
    //ctx.render_context->set_storage_buffers( std::vector<scm::gl::render_context::buffer_binding>{scm::gl::BIND_STORAGE_BUFFER} );
    ctx.render_context->apply_storage_buffer_bindings();
}

float TimeSeriesDataSet::calculate_active_cursor_position(float in_node_time_cursor) const {
    //int32_t attribute_to_visualize_index = plod_node->get_attribute_to_visualize_index();
    //looked_up_time_series_data_item->bind_to(ctx, 20, shader_program_, attribute_to_visualize_index);

    //int32_t current_timestep_offset = int(ctx.framecount % 100);

	std::cout << "In node time cursor: " << in_node_time_cursor << std::endl;

    float current_timecursor_position = in_node_time_cursor;

    if( (num_timesteps != 1) && (sequence_length != 0.0f) ) {
        if(current_timecursor_position >  sequence_length) {
            current_timecursor_position = std::fmod(current_timecursor_position, sequence_length);
        }

        std::cout << "Current timecursor position: " << current_timecursor_position << std::endl;
        std::cout << "Sequence Length: " << sequence_length << std::endl;
        std::cout << "Num Timesteps: " << num_timesteps << std::endl;        

        current_timecursor_position /= (sequence_length/num_timesteps );
    } else {
        current_timecursor_position = 0.0f;
    }

    return current_timecursor_position;
    //std::cout << "GOING TO UPLOAD TIMECURSOR POSITION: " << current_timecursor_position << std::endl;

}

}