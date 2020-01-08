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
#include <gua/renderer/TimeSeriesGPUResource.hpp>

//#include <memory>

namespace gua
{

void TimeSeriesDataSet::upload_time_range_to(RenderContext& ctx, bool deformation_enabled, bool coloring_enabled, 
											 int vis_attribut_id, int start_time_step_id, int end_time_step_id) const {

	if((!deformation_enabled) && (!coloring_enabled) ) {
		return;
	}

	std::size_t num_bytes_per_timestep = data.size() * sizeof(float) / (num_attributes * num_timesteps);

	auto time_series_data_set_ssbo_iterator = ctx.plugin_resources.find(uuid);
	if(time_series_data_set_ssbo_iterator == ctx.plugin_resources.end()) {


		auto new_time_series_gpu_resource = std::make_shared<TimeSeriesGPUResource>();
		
		int num_simultaneously_uploaded_timesteps = 2;
		int num_attributes_per_timestep = 4;

		new_time_series_gpu_resource->ssbo 
			= ctx.render_device->create_buffer(scm::gl::BIND_UNIFORM_BUFFER, 
											   scm::gl::USAGE_DYNAMIC_DRAW, 
											   num_bytes_per_timestep * num_simultaneously_uploaded_timesteps * num_attributes_per_timestep, 
											   0);

		ctx.plugin_resources[uuid] = new_time_series_gpu_resource;
	} else {
    	auto time_series_data_gpu_resource = std::dynamic_pointer_cast<TimeSeriesGPUResource>(ctx.plugin_resources.find(uuid)->second);


    	vis_attribut_id = std::max(int(0), std::min(int(num_attributes-1), int(vis_attribut_id) ) );

    	if( (start_time_step_id != time_series_data_gpu_resource->currently_uploaded_time_step_slot_0) ||
    		(end_time_step_id != time_series_data_gpu_resource->currently_uploaded_time_step_slot_1) ||

    		deformation_enabled != time_series_data_gpu_resource->currently_enabled_deformation || 
    		coloring_enabled != time_series_data_gpu_resource->currently_enabled_coloring ||
    		vis_attribut_id != time_series_data_gpu_resource->currently_attribute_to_visualize 
    	) {
    		size_t x_pos_offset_0 = num_bytes_per_timestep * start_time_step_id;
    		size_t x_pos_offset_1 = num_bytes_per_timestep * end_time_step_id;

       		size_t y_pos_offset_0 = num_bytes_per_timestep * 1 * num_timesteps + num_bytes_per_timestep * start_time_step_id;
    		size_t y_pos_offset_1 = num_bytes_per_timestep * 1 * num_timesteps + num_bytes_per_timestep * end_time_step_id;

         	size_t z_pos_offset_0 = num_bytes_per_timestep * 2 * num_timesteps + num_bytes_per_timestep * start_time_step_id;
    		size_t z_pos_offset_1 = num_bytes_per_timestep * 2 * num_timesteps + num_bytes_per_timestep * end_time_step_id;

         	size_t vis_attribute_offset_0 = num_bytes_per_timestep * vis_attribut_id * num_timesteps + num_bytes_per_timestep * start_time_step_id;
    		size_t vis_attribute_offset_1 = num_bytes_per_timestep * vis_attribut_id * num_timesteps + num_bytes_per_timestep * end_time_step_id;

    		float* mapped_time_series_ssbo 
    			= static_cast<float*>(ctx.render_context->map_buffer(time_series_data_gpu_resource->ssbo, scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER));

    		size_t write_offset = 0;
    		memcpy((char*)mapped_time_series_ssbo + write_offset, (char*) data.data() + x_pos_offset_0, num_bytes_per_timestep);
    		write_offset += num_bytes_per_timestep;
      		memcpy((char*)mapped_time_series_ssbo + write_offset, (char*) data.data() + x_pos_offset_1, num_bytes_per_timestep);
    		write_offset += num_bytes_per_timestep;
    		memcpy((char*)mapped_time_series_ssbo + write_offset, (char*) data.data() + y_pos_offset_0, num_bytes_per_timestep);
    		write_offset += num_bytes_per_timestep;
      		memcpy((char*)mapped_time_series_ssbo + write_offset, (char*) data.data() + y_pos_offset_1, num_bytes_per_timestep);
    		write_offset += num_bytes_per_timestep;
    		memcpy((char*)mapped_time_series_ssbo + write_offset, (char*) data.data() + z_pos_offset_0, num_bytes_per_timestep);
    		write_offset += num_bytes_per_timestep;
      		memcpy((char*)mapped_time_series_ssbo + write_offset, (char*) data.data() + z_pos_offset_1, num_bytes_per_timestep);
    		write_offset += num_bytes_per_timestep;
    		memcpy((char*)mapped_time_series_ssbo + write_offset, (char*) data.data() + vis_attribute_offset_0, num_bytes_per_timestep);
    		write_offset += num_bytes_per_timestep;
      		memcpy((char*)mapped_time_series_ssbo + write_offset, (char*) data.data() + vis_attribute_offset_1, num_bytes_per_timestep);
    		write_offset += num_bytes_per_timestep;
    		// ...

    		ctx.render_context->unmap_buffer(time_series_data_gpu_resource->ssbo);

    		time_series_data_gpu_resource->currently_uploaded_time_step_slot_0 = start_time_step_id;
    		time_series_data_gpu_resource->currently_uploaded_time_step_slot_1 = end_time_step_id;

    		time_series_data_gpu_resource->currently_enabled_deformation = deformation_enabled; 
    		time_series_data_gpu_resource->currently_enabled_coloring = coloring_enabled;
    		time_series_data_gpu_resource->currently_attribute_to_visualize = vis_attribut_id;
    	}
    	

	}



	//RenderContext::Mesh cmesh{};

	//    mutable std::unordered_map<std::size_t, scm::gl::buffer_ptr> shader_storage_buffer_objects;
}

void TimeSeriesDataSet::bind_to(RenderContext& ctx, int buffer_binding_point, std::shared_ptr<ShaderProgram>& shader_program, int attribute_to_visualize = 0) {
    int32_t floats_per_timestep    = data.size() / (num_attributes * num_timesteps);
    int32_t attribute_element_offset = data.size() / num_attributes;

    attribute_to_visualize = std::max(int(0), std::min(int(num_attributes-1), int(attribute_to_visualize) ) );

    shader_program->set_uniform(ctx, attribute_to_visualize, "attribute_to_visualize");
    shader_program->set_uniform(ctx, floats_per_timestep, "floats_per_attribute_timestep");
    shader_program->set_uniform(ctx, attribute_element_offset, "attribute_offset");                
    shader_program->set_uniform(ctx, extreme_values[attribute_to_visualize].first, "min_ssbo_value");        
    shader_program->set_uniform(ctx, extreme_values[attribute_to_visualize].second, "max_ssbo_value"); 
    shader_program->set_uniform(ctx, int(buffer_binding_point), "time_series_data_ssbo");

    auto current_ssbo_ptr_it = ctx.plugin_resources.find(uuid);

    if(ctx.plugin_resources.end() == current_ssbo_ptr_it) {
        exit(-1);
    }


    auto time_series_data_gpu_resource = std::dynamic_pointer_cast<TimeSeriesGPUResource>(ctx.plugin_resources.find(uuid)->second);

    ctx.render_context->bind_storage_buffer(time_series_data_gpu_resource->ssbo, buffer_binding_point, 0, data.size() * sizeof(float) );// looked_up_time_series_data_item->data.size());
    //ctx.render_context->set_storage_buffers( std::vector<scm::gl::render_context::buffer_binding>{scm::gl::BIND_STORAGE_BUFFER} );
    ctx.render_context->apply_storage_buffer_bindings();
}

float TimeSeriesDataSet::calculate_active_cursor_position(float in_node_time_cursor) const {
    float current_timecursor_position = in_node_time_cursor;

    if( (num_timesteps != 1) && (sequence_length != 0.0f) ) {
        if(current_timecursor_position >  sequence_length) {
            current_timecursor_position = std::fmod(current_timecursor_position, sequence_length);
        }
        current_timecursor_position /= (sequence_length/num_timesteps );
    } else {
        current_timecursor_position = 0.0f;
    }

    return current_timecursor_position;
}

}