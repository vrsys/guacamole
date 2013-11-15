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
#include <gua/renderer/Volume.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/utils/logger.hpp>

// external headers
#if ASSIMP_VERSION == 3
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#else
#include <assimp/assimp.hpp>
#include <assimp/aiPostProcess.h>
#include <assimp/aiScene.h>
#endif

#include <scm/gl_util/data/volume/volume_loader.h>
#include <scm/gl_util/data/imaging/texture_loader.h>
#include <scm/gl_util/primitives/box_volume.h>

namespace {
	struct Vertex {
		scm::math::vec3f pos;
		scm::math::vec2f tex;
		scm::math::vec3f normal;
		scm::math::vec3f tangent;
		scm::math::vec3f bitangent;
	};
}

namespace gua {

	////////////////////////////////////////////////////////////////////////////////

	Volume::Volume()
		: _volume_boxes_ptr(), upload_mutex_() {}

	////////////////////////////////////////////////////////////////////////////////

	Volume::Volume(std::string const& file_name)
		: 
		_volume_file_path(file_name),
		_volume_boxes_ptr(),
		upload_mutex_()
	{
		scm::gl::volume_loader scm_volume_loader;
		//scm::math::vec3ui volume_dimensions = scm_volume_loader.read_dimensions(file_name);
		scm::math::vec3ui volume_dimensions = scm::math::vec3ui(256, 256, 225);

		unsigned max_dimension_volume = scm::math::max(scm::math::max(volume_dimensions.x, volume_dimensions.y), volume_dimensions.z);

		gua::math::vec3 scaled_max_vertex_pos((float)volume_dimensions.x / (float)max_dimension_volume,
			(float)volume_dimensions.y / (float)max_dimension_volume,
			(float)volume_dimensions.z / (float)max_dimension_volume);

		MESSAGE("%f %f %f", scaled_max_vertex_pos.x, scaled_max_vertex_pos.y, scaled_max_vertex_pos.z);
		//getchar();

		bounding_box_ = math::BoundingBox<math::vec3>(math::vec3::zero(), scaled_max_vertex_pos);
		
	}

	////////////////////////////////////////////////////////////////////////////////

	void Volume::upload_to(RenderContext const& ctx) const {

		//if (!_volume_texture_ptr[ctx.id]) {
		//	WARNING("Unable to load Volume! Has no volume data.");
		//	return;
		//}

		std::unique_lock<std::mutex> lock(upload_mutex_);

		if (_volume_boxes_ptr.size() <= ctx.id){
			_volume_texture_ptr.resize(ctx.id + 1);
			_transfer_texture_ptr.resize(ctx.id + 1);
			_volume_boxes_ptr.resize(ctx.id + 1);
			_sstate.resize(ctx.id + 1);
		}

		scm::gl::volume_loader scm_volume_loader;
		_volume_texture_ptr[ctx.id] = scm_volume_loader.load_volume_data(*(ctx.render_device.get()), _volume_file_path);

		MESSAGE("%s loaded!", _volume_file_path.c_str());

		//scm::gl::texture_loader scm_image_loader; 
		_transfer_texture_ptr[ctx.id] = ctx.render_device->create_texture_2d(scm::gl::texture_2d_desc(scm::math::vec2ui(255, 1),
																			 scm::gl::data_format::FORMAT_RGBA_32F));
			

		//box_volume_geometry
		_volume_boxes_ptr[ctx.id] =
			scm::gl::box_volume_geometry_ptr(new scm::gl::box_volume_geometry(ctx.render_device, scm::math::vec3(0.0), scm::math::vec3(1.0)));
		
		_sstate[ctx.id] = ctx.render_device->create_sampler_state(
			scm::gl::FILTER_MIN_MAG_MIP_LINEAR, scm::gl::WRAP_CLAMP_TO_EDGE);
	}


	scm::gl::texture_2d_ptr Volume::create_color_map(RenderContext const& ctx,
														unsigned in_size,
														const scm::data::piecewise_function_1d<float, float>& in_alpha,
														const scm::data::piecewise_function_1d<float, scm::math::vec3f>& in_color) const
	{
			using namespace scm::gl;
			using namespace scm::math;

			scm::scoped_array<scm::math::vec3f>  color_lut;
			scm::scoped_array<float>             alpha_lut;

			color_lut.reset(new vec3f[in_size]);
			alpha_lut.reset(new float[in_size]);

			if (!scm::data::build_lookup_table(color_lut, in_color, in_size)
				|| !scm::data::build_lookup_table(alpha_lut, in_alpha, in_size)) {
				std::cout << "demo_app::create_color_map(): error during lookuptable generation" << std::endl;
				return (texture_2d_ptr());
			}
			scm::scoped_array<float> combined_lut;

			combined_lut.reset(new float[in_size * 4]);

			for (unsigned i = 0; i < in_size; ++i) {
				combined_lut[i * 4] = color_lut[i].x;
				combined_lut[i * 4 + 1] = color_lut[i].y;
				combined_lut[i * 4 + 2] = color_lut[i].z;
				combined_lut[i * 4 + 3] = alpha_lut[i];
			}

			std::vector<void*> in_data;
			in_data.push_back(combined_lut.get());

			texture_2d_ptr new_tex = ctx.render_device->create_texture_2d(scm::math::vec2ui(in_size, 1), FORMAT_RGBA_8, 1, 1, 1, FORMAT_RGBA_32F, in_data);

			if (!new_tex) {
				std::cout << "demo_app::create_color_map(): error during color map texture generation." << std::endl;
				return (texture_2d_ptr());
			}

			return (new_tex);
		}

	bool Volume::update_color_map(RenderContext const& ctx,
		scm::gl::texture_2d_ptr transfer_texture_ptr,
		unsigned in_size,
		const scm::data::piecewise_function_1d<float, float>& in_alpha,
		const scm::data::piecewise_function_1d<float, scm::math::vec3f>& in_color) const
	{
		using namespace scm::gl;
		using namespace scm::math;

		scm::scoped_array<scm::math::vec3f>  color_lut;
		scm::scoped_array<float>             alpha_lut;

		color_lut.reset(new vec3f[in_size]);
		alpha_lut.reset(new float[in_size]);

		if (!scm::data::build_lookup_table(color_lut, in_color, in_size)
			|| !scm::data::build_lookup_table(alpha_lut, in_alpha, in_size)) {
			std::cout << "demo_app::update_color_map(): error during lookuptable generation" << std::endl;
			return false;// (transfer_texture_ptr);
		}
		scm::scoped_array<float> combined_lut;

		combined_lut.reset(new float[in_size * 4]);

		for (unsigned i = 0; i < in_size; ++i) {
			combined_lut[i * 4] = color_lut[i].x;
			combined_lut[i * 4 + 1] = color_lut[i].y;
			combined_lut[i * 4 + 2] = color_lut[i].z;
			combined_lut[i * 4 + 3] = alpha_lut[i];
		}

		std::vector<void*> in_data;
		in_data.push_back(combined_lut.get());

		//texture_2d_ptr new_tex = ctx.render_device->create_texture_2d(scm::math::vec2ui(in_size, 1), FORMAT_RGBA_8, 1, 1, 1, FORMAT_RGBA_32F, in_data);
		

		if (!ctx.render_context->update_sub_texture(transfer_texture_ptr,
													scm::gl::texture_region(scm::math::vec3ui(0, 0, 0),
													scm::math::vec3ui(in_size, 1, 0)),
													1u,
													FORMAT_RGBA_8,
													in_data)) 
		{
			std::cout << "demo_app::updatecolor_map(): error during color map texture generation." << std::endl;
			return false;
		}

		return true;
	}

	////////////////////////////////////////////////////////////////////////////////

	void Volume::draw(RenderContext const& ctx) const {

		// upload to GPU if neccessary
		if (_volume_boxes_ptr.size() <= ctx.id || _volume_boxes_ptr[ctx.id] == nullptr) {
			upload_to(ctx);
		}

		scm::gl::context_vertex_input_guard vig(ctx.render_context);

		ctx.render_context->bind_texture( _volume_texture_ptr[ctx.id], _sstate[ctx.id], 5);
		ctx.render_context->bind_texture(_transfer_texture_ptr[ctx.id], _sstate[ctx.id], 6);
		scm::gl::program_ptr p = ctx.render_context->current_program();
		p->uniform("transfer_texture", 6);
		p->uniform_sampler("volume_texture", 5);		
		p->uniform("sampling_distance", 1.f/255.f);
		p->uniform("iso_value", 0.8f);
		
		ctx.render_context->apply();
		_volume_boxes_ptr[ctx.id]->draw(ctx.render_context);
		//ctx.render_context->draw_elements(mesh_->mNumFaces * 3);
	}

	////////////////////////////////////////////////////////////////////////////////

	void Volume::ray_test(Ray const& ray, PickResult::Options options,
		Node* owner, std::set<PickResult>& hits) {

		//kd_tree_.ray_test(ray, mesh_, options, owner, hits);
	}

	////////////////////////////////////////////////////////////////////////////////

}
