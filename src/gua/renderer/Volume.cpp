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
		scm::math::vec3ui volume_dimensions = scm_volume_loader.read_dimensions(file_name);
		//scm::math::vec3ui volume_dimensions = scm::math::vec3ui(256, 256, 225);

		unsigned max_dimension_volume = scm::math::max(scm::math::max(volume_dimensions.x, volume_dimensions.y), volume_dimensions.z);

		gua::math::vec3 scaled_max_vertex_pos((float)volume_dimensions.x / (float)max_dimension_volume,
			(float)volume_dimensions.y / (float)max_dimension_volume,
			(float)volume_dimensions.z / (float)max_dimension_volume);

		MESSAGE("%f %f %f", scaled_max_vertex_pos.x, scaled_max_vertex_pos.y, scaled_max_vertex_pos.z);
		//getchar();

		bounding_box_ = math::BoundingBox<math::vec3>(math::vec3::zero(), scaled_max_vertex_pos);

		// initialize transfer functions //////////////////////////////////////////////////////////////
		_alpha_transfer.clear();
		_color_transfer.clear();

#if 0
		_alpha_transfer.add_stop(0, 1.0f);
		_alpha_transfer.add_stop(0.33f, 0.5f);
		_alpha_transfer.add_stop(0.40f, 0.0f);
		_alpha_transfer.add_stop(0.60f, 0.0f);
		_alpha_transfer.add_stop(0.66f, 0.5f);
		_alpha_transfer.add_stop(1.0f, 1.0f);
#else
		_alpha_transfer.add_stop(0.0f, 0.0f);
		_alpha_transfer.add_stop(0.5f, 0.0f);
		_alpha_transfer.add_stop(1.0f, 1.0f);
#endif

#if 0
		// blue-grey-orange
		_color_transfer.add_stop(0,			scm::math::vec3f(0.0f, 1.0f, 1.0f));
		_color_transfer.add_stop(0.25f,		scm::math::vec3f(0.0f, 0.0f, 1.0f));
		_color_transfer.add_stop(0.375f,	scm::math::vec3f(0.256637f, 0.243243f, 0.245614f));
		_color_transfer.add_stop(0.50f,		scm::math::vec3f(0.765487f, 0.738739f, 0.72807f));
		_color_transfer.add_stop(0.625f,	scm::math::vec3f(0.530973f, 0.27027f, 0.0f));
		_color_transfer.add_stop(0.75f,		scm::math::vec3f(1.0f, 0.333333f, 0.0f));
		_color_transfer.add_stop(1.0f,		scm::math::vec3f(1.0f, 1.0f, 0.0f));
#else
		// blue-white-red
		_color_transfer.add_stop(0.0f, scm::math::vec3f(0.0f, 0.0f, 1.0f));
		_color_transfer.add_stop(0.5f, scm::math::vec3f(1.0f, 1.0f, 1.0f));
		_color_transfer.add_stop(1.0f, scm::math::vec3f(1.0f, 0.0f, 0.0f));
#endif
		
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
		_transfer_texture_ptr[ctx.id] = create_color_map(ctx, 255, _alpha_transfer, _color_transfer);
			

		//box_volume_geometry
		_volume_boxes_ptr[ctx.id] =
			scm::gl::box_volume_geometry_ptr(new scm::gl::box_volume_geometry(ctx.render_device, scm::math::vec3(0.0), scm::math::vec3(1.0)));
		
		_sstate[ctx.id] = ctx.render_device->create_sampler_state(
			scm::gl::FILTER_MIN_MAG_MIP_LINEAR, scm::gl::WRAP_CLAMP_TO_EDGE);
	}


	scm::gl::texture_2d_ptr 
	Volume::create_color_map(RenderContext const& ctx,
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

			//for (unsigned i = 0; i < in_size; ++i) {
			//	combined_lut[i * 4] = i;
			//	combined_lut[i * 4 + 1] = i;
			//	combined_lut[i * 4 + 2] = i;
			//	combined_lut[i * 4 + 3] = 1.0;
			//}

			std::vector<void*> in_data;
			in_data.push_back(combined_lut.get());

			texture_2d_ptr new_tex = ctx.render_device->create_texture_2d(scm::math::vec2ui(in_size, 1), FORMAT_RGBA_8, 1, 1, 1, FORMAT_RGBA_32F, in_data);

			if (!new_tex) {
				std::cout << "demo_app::create_color_map(): error during color map texture generation." << std::endl;
				return (texture_2d_ptr());
			}
			else{
				std::cout << "demo_app::create_color_map(): color map texture generated." << std::endl;
				return (new_tex);
			}			
		}

	bool Volume::update_color_map(RenderContext const& ctx,
		scm::gl::texture_2d_ptr transfer_texture_ptr,
		const scm::data::piecewise_function_1d<float, float>& in_alpha,
		const scm::data::piecewise_function_1d<float, scm::math::vec3f>& in_color) const
	{
		using namespace scm::gl;
		using namespace scm::math;

		scm::scoped_array<scm::math::vec3f>  color_lut;
		scm::scoped_array<float>             alpha_lut;

		unsigned in_size = transfer_texture_ptr->descriptor()._size.x;

		color_lut.reset(new vec3f[in_size]);
		alpha_lut.reset(new float[in_size]);

		if (!scm::data::build_lookup_table(color_lut, in_color, in_size)
			|| !scm::data::build_lookup_table(alpha_lut, in_alpha, in_size)) {
			MESSAGE("volume_data::update_color_alpha_map(): error during lookuptable generation");
			return false;
		}
		scm::scoped_array<float> combined_lut;

		combined_lut.reset(new float[in_size * 4]);

		for (unsigned i = 0; i < in_size; ++i) {
			combined_lut[i * 4] = color_lut[i].x;
			combined_lut[i * 4 + 1] = color_lut[i].y;
			combined_lut[i * 4 + 2] = color_lut[i].z;
			combined_lut[i * 4 + 3] = alpha_lut[i];
		}

		MESSAGE("generating color map texture data done.");

		MESSAGE("uploading texture data (\n size: %d KiB)...", static_cast<double>(in_size * size_of_format(FORMAT_RGBA_8)) / (1024.0));

		texture_region ur(vec3ui(0u), vec3ui(in_size, 1, 1));
		bool res = ctx.render_context->update_sub_texture(transfer_texture_ptr, ur, 0u, FORMAT_RGBA_32F, combined_lut.get());

		MESSAGE("uploading texture data done.");

		if (!res) {
			MESSAGE("volume_data::update_color_alpha_map(): error during color map texture generation.");
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
		p->uniform_sampler("volume_texture", 5);			
		p->uniform_sampler("transfer_texture", 6);
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
