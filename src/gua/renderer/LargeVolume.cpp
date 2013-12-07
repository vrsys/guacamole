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
#include <gua/renderer/LargeVolume.hpp>

#include <algorithm>
#include <cstddef>
#include <exception>
#include <iostream>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>

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

#include <scm/core/io/iomanip.h>
#include <scm/core/io/file.h>

#include <scm/gl_util/data/volume/volume_loader.h>
#include <scm/gl_util/data/imaging/texture_loader.h>
#include <scm/gl_util/primitives/box_volume.h>

#include <scm/large_data/virtual_texture/context_vtexture_guard.h>
#include <scm/large_data/virtual_texture/vtexture_2d_context.h>
#include <scm/large_data/virtual_texture/vtexture_3d.h>
#include <scm/large_data/virtual_texture/vtexture_3d_context.h>
#include <scm/large_data/virtual_texture/vtexture_system.h>

#include <boost/filesystem.hpp>
#include <boost/numeric/conversion/bounds.hpp>

namespace gua {

	LargeVolume::renderer_settings::renderer_settings()
		: //_render_method(VRM_RC_DVR_01)
		//, 
		_vtexture_fixed_lod_enabled(false)
		, _vtexture_fixed_lod(0.0)
		, _dvr_use_preintegration(false)
		, _dvr_use_adjacent_blend(true)
		, _dvr_use_animated_blend(false)
		, _dvr_use_gauss_fitting(true)
		, _dvr_use_texture_view(false)
		, _dvr_use_lod_adaptive_sampling(true)
		, _dvr_embedded_octree(false)
		, _dvr_show_iteration_count(false)
	{
	}

	bool
	LargeVolume::renderer_settings::operator==(const renderer_settings& rhs) const
	{
			return    //_render_method == rhs._render_method
				//&& 
				_vtexture_fixed_lod_enabled == rhs._vtexture_fixed_lod_enabled
				//&& _vtexture_fixed_lod         == rhs._vtexture_fixed_lod
				&& _dvr_use_preintegration == rhs._dvr_use_preintegration
				&& _dvr_use_adjacent_blend == rhs._dvr_use_adjacent_blend
				&& _dvr_use_animated_blend == rhs._dvr_use_animated_blend
				&& _dvr_use_gauss_fitting == rhs._dvr_use_gauss_fitting
				&& _dvr_use_texture_view == rhs._dvr_use_texture_view
				&& _dvr_use_lod_adaptive_sampling == rhs._dvr_use_lod_adaptive_sampling
				&& _dvr_embedded_octree == rhs._dvr_embedded_octree
				&& _dvr_show_iteration_count == rhs._dvr_show_iteration_count;
		}


	////////////////////////////////////////////////////////////////////////////////

	LargeVolume::LargeVolume()
		: _volume_boxes_ptr(), upload_mutex_() {}

	////////////////////////////////////////////////////////////////////////////////

	LargeVolume::LargeVolume(std::string const&	vfile_name,							
							scm::size_t			vol_hdd_cache_size,
							scm::size_t			vol_gpu_cache_size
							)
		: 
		_volume_file_path(vfile_name),
		_vtexture_info(),
		_renderer_settings(),
		_sample_distance(0.01),
		_volume_boxes_ptr(),
		_update_transfer_function(false),
		upload_mutex_()
	{
#if 0
		std::shared_ptr<scm::io::file> octree_file;
		octree_file.reset(new scm::io::file());
		if (!octree_file->open(vfile_name.c_str(), std::ios::in												
												| std::ios::binary, false))
		{
			WARNING("unable to read octree file ( %c ) ", vfile_name);
			octree_file->close();
		}

		octree_file->read(&(_vtexture_info.octree_header), 0, sizeof(scm::data::volume_octree_file_header));
		octree_file->close();
#else

		std::fstream octree_file(vfile_name.c_str(), std::fstream::in | std::fstream::binary);
		octree_file.read(reinterpret_cast<char*>(&(_vtexture_info.octree_header)), sizeof(scm::data::volume_octree_file_header));
		octree_file.close();

#endif

		_vtexture_info.vfile_name = vfile_name;
		_vtexture_info.vol_hdd_cache_size = vol_hdd_cache_size;
		_vtexture_info.vol_gpu_cache_size = vol_gpu_cache_size;
		_vtexture_info.vol_data_format = scm::gl::FORMAT_R_8;


		if (_vtexture_info.octree_header._data_channel_count == 1 && _vtexture_info.octree_header._data_channel_byte_per_channel == 4)
			_vtexture_info.vol_data_format = scm::gl::FORMAT_R_32F;
		else
		if (_vtexture_info.octree_header._data_channel_count == 2 && _vtexture_info.octree_header._data_channel_byte_per_channel == 1)
			_vtexture_info.vol_data_format = scm::gl::FORMAT_RG_8;
		else
			WARNING("unsupportet octree data format ( %c ) ", vfile_name.c_str());

		std::cout << _vtexture_info.vfile_name << std::endl
			<< "VVolume Dimensions: "<< _vtexture_info.octree_header._volume_dimensions << std::endl;

		//_volume_vtexture_ptr = vcontext->create_vtexture(vtex_file);
		//_volume_dimensions = _volume_vtexture_ptr->dimensions();
		_volume_dimensions = _vtexture_info.octree_header._volume_dimensions;

		unsigned max_dimension_volume = scm::math::max(scm::math::max(_volume_dimensions.x, _volume_dimensions.y), _volume_dimensions.z);

		sample_distance(1.0 / max_dimension_volume);

		_volume_dimensions_normalized = math::vec3((float)_volume_dimensions.x / (float)max_dimension_volume,
													(float)_volume_dimensions.y / (float)max_dimension_volume,
													(float)_volume_dimensions.z / (float)max_dimension_volume);

		MESSAGE("%f %f %f", _volume_dimensions_normalized.x, _volume_dimensions_normalized.y, _volume_dimensions_normalized.z);
		//getchar();

		bounding_box_ = math::BoundingBox<math::vec3>(math::vec3::zero(), _volume_dimensions_normalized);

		// initialize transfer functions //////////////////////////////////////////////////////////////
		_alpha_transfer.clear();
		_color_transfer.clear();

#if 1
		_alpha_transfer.add_stop(0, 1.0f);		
		_alpha_transfer.add_stop(0.45f, 0.0f);
		_alpha_transfer.add_stop(0.5f, 0.0f);
		_alpha_transfer.add_stop(0.55f, 0.0f);
		_alpha_transfer.add_stop(1.0f, 1.0f);
#elif 0
		_alpha_transfer.add_stop(0.0f, 0.0f);
		_alpha_transfer.add_stop(0.3f, 0.0f);
		_alpha_transfer.add_stop(0.4f, 0.2f);
		_alpha_transfer.add_stop(0.7f, 0.0f);
		_alpha_transfer.add_stop(1.0f, 1.0f);
#else
		_alpha_transfer.add_stop(0, 1.0f);		
		_alpha_transfer.add_stop(1.0f, 1.0f);
#endif

#if 1
		// blue-grey-orange
		_color_transfer.add_stop(0, scm::math::vec3f(0.0f, 1.0f, 1.0f));
		_color_transfer.add_stop(0.25f, scm::math::vec3f(0.0f, 0.0f, 1.0f));
		_color_transfer.add_stop(0.375f, scm::math::vec3f(0.256637f, 0.243243f, 0.245614f));
		_color_transfer.add_stop(0.50f, scm::math::vec3f(0.765487f, 0.738739f, 0.72807f));
		_color_transfer.add_stop(0.625f, scm::math::vec3f(0.530973f, 0.27027f, 0.0f));
		_color_transfer.add_stop(0.75f, scm::math::vec3f(1.0f, 0.333333f, 0.0f));
		_color_transfer.add_stop(1.0f, scm::math::vec3f(1.0f, 1.0f, 0.0f));
#else	
		// blue-white-red
		_color_transfer.add_stop(0.0f, scm::math::vec3f(0.0f, 0.0f, 0.0f));
		_color_transfer.add_stop(0.5f, scm::math::vec3f(0.4f, 0.0f, 0.4f));
		_color_transfer.add_stop(0.7f, scm::math::vec3f(1.0f, 1.0f, 1.0f));
		_color_transfer.add_stop(1.0f, scm::math::vec3f(1.0f, 1.0f, 1.0f));
#endif
		
	}

	////////////////////////////////////////////////////////////////////////////////

	void LargeVolume::upload_to(RenderContext const& ctx) const {

		//if (!_volume_texture_ptr[ctx.id]) {
		//	WARNING("Unable to load LargeVolume! Has no volume data.");
		//	return;
		//}

		std::unique_lock<std::mutex> lock(upload_mutex_);

		if (_volume_boxes_ptr.size() <= ctx.id){
			//_volume_texture_ptr.resize(ctx.id + 1);
			_transfer_texture_ptr.resize(ctx.id + 1);
			_gauss_texture_ptr.resize(ctx.id + 1);
			_volume_boxes_ptr.resize(ctx.id + 1);
			_sstate.resize(ctx.id + 1);
			_rstate.resize(ctx.id + 1);
		}

		try {
			_gauss_gen.reset(new scm::data::gauss_table_generator(ctx));
		}
		catch (const std::exception& e) {
			throw std::runtime_error(std::string("LargeVolume::upload_to(): error creating scm::data::gauss generator: ") + e.what());
		}

		try {
			_vtexture_system = boost::make_shared<scm::data::vtexture_system>(ctx.render_device, scm::math::vec2ui(ctx.width, ctx.height));		
		}
		catch (const std::exception& e) {
			throw std::runtime_error(std::string("LargeVolume::upload_to(): error creating _vtexture_system: ") + e.what());
		}

		_vtexture_system->profile_timing(false);
		_vtexture_system->debug_switches()._use_feedback_image_path = false;
		_vtexture_system->debug_switches()._use_feedback_lists_path = true;
		
		try {
			_vtexture_context_volume = _vtexture_system->create_vtexture_3d_context(_vtexture_info.vol_hdd_cache_size, _vtexture_info.vol_gpu_cache_size, _vtexture_info.octree_header._octree_brick_size, _vtexture_info.vol_data_format);
		}
		catch (const std::exception& e) {
			throw std::runtime_error(std::string("LargeVolume::upload_to(): error create_vtexture_3d_context: ") + e.what());
		}

		try {
			_volume_vtexture_ptr = _vtexture_context_volume->create_vtexture(_volume_file_path);
		}
		catch (const std::exception& e) {
			throw std::runtime_error(std::string("LargeVolume::upload_to(): error create_vtexture: ") + e.what());
		}


		if (!_volume_vtexture_ptr){
			std::cout << _volume_file_path << std::endl;
			WARNING("%s error!", _volume_file_path.c_str());
		}
		else{
			MESSAGE("%s loaded!", _volume_file_path.c_str());
		}
		//scm::gl::volume_loader scm_volume_loader;

		//texture_3d_ptr              load_texture_3d(render_device&       in_device,
		//	const std::string&   in_image_path,
		//	bool                 in_create_mips,
		//	bool                 in_color_mips = false,
		//	const data_format    in_force_internal_format = FORMAT_NULL);

		//_volume_texture_ptr[ctx.id] = std::shared_ptr<Texture3D>(new Texture3D(_volume_file_path));// scm_volume_loader.load_texture_3d(*(ctx.render_device.get()), _volume_file_path, false);
		//_volume_texture_ptr[ctx.id]->upload_to(ctx);

		unsigned transfer_texture_width = 255u;

		//scm::gl::texture_loader scm_image_loader; 
		_transfer_texture_ptr[ctx.id] = create_color_map(ctx, transfer_texture_width, _alpha_transfer, _color_transfer);
		_transfer_texture_ptr[ctx.id]->upload_to(ctx);

		_gauss_texture_ptr[ctx.id] = std::shared_ptr<Texture2D>(new Texture2D(transfer_texture_width, transfer_texture_width, scm::gl::FORMAT_RGBA_32F));
		_gauss_texture_ptr[ctx.id]->upload_to(ctx);


		//box_volume_geometry
		_volume_boxes_ptr[ctx.id] =
			scm::gl::box_volume_geometry_ptr(new scm::gl::box_volume_geometry(ctx.render_device, scm::math::vec3(0.0), _volume_dimensions_normalized));
		
		_sstate[ctx.id] = ctx.render_device->create_sampler_state(
			scm::gl::FILTER_MIN_MAG_MIP_LINEAR, scm::gl::WRAP_CLAMP_TO_EDGE);

		_rstate[ctx.id] = ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, 
																		scm::gl::CULL_FRONT, 
																		scm::gl::ORIENT_CCW, 
																		true);
	}

	std::shared_ptr<Texture2D>
	LargeVolume::create_color_map(RenderContext const& ctx,
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
			std::cout << "LargeVolume::create_color_map(): error during lookuptable generation" << std::endl;
			return (std::shared_ptr<Texture2D>(new Texture2D(in_size, 1)));
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

		std::shared_ptr<Texture2D> new_tex =
			std::shared_ptr<Texture2D>(new Texture2D(in_size, 1, FORMAT_RGBA_32F, in_data));// ctx.render_device->create_texture_2d(scm::math::vec2ui(in_size, 1), FORMAT_RGBA_8, 1, 1, 1, FORMAT_RGBA_32F, in_data);

		if (!new_tex) {
			std::cerr << "LargeVolume::create_color_map(): error during color map texture generation." << std::endl;
			return (std::shared_ptr<Texture2D>(new Texture2D(in_size, 1)));
		}
		else{
			std::cout << "LargeVolume::create_color_map(): color map texture generated." << std::endl;
			return (new_tex);
		}
	}

	bool LargeVolume::update_color_map(RenderContext const& ctx,
		std::shared_ptr<Texture2D> transfer_texture_ptr,
		const scm::data::piecewise_function_1d<float, float>& in_alpha,
		const scm::data::piecewise_function_1d<float, scm::math::vec3f>& in_color) const
	{
		using namespace scm::gl;
		using namespace scm::math;

		scm::scoped_array<scm::math::vec3f>  color_lut;
		scm::scoped_array<float>             alpha_lut;

		unsigned in_size = transfer_texture_ptr->width();
		
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

		MESSAGE("uploading texture data ( size: %d KiB)...", static_cast<double>(in_size * size_of_format(FORMAT_RGBA_32F)) / (1024.0));

		texture_region ur(vec3ui(0u), vec3ui(in_size, 1, 1));
		bool res = ctx.render_context->update_sub_texture(transfer_texture_ptr->get_buffer(ctx), ur, 0u, FORMAT_RGBA_32F, combined_lut.get());

		MESSAGE("uploading texture data done.");

		if (!res) {
			MESSAGE("LargeVolume::update_color_alpha_map(): error during color map texture generation.");
			return false;
		}

		return true;
	}

	////////////////////////////////////////////////////////////////////////////////

	void LargeVolume::draw(RenderContext const& ctx) const {

		// upload to GPU if neccessary
		if (_volume_boxes_ptr.size() <= ctx.id || _volume_boxes_ptr[ctx.id] == nullptr) {
			upload_to(ctx);
		}

		if (_update_transfer_function){
			for (auto color_map_texture : _transfer_texture_ptr)
			{
				update_color_map(ctx, color_map_texture, _alpha_transfer, _color_transfer);
			}
			_update_transfer_function = false;
		}

		scm::gl::context_vertex_input_guard vig(ctx.render_context);

//		ctx.render_context->bind_texture( _volume_texture_ptr[ctx.id]->get_buffer(ctx), _sstate[ctx.id], 5);
//		ctx.render_context->bind_texture(_transfer_texture_ptr[ctx.id]->get_buffer(ctx), _sstate[ctx.id], 6);
		scm::gl::program_ptr p = ctx.render_context->current_program();
		p->uniform_sampler("volume_texture", 5);			
		p->uniform_sampler("color_map", 6);
		p->uniform_sampler("gausscolor_map", 7);
		p->uniform("sampling_distance", _sample_distance);
		//p->uniform("iso_value", 0.8f);
		p->uniform("volume_bounds", _volume_dimensions_normalized);
		
		ctx.render_context->apply();
		_volume_boxes_ptr[ctx.id]->draw(ctx.render_context);		
	}

	void LargeVolume::draw_proxy(RenderContext const& ctx) const {

		// upload to GPU if neccessary
		if (_volume_boxes_ptr.size() <= ctx.id || _volume_boxes_ptr[ctx.id] == nullptr) {
			upload_to(ctx);
		}

		scm::gl::context_all_guard vig(ctx.render_context);

		ctx.render_context->set_rasterizer_state(_rstate[ctx.id]);

		ctx.render_context->apply();
		_volume_boxes_ptr[ctx.id]->draw(ctx.render_context);

	}

	void LargeVolume::set_uniforms(RenderContext const& ctx, ShaderProgram* cs) const
	{
		if (_update_transfer_function){

			update_color_map(ctx, _transfer_texture_ptr[ctx.id], _alpha_transfer, _color_transfer);
			_gauss_gen->generate_table(ctx, _transfer_texture_ptr[ctx.id], _gauss_texture_ptr[ctx.id]);

			_update_transfer_function = false;
		}

		//_volume_texture_ptr[ctx.id]->make_resident(ctx);
		//_transfer_texture_ptr[ctx.id]->make_resident(ctx);

		if (!_transfer_texture_ptr[ctx.id]){
			std::cerr << "No Transfer Texture2D ptr!" << std::endl;
			return;
		}
		
		//cs->set_uniform(ctx, _volume_texture_ptr[ctx.id], "volume_texture");
		cs->set_uniform(ctx, _transfer_texture_ptr[ctx.id], "color_map");
		cs->set_uniform(ctx, _gauss_texture_ptr[ctx.id], "gauss_color_map");
		cs->set_uniform(ctx, _sample_distance, "uni_sampling_distance");
		cs->set_uniform(ctx, _volume_dimensions_normalized, "uni_volume_bounds");

		//_volume_texture_ptr[ctx.id]->make_non_resident(ctx);
		//_transfer_texture_ptr[ctx.id]->make_non_resident(ctx);

	}

	void	LargeVolume::pre_frame_update(RenderContext const& context){
		_vtexture_system->pre_frame_update(context.render_device, context.render_context);
	}
	
	void    LargeVolume::post_frame_update(RenderContext const& context){
		_vtexture_system->post_frame_update(context.render_context);
	}

	////////////////////////////////////////////////////////////////////////////////

	void LargeVolume::ray_test(Ray const& ray, PickResult::Options options,
		Node* owner, std::set<PickResult>& hits) {

		//kd_tree_.ray_test(ray, mesh_, options, owner, hits);
	}

	////////////////////////////////////////////////////////////////////////////////
	
	float LargeVolume::sample_distance() const
	{
		return _sample_distance;
	}

	void LargeVolume::sample_distance(const float in_sample_distance)
	{
		_sample_distance = in_sample_distance;
	}
		
	void LargeVolume::set_transfer_function(const scm::data::piecewise_function_1d<float, float>& in_alpha, const scm::data::piecewise_function_1d<float, scm::math::vec3f>& in_color)
	{
		_alpha_transfer.clear();
		_color_transfer.clear();

		_alpha_transfer = in_alpha;
		_color_transfer = in_color;

		_update_transfer_function = true;
	}
		
	void LargeVolume::bind_vtexture(RenderContext const& context) const
	{
		_vtexture_system->bind_vtexture(context.render_context, _volume_vtexture_ptr, _sstate[context.id], 0);
		
	}
	
	void LargeVolume::program_uniform(RenderContext const& context, gua::ShaderProgram* shader_prg, std::string const& uniform_name) const
	{
		_vtexture_system->program_uniform(shader_prg->get_program_ptr(context), uniform_name, _volume_vtexture_ptr, 0);
	}
}
