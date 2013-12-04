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

#ifndef GUA_LARGE_VOLUME_HPP
#define GUA_LARGE_VOLUME_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/Geometry.hpp>
#include <gua/renderer/Texture2D.hpp>
#include <gua/renderer/Texture3D.hpp>
#include <gua/renderer/ShaderProgram.hpp>

// external headers
#include <scm/gl_core.h>

#include <scm/gl_core/gl_core_fwd.h>
#include <scm/gl_core/data_types.h>
#include <scm/gl_core/state_objects.h>

#include <scm/gl_util/data/volume/volume_loader.h>
#include <scm/gl_util/data/analysis/transfer_function/piecewise_function_1d.h>
#include <scm/gl_util/data/analysis/transfer_function/build_lookup_table.h>
#include <scm/gl_util/primitives/primitives_fwd.h>
#include <scm/gl_util/primitives/geometry.h>
#include <scm/gl_util/viewer/viewer_fwd.h>

#include <scm/core/platform/platform.h>
#include <scm/core/utilities/platform_warning_disable.h>

#include <scm/large_data/virtual_texture/vtexture_fwd.h>
#include <scm/large_data/volume_data/volume_header.h>


#include <mutex>
#include <thread>

#include <vector>

//struct aiMesh;


namespace gua {

	struct RenderContext;

	/**
	* Stores geometry data.
	*
	* A volume can be loaded from an Assimp volume and the draw onto multiple
	* contexts.
	* Do not use this class directly, it is just used by the Geometry class to
	* store the individual meshes of a file.
	*/
	class LargeVolume : public Geometry {
	public:

		struct renderer_settings {
			//render_method   _render_method;
			bool            _vtexture_fixed_lod_enabled;
			float           _vtexture_fixed_lod;
			bool            _dvr_use_preintegration;
			bool            _dvr_use_adjacent_blend;
			bool            _dvr_use_animated_blend;
			bool            _dvr_use_gauss_fitting;
			bool            _dvr_use_texture_view;
			bool            _dvr_use_lod_adaptive_sampling;
			bool            _dvr_embedded_octree;
			bool            _dvr_show_iteration_count;

			renderer_settings();
			bool operator==(const renderer_settings& rhs) const;
			bool operator!=(const renderer_settings& rhs) const;
		}; // struct renderer_settings

		struct vtexture3d_info {			
			std::string				vfile_name;
			scm::size_t				vol_hdd_cache_size;
			scm::size_t				vol_gpu_cache_size;
			scm::gl::data_format	vol_data_format;
			
			scm::data::volume_octree_file_header octree_header;

		}; // struct vtexture3d_info


		/**
		* Default constructor.
		*
		* Creates a new and empty LargeVolume.
		*/
		LargeVolume();

		/**
		* Constructor from an octree volume.
		*
		* Initializes the volume from a given file path
		*
		* \param volume             The Assimp volume to load the data from.
		*/
		LargeVolume(std::string const&	vfile_name,					
					scm::size_t			vol_hdd_cache_size,
					scm::size_t			vol_gpu_cache_size
					);
		
		/**
		* Draws the LargeVolume.
		*
		* Draws the LargeVolume to the given context. There is no compositing happening
		*
		* \param context          The RenderContext to draw onto.
		*/
		void draw(RenderContext const& context) const;

		void draw(const scm::gl::render_context_ptr& context,
							const scm::data::vtexture_system_ptr&    vsystem) const;
		void update(const scm::gl::render_device_ptr&  device,
										const scm::gl::render_context_ptr& context,
										const scm::data::vtexture_system_ptr&    vsystem,
										const scm::gl::camera&             cam);

		/**
		* Draws the LargeVolume.
		*
		* Draws the LargeVolume proxy to the given context.
		*
		* \param context          The RenderContext to draw onto.
		*/
		void draw_proxy(RenderContext const& context) const;

		/**
		* Sets the necessary uniform values for compositing shader
		*
		* Draws the LargeVolume proxy to the given context.
		*
		* \param shaderProgram          The RenderContext to draw onto.
		*/
		void set_uniforms(RenderContext const& ctx, ShaderProgram* cs) const;

		void ray_test(Ray const& ray, PickResult::Options options,
			Node* owner, std::set<PickResult>& hits);


		float sample_distance() const;
		void sample_distance(const float sample_distance);

		float                           sample_distance_ref() const;
		float                           sample_distance_ref_factor() const;
		void                            sample_distance_ref_factor(float d);

		//const vtex_volume&              volume() const;
		void                            update_volume_lens_box(const scm::gl::box& lb);
		void                            update_volume_lens_box_normalized(const scm::gl::box& lb);

		void set_transfer_function(const scm::data::piecewise_function_1d<float, float>& in_alpha, const scm::data::piecewise_function_1d<float, scm::math::vec3f>& in_color);

	private:
		void upload_to(RenderContext const& context) const;

		std::shared_ptr<Texture2D> create_color_map(RenderContext const& context,
			unsigned in_size,
			const scm::data::piecewise_function_1d<float, float>& in_alpha,
			const scm::data::piecewise_function_1d<float, scm::math::vec3f>& in_color) const;

		bool update_color_map(RenderContext const& context,
			std::shared_ptr<Texture2D>,
			const scm::data::piecewise_function_1d<float, float>& in_alpha,
			const scm::data::piecewise_function_1d<float, scm::math::vec3f>& in_color) const;
		
		mutable bool _update_transfer_function;

		////LargeVolume files
		//mutable std::vector<std::string>
		//	_volume_file_pathes;
		// LargeVolume File path
		std::string _volume_file_path;

		//LargeVolume boxes for each volume
		mutable std::vector<scm::gl::box_volume_geometry_ptr> 
			_volume_boxes_ptr;

		//Texture3D for volume data for each volume
		//mutable std::vector<std::shared_ptr<Texture3D>>
		//	_volume_texture_ptr;
		vtexture3d_info								_vtexture_info;
		renderer_settings							_renderer_settings;

		mutable scm::data::vtexture_system_ptr				_vtexture_system;
		mutable scm::data::vtexture_3d_context_ptr			_vtexture_context_volume;
		mutable scm::data::vtexture_3d_ptr					_volume_vtexture_ptr;

		mutable std::vector<std::shared_ptr<Texture2D>>
			_transfer_texture_ptr;

		mutable std::vector<std::shared_ptr<Texture2D>>
			_gauss_texture_ptr;

		mutable std::vector<scm::gl::sampler_state_ptr> _sstate;

		//gauss_table_generator_ptr       _gauss_gen;

#if GUA_COMPILER == GUA_COMPILER_MSVC && SCM_COMPILER_VER <= 1700
		mutable boost::mutex upload_mutex_;
#else
		mutable std::mutex upload_mutex_;
#endif

		scm::data::piecewise_function_1d<float, float>                 _alpha_transfer;
		scm::data::piecewise_function_1d<float, scm::math::vec3f>      _color_transfer;

		///LargeVolume Info
		math::vec3ui	_volume_dimensions;
		math::vec3		_volume_dimensions_normalized;
		float			_sample_distance;

		//std::shared_ptr<vtex_volume>      _vtex_volume;

		//gauss_table_generator_ptr			_gauss_gen;
		//scm::gl::texture_2d_ptr           _gauss_color_alpha_map;

	public:

	};

}

#endif  // GUA_LARGE_VOLUME_HPP
