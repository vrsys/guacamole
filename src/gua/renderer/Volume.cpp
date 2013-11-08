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

		unsigned max_dimension_volume = scm::math::max(scm::math::max(volume_dimensions.x, volume_dimensions.y), volume_dimensions.z);

		gua::math::vec3 scaled_max_vertex_pos((float)volume_dimensions.x / (float)max_dimension_volume,
			(float)volume_dimensions.y / (float)max_dimension_volume,
			(float)volume_dimensions.z / (float)max_dimension_volume);

		MESSAGE("%f %f %f", scaled_max_vertex_pos.x, scaled_max_vertex_pos.y, scaled_max_vertex_pos.z);
		getchar();

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
			_volume_boxes_ptr.resize(ctx.id + 1);
		}

		scm::gl::volume_loader scm_volume_loader;
		_volume_texture_ptr[ctx.id] = scm_volume_loader.load_volume_data(_volume_file_path);

		//box_volume_geometry
		_volume_boxes_ptr[ctx.id] =
			scm::gl::box_volume_geometry_ptr(new scm::gl::box_volume_geometry(ctx.render_device, bounding_box_.min, bounding_box_.max));
		
	}

	////////////////////////////////////////////////////////////////////////////////

	void Volume::draw(RenderContext const& ctx) const {

		// upload to GPU if neccessary
		if (_volume_boxes_ptr.size() <= ctx.id || _volume_boxes_ptr[ctx.id] == nullptr) {
			upload_to(ctx);
		}

		scm::gl::context_vertex_input_guard vig(ctx.render_context);

		scm::gl::program_ptr p = ctx.render_context->current_program();
		p->uniform("volume_texture", 5);

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
