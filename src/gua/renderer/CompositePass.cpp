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
#include <gua/renderer/CompositePass.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/StereoBuffer.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/LayerMapping.hpp>
#include <gua/renderer/Volume.hpp>
#include <gua/renderer/LargeVolume.hpp>
#include <gua/databases.hpp>

#include <scm/large_data/virtual_texture/context_vtexture_guard.h>

#include <memory>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

CompositePass::CompositePass(Pipeline* pipeline) :
	Pass(pipeline),
	composite_shader_(new ShaderProgram),
	//v_composite_shader_(new ShaderProgram),
	ray_generation_shader_(new ShaderProgram),
	volume_raygeneration_buffer_(nullptr)
{

    std::string ray_generation_vertex_shader(Resources::lookup_shader(Resources::shaders_uber_shaders_composite_ray_generation_vert));
    std::string ray_generation_fragment_shader(Resources::lookup_shader(Resources::shaders_uber_shaders_composite_ray_generation_frag));

    ray_generation_shader_->create_from_sources(ray_generation_vertex_shader, ray_generation_fragment_shader);

#if 1
    composite_shader_->create_from_files("compos_n.vert",
                                         "compos_n.frag");
#else

  std::string vertex_shader (Resources::lookup_shader(Resources::shaders_uber_shaders_composite_compose_vert));
  std::string fragment_shader(Resources::lookup_shader(Resources::shaders_uber_shaders_composite_compose_frag));

  composite_shader_->create_from_sources(vertex_shader, fragment_shader);
  composite_shader_->save_to_file(".", "compos_n");
#endif

}

////////////////////////////////////////////////////////////////////////////////

CompositePass::~CompositePass() {

  if (composite_shader_) {
    delete composite_shader_;
  }

  //if (v_composite_shader_) {
	 // delete v_composite_shader_;
  //}
  
  if (volume_raygeneration_buffer_) {
    delete volume_raygeneration_buffer_;
  }

  if (ray_generation_shader_) {
    delete ray_generation_shader_;
  }
}

////////////////////////////////////////////////////////////////////////////////

void CompositePass::create(RenderContext const& ctx,
    PipelineConfiguration const& config, std::vector<std::pair<BufferComponent,
    scm::gl::sampler_state_desc>> const& layers) {

  // reuse gbuffer from shading-pass
  gbuffer_ = inputs_[Pipeline::PipelineStage::shading];


  if (volume_raygeneration_buffer_) {
    volume_raygeneration_buffer_->remove_buffers(ctx);
    delete volume_raygeneration_buffer_;
  }

  scm::gl::sampler_state_desc state(scm::gl::FILTER_MIN_MAG_LINEAR,
                                    scm::gl::WRAP_CLAMP_TO_EDGE,
                                    scm::gl::WRAP_CLAMP_TO_EDGE);

  std::vector<std::pair<BufferComponent, scm::gl::sampler_state_desc>> layer_4f_desc;
  layer_4f_desc.push_back(std::make_pair(BufferComponent::F4, state));


  volume_raygeneration_buffer_ = new GBuffer(layer_4f_desc,
                                      config.get_left_resolution()[0],
                                      config.get_left_resolution()[1]);

  volume_raygeneration_buffer_->create(ctx);
}

////////////////////////////////////////////////////////////////////////////////

/* virtual */ void CompositePass::rendering(SerializedScene const& scene,
  RenderContext const& ctx,
  CameraMode eye,
  Camera const& camera,
  FrameBufferObject* target) 
{
	init_ressources(ctx);

	ctx.render_context->set_depth_stencil_state(depth_stencil_state_);

	// 1. render proxy geometry into fbo
	volume_raygeneration_buffer_->bind(ctx);
	{
		scm::math::vec2f resolution(volume_raygeneration_buffer_->width(), volume_raygeneration_buffer_->height());
		ctx.render_context->set_viewport(scm::gl::viewport(math::vec2(0, 0), resolution));
			
		volume_raygeneration_buffer_->clear_color_buffers(ctx, gua::utils::Color3f(0.0f, 0.0f, 0.0f));
		volume_raygeneration_buffer_->clear_depth_stencil_buffer(ctx);
			
		///TODO: Toplevel
		if (!scene.volumenodes_.empty() || !scene.vvolumenodes_.empty()/*|| !scene.transparentnodes_.empty()*/)
		{
			// gather input textures and set uniforms
			Pass::set_camera_matrices(*ray_generation_shader_, camera, pipeline_->get_current_scene(eye), eye, ctx);

			ray_generation_shader_->set_uniform(ctx, 1.f / gbuffer_->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->width(), "gua_texel_width");
			ray_generation_shader_->set_uniform(ctx, 1.f / gbuffer_->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->height(), "gua_texel_height");
            
			volume_raygeneration_buffer_->clear_color_buffers(ctx, gua::utils::Color3f(0.0f, 0.0f, 0.0f));
			volume_raygeneration_buffer_->clear_depth_stencil_buffer(ctx);

			for (auto const& node : scene.volumenodes_) {
								
				auto volume =
					std::static_pointer_cast<gua::Volume>(GeometryDatabase::instance()->lookup(node.data.get_volume()));

				if (volume) {
					ray_generation_shader_->set_uniform(
						ctx, node.transform, "gua_model_matrix");

					ray_generation_shader_->set_uniform(
						ctx, 1, "volume_frag_id");
                    																						
					ray_generation_shader_->use(ctx);
					{
						volume->draw_proxy(ctx);
					}
					ray_generation_shader_->unuse(ctx);
				}
			}

			for (auto const& node : scene.vvolumenodes_) {
										
				auto vlargevolume =
					std::static_pointer_cast<gua::LargeVolume>(GeometryDatabase::instance()->lookup(node.data.get_volume()));

				if (vlargevolume) {

					ray_generation_shader_->set_uniform(
						ctx, node.transform, "gua_model_matrix");

					ray_generation_shader_->set_uniform(
						ctx, 2, "volume_frag_id");

					ray_generation_shader_->use(ctx);
					{
						vlargevolume->draw_proxy(ctx);
					}
                    ray_generation_shader_->unuse(ctx);
						
				}
			}
		}
	}
	volume_raygeneration_buffer_->unbind(ctx);
	
	scm::gl::context_all_guard      cug(ctx.render_context);
	scm::gl::context_vtexture_guard vtg(ctx.render_context);
		
	// 2. render fullscreen quad for compositing and volume ray castinG
	Pass::set_camera_matrices(*composite_shader_, camera, pipeline_->get_current_scene(eye), eye, ctx);
	//Pass::set_camera_matrices(*v_composite_shader_, camera, pipeline_->get_current_scene(eye), eye, ctx);

	auto input_tex(inputs_[Pipeline::shading]->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->get_color_buffers(TYPE_FLOAT)[0]);
	auto normal_tex(inputs_[Pipeline::geometry]->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->get_color_buffers(TYPE_FLOAT)[0]);
	auto depth_tex(inputs_[Pipeline::geometry]->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->get_depth_buffer());

	auto raygen_tex(volume_raygeneration_buffer_->get_color_buffers(TYPE_FLOAT)[0]);

	composite_shader_->set_uniform(ctx, input_tex, "gua_color_gbuffer_in");
	composite_shader_->set_uniform(ctx, normal_tex, "gua_normal_gbuffer_in");
	composite_shader_->set_uniform(ctx, depth_tex, "gua_depth_gbuffer_in");
    composite_shader_->set_uniform(ctx, raygen_tex, "gua_ray_entry_in");
				
	composite_shader_->set_uniform(ctx, 1.f / gbuffer_->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->width(), "gua_texel_width");
	composite_shader_->set_uniform(ctx, 1.f / gbuffer_->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->height(), "gua_texel_height");

	//v_composite_shader_->set_uniform(ctx, input_tex, "gua_color_gbuffer_in");
	//v_composite_shader_->set_uniform(ctx, normal_tex, "gua_normal_gbuffer_in");
	//v_composite_shader_->set_uniform(ctx, depth_tex, "gua_depth_gbuffer_in");
	//v_composite_shader_->set_uniform(ctx, raygen_tex, "gua_ray_entry_in");
	//	
	//v_composite_shader_->set_uniform(ctx, 1.f / gbuffer_->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->width(), "gua_texel_width");
	//v_composite_shader_->set_uniform(ctx, 1.f / gbuffer_->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->height(), "gua_texel_height");


	// bind target fbo and set viewport
	target->bind(ctx);
	ctx.render_context->set_viewport(scm::gl::viewport(
		math::vec2(0, 0),
		::scm::math::vec2f(target->width(), target->height())));

	if (!scene.volumenodes_.empty() || !scene.vvolumenodes_.empty()){

		for (auto const& node : scene.volumenodes_) {

			auto volume =
				std::static_pointer_cast<gua::Volume>(GeometryDatabase::instance()->lookup(node.data.get_volume()));

			if (volume) {
				composite_shader_->set_uniform(ctx, node.transform, "gua_model_matrix");
                composite_shader_->set_uniform(ctx, scene.frustum.get_clip_near_plane(), "near_plane");                
                composite_shader_->set_uniform(ctx, scene.frustum.get_clip_near(), "near_plane_d");
                composite_shader_->set_uniform(ctx, scene.frustum.get_clip_far(), "far_plane_d");                
                composite_shader_->set_uniform(ctx, scene.frustum.get_clip_far() - scene.frustum.get_clip_near(), "diff_plane_d");

				volume->set_uniforms(ctx, composite_shader_);

				composite_shader_->use(ctx);
				{
					fullscreen_quad_->draw(ctx.render_context);
				}
				composite_shader_->unuse(ctx);
			}
		}

        for (auto const& node : scene.vvolumenodes_) {

            auto vlargevolume =
                std::static_pointer_cast<gua::LargeVolume>(GeometryDatabase::instance()->lookup(node.data.get_volume()));

            if (vlargevolume) {
                composite_shader_->set_uniform(ctx, node.transform, "gua_model_matrix");
                composite_shader_->set_uniform(ctx, scene.frustum.get_clip_near_plane(), "near_plane");
                composite_shader_->set_uniform(ctx, scene.frustum.get_clip_near(), "near_plane_d");
                composite_shader_->set_uniform(ctx, scene.frustum.get_clip_far(), "far_plane_d");
                composite_shader_->set_uniform(ctx, scene.frustum.get_clip_far() - scene.frustum.get_clip_near(), "diff_plane_d");
                                                
                vlargevolume->set_uniforms(ctx, composite_shader_);
                              
                vlargevolume->pre_frame_update(ctx);

                vlargevolume->bind_vtexture(ctx);
                vlargevolume->program_uniform(ctx, composite_shader_, "vtex_volume");

                composite_shader_->use(ctx);
                {
                    fullscreen_quad_->draw(ctx.render_context);
                }
                composite_shader_->unuse(ctx);

                vlargevolume->post_frame_update(ctx);
            }
        }
        
        /*{

			auto vlargevolume =
				std::static_pointer_cast<gua::LargeVolume>(GeometryDatabase::instance()->lookup(node.data.get_volume()));

			if (vlargevolume) {
				v_composite_shader_->set_uniform(
					ctx, node.transform, "gua_model_matrix");

                v_composite_shader_->set_uniform(
                    ctx, scene.frustum.get_clip_near_plane(), "near_plane");
                
				vlargevolume->set_uniforms(ctx, v_composite_shader_);

				vlargevolume->pre_frame_update(ctx);
					
				vlargevolume->bind_vtexture(ctx);
				vlargevolume->program_uniform(ctx, v_composite_shader_, "vtex_volume");
                					
				v_composite_shader_->use(ctx);
				{
					fullscreen_quad_->draw(ctx.render_context);
				}
				v_composite_shader_->unuse(ctx);

				vlargevolume->post_frame_update(ctx);
			}
		}*/
	}
	else{
		composite_shader_->use(ctx);
		{
			fullscreen_quad_->draw(ctx.render_context);
		}
		composite_shader_->unuse(ctx);
	}
	target->unbind(ctx);

	ctx.render_context->reset_state_objects();
	
}

////////////////////////////////////////////////////////////////////////////////

void CompositePass::init_ressources(RenderContext const& ctx) {

  if (!depth_stencil_state_) {
    depth_stencil_state_ = ctx.render_device->create_depth_stencil_state(false, false, scm::gl::COMPARISON_NEVER);
  }

  if (!fullscreen_quad_) {
    fullscreen_quad_ = scm::gl::quad_geometry_ptr(new scm::gl::quad_geometry(ctx.render_device, math::vec2(-1.f, -1.f), math::vec2(1.f, 1.f)));
  }
}

////////////////////////////////////////////////////////////////////////////////

/* virtual */ LayerMapping const* CompositePass::get_gbuffer_mapping() const {
  throw std::runtime_error("no gbuffer mapping available for composite pass");
}

////////////////////////////////////////////////////////////////////////////////

void CompositePass::print_shaders(std::string const& directory,
                               std::string const& name) const  {
	composite_shader_->save_to_file(directory, name + "/composite_shader");
	//v_composite_shader_->save_to_file(directory, name + "/v_composite_shader");
	ray_generation_shader_->save_to_file(directory, name + "/ray_generation_shader");
}

////////////////////////////////////////////////////////////////////////////////

bool CompositePass::pre_compile_shaders(RenderContext const& ctx) {

	ctx.render_device->add_include_files("./../../../guacamole/resources/shaders/uber_shaders/composite/virtual_volume", "/scm/virtual_volume");
	ctx.render_device->add_include_files("./../../../guacamole/externals/inc/schism/scm_large_data/src/scm/large_data/virtual_texture/shader", "/scm/data/vtexture");
	ctx.render_device->add_include_files("./../../../guacamole/resources/shaders/uber_shaders/composite/volume_utils", "/scm/data");

	if (composite_shader_)            return composite_shader_->upload_to(ctx);
	//if (v_composite_shader_)            return v_composite_shader_->upload_to(ctx);
    if (ray_generation_shader_)       return ray_generation_shader_->upload_to(ctx);

    return false;
}

////////////////////////////////////////////////////////////////////////////////

void CompositePass::render_scene(Camera const& camera, RenderContext const& ctx) {

  for (int i(0); i < gbuffer_->get_eye_buffers().size(); ++i) {

    FrameBufferObject* fbo(gbuffer_->get_eye_buffers()[i]);

    CameraMode eye(CameraMode::CENTER);
    if (gbuffer_->get_eye_buffers().size() > 1 && i == 0)
      eye = CameraMode::LEFT;
    if (gbuffer_->get_eye_buffers().size() > 1 && i == 1)
      eye = CameraMode::RIGHT;

    fbo->bind(ctx);

    ctx.render_context->set_viewport(scm::gl::viewport(
      math::vec2(0, 0), ::scm::math::vec2f(fbo->width(), fbo->height())));

    rendering(pipeline_->get_current_scene(eye), ctx, eye, camera, fbo);

    fbo->unbind(ctx);
}
}

}
