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
#include <gua/spoints/SPointsRenderer.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/guacamole.hpp>
#include <gua/databases.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/memory.hpp>

#include <gua/renderer/ResourceFactory.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/MaterialShader.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/WindowBase.hpp>
#include <gua/spoints/SPointsResource.hpp>
#include <gua/spoints/SPointsNode.hpp>
#include <gua/spoints/spoints_geometry/NetKinectArray.hpp>

#include <scm/gl_core/render_device/context_guards.h>

namespace {

struct VertexOnly {
  scm::math::vec3f pos;
};

std::vector<unsigned> proxy_mesh_indices(int size,
                                         unsigned width,
                                         unsigned height) {
  std::vector<unsigned> index_array(size);
  unsigned v(0);
  for (unsigned h(0); h < (height - 1); ++h) {
    for (unsigned w(0); w < (width - 1); ++w) {
      index_array[v] = (w + h * width);
      ++v;
      index_array[v] = (w + h * width + 1);
      ++v;
      index_array[v] = (w + h * width + width);
      ++v;
      index_array[v] = (w + h * width + width);
      ++v;
      index_array[v] = (w + h * width + 1);
      ++v;
      index_array[v] = (w + h * width + 1 + width);
      ++v;
    }
  }
  return index_array;
}

gua::RenderContext::Mesh create_proxy_mesh(gua::RenderContext& ctx,
                                           unsigned height_depthimage,
                                           unsigned width_depthimage) {
  int num_vertices = height_depthimage * width_depthimage;
  // int num_indices = height_depthimage * width_depthimage;
  int num_triangles = ((height_depthimage - 1) * (width_depthimage - 1)) * 2;
  int num_triangle_indices = 3 * num_triangles;
  // int num_line_indices = (height_depthimage - 1) * ((width_depthimage - 1) * 3 +
  //                                                   1) + (width_depthimage - 1);

  float step = 1.0f / width_depthimage;

  gua::RenderContext::Mesh proxy_mesh{};
  proxy_mesh.indices_topology = scm::gl::PRIMITIVE_TRIANGLE_LIST;
  proxy_mesh.indices_type = scm::gl::TYPE_UINT;
  proxy_mesh.indices_count = num_triangle_indices;

  proxy_mesh.vertices =
      ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       num_vertices * sizeof(VertexOnly),
                                       0);

  VertexOnly* data(static_cast<VertexOnly*>(ctx.render_context->map_buffer(
      proxy_mesh.vertices, scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER)));

  unsigned v(0);
  for (float h = 0.5 * step; h < height_depthimage* step; h += step) {
    for (float w = 0.5 * step; w < width_depthimage* step; w += step) {
      data[v].pos = scm::math::vec3f(w, h, 0.0f);
      ++v;
    }
  }

  ctx.render_context->unmap_buffer(proxy_mesh.vertices);

  std::vector<unsigned> indices = proxy_mesh_indices(
      num_triangle_indices, width_depthimage, height_depthimage);

  proxy_mesh.indices = ctx.render_device
      ->create_buffer(scm::gl::BIND_INDEX_BUFFER,
                      scm::gl::USAGE_STATIC_DRAW,
                      num_triangle_indices * sizeof(unsigned int),
                      indices.data());

  proxy_mesh.vertex_array = ctx.render_device->create_vertex_array(
      scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC3F, sizeof(VertexOnly)),
      { proxy_mesh.vertices });
  return proxy_mesh;
  //ctx.render_context->apply(); // necessary ???
}

void draw_proxy_mesh(gua::RenderContext const& ctx,
                     gua::RenderContext::Mesh const& mesh) {
  scm::gl::context_vertex_input_guard vig(ctx.render_context);
  ctx.render_context->bind_vertex_array(mesh.vertex_array);
  ctx.render_context->bind_index_buffer(
      mesh.indices, mesh.indices_topology, mesh.indices_type);

  ctx.render_context->apply();
  ctx.render_context->draw_elements(mesh.indices_count);
}

}

namespace gua {

////////////////////////////////////////////////////////////////////////////////

SPointsRenderer::SPointsRenderer() : initialized_(false) {
  ResourceFactory factory;


  // create final shader description
  program_stages_.push_back(ShaderProgramStage(
      scm::gl::STAGE_VERTEX_SHADER,
      factory.read_shader_file("resources/shaders/forward_point_rendering.vert")));
  program_stages_.push_back(ShaderProgramStage(
      scm::gl::STAGE_FRAGMENT_SHADER,
      factory.read_shader_file("resources/shaders/forward_point_rendering.frag")));



}


////////////////////////////////////////////////////////////////////////////////
void SPointsRenderer::render(Pipeline& pipe,
                             PipelinePassDescription const& desc) {



  ///////////////////////////////////////////////////////////////////////////
  //  retrieve current view state
  ///////////////////////////////////////////////////////////////////////////
  auto& scene = *pipe.current_viewstate().scene;
  auto const& camera = pipe.current_viewstate().camera;
  // auto const& frustum = pipe.current_viewstate().frustum;
  auto& target = *pipe.current_viewstate().target;

  auto const& ctx(pipe.get_context());


  if (!initialized_) {
    initialized_ = true;
    points_rasterizer_state_ = ctx.render_device
      ->create_rasterizer_state(scm::gl::FILL_SOLID,
                                scm::gl::CULL_NONE,
                                scm::gl::ORIENT_CCW,
                                false,
                                false,
                                0.0,
                                false,
                                false,
                                scm::gl::point_raster_state(true));
  }

  auto objects(scene.nodes.find(std::type_index(typeid(node::SPointsNode))));
  int view_id(camera.config.get_view_id());


  if (objects != scene.nodes.end() && objects->second.size() > 0) {

    float last_known_point_size = std::numeric_limits<float>::max();
    for (auto& o : objects->second) {

      auto spoints_node(reinterpret_cast<node::SPointsNode*>(o));
      auto spoints_desc(spoints_node->get_spoints_description());

      if (!GeometryDatabase::instance()->contains(spoints_desc)) {
        gua::Logger::LOG_WARNING << "SPointsRenderer::draw(): No such spoints."
                                 << spoints_desc << ", " << std::endl;
        continue;
      }

      auto spoints_resource = std::static_pointer_cast<SPointsResource>(
          GeometryDatabase::instance()->lookup(spoints_desc));
      if (!spoints_resource) {
        gua::Logger::LOG_WARNING << "SPointsRenderer::draw(): Invalid spoints."
                                 << std::endl;
        continue;
      }


      auto const& model_matrix(spoints_node->get_cached_world_transform());
      auto normal_matrix(scm::math::transpose(
          scm::math::inverse(spoints_node->get_cached_world_transform())));
      auto view_matrix(pipe.current_viewstate().frustum.get_view());


      scm::math::mat4f mv_matrix = scm::math::mat4f(view_matrix) * scm::math::mat4f(model_matrix);
      scm::math::mat4f projection_matrix = scm::math::mat4f(pipe.current_viewstate().frustum.get_projection());



      const float scaling = scm::math::length(
          (model_matrix * view_matrix) * scm::math::vec4d(1.0, 0.0, 0.0, 0.0));


      spoints::matrix_package current_package;
      memcpy((char*) &current_package, (char*) mv_matrix.data_array, 16 * sizeof(float) );
      memcpy( ((char*) &current_package) +  16 * sizeof(float), (char*) projection_matrix.data_array, 16 * sizeof(float) );
      
      scm::math::vec2ui const& render_target_dims = camera.config.get_resolution();



      current_package.res_xy[0] = render_target_dims.x;
      current_package.res_xy[1] = render_target_dims.y;


    auto camera_id = pipe.current_viewstate().viewpoint_uuid;
    //auto view_direction = pipe.current_viewstate().view_direction;
    //std::size_t gua_view_id = (camera_id << 8) | (std::size_t(view_direction));


    bool is_camera = (!pipe.current_viewstate().shadow_mode);

    bool stereo_mode = (pipe.current_viewstate().camera.config.get_enable_stereo());

    std::size_t view_uuid = camera_id;


    spoints::camera_matrix_package cm_package;
    cm_package.k_package.is_camera = is_camera;
    cm_package.k_package.view_uuid = view_uuid;
    cm_package.k_package.stereo_mode = stereo_mode;
    cm_package.k_package.framecount = pipe.get_context().framecount;
    cm_package.k_package.render_context_id = pipe.get_context().id;
    cm_package.mat_package = current_package;



    spoints_resource->push_matrix_package(cm_package);

    spoints_resource->update_buffers(pipe.get_context(), pipe);
    //auto const& spoints_data = spointsdata_[spoints_resource->uuid()];




      // get material dependent shader
      std::shared_ptr<ShaderProgram> current_shader;

      MaterialShader* current_material =
          spoints_node->get_material()->get_shader();
      if (current_material) {

        auto shader_iterator = programs_.find(current_material);
        if (shader_iterator != programs_.end()) {
          current_shader = shader_iterator->second;
        } else {
          auto smap = global_substitution_map_;
          for (const auto& i : current_material->generate_substitution_map())
            smap[i.first] = i.second;

          current_shader = std::make_shared<ShaderProgram>();
          current_shader->set_shaders(
              program_stages_, std::list<std::string>(), false, smap);
          programs_[current_material] = current_shader;
        }
      } else {
        Logger::LOG_WARNING << "SPointsPass::render(): Cannot find material: "
                            << spoints_node->get_material()->get_shader_name()
                            << std::endl;
      }




      current_shader->use(ctx);
    
      current_shader->set_uniform(
        ctx,
        scm::math::mat4f(model_matrix),
        "kinect_model_matrix");

      float const screen_space_point_size = spoints_node->get_screen_space_point_size();

      current_shader->set_uniform(
        ctx,
        screen_space_point_size,
        "point_size");

      Vec<float> quant_step_vec = spoints_resource->getQuantizationStepSize();
      //std::cout << "QUANT STEP " << quant_step_vec << std::endl;
      float quant_step = std::max(quant_step_vec.z, std::max(quant_step_vec.x, quant_step_vec.y));

      current_shader->set_uniform(
        ctx,
        quant_step,
        "quant_step");

      bool write_depth = true;
      target.bind(ctx, write_depth);
      target.set_viewport(ctx);

      ctx.render_context->set_rasterizer_state(points_rasterizer_state_);
      ctx.render_context->apply();

      spoints_resource->draw(ctx);

      target.unbind(ctx);
    }

  
  }

}

////////////////////////////////////////////////////////////////////////////////

}
