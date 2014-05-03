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
#include <gua/renderer/PBRRessource.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/utils/Logger.hpp>

// external headers



namespace gua {

////////////////////////////////////////////////////////////////////////////////

PBRRessource::PBRRessource()
    : buffers_(), upload_mutex_(), point_cloud_(nullptr) {}

////////////////////////////////////////////////////////////////////////////////

PBRRessource::PBRRessource(std::shared_ptr<pbr::ren::RawPointCloud> point_cloud)
    : buffers_(),
      upload_mutex_(),
      point_cloud_(point_cloud){

    //set already created BB

    if (!point_cloud_->is_loaded())
    {
       Logger::LOG_WARNING << "Point Cloud was not loaded!" << std::endl;
    }
    else
    {
    	scm::gl::boxf loaded_bb = point_cloud->aabb();
    	bounding_box_.min = loaded_bb.min_vertex();
   	bounding_box_.max = loaded_bb.max_vertex();
    }
}

////////////////////////////////////////////////////////////////////////////////

void PBRRessource::upload_to(RenderContext const& ctx) const {

  if (!point_cloud_->is_loaded()) {
    Logger::LOG_WARNING << "Point Cloud was not loaded!" << std::endl;
    return;
  }

  std::unique_lock<std::mutex> lock(upload_mutex_);

  if (buffers_.size() <= ctx.id) {
    buffers_.resize(ctx.id + 1);
  }

  buffers_[ctx.id] =
      ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                                       scm::gl::USAGE_DYNAMIC_COPY,
                                       point_cloud_->num_surfels() * sizeof(pbr::ren::RawPointCloud::SerializedSurfel),
                                       &(point_cloud_->data()[0]));



  std::vector<scm::gl::buffer_ptr> buffer_arrays;
  buffer_arrays.push_back(buffers_[ctx.id]);

  vertex_array_[ctx.id] = ctx.render_device->create_vertex_array(
      scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC3F, sizeof(pbr::ren::RawPointCloud::SerializedSurfel))(
          0, 1, scm::gl::TYPE_UBYTE, sizeof(pbr::ren::RawPointCloud::SerializedSurfel))(
          0, 2, scm::gl::TYPE_UBYTE, sizeof(pbr::ren::RawPointCloud::SerializedSurfel))(
          0, 3, scm::gl::TYPE_UBYTE, sizeof(pbr::ren::RawPointCloud::SerializedSurfel))(
          0, 4, scm::gl::TYPE_UBYTE, sizeof(pbr::ren::RawPointCloud::SerializedSurfel))(
          0, 5, scm::gl::TYPE_FLOAT, sizeof(pbr::ren::RawPointCloud::SerializedSurfel))(
          0, 6, scm::gl::TYPE_VEC3F, sizeof(pbr::ren::RawPointCloud::SerializedSurfel)),
      buffer_arrays);

}

////////////////////////////////////////////////////////////////////////////////

void PBRRessource::draw(RenderContext const& ctx) const {

  // upload to GPU if neccessary
  if (buffers_.size() <= ctx.id || buffers_[ctx.id] == nullptr) {
    upload_to(ctx);
  }

  scm::gl::context_vertex_input_guard vig(ctx.render_context);

  ctx.render_context->bind_vertex_array(vertex_array_[ctx.id]);

  ctx.render_context->apply();


  //ctx.render_context->draw_elements(point_cloud_->num_surfels());
}

////////////////////////////////////////////////////////////////////////////////

void PBRRessource::ray_test(Ray const& ray, PickResult::Options options,
                    Node* owner, std::set<PickResult>& hits) {

	return;
}


////////////////////////////////////////////////////////////////////////////////

/*virtual*/ GeometryUberShader* PBRRessource::get_ubershader() const {
  return Singleton<PBRUberShader>::instance();
}

}
