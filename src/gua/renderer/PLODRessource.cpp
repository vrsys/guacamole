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
#include <gua/renderer/PLODRessource.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/utils/Logger.hpp>

// external headers

#include <iostream>


namespace gua {

////////////////////////////////////////////////////////////////////////////////

PLODRessource::PLODRessource()
    : upload_mutex_() {}

////////////////////////////////////////////////////////////////////////////////

PLODRessource::PLODRessource(const pbr::ren::LodPointCloud* point_cloud)
    : upload_mutex_() {


     //set already created BB
  	 scm::gl::boxf loaded_bb = point_cloud->kdn_tree()->bounding_boxes()[0];
  	 bounding_box_.min = loaded_bb.min_vertex();
 	   bounding_box_.max = loaded_bb.max_vertex();
}

////////////////////////////////////////////////////////////////////////////////

void PLODRessource::upload_to(RenderContext const& ctx) const {
/*

  if (!point_cloud_->is_loaded()) {
    Logger::LOG_WARNING << "Point Cloud was not loaded!" << std::endl;
    return;
  }

  std::unique_lock<std::mutex> lock(upload_mutex_);

  if (buffers_.size() <= ctx.id) {
    buffers_.resize(ctx.id + 1);
    vertex_array_.resize(ctx.id + 1);
  }


  buffers_[ctx.id] =
      ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       point_cloud_->num_surfels() * sizeof(pbr::ren::RawPointCloud::SerializedSurfel),
                                       &(point_cloud_->data()[0]));


  std::vector<scm::gl::buffer_ptr> buffer_arrays;
  buffer_arrays.push_back(buffers_[ctx.id]);


  vertex_array_[ctx.id] = ctx.render_device->create_vertex_array(
      scm::gl::vertex_format(
          0, 0, scm::gl::TYPE_VEC3F, sizeof(pbr::ren::RawPointCloud::SerializedSurfel))(
          0, 1, scm::gl::TYPE_UBYTE, sizeof(pbr::ren::RawPointCloud::SerializedSurfel))(
          0, 2, scm::gl::TYPE_UBYTE, sizeof(pbr::ren::RawPointCloud::SerializedSurfel))(
          0, 3, scm::gl::TYPE_UBYTE, sizeof(pbr::ren::RawPointCloud::SerializedSurfel))(
          0, 4, scm::gl::TYPE_UBYTE, sizeof(pbr::ren::RawPointCloud::SerializedSurfel))(
          0, 5, scm::gl::TYPE_FLOAT, sizeof(pbr::ren::RawPointCloud::SerializedSurfel))(
          0, 6, scm::gl::TYPE_VEC3F, sizeof(pbr::ren::RawPointCloud::SerializedSurfel)),
       buffer_arrays);
*/
}

////////////////////////////////////////////////////////////////////////////////
void PLODRessource::draw(RenderContext const& ctx) const
{/*dummy*/}
////////////////////////////////////////////////////////////////////////////////

void PLODRessource::draw(RenderContext const& ctx, pbr::context_t context_id, pbr::view_t view_id, pbr::model_t model_id, scm::gl::vertex_array_ptr const& vertex_array) const 
{

    pbr::ren::ModelDatabase* database = pbr::ren::ModelDatabase::GetInstance();
    pbr::ren::CutDatabase* cuts = pbr::ren::CutDatabase::GetInstance();
    
    pbr::ren::Cut& cut = cuts->GetCut(context_id, view_id, model_id);
    std::vector<pbr::ren::Cut::NodeSlotAggregate> node_list = cut.complete_set();
    const pbr::ren::KdnTree* kdn_tree = database->GetModel(model_id)->kdn_tree();

    uint32_t surfels_per_node = database->surfels_per_node();
    uint32_t surfels_per_node_of_model = kdn_tree->surfels_per_node();


  //if (buffers_.size() <= ctx.id || buffers_[ctx.id] == nullptr) {
  //  upload_to(ctx);
  //}

  scm::gl::context_vertex_input_guard vig(ctx.render_context);

  ctx.render_context->bind_vertex_array(vertex_array);

  ctx.render_context->apply();
  
  pbr::node_t node_counter = 0;
  
  for(std::vector<pbr::ren::Cut::NodeSlotAggregate>::const_iterator k = node_list.begin(); k != node_list.end(); ++k, ++node_counter)
  {
      ctx.render_context->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, (k->slot_id_) * surfels_per_node, surfels_per_node_of_model);
  
  }

}

////////////////////////////////////////////////////////////////////////////////

void PLODRessource::ray_test(Ray const& ray, PickResult::Options options,
                    Node* owner, std::set<PickResult>& hits) {

	return;
}


////////////////////////////////////////////////////////////////////////////////

/*virtual*/ GeometryUberShader* PLODRessource::get_ubershader() const {
  return Singleton<PLODUberShader>::instance();
}

}
