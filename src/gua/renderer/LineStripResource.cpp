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
#include <gua/renderer/LineStripResource.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/node/LineStripNode.hpp>
#include <gua/utils/Logger.hpp>

#include <scm/gl_core/constants.h>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

LineStripResource::LineStripResource()
    : kd_tree_(), line_strip_() {}

////////////////////////////////////////////////////////////////////////////////

LineStripResource::LineStripResource(LineStrip const& line_strip, bool build_kd_tree)
    : kd_tree_(), line_strip_(line_strip) {
  std::cout << "Started creating lsResource\n";
  if (line_strip_.num_occupied_vertex_slots > 0) {
    bounding_box_ = math::BoundingBox<math::vec3>();

    for (int v(0); v < line_strip_.num_occupied_vertex_slots; ++v) {
        std::cout << "accessing index: " << v << "\n";
      bounding_box_.expandBy(math::vec3{line_strip_.positions[v]});
    }

    if (build_kd_tree) {
      //kd_tree_.generate(line_strip);
    }
  }

  std::cout << "DONE LOADING RESOURCE\n";
}

////////////////////////////////////////////////////////////////////////////////

void LineStripResource::upload_to(RenderContext& ctx) const {
  RenderContext::LineStrip clinestrip{};
  clinestrip.vertex_topology = scm::gl::PRIMITIVE_LINE_STRIP_ADJACENCY;
  clinestrip.vertex_reservoir_size = line_strip_.vertex_reservoir_size;
  clinestrip.num_occupied_vertex_slots = line_strip_.num_occupied_vertex_slots;


  if (line_strip_.vertex_reservoir_size == 0) {
    Logger::LOG_WARNING << "Unable to load LineStrip! Has no vertex data." << std::endl;
    return;
  }

 
  clinestrip.vertices =
      ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                                       scm::gl::USAGE_DYNAMIC_DRAW,
                                       (line_strip_.vertex_reservoir_size+2) * sizeof(LineStrip::Vertex),
                                       0);

  LineStrip::Vertex* data(static_cast<LineStrip::Vertex*>(ctx.render_context->map_buffer(
      clinestrip.vertices, scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER)));

  line_strip_.copy_to_buffer(data);

  //std::cout << buffer_content << ""
  ctx.render_context->unmap_buffer(clinestrip.vertices);

  clinestrip.vertex_array = ctx.render_device->create_vertex_array(
      line_strip_.get_vertex_format(),
      {clinestrip.vertices});
  ctx.line_strips[uuid()] = clinestrip;

  ctx.render_context->apply();
}

////////////////////////////////////////////////////////////////////////////////

void LineStripResource::draw(RenderContext& ctx) const {
  //DUMMY
}


////////////////////////////////////////////////////////////////////////////////

void LineStripResource::draw(RenderContext& ctx, bool render_vertices_as_points) const {
  auto iter = ctx.line_strips.find(uuid());
  if (iter == ctx.line_strips.end()) {
    // upload to GPU if neccessary
    upload_to(ctx);
    iter = ctx.line_strips.find(uuid());
  }
  


  ctx.render_context->bind_vertex_array(iter->second.vertex_array);
  //ctx.render_context->bind_index_buffer(iter->second.indices, iter->second.indices_topology, iter->second.indices_type);
  ctx.render_context->apply_vertex_input();
  
  if(!render_vertices_as_points) {
    ctx.render_context->draw_arrays(iter->second.vertex_topology, 0, iter->second.num_occupied_vertex_slots+2);
  } else {
    ctx.render_context->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, 1, iter->second.num_occupied_vertex_slots);
  }
  //ctx.render_context->draw_elements(iter->second.indices_count);
  std::cout << "DREW THE LINES\n";
  
}


////////////////////////////////////////////////////////////////////////////////


void LineStripResource::ray_test(Ray const& ray, int options,
                    node::Node* owner, std::set<PickResult>& hits) {

  //kd_tree_.ray_test(ray, line_strip_, options, owner, hits);
}

////////////////////////////////////////////////////////////////////////////////

math::vec3 LineStripResource::get_vertex(unsigned int i) const {
  return math::vec3(
      line_strip_.positions[i].x, line_strip_.positions[i].y, line_strip_.positions[i].z);
}

////////////////////////////////////////////////////////////////////////////////

}
