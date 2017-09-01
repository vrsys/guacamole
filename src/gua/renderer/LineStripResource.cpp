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

namespace gua {

////////////////////////////////////////////////////////////////////////////////

LineStripResource::LineStripResource()
    : kd_tree_(), line_strip_() {}

////////////////////////////////////////////////////////////////////////////////

LineStripResource::LineStripResource(LineStrip const& line_strip, bool build_kd_tree)
    : kd_tree_(), line_strip_(line_strip) {

  if (line_strip_.num_occupied_vertex_slots > 0) {
    bounding_box_ = math::BoundingBox<math::vec3>();

    for (unsigned v(0); v < line_strip_.num_occupied_vertex_slots; ++v) {
      bounding_box_.expandBy(math::vec3{line_strip_.positions[v]});
    }

    if (build_kd_tree) {
      //kd_tree_.generate(line_strip);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////

void LineStripResource::upload_to(RenderContext& ctx) const {
  RenderContext::Mesh cmesh{};
  cmesh.indices_topology = scm::gl::PRIMITIVE_LINE_STRIP_ADJACENCY;
  cmesh.indices_type = scm::gl::TYPE_UINT;
  cmesh.indices_count = mesh_.num_triangles * 3;

  if (!mesh_.num_vertices > 0) {
    Logger::LOG_WARNING << "Unable to load Mesh! Has no vertex data." << std::endl;
    return;
  }

  cmesh.vertices =
      ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       mesh_.num_vertices * sizeof(Mesh::Vertex),
                                       0);

  Mesh::Vertex* data(static_cast<Mesh::Vertex*>(ctx.render_context->map_buffer(
      cmesh.vertices, scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER)));

  mesh_.copy_to_buffer(data);

  ctx.render_context->unmap_buffer(cmesh.vertices);

  cmesh.indices =
      ctx.render_device->create_buffer(scm::gl::BIND_INDEX_BUFFER,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       mesh_.num_triangles * 3 * sizeof(unsigned),
                                       mesh_.indices.data());

  cmesh.vertex_array = ctx.render_device->create_vertex_array(
      mesh_.get_vertex_format(),
      {cmesh.vertices});
  ctx.meshes[uuid()] = cmesh;

  ctx.render_context->apply();
}

////////////////////////////////////////////////////////////////////////////////

void TriMeshRessource::draw(RenderContext& ctx) const {
  auto iter = ctx.meshes.find(uuid());
  if (iter == ctx.meshes.end()) {
    // upload to GPU if neccessary
    upload_to(ctx);
    iter = ctx.meshes.find(uuid());
  }
  ctx.render_context->bind_vertex_array(iter->second.vertex_array);
  ctx.render_context->bind_index_buffer(iter->second.indices, iter->second.indices_topology, iter->second.indices_type);
  ctx.render_context->apply_vertex_input();
  ctx.render_context->draw_elements(iter->second.indices_count);
}

////////////////////////////////////////////////////////////////////////////////

void TriMeshRessource::ray_test(Ray const& ray, int options,
                    node::Node* owner, std::set<PickResult>& hits) {
  kd_tree_.ray_test(ray, mesh_, options, owner, hits);
}

////////////////////////////////////////////////////////////////////////////////

math::vec3 TriMeshRessource::get_vertex(unsigned int i) const {
  return math::vec3(
      mesh_.positions[i].x, mesh_.positions[i].y, mesh_.positions[i].z);
}

////////////////////////////////////////////////////////////////////////////////

std::vector<unsigned int> TriMeshRessource::get_face(unsigned int i) const {
  std::vector<unsigned int> face;
  face.push_back(mesh_.indices[3*i]);
  face.push_back(mesh_.indices[3*i+1]);
  face.push_back(mesh_.indices[3*i+2]);
  return face;
}

////////////////////////////////////////////////////////////////////////////////

}
