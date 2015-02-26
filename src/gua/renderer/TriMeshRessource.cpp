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
#include <gua/renderer/TriMeshRessource.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/node/TriMeshNode.hpp>
#include <gua/utils/Logger.hpp>

// external headers
#include <assimp/postprocess.h>
#include <assimp/scene.h>

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

TriMeshRessource::TriMeshRessource()
    : vertices_(), indices_(), vertex_array_(), upload_mutex_(), ai_mesh_(nullptr), mesh_() {}

////////////////////////////////////////////////////////////////////////////////

TriMeshRessource::TriMeshRessource(aiMesh* mesh, std::shared_ptr<Assimp::Importer> const& importer,
           bool build_kd_tree)
    : vertices_(),
      indices_(),
      vertex_array_(),
      upload_mutex_(),
      mesh_(*mesh),
      ai_mesh_(mesh),
      importer_(importer) {

  if (mesh_.num_vertices > 0) {
    bounding_box_ = math::BoundingBox<math::vec3>();

    for (unsigned v(0); v < mesh_.num_vertices; ++v) {
      bounding_box_.expandBy(mesh_.positions[v]);
    }

    if (build_kd_tree) {
      kd_tree_.generate(ai_mesh_);
    }
  }
}
////////////////////////////////////////////////////////////////////////////////

TriMeshRessource::TriMeshRessource(Mesh const& mesh)
    : vertices_(),
      indices_(),
      vertex_array_(),
      upload_mutex_(),
      mesh_(mesh),
      ai_mesh_(nullptr),
      importer_(nullptr) {

  if (mesh_.num_vertices > 0) {
    bounding_box_ = math::BoundingBox<math::vec3>();

    for (unsigned v(0); v < mesh_.num_vertices; ++v) {
      bounding_box_.expandBy(mesh_.positions[v]);
    }    
  }
}

////////////////////////////////////////////////////////////////////////////////

void TriMeshRessource::upload_to(RenderContext const& ctx) const {

  if (vertices_.size() <= ctx.id || vertices_[ctx.id] == nullptr) {
    if (!mesh_.num_vertices > 0) {
      Logger::LOG_WARNING << "Unable to load Mesh! Has no vertex data." << std::endl;
      return;
    }

    std::unique_lock<std::mutex> lock(upload_mutex_);

    if (vertices_.size() <= ctx.id) {
      vertices_.resize(ctx.id + 1);
      indices_.resize(ctx.id + 1);
      vertex_array_.resize(ctx.id + 1);
    }

    vertices_[ctx.id] =
        ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                                         scm::gl::USAGE_STATIC_DRAW,
                                         mesh_.num_vertices * sizeof(Vertex),
                                         0);



    Vertex* data(static_cast<Vertex*>(ctx.render_context->map_buffer(
        vertices_[ctx.id], scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER)));

    mesh_.copy_to_buffer_static(data);

    ctx.render_context->unmap_buffer(vertices_[ctx.id]);

    indices_[ctx.id] =
        ctx.render_device->create_buffer(scm::gl::BIND_INDEX_BUFFER,
                                         scm::gl::USAGE_STATIC_DRAW,
                                         mesh_.num_triangles * 3 * sizeof(unsigned),
                                         &mesh_.indices[0]);

    vertex_array_[ctx.id] = ctx.render_device->create_vertex_array(
        scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
            0, 1, scm::gl::TYPE_VEC2F, sizeof(Vertex))(
            0, 2, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
            0, 3, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
            0, 4, scm::gl::TYPE_VEC3F, sizeof(Vertex)),
        {vertices_[ctx.id]});

    ctx.render_context->apply();
  }
}

////////////////////////////////////////////////////////////////////////////////

void TriMeshRessource::draw(RenderContext const& ctx) const {

  // upload to GPU if neccessary
  upload_to(ctx);

  ctx.render_context->bind_vertex_array(vertex_array_[ctx.id]);
  ctx.render_context->bind_index_buffer(indices_[ctx.id], scm::gl::PRIMITIVE_TRIANGLE_LIST, scm::gl::TYPE_UINT);
  ctx.render_context->apply_vertex_input();
  ctx.render_context->draw_elements(mesh_.num_triangles * 3);
}

////////////////////////////////////////////////////////////////////////////////

void TriMeshRessource::ray_test(Ray const& ray, int options,
                    node::Node* owner, std::set<PickResult>& hits) {
  kd_tree_.ray_test(ray, ai_mesh_, options, owner, hits);
}

////////////////////////////////////////////////////////////////////////////////

unsigned int TriMeshRessource::num_vertices() const { return mesh_.num_vertices; }

////////////////////////////////////////////////////////////////////////////////

unsigned int TriMeshRessource::num_faces() const { return mesh_.num_triangles; }

////////////////////////////////////////////////////////////////////////////////

math::vec3 TriMeshRessource::get_vertex(unsigned int i) const {

  return math::vec3(
      mesh_.positions[i].x, mesh_.positions[i].y, mesh_.positions[i].z);
}

////////////////////////////////////////////////////////////////////////////////

std::vector<unsigned int> TriMeshRessource::get_face(unsigned int i) const {

  std::vector<unsigned int> face{mesh_.indices[i]};
  
  return face;
}

////////////////////////////////////////////////////////////////////////////////

}
