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
#include <gua/utils/Logger.hpp>

// external headers
#if ASSIMP_VERSION == 3
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#else
#include <assimp/assimp.hpp>
#include <assimp/aiPostProcess.h>
#include <assimp/aiScene.h>
#endif

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
    : vertices_(), indices_(), vertex_array_(), upload_mutex_(), mesh_(nullptr) {}

////////////////////////////////////////////////////////////////////////////////

TriMeshRessource::TriMeshRessource(aiMesh* mesh, std::shared_ptr<Assimp::Importer> const& importer,
           bool build_kd_tree)
    : vertices_(),
      indices_(),
      vertex_array_(),
      upload_mutex_(),
      mesh_(mesh),
      importer_(importer) {

  if (mesh_->HasPositions()) {
    bounding_box_ = math::BoundingBox<math::vec3>();

    for (unsigned v(0); v < mesh_->mNumVertices; ++v) {
      bounding_box_.expandBy(scm::math::vec3(
          mesh_->mVertices[v].x, mesh_->mVertices[v].y, mesh_->mVertices[v].z));
    }

    if (build_kd_tree) {
      kd_tree_.generate(mesh_);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////

void TriMeshRessource::upload_to(RenderContext const& ctx) const {

  if (!mesh_->HasPositions()) {
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
                                       mesh_->mNumVertices * sizeof(Vertex),
                                       0);



  Vertex* data(static_cast<Vertex*>(ctx.render_context->map_buffer(
      vertices_[ctx.id], scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER)));

  for (unsigned v(0); v < mesh_->mNumVertices; ++v) {
    data[v].pos = scm::math::vec3(
        mesh_->mVertices[v].x, mesh_->mVertices[v].y, mesh_->mVertices[v].z);

    if (mesh_->HasTextureCoords(0)) {
      data[v].tex = scm::math::vec2(mesh_->mTextureCoords[0][v].x,
                                    mesh_->mTextureCoords[0][v].y);
    } else {
      data[v].tex = scm::math::vec2(0.f, 0.f);
    }

    if (mesh_->HasNormals()) {
      data[v].normal = scm::math::vec3(
          mesh_->mNormals[v].x, mesh_->mNormals[v].y, mesh_->mNormals[v].z);
    } else {
      data[v].normal = scm::math::vec3(0.f, 0.f, 0.f);
    }

    if (mesh_->HasTangentsAndBitangents()) {
      data[v].tangent = scm::math::vec3(
          mesh_->mTangents[v].x, mesh_->mTangents[v].y, mesh_->mTangents[v].z);

      data[v].bitangent = scm::math::vec3(mesh_->mBitangents[v].x,
                                          mesh_->mBitangents[v].y,
                                          mesh_->mBitangents[v].z);
    } else {
      data[v].tangent = scm::math::vec3(0.f, 0.f, 0.f);
      data[v].bitangent = scm::math::vec3(0.f, 0.f, 0.f);
    }
  }

  ctx.render_context->unmap_buffer(vertices_[ctx.id]);

  std::vector<unsigned> index_array(mesh_->mNumFaces * 3);

  for (unsigned t = 0; t < mesh_->mNumFaces; ++t) {
    const struct aiFace* face = &mesh_->mFaces[t];

    index_array[t * 3] = face->mIndices[0];
    index_array[t * 3 + 1] = face->mIndices[1];
    index_array[t * 3 + 2] = face->mIndices[2];
  }

  indices_[ctx.id] =
      ctx.render_device->create_buffer(scm::gl::BIND_INDEX_BUFFER,
                                       scm::gl::USAGE_STATIC_DRAW,
                                       mesh_->mNumFaces * 3 * sizeof(unsigned),
                                       &index_array[0]);

  std::vector<scm::gl::buffer_ptr> buffer_arrays;
  buffer_arrays.push_back(vertices_[ctx.id]);

  vertex_array_[ctx.id] = ctx.render_device->create_vertex_array(
      scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
          0, 1, scm::gl::TYPE_VEC2F, sizeof(Vertex))(
          0, 2, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
          0, 3, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
          0, 4, scm::gl::TYPE_VEC3F, sizeof(Vertex)),
      buffer_arrays);

}

////////////////////////////////////////////////////////////////////////////////

void TriMeshRessource::draw(RenderContext const& ctx) const {

  // upload to GPU if neccessary
  if (vertices_.size() <= ctx.id || vertices_[ctx.id] == nullptr) {
    upload_to(ctx);
  }

  scm::gl::context_vertex_input_guard vig(ctx.render_context);

  ctx.render_context->bind_vertex_array(vertex_array_[ctx.id]);

  ctx.render_context->bind_index_buffer(
      indices_[ctx.id], scm::gl::PRIMITIVE_TRIANGLE_LIST, scm::gl::TYPE_UINT);

  ctx.render_context->apply();
  ctx.render_context->draw_elements(mesh_->mNumFaces * 3);
}

////////////////////////////////////////////////////////////////////////////////

void TriMeshRessource::ray_test(Ray const& ray, PickResult::Options options,
                    Node* owner, std::set<PickResult>& hits) {

  kd_tree_.ray_test(ray, mesh_, options, owner, hits);
}

////////////////////////////////////////////////////////////////////////////////

unsigned int TriMeshRessource::num_vertices() const { return mesh_->mNumVertices; }

////////////////////////////////////////////////////////////////////////////////

unsigned int TriMeshRessource::num_faces() const { return mesh_->mNumFaces; }

////////////////////////////////////////////////////////////////////////////////

scm::math::vec3 TriMeshRessource::get_vertex(unsigned int i) const {

  return scm::math::vec3(
      mesh_->mVertices[i].x, mesh_->mVertices[i].y, mesh_->mVertices[i].z);
}

////////////////////////////////////////////////////////////////////////////////

std::vector<unsigned int> TriMeshRessource::get_face(unsigned int i) const {

  std::vector<unsigned int> face(mesh_->mFaces[i].mNumIndices);
  for (unsigned int j = 0; j < mesh_->mFaces[i].mNumIndices; ++j)
    face[j] = mesh_->mFaces[i].mIndices[j];
  return face;
}

////////////////////////////////////////////////////////////////////////////////

/*virtual*/ std::shared_ptr<GeometryUberShader> TriMeshRessource::create_ubershader() const {
  return std::make_shared<TriMeshUberShader>();
}

}
