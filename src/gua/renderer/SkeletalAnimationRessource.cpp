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
#include <gua/renderer/SkeletalAnimationRessource.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/node/SkeletalAnimationNode.hpp>
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

SkeletalAnimationRessource::SkeletalAnimationRessource()
    : vertices_(), indices_(), vertex_array_(), upload_mutex_(), scene_(nullptr) {}

////////////////////////////////////////////////////////////////////////////////

SkeletalAnimationRessource::SkeletalAnimationRessource(aiScene const* scene, std::shared_ptr<Assimp::Importer> const& importer,
           bool build_kd_tree)
    : vertices_(),
      indices_(),
      vertex_array_(),
      upload_mutex_(),
      scene_(scene),
      importer_(importer) {


  //if (mesh_->HasPositions()) {
  bounding_box_ = math::BoundingBox<math::vec3>();

  for (unsigned i(0); i < scene_->mNumMeshes; ++i){

    for (unsigned v(0); v < scene->mMeshes[i]->mNumVertices; ++v) {
      bounding_box_.expandBy(scm::math::vec3(
          scene->mMeshes[i]->mVertices[v].x, scene->mMeshes[i]->mVertices[v].y, scene->mMeshes[i]->mVertices[v].z));
    }

  }

    // TODO
    /*if (build_kd_tree) {
      kd_tree_.generate(mesh_);
    }
  //}*/
}

////////////////////////////////////////////////////////////////////////////////

void SkeletalAnimationRessource::InitMesh(uint MeshIndex,
                    const aiMesh* paiMesh,
                    std::vector<scm::math::vec3>& Positions,
                    std::vector<scm::math::vec3>& Normals,
                    std::vector<scm::math::vec2>& TexCoords,
                    /*std::vector<VertexBoneData>& Bones,*/
                    std::vector<uint>& Indices)
{    
    const scm::math::vec3 Zero3D(0.0f, 0.0f, 0.0f);
    
    // Populate the vertex attribute vectors
    for (uint i = 0 ; i < paiMesh->mNumVertices ; i++) { // TODO catch: haspositions and hasnormals
        scm::math::vec3 pPos      = scm::math::vec3(paiMesh->mVertices[i][0],paiMesh->mVertices[i][1],paiMesh->mVertices[i][2]);
        scm::math::vec3 pNormal   = scm::math::vec3(paiMesh->mNormals[i][0],paiMesh->mNormals[i][1],paiMesh->mNormals[i][2]);
        scm::math::vec3 pTexCoord = paiMesh->HasTextureCoords(0) ? scm::math::vec3(paiMesh->mTextureCoords[0][i][0],paiMesh->mTextureCoords[0][i][1],paiMesh->mTextureCoords[0][i][2]) : scm::math::vec3(0,0,0);

        Positions.push_back(pPos);
        Normals.push_back(pNormal);
        TexCoords.push_back(scm::math::vec2(pTexCoord[0], pTexCoord[1]));        
    }
    
    //TODO
    //LoadBones(MeshIndex, paiMesh, Bones);
    
    // Populate the index buffer
    for (uint i = 0 ; i < paiMesh->mNumFaces ; i++) {
        const aiFace& Face = paiMesh->mFaces[i];
        assert(Face.mNumIndices == 3);
        Indices.push_back(Face.mIndices[0] + entries_[MeshIndex].BaseVertex);
        Indices.push_back(Face.mIndices[1] + entries_[MeshIndex].BaseVertex);
        Indices.push_back(Face.mIndices[2] + entries_[MeshIndex].BaseVertex);
    }
}

////////////////////////////////////////////////////////////////////////////////

void SkeletalAnimationRessource::upload_to(RenderContext const& ctx) /*const*/{

  
  if (vertices_.size() <= ctx.id || vertices_[ctx.id] == nullptr) {

    //new
    ///////////////////////////////////////////////////////////////////////

    entries_.resize(scene_->mNumMeshes);

    std::vector<scm::math::vec3> Positions;
    std::vector<scm::math::vec3> Normals;
    std::vector<scm::math::vec2> TexCoords;
    //std::vector<VertexBoneData> Bones; // TODO
    std::vector<uint> Indices;
       
    uint NumVertices = 0;
    uint NumIndices = 0;
    
    // Count the number of vertices and indices
    for (uint i = 0 ; i < entries_.size() ; i++) {
        entries_[i].MaterialIndex = scene_->mMeshes[i]->mMaterialIndex;        
        entries_[i].NumIndices    = scene_->mMeshes[i]->mNumFaces * 3;
        entries_[i].BaseVertex    = NumVertices;
        entries_[i].BaseIndex     = NumIndices;
        
        NumVertices += scene_->mMeshes[i]->mNumVertices;
        NumIndices  += entries_[i].NumIndices;
    }

    num_faces_ = NumIndices/3;
    num_vertices_ = NumVertices;
    
    // Reserve space in the vectors for the vertex attributes and indices
    Positions.reserve(NumVertices);
    Normals.reserve(NumVertices);
    TexCoords.reserve(NumVertices);
    //Bones.resize(NumVertices); // TODO
    Indices.reserve(NumIndices);
        
    // Initialize the meshes in the scene one by one
    for (uint i = 0 ; i < entries_.size() ; i++) {
        const aiMesh* paiMesh = scene_->mMeshes[i];
        InitMesh(i, paiMesh, Positions, Normals, TexCoords, /*Bones,*/ Indices);
    }

    ///////////////////////////////////////////////////////////////////////

    std::unique_lock<std::mutex> lock(upload_mutex_);

    if (vertices_.size() <= ctx.id) {
      vertices_.resize(ctx.id + 1);
      indices_.resize(ctx.id + 1);
      vertex_array_.resize(ctx.id + 1);
    }

    vertices_[ctx.id] =
        ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                                         scm::gl::USAGE_STATIC_DRAW,
                                         num_vertices_ * sizeof(Vertex),
                                         0);


    Vertex* data(static_cast<Vertex*>(ctx.render_context->map_buffer(
        vertices_[ctx.id], scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER)));

    for (unsigned v(0); v < num_vertices_; ++v) {
      data[v].pos = scm::math::vec3(
          Positions[v].x, Positions[v].y, Positions[v].z);

      //if (mesh_->HasTextureCoords(0)) {
      data[v].tex = scm::math::vec2(TexCoords[v].x,
                                    TexCoords[v].y);
      //} else {
      //  data[v].tex = scm::math::vec2(0.f, 0.f);
      //}

      //if (mesh_->HasNormals()) {
      data[v].normal = scm::math::vec3(
            Normals[v].x, Normals[v].y, Normals[v].z);
      //} else {
      //  data[v].normal = scm::math::vec3(0.f, 0.f, 0.f);
      //}

      //if (mesh_->HasTangentsAndBitangents()) {
      /*if (false) {                              // TODO!!!!!!!!!!!!!!!!!!!!!
        data[v].tangent = scm::math::vec3(
            mesh_->mTangents[v].x, mesh_->mTangents[v].y, mesh_->mTangents[v].z);

        data[v].bitangent = scm::math::vec3(mesh_->mBitangents[v].x,
                                            mesh_->mBitangents[v].y,
                                            mesh_->mBitangents[v].z);
      } else {*/
      data[v].tangent = scm::math::vec3(0.f, 0.f, 0.f);
      data[v].bitangent = scm::math::vec3(0.f, 0.f, 0.f);
      //}
    }

    ctx.render_context->unmap_buffer(vertices_[ctx.id]);

    indices_[ctx.id] =
        ctx.render_device->create_buffer(scm::gl::BIND_INDEX_BUFFER,
                                         scm::gl::USAGE_STATIC_DRAW,
                                         num_faces_ * 3 * sizeof(unsigned),
                                         &Indices[0]);

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

void SkeletalAnimationRessource::draw(RenderContext const& ctx) /*const*/ {

  // upload to GPU if neccessary
  upload_to(ctx);

  //TODO: update bone transoformations

  ctx.render_context->bind_vertex_array(vertex_array_[ctx.id]);
  ctx.render_context->bind_index_buffer(indices_[ctx.id], scm::gl::PRIMITIVE_TRIANGLE_LIST, scm::gl::TYPE_UINT);
  ctx.render_context->apply_vertex_input();
  ctx.render_context->draw_elements(num_faces_ * 3);
}

////////////////////////////////////////////////////////////////////////////////

void SkeletalAnimationRessource::ray_test(Ray const& ray, int options,
                    node::Node* owner, std::set<PickResult>& hits) {
  //TODO
  //kd_tree_.ray_test(ray, mesh_, options, owner, hits);
}

////////////////////////////////////////////////////////////////////////////////

unsigned int SkeletalAnimationRessource::num_vertices() const { return num_vertices_; }

////////////////////////////////////////////////////////////////////////////////

unsigned int SkeletalAnimationRessource::num_faces() const { return num_faces_; }

////////////////////////////////////////////////////////////////////////////////

scm::math::vec3 SkeletalAnimationRessource::get_vertex(unsigned int i) const {

  // TODO
  /*return scm::math::vec3(
      mesh_->mVertices[i].x, mesh_->mVertices[i].y, mesh_->mVertices[i].z);*/
  return scm::math::vec3();
}

////////////////////////////////////////////////////////////////////////////////

std::vector<unsigned int> SkeletalAnimationRessource::get_face(unsigned int i) const {

  //TODO
  /*std::vector<unsigned int> face(mesh_->mFaces[i].mNumIndices);
  for (unsigned int j = 0; j < mesh_->mFaces[i].mNumIndices; ++j)
    face[j] = mesh_->mFaces[i].mIndices[j];
  return face;*/
  return std::vector<unsigned int>();
}

////////////////////////////////////////////////////////////////////////////////

}
