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
//#include <assimp/scene.h>

namespace {



struct Vertex {
  scm::math::vec3 pos;
  scm::math::vec2f tex;
  scm::math::vec3 normal;
  scm::math::vec3 tangent;
  scm::math::vec3 bitangent;
  scm::math::vec4f bone_weights;
  scm::math::vec4i bone_ids;
};
}

namespace gua {

void SkeletalAnimationRessource::VertexBoneData::AddBoneData(uint BoneID, float Weight)
{
    uint numWeights = (sizeof(IDs)/sizeof(IDs[0]));
    for (uint i = 0 ; i <  numWeights; i++) {
        if (Weights[i] == 0.0) {
            IDs[i]     = BoneID;
            Weights[i] = Weight;
            return;
        }        
    }
    // should never get here - more bones than we have space for
    Logger::LOG_WARNING << "Warning: Ignoring bone associated to vertex (more than " << numWeights << ")" << std::endl;
    //assert(false);
}

////////////////////////////////////////////////////////////////////////////////

SkeletalAnimationRessource::SkeletalAnimationRessource()
    : vertices_(), indices_(), vertex_array_(), upload_mutex_(), mesh_(nullptr){}

////////////////////////////////////////////////////////////////////////////////

SkeletalAnimationRessource::SkeletalAnimationRessource(aiMesh const* mesh,std::shared_ptr<SkeletalAnimationDirector> animation_director , std::shared_ptr<Assimp::Importer> const& importer,
           bool build_kd_tree)
    : vertices_(),
      indices_(),
      vertex_array_(),
      upload_mutex_(),
      mesh_(mesh),
      animation_director_(animation_director),
      importer_(importer){

  //TODO generate BBox and KDTree
  //if (mesh_->HasPositions()) {
  bounding_box_ = math::BoundingBox<math::vec3>();


  // without bone influence
  for (unsigned v(0); v < mesh->mNumVertices; ++v) {
    bounding_box_.expandBy(scm::math::vec3(
        mesh->mVertices[v].x, mesh->mVertices[v].y, mesh->mVertices[v].z));
  }


    // TODO
    /*if (build_kd_tree) {
      kd_tree_.generate(mesh_);
    }
  //}*/

}

////////////////////////////////////////////////////////////////////////////////

void SkeletalAnimationRessource::LoadBones(std::vector<VertexBoneData>& Bones)
{
  for (uint i = 0 ; i < mesh_->mNumBones ; i++) {

    std::string BoneName(mesh_->mBones[i]->mName.data);      
    uint BoneIndex = animation_director_->getBoneID(BoneName);        
    
    for (uint j = 0 ; j < mesh_->mBones[i]->mNumWeights ; j++) {
      //uint VertexID = entries_[MeshIndex].BaseVertex + mesh_->mBones[i]->mWeights[j].mVertexId;//no BaseVertex needed anymore???
      uint VertexID = mesh_->mBones[i]->mWeights[j].mVertexId;
      float Weight  = mesh_->mBones[i]->mWeights[j].mWeight;                   
      Bones[VertexID].AddBoneData(BoneIndex, Weight);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////

void SkeletalAnimationRessource::InitMesh(std::vector<scm::math::vec3>& Positions,
                    std::vector<scm::math::vec3>& Normals,
                    std::vector<scm::math::vec2>& TexCoords,
                    std::vector<scm::math::vec3>& Tangents,
                    std::vector<scm::math::vec3>& Bitangents,
                    std::vector<VertexBoneData>& Bones,
                    std::vector<uint>& Indices)
{    
  const scm::math::vec3 Zero3D(0.0f, 0.0f, 0.0f);
  
  // Populate the vertex attribute vectors
  for (uint i = 0 ; i < mesh_->mNumVertices ; i++) { // TODO catch: haspositions and hasnormals
    
    scm::math::vec3 pPos = scm::math::vec3(0.f, 0.f, 0.f);
    if(mesh_->HasPositions()) {
      pPos = scm::math::vec3(mesh_->mVertices[i].x,mesh_->mVertices[i].y,mesh_->mVertices[i].z);
    }

    scm::math::vec3 pNormal = scm::math::vec3(0.f, 0.f, 0.f);
    if(mesh_->HasNormals()) {
      pNormal = scm::math::vec3(mesh_->mNormals[i].x,mesh_->mNormals[i].y,mesh_->mNormals[i].z);
    }

    scm::math::vec3 pTexCoord = scm::math::vec3(0.0f,0.0f,0.0f);
    if(mesh_->HasTextureCoords(0)) {}
    {
      pTexCoord = scm::math::vec3(mesh_->mTextureCoords[0][i].x,mesh_->mTextureCoords[0][i].y,mesh_->mTextureCoords[0][i].z);
    }

    scm::math::vec3 pTangent = scm::math::vec3(0.f, 0.f, 0.f);
    scm::math::vec3 pBitangent = scm::math::vec3(0.f, 0.f, 0.f);
    if (mesh_->HasTangentsAndBitangents()) {
      pTangent = scm::math::vec3(mesh_->mTangents[i].x, mesh_->mTangents[i].y, mesh_->mTangents[i].z);

      pBitangent = scm::math::vec3(mesh_->mBitangents[i].x, mesh_->mBitangents[i].y, mesh_->mBitangents[i].z);
    }

    Positions.push_back(pPos);
    Normals.push_back(pNormal);
    Bitangents.push_back(pBitangent);
    Tangents.push_back(pTangent);
    TexCoords.push_back(scm::math::vec2(pTexCoord[0], pTexCoord[1]));
  }
  
  LoadBones(Bones);
  
  // Populate the index buffer
  for (uint i = 0 ; i < mesh_->mNumFaces ; i++) {
    const aiFace& Face = mesh_->mFaces[i];

    if(Face.mNumIndices != 3) {
      Logger::LOG_ERROR << "InitMesh - face doesnt have 3 vertices" << std::endl;
      assert(false);
    }

    /*Indices.push_back(Face.mIndices[0] + entries_[MeshIndex].BaseVertex);
    Indices.push_back(Face.mIndices[1] + entries_[MeshIndex].BaseVertex);
    Indices.push_back(Face.mIndices[2] + entries_[MeshIndex].BaseVertex);*/

    Indices.push_back(Face.mIndices[0]);
    Indices.push_back(Face.mIndices[1]);
    Indices.push_back(Face.mIndices[2]);
  }
}

////////////////////////////////////////////////////////////////////////////////

void SkeletalAnimationRessource::upload_to(RenderContext const& ctx) /*const*/{

  if (vertices_.size() <= ctx.id || vertices_[ctx.id] == nullptr) {

    //overide old bbox, now with bones' influence
    bounding_box_ = math::BoundingBox<math::vec3>();

    std::vector<scm::math::vec3> Positions;
    std::vector<scm::math::vec3> Normals;
    std::vector<scm::math::vec2> TexCoords;
    std::vector<scm::math::vec3> Tangents;
    std::vector<scm::math::vec3> Bitangents;
    std::vector<VertexBoneData> Bones;
    std::vector<uint> Indices;
       
    uint NumIndices = mesh_-> mNumFaces * 3;

    num_faces_ = NumIndices/3;
    num_vertices_ = mesh_-> mNumVertices;;
    
    // Reserve space in the vectors for the vertex attributes and indices
    Positions.reserve(num_vertices_);
    Normals.reserve(num_vertices_);
    TexCoords.reserve(num_vertices_);
    Tangents.reserve(num_vertices_);
    Bitangents.reserve(num_vertices_);
    Bones.resize(num_vertices_);
    Indices.reserve(NumIndices);
    
    InitMesh(Positions, Normals, TexCoords, Tangents, Bitangents, Bones, Indices);


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

      data[v].pos = scm::math::vec3( Positions[v].x, Positions[v].y, Positions[v].z);

      data[v].tex = scm::math::vec2(TexCoords[v].x, TexCoords[v].y);

      data[v].normal = scm::math::vec3(Normals[v].x, Normals[v].y, Normals[v].z);

      data[v].tangent = scm::math::vec3(Tangents[v].x, Tangents[v].y, Tangents[v].z);

      data[v].bitangent = scm::math::vec3(Bitangents[v].x, Bitangents[v].y, Bitangents[v].z);

      data[v].bone_weights = scm::math::vec4f(Bones[v].Weights[0],Bones[v].Weights[1],Bones[v].Weights[2],Bones[v].Weights[3]);
      
      data[v].bone_ids = scm::math::vec4i(Bones[v].IDs[0],Bones[v].IDs[1],Bones[v].IDs[2],Bones[v].IDs[3]);
    

      //expand initial bbox with bones' influence
      auto bone_transformation = animation_director_->get_bone_transforms();

      scm::math::mat4 BoneTransform =  bone_transformation[Bones[v].IDs[0]] * Bones[v].Weights[0];
         BoneTransform += bone_transformation[Bones[v].IDs[1]] * Bones[v].Weights[1];
         BoneTransform += bone_transformation[Bones[v].IDs[2]] * Bones[v].Weights[2];
         BoneTransform += bone_transformation[Bones[v].IDs[3]] * Bones[v].Weights[3];

      auto final_pos  =  BoneTransform * scm::math::vec4(Positions[v].x, Positions[v].y, Positions[v].z, 1.0);

      bounding_box_.expandBy(scm::math::vec3(final_pos.x,final_pos.y,final_pos.z));


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
            0, 4, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
            0, 5, scm::gl::TYPE_VEC4F, sizeof(Vertex))(
            0, 6, scm::gl::TYPE_VEC4I, sizeof(Vertex)),
        {vertices_[ctx.id]});

    
    ctx.render_context->apply();
  }

}

////////////////////////////////////////////////////////////////////////////////

void SkeletalAnimationRessource::draw(RenderContext const& ctx) /*const*/ {

  // upload to GPU if neccessary
  upload_to(ctx);

  ctx.render_context->bind_vertex_array(vertex_array_[ctx.id]);
  ctx.render_context->bind_index_buffer(indices_[ctx.id], scm::gl::PRIMITIVE_TRIANGLE_LIST, scm::gl::TYPE_UINT);
  ctx.render_context->apply_vertex_input();
  ctx.render_context->draw_elements(num_faces_ * 3);
}

////////////////////////////////////////////////////////////////////////////////

void SkeletalAnimationRessource::ray_test(Ray const& ray, int options,
                    node::Node* owner, std::set<PickResult>& hits) {
  //TODO raycasting
  Logger::LOG_ERROR << "get_vertex() dynamic ray testing not supported " << std::endl;
  //kd_tree_.ray_test(ray, mesh_, options, owner, hits);
}

////////////////////////////////////////////////////////////////////////////////

unsigned int SkeletalAnimationRessource::num_vertices() const { return num_vertices_; }

////////////////////////////////////////////////////////////////////////////////

unsigned int SkeletalAnimationRessource::num_faces() const { return num_faces_; }

////////////////////////////////////////////////////////////////////////////////

scm::math::vec3 SkeletalAnimationRessource::get_vertex(unsigned int i) const {

  //TODO physics handling
  Logger::LOG_ERROR << "get_vertex() dynamic vertex positions not supported " << std::endl;
  return scm::math::vec3();
}

////////////////////////////////////////////////////////////////////////////////

std::vector<unsigned int> SkeletalAnimationRessource::get_face(unsigned int i) const {

  //TODO cpu representation of mesh
  Logger::LOG_ERROR << "get_face() of merged neshes not supported " << std::endl;
  /*std::vector<unsigned int> face(mesh_->mFaces[i].mNumIndices);
  for (unsigned int j = 0; j < mesh_->mFaces[i].mNumIndices; ++j)
    face[j] = mesh_->mFaces[i].mIndices[j];
  return face;*/
  return std::vector<unsigned int>();
}

////////////////////////////////////////////////////////////////////////////////
}