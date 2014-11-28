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

#define ARRAY_SIZE_IN_ELEMENTS(a) (sizeof(a)/sizeof(a[0]))

namespace {

scm::math::mat4f to_gua(aiMatrix4x4 const& m){
  scm::math::mat4f res(m.a1,m.b1,m.c1,m.d1
                      ,m.a2,m.b2,m.c2,m.d2
                      ,m.a3,m.b3,m.c3,m.d3
                      ,m.a4,m.b4,m.c4,m.d4);
  return res;
}

scm::math::vec3 to_gua(aiVector3D const& v){
  scm::math::vec3 res(v.x, v.y, v.z);
  return res;
}

scm::math::quatf to_gua(aiQuaternion const& q){
  scm::math::quatf res(q.w, q.x, q.y, q.z);
  return res;
}


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
    for (uint i = 0 ; i < ARRAY_SIZE_IN_ELEMENTS(IDs) ; i++) {
        if (Weights[i] == 0.0) {
            IDs[i]     = BoneID;
            Weights[i] = Weight;
            return;
        }        
    }
    std::cout << "Warning: Ignoring bone associated to vertex (more than " << ARRAY_SIZE_IN_ELEMENTS(IDs) << ")" << std::endl;
    // should never get here - more bones than we have space for
    //assert(0);
}

////////////////////////////////////////////////////////////////////////////////

SkeletalAnimationRessource::SkeletalAnimationRessource()
    : vertices_(), indices_(), vertex_array_(), upload_mutex_(), scene_(nullptr), bone_transforms_block_(nullptr), timer_() {timer_.start();}

////////////////////////////////////////////////////////////////////////////////

SkeletalAnimationRessource::SkeletalAnimationRessource(aiScene const* scene, std::shared_ptr<Assimp::Importer> const& importer,
           bool build_kd_tree)
    : vertices_(),
      indices_(),
      vertex_array_(),
      upload_mutex_(),
      scene_(scene),
      importer_(importer),
      bone_transforms_block_(nullptr),
      num_bones_(0),
      timer_() {


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

  timer_.start();
}

////////////////////////////////////////////////////////////////////////////////

void SkeletalAnimationRessource::LoadBones(uint MeshIndex, const aiMesh* pMesh, std::vector<VertexBoneData>& Bones)
{

    for (uint i = 0 ; i < pMesh->mNumBones ; i++) {

        uint BoneIndex = 0;        
        std::string BoneName(pMesh->mBones[i]->mName.data);      

        
        if (bone_mapping_.find(BoneName) == bone_mapping_.end()) {

            // Allocate an index for a new bone
            BoneIndex = num_bones_;
            num_bones_++;            
            BoneInfo bi;      

            bone_info_.push_back(bi);
            bone_info_[BoneIndex].BoneOffset = to_gua(pMesh->mBones[i]->mOffsetMatrix);    
            bone_mapping_[BoneName] = BoneIndex;
        }
        else {
            BoneIndex = bone_mapping_[BoneName];
        }
        
        for (uint j = 0 ; j < pMesh->mBones[i]->mNumWeights ; j++) {
            uint VertexID = entries_[MeshIndex].BaseVertex + pMesh->mBones[i]->mWeights[j].mVertexId;
            float Weight  = pMesh->mBones[i]->mWeights[j].mWeight;                   
            Bones[VertexID].AddBoneData(BoneIndex, Weight);
        }
    }

}

////////////////////////////////////////////////////////////////////////////////

void SkeletalAnimationRessource::InitMesh(uint MeshIndex,
                    const aiMesh* paiMesh,
                    std::vector<scm::math::vec3>& Positions,
                    std::vector<scm::math::vec3>& Normals,
                    std::vector<scm::math::vec2>& TexCoords,
                    std::vector<scm::math::vec3>& Tangents,
                    std::vector<scm::math::vec3>& Bitangents,
                    std::vector<VertexBoneData>& Bones,
                    std::vector<uint>& Indices)
{    
    const scm::math::vec3 Zero3D(0.0f, 0.0f, 0.0f);
    
    // Populate the vertex attribute vectors
    for (uint i = 0 ; i < paiMesh->mNumVertices ; i++) { // TODO catch: haspositions and hasnormals
        
        scm::math::vec3 pPos = scm::math::vec3(0.f, 0.f, 0.f);
        if(paiMesh->HasPositions()) {
          pPos = scm::math::vec3(paiMesh->mVertices[i].x,paiMesh->mVertices[i].y,paiMesh->mVertices[i].z);
        }

        scm::math::vec3 pNormal = scm::math::vec3(0.f, 0.f, 0.f);
        if(paiMesh->HasNormals()) {
          pNormal = scm::math::vec3(paiMesh->mNormals[i].x,paiMesh->mNormals[i].y,paiMesh->mNormals[i].z);
        }

        scm::math::vec3 pTexCoord = scm::math::vec3(0,0,0);
        if(paiMesh->HasTextureCoords(0)) {}
        {
          pTexCoord = scm::math::vec3(paiMesh->mTextureCoords[0][i].x,paiMesh->mTextureCoords[0][i].y,paiMesh->mTextureCoords[0][i].z);
        }

        scm::math::vec3 pTangent = scm::math::vec3(0.f, 0.f, 0.f);
        scm::math::vec3 pBitangent = scm::math::vec3(0.f, 0.f, 0.f);
        if (paiMesh->HasTangentsAndBitangents()) {
          pTangent = scm::math::vec3(paiMesh->mTangents[i].x, paiMesh->mTangents[i].y, paiMesh->mTangents[i].z);

          pBitangent = scm::math::vec3(paiMesh->mBitangents[i].x, paiMesh->mBitangents[i].y, paiMesh->mBitangents[i].z);
        }

        Positions.push_back(pPos);
        Normals.push_back(pNormal);
        Bitangents.push_back(pBitangent);
        Tangents.push_back(pTangent);
        TexCoords.push_back(scm::math::vec2(pTexCoord[0], pTexCoord[1]));

    }
    
    LoadBones(MeshIndex, paiMesh, Bones);
    
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

    entries_.resize(scene_->mNumMeshes);

    std::vector<scm::math::vec3> Positions;
    std::vector<scm::math::vec3> Normals;
    std::vector<scm::math::vec2> TexCoords;
    std::vector<scm::math::vec3> Tangents;
    std::vector<scm::math::vec3> Bitangents;
    std::vector<VertexBoneData> Bones;
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
    Tangents.reserve(NumVertices);
    Bitangents.reserve(NumVertices);
    Bones.resize(NumVertices);
    Indices.reserve(NumIndices);
        
    // Initialize the meshes in the scene one by one
    for (uint i = 0 ; i < entries_.size() ; i++) {
        const aiMesh* paiMesh = scene_->mMeshes[i];
        InitMesh(i, paiMesh, Positions, Normals, TexCoords, Tangents, Bitangents, Bones, Indices);
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

    bone_transforms_block_ = std::make_shared<BoneTransformUniformBlock>(ctx.render_device);
    
    std::vector< math::mat4 > tmp_transforms;

    std::for_each( bone_info_.begin(), bone_info_.end(), [&tmp_transforms](BoneInfo const& bi){tmp_transforms.push_back(bi.FinalTransformation);});

    bone_transforms_block_->update(ctx.render_context, tmp_transforms);
    ctx.render_context->bind_uniform_buffer( bone_transforms_block_->block().block_buffer(), 1 );

    ctx.render_context->apply();
  }

}

////////////////////////////////////////////////////////////////////////////////

void SkeletalAnimationRessource::draw(RenderContext const& ctx) /*const*/ {

  // upload to GPU if neccessary
  upload_to(ctx);

  updateBoneTransforms(ctx);

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

  //TODO
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

void SkeletalAnimationRessource::updateBoneTransforms(RenderContext const& ctx)
{
    std::vector<scm::math::mat4f> Transforms;

    BoneTransform(timer_.get_elapsed(), Transforms);

    std::vector<math::mat4> tmp_transforms;

    std::for_each( bone_info_.begin(), bone_info_.end(), [&tmp_transforms](BoneInfo const& bi){tmp_transforms.push_back(bi.FinalTransformation);});
    
    bone_transforms_block_->update(ctx.render_context, tmp_transforms);
    ctx.render_context->bind_uniform_buffer( bone_transforms_block_->block().block_buffer(), 1 );

}


void SkeletalAnimationRessource::BoneTransform(float TimeInSeconds, std::vector<scm::math::mat4f>& Transforms)
{
    scm::math::mat4f Identity = scm::math::mat4f::identity();
    
    //if no frame frquequency is given, set to 25
    float TicksPerSecond = (float)(scene_->mAnimations[0]->mTicksPerSecond != 0 ? scene_->mAnimations[0]->mTicksPerSecond : 25.0f);
    float TimeInTicks = TimeInSeconds * TicksPerSecond;
    float AnimationTime = fmod(TimeInTicks, (float)scene_->mAnimations[0]->mDuration);

    ReadNodeHierarchy(AnimationTime, scene_->mRootNode, Identity);

    Transforms.resize(num_bones_);

    for (uint i = 0 ; i < num_bones_ ; i++) {
        Transforms[i] = bone_info_[i].FinalTransformation;
    }
}

void SkeletalAnimationRessource::ReadNodeHierarchy(float AnimationTime, const aiNode* pNode, const scm::math::mat4f& ParentTransform)
{    
    std::string NodeName(pNode->mName.data);
    
    const aiAnimation* pAnimation = scene_->mAnimations[0];
        
    scm::math::mat4f NodeTransformation(to_gua(pNode->mTransformation));
     
    const aiNodeAnim* pNodeAnim = FindNodeAnim(pAnimation, NodeName);
    
    if (pNodeAnim) {
      // Interpolate scaling and generate scaling transformation matrix
      scm::math::vec3 Scaling;
      CalcInterpolatedScaling(Scaling, AnimationTime, pNodeAnim);
      scm::math::mat4f ScalingM = scm::math::make_scale(Scaling);

      // Interpolate rotation and generate rotation transformation matrix
      scm::math::quatf RotationQ;
      CalcInterpolatedRotation(RotationQ, AnimationTime, pNodeAnim); 
      scm::math::mat4f RotationM = RotationQ.to_matrix();

      // Interpolate translation and generate translation transformation matrix
      scm::math::vec3 Translation;
      CalcInterpolatedPosition(Translation, AnimationTime, pNodeAnim);
      scm::math::mat4f TranslationM = scm::math::make_translation(Translation);
      
      // Combine the above transformations
      NodeTransformation = TranslationM * RotationM * ScalingM;
    }
       
    scm::math::mat4f GlobalTransformation = ParentTransform * NodeTransformation;
    
    if (bone_mapping_.find(NodeName) != bone_mapping_.end()) {
        uint BoneIndex = bone_mapping_[NodeName];
        //bone_info_[BoneIndex].FinalTransformation = m_GlobalInverseTransform * GlobalTransformation * bone_info_[BoneIndex].BoneOffset;
        bone_info_[BoneIndex].FinalTransformation = GlobalTransformation * bone_info_[BoneIndex].BoneOffset;
    }
    
    for (uint i = 0 ; i < pNode->mNumChildren ; i++) {
        ReadNodeHierarchy(AnimationTime, pNode->mChildren[i], GlobalTransformation);
    }
}
////////////////////////////////////////////////////////////////////////////////

const aiNodeAnim* SkeletalAnimationRessource::FindNodeAnim(const aiAnimation* pAnimation, const std::string NodeName)
{
    for (uint i = 0 ; i < pAnimation->mNumChannels ; i++) {
        const aiNodeAnim* pNodeAnim = pAnimation->mChannels[i];
        
        if (std::string(pNodeAnim->mNodeName.data) == NodeName) {
            return pNodeAnim;
        }
    }
    
    return NULL;
}

uint SkeletalAnimationRessource::FindPosition(float AnimationTime, const aiNodeAnim* pNodeAnim)
{    
    for (uint i = 0 ; i < pNodeAnim->mNumPositionKeys - 1 ; i++) {
        if (AnimationTime < (float)pNodeAnim->mPositionKeys[i + 1].mTime) {
            return i;
        }
    }
    
    assert(0);

    return 0;
}


uint SkeletalAnimationRessource::FindRotation(float AnimationTime, const aiNodeAnim* pNodeAnim)
{
    assert(pNodeAnim->mNumRotationKeys > 0);

    for (uint i = 0 ; i < pNodeAnim->mNumRotationKeys - 1 ; i++) {
        if (AnimationTime < (float)pNodeAnim->mRotationKeys[i + 1].mTime) {
            return i;
        }
    }
    
    assert(0);

    return 0;
}


uint SkeletalAnimationRessource::FindScaling(float AnimationTime, const aiNodeAnim* pNodeAnim)
{
    assert(pNodeAnim->mNumScalingKeys > 0);
    
    for (uint i = 0 ; i < pNodeAnim->mNumScalingKeys - 1 ; i++) {
        if (AnimationTime < (float)pNodeAnim->mScalingKeys[i + 1].mTime) {
            return i;
        }
    }
    
    assert(0);

    return 0;
}


void SkeletalAnimationRessource::CalcInterpolatedPosition(scm::math::vec3& Out, float AnimationTime, const aiNodeAnim* pNodeAnim)
{
    if (pNodeAnim->mNumPositionKeys == 1) {
        Out = to_gua(pNodeAnim->mPositionKeys[0].mValue);
        return;
    }
            
    uint PositionIndex = FindPosition(AnimationTime, pNodeAnim);
    uint NextPositionIndex = (PositionIndex + 1);
    assert(NextPositionIndex < pNodeAnim->mNumPositionKeys);
    float DeltaTime = (float)(pNodeAnim->mPositionKeys[NextPositionIndex].mTime - pNodeAnim->mPositionKeys[PositionIndex].mTime);
    float Factor = (AnimationTime - (float)pNodeAnim->mPositionKeys[PositionIndex].mTime) / DeltaTime;
    assert(Factor >= 0.0f && Factor <= 1.0f);
    const aiVector3D& Start = pNodeAnim->mPositionKeys[PositionIndex].mValue;
    const aiVector3D& End = pNodeAnim->mPositionKeys[NextPositionIndex].mValue;
    aiVector3D Delta = End - Start;
    Out = to_gua(Start + Factor * Delta);
}


void SkeletalAnimationRessource::CalcInterpolatedRotation(scm::math::quatf& Out, float AnimationTime, const aiNodeAnim* pNodeAnim)
{
  // we need at least two values to interpolate...
    if (pNodeAnim->mNumRotationKeys == 1) {
        Out = to_gua(pNodeAnim->mRotationKeys[0].mValue);
        return;
    }
    
    uint RotationIndex = FindRotation(AnimationTime, pNodeAnim);
    uint NextRotationIndex = (RotationIndex + 1);
    assert(NextRotationIndex < pNodeAnim->mNumRotationKeys);
    float DeltaTime = (float)(pNodeAnim->mRotationKeys[NextRotationIndex].mTime - pNodeAnim->mRotationKeys[RotationIndex].mTime);
    float Factor = (AnimationTime - (float)pNodeAnim->mRotationKeys[RotationIndex].mTime) / DeltaTime;
    assert(Factor >= 0.0f && Factor <= 1.0f);
    const aiQuaternion& StartRotationQ = pNodeAnim->mRotationKeys[RotationIndex].mValue;
    const aiQuaternion& EndRotationQ   = pNodeAnim->mRotationKeys[NextRotationIndex].mValue;  
    aiQuaternion temp;  
    aiQuaternion::Interpolate(temp, StartRotationQ, EndRotationQ, Factor);
    Out = to_gua(temp.Normalize());
}


void SkeletalAnimationRessource::CalcInterpolatedScaling(scm::math::vec3& Out, float AnimationTime, const aiNodeAnim* pNodeAnim)
{
    if (pNodeAnim->mNumScalingKeys == 1) {
        Out = to_gua(pNodeAnim->mScalingKeys[0].mValue);
        return;
    }

    uint ScalingIndex = FindScaling(AnimationTime, pNodeAnim);
    uint NextScalingIndex = (ScalingIndex + 1);
    assert(NextScalingIndex < pNodeAnim->mNumScalingKeys);
    float DeltaTime = (float)(pNodeAnim->mScalingKeys[NextScalingIndex].mTime - pNodeAnim->mScalingKeys[ScalingIndex].mTime);
    float Factor = (AnimationTime - (float)pNodeAnim->mScalingKeys[ScalingIndex].mTime) / DeltaTime;
    assert(Factor >= 0.0f && Factor <= 1.0f);
    const aiVector3D& Start = pNodeAnim->mScalingKeys[ScalingIndex].mValue;
    const aiVector3D& End   = pNodeAnim->mScalingKeys[NextScalingIndex].mValue;
    aiVector3D Delta = End - Start;
    Out = to_gua(Start + Factor * Delta);
}
////////////////////////////////////////////////////////////////////////////////

}