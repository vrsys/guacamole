// class header
#include <gua/utils/SkinnedMesh.hpp>
#include <gua/utils/Timer.hpp>

//external headers
#include <iostream>
#include <queue>
#include <map>

namespace gua{

SkinnedMesh::SkinnedMesh():
 Mesh{},
 bone_ids{},
 bone_weights{},
 bone_counts{}
{}

SkinnedMesh::SkinnedMesh(FbxMesh& mesh, Bone const& root) {
 Timer timer{};
  timer.start();

  bool has_uvs = mesh.GetElementUVCount() > 0;
  bool has_tangents = (mesh.GetElementTangentCount() == 0 || mesh.GetElementBinormalCount() == 0) && !has_uvs;

  //get the temporary elements, actual work is done in this function
  auto verts_and_tris = get_verts_and_tris(mesh);

  std::vector<std::vector<temp_vert>>const& vert_positions = std::get<0>(verts_and_tris);
  num_vertices = std::get<2>(verts_and_tris);

  std::vector<temp_tri>& temp_tris = std::get<1>(verts_and_tris);
  num_triangles = temp_tris.size();

  bool has_weights = false;
  std::vector<bone_influences> ctrlpt_weights{};
  //get weights of original control points
  if(root.name != "none") {
    ctrlpt_weights = get_weights(mesh, root);
    has_weights = ctrlpt_weights.size() > 0;
  }

  // Reserve space in the vectors for the vertex attributes and indices
  positions.reserve(num_vertices);
  normals.reserve(num_vertices);
  if(has_uvs) {
    texCoords.reserve(num_vertices);
  }
  else {
    texCoords.resize(num_vertices, scm::math::vec2f(0.0f));
  }
  if(has_tangents) {
    tangents.reserve(num_vertices);
  }
  else {
    tangents.resize(num_vertices, scm::math::vec3f(0.0f));
    bitangents.resize(num_vertices, scm::math::vec3f(0.0f));
  }

  if(has_weights) {
    bone_counts.reserve(num_vertices);
    bone_ids.reserve(num_vertices);
    bone_weights.reserve(num_vertices);
  }
  else {
    bone_counts.resize(num_vertices, 0);
    bone_ids.resize(num_vertices, 0);
    bone_weights.resize(num_vertices, 0.0f);
  }
  
  FbxVector4* control_points = mesh.GetControlPoints();
  //load reduced attributes
  unsigned curr_vert = 0;
  //iterate over control points
  for(std::vector<temp_vert> const& verts : vert_positions) {
    //iterate over vertices at that point
    for(temp_vert const& vert : verts) {
      //update containing triangles with actual index of this vertex in member vectors
      for(auto const& tri : vert.tris) {
        temp_tris[tri.first].verts[tri.second] = curr_vert;
      }
      //push properties to attribute vectors
      positions.push_back(to_gua::vec3(control_points[vert.point]));
      normals.push_back(vert.normal);

      if(has_tangents) {
        tangents.push_back(vert.tangent);
        bitangents.push_back(vert.bitangent);
      }
      if(has_uvs) {
        texCoords.push_back(vert.uv);
      }
      if(has_weights) {
        bone_influences const& curr_influence{ctrlpt_weights[vert.point]};
        //push all bone influences for current vert
        for(uint i(0); i < curr_influence.weights.size(); ++i){
          bone_ids.push_back(curr_influence.IDs[i]);
          bone_weights.push_back(curr_influence.weights[i]);
        }

        bone_counts.push_back(curr_influence.weights.size());
      }
      ++curr_vert;
    }
  }

  //load reduced triangles
  indices.reserve(num_triangles * 3);
  for(temp_tri const& tri : temp_tris) {
    indices.push_back(tri.verts[0]);
    indices.push_back(tri.verts[1]);
    indices.push_back(tri.verts[2]);
  }

  //output reduction info
  // Logger::LOG_DEBUG << "Number of vertices reduced from " << old_num_vertices << " to " << num_vertices << " ,time taken: " << timer.get_elapsed() << std::endl;
}

SkinnedMesh::SkinnedMesh(aiMesh const& mesh, Bone const& root):
  Mesh{mesh}
 {   
  //get weights and write them to vectors
  auto temp_weights = get_weights(mesh, root);
  for(auto const& w : temp_weights){
    for(uint i(0); i < w.weights.size(); ++i){
      bone_ids.push_back(w.IDs[i]);
      bone_weights.push_back(w.weights[i]);
    }
    bone_counts.push_back(w.weights.size());
  }
}

std::vector<SkinnedMesh::bone_influences> SkinnedMesh::get_weights(aiMesh const& mesh, Bone const& root) {
  std::map<std::string, int> bone_mapping_; // maps a bone name to its index
  root.collect_indices(bone_mapping_);
  
  std::vector<bone_influences> temp_weights{mesh.mNumVertices};

  for (uint i = 0 ; i < mesh.mNumBones ; i++) {
    std::string bone_name(mesh.mBones[i]->mName.data);      
    uint BoneIndex = bone_mapping_.at(bone_name);        
    
    for (uint j = 0 ; j < mesh.mBones[i]->mNumWeights ; j++) {
      uint VertexID = mesh.mBones[i]->mWeights[j].mVertexId;
      float Weight  = mesh.mBones[i]->mWeights[j].mWeight;                   
      temp_weights[VertexID].add_bone(BoneIndex, Weight);
    }
  }

  return temp_weights;
}

std::vector<SkinnedMesh::bone_influences> SkinnedMesh::get_weights(FbxMesh const& mesh, Bone const& root) {
  std::map<std::string, int> bone_mapping_; // maps a bone name to its index
  root.collect_indices(bone_mapping_);

  //check for skinning
  FbxSkin* skin = NULL;
  for(unsigned i = 0; i < mesh.GetDeformerCount(); ++i) {
    FbxDeformer* defPtr ={mesh.GetDeformer(i)};
    if(defPtr->GetDeformerType() == FbxDeformer::eSkin) {
      skin = static_cast<FbxSkin*>(defPtr);
      break;
    }
  }

  if(!skin) {
    Logger::LOG_WARNING << "Mesh does not contain skin deformer, ignoring weights" << std::endl;
    return std::vector<bone_influences>{};
  }
  //set up temporary weights, for control points, not actual vertices
  std::vector<bone_influences> temp_weights{unsigned(mesh.GetControlPointsCount())};
  //one cluster corresponds to one bone
  for(unsigned i = 0; i < skin->GetClusterCount(); ++i) {
    FbxCluster* cluster = skin->GetCluster(i);
    FbxNode* Bone = cluster->GetLink();

    if(!Bone) {
      Logger::LOG_ERROR << "associated Bone does not exist!" << std::endl;
      assert(false);      
    }

    std::string bone_name(Bone->GetName());
    uint bone_index;
    if(bone_mapping_.find(bone_name) != bone_mapping_.end()) {
      bone_index = bone_mapping_.at(bone_name);
    }      
    else {
      Logger::LOG_ERROR << "Bone with name '" << bone_name << "' does not exist!, ignoring weights" << std::endl;
      return std::vector<bone_influences>{};          
    }

    int* indices = cluster->GetControlPointIndices();
    double* weights = cluster->GetControlPointWeights();
    for(unsigned i = 0; i < cluster->GetControlPointIndicesCount(); ++i) {
      //update mapping info of current control point
      temp_weights[indices[i]].add_bone(bone_index, weights[i]);
    }
  }

  return temp_weights;
}

void SkinnedMesh::copy_to_buffer(SkinnedVertex* vertex_buffer, uint resource_offset)  const {
  uint bone_offset{resource_offset};
  for (unsigned v(0); v < num_vertices; ++v) {

    vertex_buffer[v].pos = positions[v];

    vertex_buffer[v].tex = texCoords[v];

    vertex_buffer[v].normal = normals[v];

    vertex_buffer[v].tangent = tangents[v];

    vertex_buffer[v].bitangent = bitangents[v];

    vertex_buffer[v].bone_id_offset = bone_offset;

    vertex_buffer[v].nr_of_bones = bone_counts[v];
    
    bone_offset += bone_counts[v];
  }
}

scm::gl::vertex_format SkinnedMesh::get_vertex_format() const {
  return scm::gl::vertex_format(
    0, 0, scm::gl::TYPE_VEC3F, sizeof(SkinnedVertex))(
    0, 1, scm::gl::TYPE_VEC2F, sizeof(SkinnedVertex))(
    0, 2, scm::gl::TYPE_VEC3F, sizeof(SkinnedVertex))(
    0, 3, scm::gl::TYPE_VEC3F, sizeof(SkinnedVertex))(
    0, 4, scm::gl::TYPE_VEC3F, sizeof(SkinnedVertex))(
    0, 5, scm::gl::TYPE_UINT, sizeof(SkinnedVertex))(
    0, 6, scm::gl::TYPE_UINT, sizeof(SkinnedVertex));
}

std::vector<uint> const& 
SkinnedMesh::get_bone_ids()const{
  return bone_ids;
}

std::vector<float> const& 
SkinnedMesh::get_bone_weights()const{
  return bone_weights;
}

} // namespace gua