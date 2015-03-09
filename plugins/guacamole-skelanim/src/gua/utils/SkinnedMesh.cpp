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

SkinnedMesh::SkinnedMesh(FbxMesh& mesh, Bone const& root):
 Mesh{mesh}
{
  bool has_weights = false;
  std::vector<bone_influences> ctrlpt_weights{};
  //get weights of original control points
  if(root.name != "none") {
    ctrlpt_weights = get_weights(mesh, root);
    has_weights = ctrlpt_weights.size() > 0;
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

  scm::math::vec3f last_pos{positions[0]};
  unsigned weight_index = 0;
  //iterate over vertices
  for(auto const& pos : positions) {
    //if position changes its the next control point -> next weight
    if(pos != last_pos) {
      ++weight_index;
    }
    if(has_weights) {
      bone_influences const& curr_influence{ctrlpt_weights[weight_index]};
      //push all bone influences for current vert
      for(uint i(0); i < curr_influence.weights.size(); ++i){
        bone_ids.push_back(curr_influence.IDs[i]);
        bone_weights.push_back(curr_influence.weights[i]);
      }

      bone_counts.push_back(curr_influence.weights.size());
    }
    last_pos = pos;
  }
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