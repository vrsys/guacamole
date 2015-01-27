// class header
#include <gua/renderer/SkeletalAnimationUtils.hpp>
//external headers
#include <iostream>

namespace to_gua{

scm::math::mat4f mat4(aiMatrix4x4 const& m) {
  scm::math::mat4f res(m.a1,m.b1,m.c1,m.d1
                      ,m.a2,m.b2,m.c2,m.d2
                      ,m.a3,m.b3,m.c3,m.d3
                      ,m.a4,m.b4,m.c4,m.d4);
  return res;
}
template<typename T>
scm::math::vec3 vec3(T const& v) {
  scm::math::vec3 res(v[0], v[1], v[2]);
  return res;
}

template<typename T>
scm::math::vec2 vec2(T const& v) {
  scm::math::vec2 res(v[0], v[1]);
  return res;
}

template<typename T>
scm::math::vec4 vec4(T const& v) {
  scm::math::vec4 res(v[0], v[1], v[2], v[3]);
  return res;
}

<<<<<<< HEAD
scm::math::quatf quat(aiQuaternion const& q) {
  scm::math::quatf res(q.w, q.x, q.y, q.z);
  return res;
}

}

namespace gua {

std::vector<std::shared_ptr<SkeletalAnimation>> SkeletalAnimationUtils::load_animations(aiScene const& scene) {
=======
std::vector<std::shared_ptr<SkeletalAnimation>> SkeletalAnimationUtils::load_animations(aiScene const* scene, std::string const& file_name) {
>>>>>>> add animation name interface for animation control in avango
  std::vector<std::shared_ptr<SkeletalAnimation>> animations{};
  if(!scene.HasAnimations()) Logger::LOG_WARNING << "scene contains no animations!" << std::endl;
 
<<<<<<< HEAD
  for(uint i = 0; i < scene.mNumAnimations; ++i) {
    animations.push_back(std::make_shared<SkeletalAnimation>(*(scene.mAnimations[i])));
=======
  for(uint i = 0; i < scene->mNumAnimations; ++i) {
    animations.push_back(std::make_shared<SkeletalAnimation>(scene->mAnimations[i], file_name));
>>>>>>> add animation name interface for animation control in avango
  }

  return animations;
}

void SkeletalAnimationUtils::calculate_matrices(float timeInSeconds, Node const& root, SkeletalAnimation const& pAnim, std::vector<scm::math::mat4f>& transforms) {
 
  float timeNormalized = 0;
  Pose pose{};

  timeNormalized = timeInSeconds / pAnim.get_duration();
  timeNormalized = scm::math::fract(timeNormalized);

  pose = pAnim.calculate_pose(timeNormalized);

  scm::math::mat4f identity = scm::math::mat4f::identity();
  root.accumulate_matrices(transforms, pose, identity);
}

void SkeletalAnimationUtils::calculate_matrices(Node const& root, std::vector<scm::math::mat4f>& transforms) {

  Pose pose{};

  scm::math::mat4f identity = scm::math::mat4f::identity();
  root.accumulate_matrices(transforms, pose, identity);
}

////////////////////////////////////////////////////////////////////////////////
float blend::cos(float x) {
  x *= scm::math::pi_f;
  return 0.5f * (1 - scm::math::cos(x));
}

float blend::linear(float x) {
  //values from 0 to 2 accepted
  x = fmod(x, 2.0f);
  x = 1 - scm::math::abs(x - 1);
  return x;
}

float blend::smoothstep(float x) {
  x = fmod(x, 2.0f);
  x = 1 - scm::math::abs(x - 1);
  
  return 3 * x * x - 2 * x * x * x;
}

float blend::swap(float x)
{
  x = fmod(x, 2.0f);
  return (x > 0.5) ? 1 : 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Node::Node():
  index{-1},
  name{"none"},
  parentName{"none"},
  numChildren{0},
  transformation{},
  offsetMatrix{scm::math::mat4f::identity()}
{}

Node::Node(aiNode const& node):
  index{-1},
  name{node.mName.C_Str()},
  parentName{node.mParent != NULL ? node.mParent->mName.C_Str() : "none"},
  numChildren{node.mNumChildren},
  transformation{to_gua::mat4(node.mTransformation)},
  offsetMatrix{scm::math::mat4f::identity()}
{
  for(unsigned i = 0; i < node.mNumChildren; ++i) {
    std::shared_ptr<Node> child = std::make_shared<Node>(*(node.mChildren[i]));
    children.push_back(child);
  }
}

Node::Node(aiScene const& scene) {
  //construct hierarchy
  *this = Node{*(scene.mRootNode)};

  std::map<std::string, std::pair<uint, scm::math::mat4f>> bone_info{};
  unsigned num_bones = 0;
  for (uint i = 0 ; i < scene.mNumMeshes ; i++) {
    for (uint b = 0; b < scene.mMeshes[i]->mNumBones; ++b){

      std::string BoneName(scene.mMeshes[i]->mBones[b]->mName.data);  
      if (bone_info.find(BoneName) == bone_info.end()) {
         
        bone_info[BoneName] = std::make_pair(num_bones, to_gua::mat4(scene.mMeshes[i]->mBones[b]->mOffsetMatrix));   
        ++num_bones;
      }
    }
  }
  this->set_properties(bone_info);
}

Node::~Node()
{}

std::shared_ptr<Node> Node::find(std::string const& name) const {

  for(std::shared_ptr<Node> const& child : children) {
    if(child->name == name) return child;

    std::shared_ptr<Node> found = child->find(name);
    if(found) return found;
  }

  return nullptr;
}

void Node::collect_indices(std::map<std::string, int>& ids) const {
  ids[name] = index;

  for(std::shared_ptr<Node> const& child : children) {
    child->collect_indices(ids);
  }
}

void Node::set_properties(std::map<std::string, std::pair<uint, scm::math::mat4f>> const& infos) {
  if(infos.find(name) != infos.end()) {
    offsetMatrix = infos.at(name).second;
    index = infos.at(name).first;
  }

  for(std::shared_ptr<Node>& child : children) {
    child->set_properties(infos);
  }
}

void Node::accumulate_matrices(std::vector<scm::math::mat4f>& transformMat4s, Pose const& pose, scm::math::mat4f const& parentTransform) const {
  scm::math::mat4f nodeTransformation{transformation};

  if(pose.contains(name)) { 
    nodeTransformation = pose.get_transform(name).to_matrix();  
  }
  
  scm::math::mat4f finalTransformation = parentTransform * nodeTransformation;

  //update transform if bone is mapped
  if (index >= 0) {
    transformMat4s[index] = finalTransformation * offsetMatrix;
  }
  
  for (std::shared_ptr<Node> const& child : children) {
     child->accumulate_matrices(transformMat4s, pose, finalTransformation);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Mesh::Mesh():
 positions{},
 normals{},
 texCoords{},
 tangents{},
 bitangents{},
 weights{},
 indices{}
{}

Mesh::Mesh(FbxMesh& mesh) {
  num_vertices = mesh.GetControlPointsCount(); 

  std::string UV_set;
  bool has_uvs = true;
//UV coordinates
  if(mesh.GetElementUVCount() == 0) {
    Logger::LOG_WARNING << "Mesh has no texture coordinates" << std::endl;
    has_uvs = false;
  }
  else {
    if(mesh.GetElementUVCount() > 1) {
      Logger::LOG_WARNING << "Mesh has multiple UV sets, only using first one" << std::endl;
    }
    UV_set = mesh.GetElementUV(0)->GetName();
  }

//normals
  if(mesh.GetElementNormalCount() == 0) {
    //dont override exiting normals and generate by control point, not vertex
    mesh.GenerateNormals(false, true);
  }

//tangents
  if(mesh.GetElementTangentCount() == 0 || mesh.GetElementBinormalCount() == 0) {
    Logger::LOG_DEBUG << "Generating tangents" << std::endl;
    mesh.GenerateTangentsData(UV_set.c_str(), true);
  }

//polygons
  if(mesh.GetPolygonCount() < 1) {
    Logger::LOG_ERROR << "No polygons in mesh" << std::endl;
    assert(0);
  }

  if(!mesh.IsTriangleMesh()) {
    Logger::LOG_DEBUG << "Triangulating mesh" << std::endl;
  }

  struct temp_vert {
    temp_vert(unsigned pt, unsigned norm, unsigned u, unsigned tr, unsigned ind):
     point{pt},
     normal{norm},
     uv{u},
     tris{}
    {
      tris.push_back(std::make_pair(tr, ind));
    }
    unsigned point;
    unsigned normal;
    unsigned uv;
    std::vector<std::pair<unsigned, unsigned>> tris; //tris which share vertex
  };

  struct temp_tri {
    temp_tri(unsigned a, unsigned b, unsigned c):
     verts{a, b, c}
    {}
    std::array<unsigned, 3> verts;
  };

  std::vector<std::vector<temp_vert>> temp_verts{unsigned(mesh.GetControlPointsCount()), std::vector<temp_vert>{}};

  std::vector<temp_tri> temp_tris{};
  
  FbxVector4* control_points = mesh.GetControlPoints();
  int* poly_vertices = mesh.GetPolygonVertices();
  
  FbxArray<FbxVector4> poly_normals;
  mesh.GetPolygonVertexNormals(poly_normals);
  unsigned dupl_normals = 0;
  //index normals, filter out duplicates
  std::vector<unsigned> normal_indices{};
  for(unsigned i = 0; i < poly_normals.Size(); ++i) {
    unsigned d = poly_normals.Find(poly_normals[i]);
    if(d == i) {
      normal_indices.push_back(i);
    }
    else {
      normal_indices.push_back(d); 
      ++dupl_normals;
    }
  } 
  std::cout << dupl_normals << " normal duplications" << std::endl;

  FbxArray<FbxVector2> poly_uvs;
  mesh.GetPolygonVertexUVs(UV_set.c_str(), poly_uvs);
  //index uvs, filter out duplicates
  unsigned dupl_uvs = 0;
  std::vector<unsigned> uv_indices{};
  for(unsigned i = 0; i < poly_uvs.Size(); ++i) {
    unsigned d = poly_uvs.Find(poly_uvs[i]);
    if(d == i) {
      uv_indices.push_back(i);
    }
    else {
      uv_indices.push_back(d); 
      ++dupl_uvs;
    }
  } 
  std::cout << dupl_uvs << " UV duplications" << std::endl;

  //tangents
  FbxGeometryElementTangent* tangent_info = mesh.GetElementTangent(0);
  // FbxArray<FbxVector4> poly_tangents = tangent_info->GetDirectArray();
  //tangents per position
  if(tangent_info->GetMappingMode() == FbxGeometryElement::eByControlPoint) {
    if(tangent_info->GetReferenceMode() == FbxGeometryElement::eDirect) {
      tangent_info->GetDirectArray().GetAt(0);

    }
    else if(tangent_info->GetReferenceMode() == FbxGeometryElement::eIndexToDirect) {
      unsigned index = tangent_info->GetIndexArray().GetAt(0);
      tangent_info->GetDirectArray().GetAt(index);
    }
    else {
      Logger::LOG_ERROR << "Type of reference not supported" << std::endl;
    }
  }
  //tangents per vertex
  else if(tangent_info->GetMappingMode() == FbxGeometryElement::eByPolygonVertex){
    if(tangent_info->GetReferenceMode() == FbxGeometryElement::eDirect) {
      tangent_info->GetDirectArray().GetAt(0);

    }
    else if(tangent_info->GetReferenceMode() == FbxGeometryElement::eIndexToDirect) {
      unsigned index = tangent_info->GetIndexArray().GetAt(0);
      tangent_info->GetDirectArray().GetAt(index);
    }
    else {
      Logger::LOG_ERROR << "Type of reference not supported" << std::endl;
    }
  }
  else {
    Logger::LOG_ERROR << "Type of mapping not supported" << std::endl;
  }

  num_triangles = 0; 
  for(unsigned i = 0; i < mesh.GetPolygonCount(); ++i)
  {
    //triangulate face if necessary
    for(unsigned j = 2; j < mesh.GetPolygonSize(i); ++j)
    {
      unsigned start_index = mesh.GetPolygonVertexIndex(i);
      std::array<unsigned, 3> indices{start_index, start_index + j - 1, start_index + j};

      temp_tri tri{unsigned(poly_vertices[indices[0]]), unsigned(poly_vertices[indices[1]]), unsigned(poly_vertices[indices[2]])};
      temp_tris.push_back(tri);

      //add vertices to to array
      temp_verts[tri.verts[0]].push_back(temp_vert{tri.verts[0], normal_indices[indices[0]], uv_indices[indices[0]], num_triangles, 0});
      temp_verts[tri.verts[1]].push_back(temp_vert{tri.verts[1], normal_indices[indices[1]], uv_indices[indices[1]], num_triangles, 1});
      temp_verts[tri.verts[2]].push_back(temp_vert{tri.verts[2], normal_indices[indices[2]], uv_indices[indices[2]], num_triangles, 2});
      
      ++num_triangles;
    }
  }
  //filter out duplicate vertices
  num_vertices = 0;
  unsigned old_num_vertices = 0;
  unsigned dupl_verts = 0;
  for(std::vector<temp_vert>& verts : temp_verts) {
    old_num_vertices += verts.size();
    for(auto iter = verts.begin(); iter != verts.end(); ++iter) {
      for(std::vector<temp_vert>::iterator iter2 = std::next(iter); iter2 != verts.end(); ++iter2) {
        //match by normals and if exisiting, uvs
        bool duplicate = has_uvs ? (iter2->uv == iter->uv && iter2->normal == iter->normal) :  (iter2->normal == iter->normal); 
        //duplicate -> merge vertices
        if(duplicate) {

          iter->tris.push_back(iter2->tris[0]);

          verts.erase(iter2);
          ++dupl_verts;
          break;
        }
      }
    }
    num_vertices += verts.size();
  }
  std::cout << dupl_verts << " vertex duplications" << std::endl;

  // Reserve space in the vectors for the vertex attributes and indices
  //for now resize and initialize with 0
  positions.reserve(num_vertices);
  normals.reserve(num_vertices);
  if(!has_uvs) {
    texCoords.resize(num_vertices, scm::math::vec2(0.0f));
  }
  else {
    texCoords.reserve(num_vertices);
  }
  tangents.resize(num_vertices);
  bitangents.resize(num_vertices);
  weights.resize(num_vertices);

  //load reduced attributes
  unsigned curr_vert = 0;
  for(std::vector<temp_vert> const& verts : temp_verts) {
    for(temp_vert const& vert : verts) {
      //update vert index of tris using this vertex
      for(auto const& tri : vert.tris) {
        temp_tris[tri.first].verts[tri.second] = curr_vert;
      }
      positions.push_back(to_gua::vec3(control_points[vert.point]));
      normals.push_back(to_gua::vec3(poly_normals[vert.normal]));
      if(has_uvs) texCoords.push_back(to_gua::vec3(poly_uvs[vert.uv]));
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
  Logger::LOG_DEBUG << "Number of vertices reduced from " << old_num_vertices << " to " << num_vertices << std::endl;
}

Mesh::Mesh(aiMesh const& mesh, Node const& root) {   
  num_triangles = mesh.mNumFaces;
  num_vertices = mesh.mNumVertices; 
  
  // Reserve space in the vectors for the vertex attributes and indices
  positions.reserve(num_vertices);
  normals.reserve(num_vertices);
  texCoords.reserve(num_vertices);
  tangents.reserve(num_vertices);
  bitangents.reserve(num_vertices);
  weights.resize(num_vertices);
  indices.reserve(num_triangles * 3);


  // Populate the vertex attribute vectors
  for (uint i = 0 ; i < mesh.mNumVertices ; i++) {
    
    scm::math::vec3 pPos = scm::math::vec3(0.0f);
    if(mesh.HasPositions()) {
      pPos = to_gua::vec3(mesh.mVertices[i]);
    }

    scm::math::vec3 pNormal = scm::math::vec3(0.0f);
    if(mesh.HasNormals()) {
      pNormal = to_gua::vec3(mesh.mNormals[i]);
    }

    scm::math::vec2 pTexCoord = scm::math::vec2(0.0f);
    if(mesh.HasTextureCoords(0)) {
      pTexCoord = scm::math::vec2(mesh.mTextureCoords[0][i].x, mesh.mTextureCoords[0][i].y);
    }

    scm::math::vec3 pTangent = scm::math::vec3(0.0f);
    scm::math::vec3 pBitangent = scm::math::vec3(0.0f);
    if (mesh.HasTangentsAndBitangents()) {
      pTangent = to_gua::vec3(mesh.mTangents[i]);

      pBitangent = to_gua::vec3(mesh.mBitangents[i]);
    }

    positions.push_back(pPos);
    normals.push_back(pNormal);
    bitangents.push_back(pBitangent);
    tangents.push_back(pTangent);
    texCoords.push_back(pTexCoord);
  }

  init_weights(mesh, root);
  
  // Populate the index buffer
  for (uint i = 0 ; i < mesh.mNumFaces ; i++) {
    const aiFace& Face = mesh.mFaces[i];

    if(Face.mNumIndices != 3) {
      Logger::LOG_ERROR << "InitMesh - face doesnt have 3 vertices" << std::endl;
      assert(false);
    }

    indices.push_back(Face.mIndices[0]);
    indices.push_back(Face.mIndices[1]);
    indices.push_back(Face.mIndices[2]);
  }
}


void Mesh::init_weights(aiMesh const& mesh, Node const& root) {
  std::map<std::string, int> bone_mapping_; // maps a bone name to its index
  root.collect_indices(bone_mapping_);

  for (uint i = 0 ; i < mesh.mNumBones ; i++) {
    std::string bone_name(mesh.mBones[i]->mName.data);      
    uint BoneIndex = bone_mapping_.at(bone_name);        
    
    for (uint j = 0 ; j < mesh.mBones[i]->mNumWeights ; j++) {
      uint VertexID = mesh.mBones[i]->mWeights[j].mVertexId;
      float Weight  = mesh.mBones[i]->mWeights[j].mWeight;                   
      weights[VertexID].AddBoneData(BoneIndex, Weight);
    }
  }
}

void Mesh::copy_to_buffer(Vertex* vertex_buffer)  const {
  for (unsigned v(0); v < num_vertices; ++v) {

    vertex_buffer[v].pos = positions[v];

    vertex_buffer[v].tex = texCoords[v];

    vertex_buffer[v].normal = normals[v];

    vertex_buffer[v].tangent = tangents[v];

    vertex_buffer[v].bitangent = bitangents[v];

    vertex_buffer[v].bone_weights = scm::math::vec4f(weights[v].weights[0],weights[v].weights[1],weights[v].weights[2],weights[v].weights[3]);
    
    vertex_buffer[v].bone_ids = scm::math::vec4i(weights[v].IDs[0],weights[v].IDs[1],weights[v].IDs[2],weights[v].IDs[3]);
  }
}
void Mesh::copy_to_buffer_static(Vertex* vertex_buffer)  const {
  for (unsigned v(0); v < num_vertices; ++v) {

    vertex_buffer[v].pos = positions[v];

    vertex_buffer[v].tex = texCoords[v];

    vertex_buffer[v].normal = normals[v];

    vertex_buffer[v].tangent = tangents[v];

    vertex_buffer[v].bitangent = bitangents[v];
  }
}

void Mesh::from_fbx_scene(FbxNode* node, std::vector<FbxMesh*>& meshes ) {
  if(node != NULL) {
    if(node->GetGeometry() != NULL) {
      std::cout << " has geometry" << std::endl;
      if(node->GetGeometry()->GetAttributeType() == FbxNodeAttribute::eMesh) {
      std::cout << " is mesh" << std::endl;
      meshes.push_back(dynamic_cast<FbxMesh*>(node->GetGeometry()));
      }
    }

    // std::cout << " children:" << std::endl;
    // for(unsigned i = 0; i < node->GetChildNameCount(); ++i) {
    //   std::cout << node->GetChildName(i) << std::endl;
    // }

    for(unsigned i = 0; i < node->GetChildCount(); ++i) {
      if(node->GetChild(i) != NULL) {
        from_fbx_scene(node->GetChild(i), meshes);
      }
    }
  }
  else std::cout << "node is null" << std::endl;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Transformation::Transformation():
  scaling{1.0f},
  rotation{scm::math::quatf::identity()},
  translation{0.0f}
{}

Transformation::Transformation(scm::math::vec3 const& scale, scm::math::quatf const& rotate, scm::math::vec3 const& translate):
  scaling{scale},
  rotation{rotate},
  translation{translate}
{}

Transformation::~Transformation()
{}

scm::math::mat4f Transformation::to_matrix() const {
  return scm::math::make_translation(translation) * rotation.to_matrix() * scm::math::make_scale(scaling);
}

Transformation Transformation::blend(Transformation const& t, float const factor) const {
  return Transformation{scaling * (1 - factor) + t.scaling * factor, slerp(rotation, t.rotation, factor), translation * (1 - factor) + t.translation * factor};
}

Transformation Transformation::operator+(Transformation const& t) const {
  return Transformation{scaling + t.scaling, scm::math::normalize(t.rotation * rotation), translation + t.translation};
}
Transformation& Transformation::operator+=(Transformation const& t) {
  *this = *this + t;
  return *this;
}

Transformation Transformation::operator*(float const factor) const {
  return Transformation{scaling * factor, slerp(scm::math::quatf::identity(), rotation, factor), translation * factor};
}
Transformation& Transformation::operator*=(float const f) {
  *this = *this * f;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
BoneAnimation::BoneAnimation():
  name{"default"},
  scalingKeys{},
  rotationKeys{},
  translationKeys{}
{}

BoneAnimation::~BoneAnimation()
{}

BoneAnimation::BoneAnimation(aiNodeAnim* anim):
  name{anim->mNodeName.C_Str()}
 {

  for(unsigned i = 0; i < anim->mNumScalingKeys; ++i) {
    scalingKeys.push_back(Key<scm::math::vec3>{anim->mScalingKeys[i].mTime, to_gua::vec3(anim->mScalingKeys[i].mValue)});
  }
  for(unsigned i = 0; i < anim->mNumRotationKeys; ++i) {
    rotationKeys.push_back(Key<scm::math::quatf>{anim->mRotationKeys[i].mTime, to_gua::quat(anim->mRotationKeys[i].mValue)});
  }
  for(unsigned i = 0; i < anim->mNumPositionKeys; ++i) {
    translationKeys.push_back(Key<scm::math::vec3>{anim->mPositionKeys[i].mTime, to_gua::vec3(anim->mPositionKeys[i].mValue)});
  }
}

Transformation BoneAnimation::calculate_transform(float time) const {
  return Transformation{calculate_value(time, scalingKeys), calculate_value(time, rotationKeys), calculate_value(time, translationKeys)};
}

std::string const& BoneAnimation::get_name() const {
  return name;
}

scm::math::vec3 BoneAnimation::interpolate(scm::math::vec3 val1, scm::math::vec3 val2, float factor) const {
  return val1 * (1 - factor) + val2 * factor;
}

scm::math::quatf BoneAnimation::interpolate(scm::math::quatf val1, scm::math::quatf val2, float factor) const {
  return normalize(slerp(val1, val2, factor));
}

template<class T> 
uint BoneAnimation::find_key(float animationTime, std::vector<Key<T>> keys) const {    
  if(keys.size() < 1) {
    Logger::LOG_ERROR << "no keys" << std::endl;
    assert(false);
  } 

  for(uint i = 0 ; i < keys.size() - 1 ; i++) {
    if(animationTime < (float)keys[i + 1].time) {
      return i;
    }
  }

  Logger::LOG_ERROR << "no key found" << std::endl;
  assert(false);

  return 0;
}

template<class T> 
T BoneAnimation::calculate_value(float time, std::vector<Key<T>> keys) const {
  if(keys.size() == 1) {
     return keys[0].value;
  }

  uint lastIndex = find_key(time, keys);
  uint nextIndex = (lastIndex + 1);

  if(nextIndex > keys.size()) {
    Logger::LOG_ERROR << "frame out of range" << std::endl;
    assert(false);
  }

  float deltaTime = (float)(keys[nextIndex].time - keys[lastIndex].time);
  float factor = (time - (float)keys[lastIndex].time) / deltaTime;
  //assert(factor >= 0.0f && factor <= 1.0f);
  T const& key1 = keys[lastIndex].value;
  T const& key2 = keys[nextIndex].value;

  return interpolate(key1, key2, factor);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
SkeletalAnimation::SkeletalAnimation():
  name{"default"},
  numFrames{0},
  numFPS{0},
  duration{0},
  numBoneAnims{0},
  boneAnims{}
{}

<<<<<<< HEAD
SkeletalAnimation::SkeletalAnimation(aiAnimation const& anim):
  name{anim.mName.C_Str()},
  numFrames{unsigned(anim.mDuration)},
  numFPS{anim.mTicksPerSecond > 0 ? anim.mTicksPerSecond : 25},
=======
SkeletalAnimation::SkeletalAnimation(aiAnimation* anim, std::string const& file_name):
  //name{anim->mName.C_Str()},
  name{file_name},
  numFrames{unsigned(anim->mDuration)},
  numFPS{anim->mTicksPerSecond > 0 ? anim->mTicksPerSecond : 25},
>>>>>>> add animation name interface for animation control in avango
  duration{double(numFrames) / numFPS},
  numBoneAnims{anim.mNumChannels},
  boneAnims{}
{
  for(unsigned i = 0; i < numBoneAnims; ++i) {
    boneAnims.push_back(BoneAnimation{anim.mChannels[i]});
  }
}

SkeletalAnimation::~SkeletalAnimation()
{}

Pose SkeletalAnimation::calculate_pose(float time) const { 
  Pose pose{};

  float currFrame = time * float(numFrames);
   
  for(BoneAnimation const& boneAnim : boneAnims) {
    Transformation boneTransform = boneAnim.calculate_transform(currFrame);

    pose.set_transform(boneAnim.get_name(), boneTransform);
  }  

  return pose;
}

double SkeletalAnimation::get_duration() const {
  return duration;
}

std::string SkeletalAnimation::get_name() const {
  return name;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Pose::Pose():
  transforms{}
{}

Pose::~Pose()
{}

bool Pose::contains(std::string const& name ) const {
  return transforms.find(name) != transforms.end();
}

Transformation const& Pose::get_transform(std::string const& name) const{
  try {
    return transforms.at(name);
  }
  catch(std::exception const& e) {
    Logger::LOG_ERROR << "bone '" << name << "' not contained in pose" << std::endl;
    return transforms.begin()->second;
  }
}

void Pose::set_transform(std::string const& name, Transformation const& value) {
  transforms[name] = value;
}

void Pose::blend(Pose const& pose2, float blendFactor) {
  for_each(pose2.transforms.cbegin(), pose2.transforms.cend(), [this, &blendFactor](std::pair<std::string, Transformation> const& p) {
    if(contains(p.first)) {
      set_transform(p.first, get_transform(p.first).blend(p.second, blendFactor));
    }
    else {
      set_transform(p.first, p.second);
    }
  });
  // *this = *this * (1 - blendFactor) + pose2 * blendFactor;
}

Pose& Pose::operator+=(Pose const& pose2) {
  for_each(pose2.transforms.cbegin(), pose2.transforms.cend(), [this](std::pair<std::string, Transformation> const& p) {
    if(contains(p.first)) {
      set_transform(p.first, get_transform(p.first) + p.second);
    }
    else {
      set_transform(p.first, p.second);
    }
  });
  return *this;
}
Pose Pose::operator+(Pose const& p2) const {
  Pose temp{*this};
  temp += p2;
  return temp;
}

Pose& Pose::operator*=(float const factor) {
  for(auto& p : transforms)
  {
    p.second *=factor;
  }
  return *this;
}
Pose Pose::operator*(float const factor) const {
  Pose temp{*this};
  temp *= factor;
  return temp;
}

void Pose::partial_replace(Pose const& pose2, std::shared_ptr<Node> const& pNode) {
  if(pose2.contains(pNode->name)) {
    set_transform(pNode->name, pose2.get_transform(pNode->name));
  }

  for(std::shared_ptr<Node>& child : pNode->children) {
    partial_replace(pose2, child);
  }
}

} // namespace gua