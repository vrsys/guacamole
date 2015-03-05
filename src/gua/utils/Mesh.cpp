// class header
#include <gua/utils/Mesh.hpp>
//external headers
#include <iostream>
#include <queue>

namespace to_gua{

scm::math::mat4f mat4(aiMatrix4x4 const& m) {
  scm::math::mat4f res(m.a1,m.b1,m.c1,m.d1
                      ,m.a2,m.b2,m.c2,m.d2
                      ,m.a3,m.b3,m.c3,m.d3
                      ,m.a4,m.b4,m.c4,m.d4);
  return res;
}
scm::math::quatf quat(aiQuaternion const& q) {
  scm::math::quatf res(q.w, q.x, q.y, q.z);
  return res;
}
#ifdef GUACAMOLE_FBX
scm::math::mat4f mat4(FbxAMatrix const& m) {
  scm::math::mat4f res(m[0][0],m[0][1],m[0][2],m[0][3],
                      m[1][0],m[1][1],m[1][2],m[1][3],
                      m[2][0],m[2][1],m[2][2],m[2][3],
                      m[3][0],m[3][1],m[3][2],m[3][3]);
  return res;
}
scm::math::quatf quat(FbxQuaternion const& q) {
  scm::math::quatf res(q[3], q[0], q[1], q[2]);
  return res;
}
#endif
}

namespace gua {

Mesh::Mesh():
 positions{},
 normals{},
 texCoords{},
 tangents{},
 bitangents{},
 indices{}
{}
#ifdef GUACAMOLE_FBX
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

  FbxVector4 translation = mesh.GetNode()->GetGeometricTranslation(FbxNode::eSourcePivot);
  FbxVector4 rotation = mesh.GetNode()->GetGeometricRotation(FbxNode::eSourcePivot);
  FbxVector4 scaling = mesh.GetNode()->GetGeometricScaling(FbxNode::eSourcePivot);
  FbxAMatrix geo_transform(translation, rotation, scaling);

  FbxAMatrix identity = FbxAMatrix{};
  identity.SetIdentity();
  if(geo_transform != identity) {
    Logger::LOG_WARNING << "Mesh has Geometric Transform, vertices will be skewed." << std::endl;
  }

//normals
  if(mesh.GetElementNormalCount() == 0) {
    //dont override exiting normals and generate by control point, not vertex
    mesh.GenerateNormals(false, true);
  }

//tangents
  bool has_tangents = true;
  if(mesh.GetElementTangentCount() == 0 || mesh.GetElementBinormalCount() == 0) {
    if(has_uvs) { 
      Logger::LOG_DEBUG << "Generating tangents" << std::endl;
      mesh.GenerateTangentsData(UV_set.c_str(), true);
    }
    else {
      Logger::LOG_DEBUG << "No UVs, can't generate tangents" << std::endl;
      has_tangents = false;
    }
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
    temp_vert(unsigned oindex, unsigned pt, unsigned norm, unsigned tr, unsigned ind):
     old_index{oindex},
     point{pt},
     normal{norm},
     uv{0},
     tangent{0},
     bitangent{0},
     tris{}
    {
      tris.push_back(std::make_pair(tr, ind));
    }
    unsigned old_index;
    unsigned point;
    unsigned normal;
    unsigned tangent;
    unsigned bitangent;
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
  Logger::LOG_DEBUG << dupl_normals << " normal duplications" << std::endl;

  FbxArray<FbxVector2> poly_uvs{};
  std::vector<unsigned> uv_indices{};
  if(has_uvs) {
    mesh.GetPolygonVertexUVs(UV_set.c_str(), poly_uvs);
    //index uvs, filter out duplicates
    unsigned dupl_uvs = 0;
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
    Logger::LOG_DEBUG << dupl_uvs << " UV duplications" << std::endl;
  }

  //this function gets a geometry layer and returns the function to access it depending on mapping & referencing
  auto get_access_function = [](FbxLayerElementTemplate<FbxVector4> const& layer) {
    std::function<unsigned(temp_vert const&)> access_function;
    //mapping to control point
    if(layer.GetMappingMode() == FbxGeometryElement::eByControlPoint) {
      if(layer.GetReferenceMode() == FbxGeometryElement::eDirect) {
        access_function =[](temp_vert const& vert)->unsigned {
          return vert.point;
        };
      }
      else if(layer.GetReferenceMode() == FbxGeometryElement::eIndexToDirect) {
        access_function = [&layer](temp_vert const& vert)->unsigned {
          return layer.GetIndexArray().GetAt(vert.point);
        };
      }
      else {
        Logger::LOG_ERROR << "Type of reference not supported" << std::endl;
      }
    }
    //mapping to vertex
    else if(layer.GetMappingMode() == FbxGeometryElement::eByPolygonVertex){
      if(layer.GetReferenceMode() == FbxGeometryElement::eDirect) {
        access_function =[](temp_vert const& vert)->unsigned {
          return vert.old_index;
        };
      }
      else if(layer.GetReferenceMode() == FbxGeometryElement::eIndexToDirect) {
        access_function = [&layer](temp_vert const& vert)->unsigned {
          return layer.GetIndexArray().GetAt(vert.old_index);
        };
      }
      else {
        Logger::LOG_ERROR << "Type of reference not supported" << std::endl;
      }
    }
    else {
      Logger::LOG_ERROR << "Type of mapping not supported" << std::endl;
    }

    return access_function;
  };

  //define function to get tangent indices
  std::function<unsigned(temp_vert const&)> get_tangent;
  std::function<unsigned(temp_vert const&)> get_bitangent;

  FbxArray<FbxVector4> poly_tangents;
  FbxArray<FbxVector4> poly_bitangents;

  std::vector<unsigned> tangent_indices{};
  std::vector<unsigned> bitangent_indices{};
  if(has_tangents){
    FbxGeometryElementTangent* tangent_info = mesh.GetElementTangent(0);
    tangent_info->GetDirectArray().CopyTo(poly_tangents);
    get_tangent = get_access_function(*tangent_info);
    
    unsigned dupl_tangents = 0;
    //index tangents, filter out duplicates to allow vertex duplication testing by tangent
    for(unsigned i = 0; i < poly_tangents.Size(); ++i) {
      unsigned d = poly_tangents.Find(poly_tangents[i]);
      if(d == i) {
        tangent_indices.push_back(i);
      }
      else {
        tangent_indices.push_back(d); 
        ++dupl_tangents;
      }
    } 
    Logger::LOG_DEBUG << dupl_tangents << " tangent duplications" << std::endl;

    FbxGeometryElementBinormal* bitangent_info = mesh.GetElementBinormal(0);
    bitangent_info->GetDirectArray().CopyTo(poly_bitangents);
    get_bitangent = get_access_function(*bitangent_info);

    unsigned dupl_bitangents = 0;
    //index tangents, filter out duplicatest to allow vertex duplication testing by tangent
    for(unsigned i = 0; i < poly_bitangents.Size(); ++i) {
      unsigned d = poly_bitangents.Find(poly_bitangents[i]);
      if(d == i) {
        bitangent_indices.push_back(i);
      }
      else {
        bitangent_indices.push_back(d); 
        ++dupl_bitangents;
      }
    } 
    Logger::LOG_DEBUG << dupl_bitangents << " bitangent duplications" << std::endl;
  }

  num_triangles = 0; 
  //starting index of the polygon in the index array
  unsigned start_index = 0;
  //how many tris the last polygon contained
  unsigned tris_added = 0;
  for(unsigned i = 0; i < mesh.GetPolygonCount(); ++i)
  {
    //triangulate face if necessary
    for(unsigned j = 2; j < mesh.GetPolygonSize(i); ++j)
    {
      std::array<unsigned, 3> indices{start_index, start_index + j - 1, start_index + j};

      temp_tri tri{unsigned(poly_vertices[indices[0]]), unsigned(poly_vertices[indices[1]]), unsigned(poly_vertices[indices[2]])};
      temp_tris.push_back(tri);

      //add vertices to to array
      std::vector<temp_vert>& curr_point1 = temp_verts[tri.verts[0]];
      curr_point1.push_back(temp_vert{indices[0], tri.verts[0], normal_indices[indices[0]], num_triangles, 0});
      
      std::vector<temp_vert>& curr_point2 = temp_verts[tri.verts[1]];
      curr_point2.push_back(temp_vert{indices[1], tri.verts[1], normal_indices[indices[1]], num_triangles, 1});
      
      std::vector<temp_vert>& curr_point3 = temp_verts[tri.verts[2]];
      curr_point3.push_back(temp_vert{indices[2], tri.verts[2], normal_indices[indices[2]], num_triangles, 2});
      //set optional data
      if(has_uvs) {
        curr_point1[curr_point1.size()-1].uv = uv_indices[indices[0]];
        
        curr_point2[curr_point2.size()-1].uv = uv_indices[indices[1]];
        
        curr_point3[curr_point3.size()-1].uv = uv_indices[indices[2]]; 
      }
      if(has_tangents) {
        curr_point1[curr_point1.size()-1].tangent = tangent_indices[get_tangent(curr_point1[curr_point1.size()-1])];
        curr_point1[curr_point1.size()-1].bitangent = bitangent_indices[get_bitangent(curr_point1[curr_point1.size()-1])];
        
        curr_point2[curr_point2.size()-1].tangent = tangent_indices[get_tangent(curr_point2[curr_point2.size()-1])];
        curr_point2[curr_point2.size()-1].bitangent = bitangent_indices[get_bitangent(curr_point2[curr_point2.size()-1])];
        
        curr_point3[curr_point3.size()-1].tangent = tangent_indices[get_tangent(curr_point3[curr_point3.size()-1])];
        curr_point3[curr_point3.size()-1].bitangent = bitangent_indices[get_bitangent(curr_point3[curr_point3.size()-1])];
      }
      ++tris_added;
      ++num_triangles;
    }
    start_index += 2 + tris_added;
    tris_added = 0;
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
        bool duplicate = iter2->normal == iter->normal;
        duplicate = has_uvs ? (iter2->uv == iter->uv && duplicate) : duplicate; 
        duplicate = has_tangents ? (iter2->tangent == iter->tangent && iter2->bitangent == iter->bitangent && duplicate) : duplicate; 
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
  Logger::LOG_DEBUG << dupl_verts << " vertex duplications" << std::endl;

  // Reserve space in the vectors for the vertex attributes and indices
  positions.reserve(num_vertices);
  normals.reserve(num_vertices);
  if(has_uvs) {
    texCoords.reserve(num_vertices);
  }
  else {
    texCoords.resize(num_vertices, scm::math::vec2(0.0f));
  }
  if(has_tangents) {
    tangents.reserve(num_vertices);
  }
  else {
    tangents.resize(num_vertices, scm::math::vec3f(0.0f));
    bitangents.resize(num_vertices, scm::math::vec3f(0.0f));
  }
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
      if(has_tangents) {
        tangents.push_back(to_gua::vec3(poly_tangents[vert.tangent]));
        bitangents.push_back(to_gua::vec3(poly_bitangents[vert.bitangent]));
      }
      if(has_uvs) {
        texCoords.push_back(to_gua::vec3(poly_uvs[vert.uv]));
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
  Logger::LOG_DEBUG << "Number of vertices reduced from " << old_num_vertices << " to " << num_vertices << std::endl;
}
#endif // GUACAMOLE_FBX

Mesh::Mesh(aiMesh const& mesh) {   
  num_triangles = mesh.mNumFaces;
  num_vertices = mesh.mNumVertices; 
  
  // Reserve space in the vectors for the vertex attributes and indices
  positions.reserve(num_vertices);
  normals.reserve(num_vertices);
  texCoords.reserve(num_vertices);
  tangents.reserve(num_vertices);
  bitangents.reserve(num_vertices);
  indices.reserve(num_triangles * 3);


  // Populate the vertex attribute vectors
  for (uint i = 0 ; i < mesh.mNumVertices ; i++) {
    
    scm::math::vec3f pPos = scm::math::vec3f(0.0f);
    if(mesh.HasPositions()) {
      pPos = to_gua::vec3(mesh.mVertices[i]);
    }

    scm::math::vec3f pNormal = scm::math::vec3f(0.0f);
    if(mesh.HasNormals()) {
      pNormal = to_gua::vec3(mesh.mNormals[i]);
    }

    scm::math::vec2f pTexCoord = scm::math::vec2(0.0f);
    if(mesh.HasTextureCoords(0)) {
      pTexCoord = scm::math::vec2(mesh.mTextureCoords[0][i].x, mesh.mTextureCoords[0][i].y);
    }

    scm::math::vec3f pTangent = scm::math::vec3f(0.0f);
    scm::math::vec3f pBitangent = scm::math::vec3f(0.0f);
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
  
  // Populate the index buffer
  for (uint i = 0 ; i < mesh.mNumFaces ; i++) {
    const aiFace& face = mesh.mFaces[i];
    // triangulate face if necessary
    for(unsigned j = 2; j < face.mNumIndices; ++j) {
      indices.push_back(face.mIndices[0]);
      indices.push_back(face.mIndices[j - 1]);
      indices.push_back(face.mIndices[j]);
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
  }
}

scm::gl::vertex_format Mesh::get_vertex_format() const {
  return scm::gl::vertex_format(
    0, 0, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
    0, 1, scm::gl::TYPE_VEC2F, sizeof(Vertex))(
    0, 2, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
    0, 3, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
    0, 4, scm::gl::TYPE_VEC3F, sizeof(Vertex));
}

} // namespace gua