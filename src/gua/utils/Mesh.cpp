// class header
#include <gua/utils/Mesh.hpp>

// guacamole headers
#include <gua/utils/Logger.hpp>
#include <gua/utils/ToGua.hpp>
// #include <gua/utils/Timer.hpp>

// external headers
#include <iostream>
#include <assimp/scene.h>
#ifdef GUACAMOLE_FBX
#include <fbxsdk.h>
#endif // GUACAMOLE_FBX

namespace gua
{
Mesh::Mesh() : positions{}, normals{}, texCoords{}, tangents{}, bitangents{}, indices{} {}


Mesh::Mesh(const char* filename) : positions{}, normals{}, texCoords{}, tangents{}, bitangents{}, indices{}, num_vertices{}, num_triangles{}
{
    FILE* f = fopen( filename, "rb");
    if(nullptr == f){
        std::cout << "ERROR: Mesh: could not open file " << filename << " Mesh will remain empty" << std::endl;
        return;
    }

    // header
    unsigned int num_positions(0);
    unsigned int num_normals(0);
    unsigned int num_texCoords(0);
    unsigned int num_tangents(0);
    unsigned int num_bitangents(0);
    unsigned int num_indices(0);
    fread(&num_vertices, sizeof(unsigned), 1, f);
    fread(&num_triangles, sizeof(unsigned), 1, f);
    fread(&num_positions, sizeof(unsigned), 1, f);
    fread(&num_normals, sizeof(unsigned), 1, f);
    fread(&num_texCoords, sizeof(unsigned), 1, f);
    fread(&num_tangents, sizeof(unsigned), 1, f);
    fread(&num_bitangents, sizeof(unsigned), 1, f);
    fread(&num_indices, sizeof(unsigned), 1, f);

    positions.resize(num_positions);
    normals.resize(num_normals);
    texCoords.resize(num_texCoords);
    tangents.resize(num_tangents);
    bitangents.resize(num_bitangents);
    indices.resize(num_indices);

    // data
    fread(&positions[0], sizeof(scm::math::vec3f), num_positions, f);
    fread(&normals[0], sizeof(scm::math::vec3f), num_normals, f);
    fread(&texCoords[0], sizeof(scm::math::vec2f), num_texCoords, f);
    fread(&tangents[0], sizeof(scm::math::vec3f), num_tangents, f);        
    fread(&bitangents[0], sizeof(scm::math::vec3f), num_bitangents, f);
    fread(&indices[0], sizeof(unsigned), num_indices, f);

    fclose(f);

}

#ifdef GUACAMOLE_FBX
Mesh::Mesh(FbxMesh& mesh, int material_index) { construct(mesh, material_index); }

std::vector<unsigned> Mesh::construct(FbxMesh& mesh, int material_index)
{
    // Timer timer{};
    // timer.start();

    // if the given materialindex is valid, assume that only the polys with this material should be loaded
    bool split_materials = material_index >= 0;
    FbxGeometryElementMaterial const* material_layer = mesh.GetElementMaterial(0);

    std::function<unsigned(unsigned)> get_material;

    if(material_layer->GetMappingMode() == FbxGeometryElement::eByPolygon)
    {
        if(material_layer->GetReferenceMode() == FbxGeometryElement::eIndexToDirect)
        {
            get_material = [&material_layer](unsigned polygon_index) -> unsigned { return material_layer->GetIndexArray().GetAt(polygon_index); };
        }
        else
        {
            Logger::LOG_ERROR << "Type of material reference not supported" << std::endl;
        }
    }
    else if(material_layer->GetMappingMode() == FbxGeometryElement::eAllSame)
    {
        get_material = [&material_layer](unsigned polygon_index) -> unsigned { return material_layer->GetIndexArray().GetAt(0); };
    }
    else
    {
        Logger::LOG_ERROR << "Type of material mapping not supported" << std::endl;
    }

    // polygons
    if(mesh.GetPolygonCount() < 1)
    {
        Logger::LOG_ERROR << "No polygons in mesh" << std::endl;
        assert(0);
    }

    // normals
    if(mesh.GetElementNormalCount() == 0)
    {
        // dont override exiting normals and generate by control point, not vertex
        mesh.GenerateNormals(false, true);
    }
    bool vertex_normals = mesh.GetElementNormal(0)->GetMappingMode() == FbxGeometryElement::eByPolygonVertex;

    // UV coordinates
    bool has_uvs = true;
    if(mesh.GetElementUVCount() == 0)
    {
        Logger::LOG_WARNING << "Mesh has no texture coordinates" << std::endl;
        has_uvs = false;
    }
    else
    {
        if(mesh.GetElementUVCount() > 1)
        {
            Logger::LOG_WARNING << "Mesh has multiple UV sets, only using first one" << std::endl;
        }
    }

    // tangents
    bool has_tangents = true;
    if(mesh.GetElementTangentCount() == 0 || mesh.GetElementBinormalCount() == 0)
    {
        if(has_uvs)
        {
            mesh.GenerateTangentsData(0, true);
        }
        else
        {
            Logger::LOG_DEBUG << "No UVs, can't generate tangents" << std::endl;
            has_tangents = false;
        }
    }

    FbxVector4 translation = mesh.GetNode()->GetGeometricTranslation(FbxNode::eSourcePivot);
    FbxVector4 rotation = mesh.GetNode()->GetGeometricRotation(FbxNode::eSourcePivot);
    FbxVector4 scaling = mesh.GetNode()->GetGeometricScaling(FbxNode::eSourcePivot);
    FbxAMatrix geo_transform(translation, rotation, scaling);

    FbxAMatrix identity = FbxAMatrix{};
    identity.SetIdentity();
    if(geo_transform != identity)
    {
        Logger::LOG_WARNING << "Mesh has Geometric Transform, vertices may be skewed." << std::endl;
    }

    // one vector of temp_vert represents one control point, every temp_vert in that vector is one vertex at that point
    std::vector<std::list<temp_vert>> control_points{unsigned(mesh.GetControlPointsCount()), std::list<temp_vert>{}};
    std::vector<temp_tri> temp_tris{};

    // vertex indices of polygons
    int* poly_vertices = mesh.GetPolygonVertices();

    // define function to access vertex properties
    std::function<unsigned(temp_vert const&)> get_normal = get_access_function(*mesh.GetElementNormal(0));
    FbxArray<FbxVector4> poly_normals{};
    // FbxLayerElementArrayTemplate<FbxVector4> poly_normals{mesh.GetElementNormal(0)->GetDirectArray()};
    mesh.GetElementNormal(0)->GetDirectArray().CopyTo(poly_normals);

    std::function<unsigned(temp_vert const&)> get_uv;

    FbxArray<FbxVector2> poly_uvs{};
    if(has_uvs)
    {
        mesh.GetElementUV(0)->GetDirectArray().CopyTo(poly_uvs);
        get_uv = get_access_function(*mesh.GetElementUV(0));
    }

    std::function<unsigned(temp_vert const&)> get_tangent;
    std::function<unsigned(temp_vert const&)> get_bitangent;

    FbxArray<FbxVector4> poly_tangents{};
    FbxArray<FbxVector4> poly_bitangents{};
    // FbxLayerElementArrayTemplate<FbxVector4> const& poly_tangents{has_tangents ? mesh.GetElementTangent(0)->GetDirectArray() : FbxLayerElementArrayTemplate<FbxVector4>{EFbxType::eFbxDouble4}};
    // FbxLayerElementArrayTemplate<FbxVector4> const& poly_bitangents{has_tangents ? mesh.GetElementBinormal(0)->GetDirectArray() : FbxLayerElementArrayTemplate<FbxVector4>{EFbxType::eFbxDouble4}};

    if(has_tangents)
    {
        mesh.GetElementTangent(0)->GetDirectArray().CopyTo(poly_tangents);
        mesh.GetElementBinormal(0)->GetDirectArray().CopyTo(poly_bitangents);

        get_tangent = get_access_function(*mesh.GetElementTangent(0));
        get_bitangent = get_access_function(*mesh.GetElementBinormal(0));
    }

    num_triangles = 0;
    // starting index of the polygon in the index array
    unsigned start_index = 0;
    // iterate over polygons
    for(int i = 0; i < mesh.GetPolygonCount(); ++i)
    {
        if(!split_materials || get_material(i) == material_index)
        {
            // triangulate face if necessary
            for(int j = 2; j < mesh.GetPolygonSize(i); ++j)
            {
                // get indices of vertices in attribute arrays
                std::array<unsigned, 3> indices{start_index, start_index + j - 1, start_index + j};
                // create triangle from vertex indices
                temp_tri tri{unsigned(poly_vertices[indices[0]]), unsigned(poly_vertices[indices[1]]), unsigned(poly_vertices[indices[2]])};
                temp_tris.push_back(tri);

                // create new vertices that form triangle
                temp_vert vert1{indices[0], tri.verts[0], num_triangles, 0};
                temp_vert vert2{indices[1], tri.verts[1], num_triangles, 1};
                temp_vert vert3{indices[2], tri.verts[2], num_triangles, 2};

                // set normals only if they vary by vertex
                if(vertex_normals)
                {
                    vert1.normal = to_gua::vec3f(poly_normals[get_normal(vert1)]);
                    vert2.normal = to_gua::vec3f(poly_normals[get_normal(vert2)]);
                    vert3.normal = to_gua::vec3f(poly_normals[get_normal(vert3)]);
                }

                // set optional data
                if(has_uvs)
                {
                    vert1.uv = to_gua::vec2f(poly_uvs[get_uv(vert1)]);
                    vert2.uv = to_gua::vec2f(poly_uvs[get_uv(vert2)]);
                    vert3.uv = to_gua::vec2f(poly_uvs[get_uv(vert3)]);
                }
                if(has_tangents)
                {
                    vert1.tangent = to_gua::vec3f(poly_tangents[get_tangent(vert1)]);
                    vert2.tangent = to_gua::vec3f(poly_tangents[get_tangent(vert2)]);
                    vert3.tangent = to_gua::vec3f(poly_tangents[get_tangent(vert3)]);

                    vert1.bitangent = to_gua::vec3f(poly_bitangents[get_bitangent(vert1)]);
                    vert2.bitangent = to_gua::vec3f(poly_bitangents[get_bitangent(vert2)]);
                    vert3.bitangent = to_gua::vec3f(poly_bitangents[get_bitangent(vert3)]);
                }

                // add new vertices to respective control points
                control_points[tri.verts[0]].push_back(vert1);
                control_points[tri.verts[1]].push_back(vert2);
                control_points[tri.verts[2]].push_back(vert3);

                ++num_triangles;
            }
        }
        start_index += mesh.GetPolygonSize(i);
    }

    // filter out duplicate vertices
    num_vertices = 0;
    unsigned old_num_vertices = 0;
    unsigned dupl_verts = 0;
    for(auto ctrl_pt_iter = control_points.begin(); ctrl_pt_iter != control_points.end(); ++ctrl_pt_iter)
    {
        // skip control points without vertices
        if(ctrl_pt_iter->empty())
        {
            ++ctrl_pt_iter;
        }
        if(ctrl_pt_iter == control_points.end())
            break;

        old_num_vertices += ctrl_pt_iter->size();

        for(auto iter = ctrl_pt_iter->begin(); iter != ctrl_pt_iter->end(); ++iter)
        {
            // iterate over vertices behind current vertex
            for(auto iter2 = std::next(iter); iter2 != ctrl_pt_iter->end(); ++iter2)
            {
                // match by normals and if exisiting, other attributes
                bool duplicate = true;
                if(vertex_normals)
                    duplicate = duplicate && iter2->normal == iter->normal;
                if(has_uvs)
                    duplicate = duplicate && iter2->uv == iter->uv;
                if(has_tangents)
                    duplicate = duplicate && iter2->tangent == iter->tangent && iter2->bitangent == iter->bitangent;
                // duplicate -> merge vertices
                if(duplicate)
                {
                    // add triangle of duplicate vertex to current vertex
                    iter->tris.push_back(iter2->tris[0]);
                    // removing element invalidates iterators
                    // assign result to first iter to continue iterating
                    iter2 = ctrl_pt_iter->erase(iter2);
                    if(iter2 == ctrl_pt_iter->end())
                        break;
                    ++dupl_verts;
                }
            }
        }
        // add number of filtered verts at current control point to total vertex number
        num_vertices += ctrl_pt_iter->size();
    }

    bool empty = true;
    for(auto const& cp : control_points)
    {
        if(!cp.empty())
        {
            empty = false;
            break;
        }
    }
    if(empty)
    {
        throw std::runtime_error("vertex list empty");
    }
    // Reserve space in the vectors for the vertex attributes and indices
    positions.reserve(num_vertices);
    normals.reserve(num_vertices);
    if(has_uvs)
    {
        texCoords.reserve(num_vertices);
    }
    else
    {
        texCoords.resize(num_vertices, scm::math::vec2f(0.0f));
    }
    if(has_tangents)
    {
        tangents.reserve(num_vertices);
        bitangents.reserve(num_vertices);
    }
    else
    {
        tangents.resize(num_vertices, scm::math::vec3f(0.0f));
        bitangents.resize(num_vertices, scm::math::vec3f(0.0f));
    }

    // save which vertex lies on which controlpoint
    std::vector<unsigned> point_indices{};

    // load reduced attributes
    unsigned curr_vert = 0;
    scm::math::vec3f curr_position{};
    scm::math::vec3f curr_normal{};
    // iterate over control points
    for(unsigned i = 0; i < control_points.size(); ++i)
    {
        // skip control points without vertices
        if(control_points[i].empty())
            continue;
        // get position once per point, all vertices at this control point have this position
        curr_position = to_gua::vec3f(mesh.GetControlPointAt(i));

        // if normal is not unique to vertex, all vertices at this point also have the same normal
        if(!vertex_normals)
        {
            auto const& positions = control_points.at(i);
            auto const& front = positions.front();
            auto const& normal_idx = get_normal(front);
            auto const& normal = poly_normals[normal_idx];
            curr_normal = to_gua::vec3f(normal);
            // curr_normal = to_gua::vec3f(poly_normals[get_normal(control_points[i].front())]);
        }

        // iterate over vertices at that point
        for(auto const& vert : control_points[i])
        {
            // update containing triangles with actual index of this vertex in member vectors
            for(auto const& tri : vert.tris)
            {
                temp_tris[tri.first].verts[tri.second] = curr_vert;
            }
            // push properties to attribute vectors
            positions.push_back(curr_position);

            if(vertex_normals)
            {
                normals.push_back(vert.normal);
            }
            else
            {
                normals.push_back(curr_normal);
            }

            if(has_tangents)
            {
                tangents.push_back(vert.tangent);
                bitangents.push_back(vert.bitangent);
            }
            if(has_uvs)
            {
                texCoords.push_back(vert.uv);
            }
            ++curr_vert;

            point_indices.push_back(i);
        }
    }
    // free memory
    std::vector<std::list<temp_vert>>{}.swap(control_points);

    // load reduced triangles
    indices.reserve(num_triangles * 3);
    for(auto const& tri : temp_tris)
    {
        indices.push_back(tri.verts[0]);
        indices.push_back(tri.verts[1]);
        indices.push_back(tri.verts[2]);
    }

    // output reduction info
    // Logger::LOG_DEBUG << "Number of vertices reduced from " << old_num_vertices << " to " << num_vertices << " ,time taken: " << timer.get_elapsed() << std::endl;

    return point_indices;
}

// this function gets a geometry layer and returns the function to access it depending on mapping & referencing
template <typename T>
std::function<unsigned(Mesh::temp_vert const&)> Mesh::get_access_function(FbxLayerElementTemplate<T> const& layer)
{
    std::function<unsigned(temp_vert const&)> access_function;
    // mapping to control point
    if(layer.GetMappingMode() == FbxGeometryElement::eByControlPoint)
    {
        if(layer.GetReferenceMode() == FbxGeometryElement::eDirect)
        {
            access_function = [](temp_vert const& vert) -> unsigned {
                // std::cout << "accessing " << vert.point << std::endl;
                return vert.point;
            };
        }
        else if(layer.GetReferenceMode() == FbxGeometryElement::eIndexToDirect)
        {
            access_function = [&layer](temp_vert const& vert) -> unsigned { return layer.GetIndexArray().GetAt(vert.point); };
        }
        else
        {
            Logger::LOG_ERROR << "Type of reference not supported" << std::endl;
        }
    }
    // mapping to vertex
    else if(layer.GetMappingMode() == FbxGeometryElement::eByPolygonVertex)
    {
        if(layer.GetReferenceMode() == FbxGeometryElement::eDirect)
        {
            access_function = [](temp_vert const& vert) -> unsigned { return vert.old_index; };
        }
        else if(layer.GetReferenceMode() == FbxGeometryElement::eIndexToDirect)
        {
            access_function = [&layer](temp_vert const& vert) -> unsigned { return layer.GetIndexArray().GetAt(vert.old_index); };
        }
        else
        {
            Logger::LOG_ERROR << "Type of reference not supported" << std::endl;
        }
    }
    else
    {
        Logger::LOG_ERROR << "Type of mapping not supported" << std::endl;
    }

    return access_function;
};

#endif // GUACAMOLE_FBX

Mesh::Mesh(aiMesh const& mesh)
{

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
    for(unsigned int i = 0; i < mesh.mNumVertices; i++)
    {
        scm::math::vec3f pPos = scm::math::vec3f(0.0f);
        if(mesh.HasPositions())
        {
            pPos = to_gua::vec3f(mesh.mVertices[i]);
        }

        scm::math::vec3f pNormal = scm::math::vec3f(0.0f);
        if(mesh.HasNormals())
        {
            pNormal = to_gua::vec3f(mesh.mNormals[i]);
        }

        scm::math::vec2f pTexCoord = scm::math::vec2(0.0f);
        if(mesh.HasTextureCoords(0))
        {
            pTexCoord = scm::math::vec2(mesh.mTextureCoords[0][i].x, mesh.mTextureCoords[0][i].y);
        }

        scm::math::vec3f pTangent = scm::math::vec3f(0.0f);
        scm::math::vec3f pBitangent = scm::math::vec3f(0.0f);
        if(mesh.HasTangentsAndBitangents())
        {
            pTangent = to_gua::vec3f(mesh.mTangents[i]);

            pBitangent = to_gua::vec3f(mesh.mBitangents[i]);
        }

        positions.push_back(pPos);
        normals.push_back(pNormal);
        bitangents.push_back(pBitangent);
        tangents.push_back(pTangent);
        texCoords.push_back(pTexCoord);
    }

    // Populate the index buffer
    for(unsigned int i = 0; i < mesh.mNumFaces; i++)
    {
        const aiFace& face = mesh.mFaces[i];
        // triangulate face if necessary
        for(unsigned j = 2; j < face.mNumIndices; ++j)
        {
            indices.push_back(face.mIndices[0]);
            indices.push_back(face.mIndices[j - 1]);
            indices.push_back(face.mIndices[j]);
        }
    }

}

void Mesh::copy_to_buffer(Vertex* vertex_buffer) const
{
    for(unsigned v(0); v < num_vertices; ++v)
    {
        vertex_buffer[v].pos = positions[v];

        vertex_buffer[v].tex = texCoords[v];

        vertex_buffer[v].normal = normals[v];

        if((!tangents.empty()) && (!bitangents.empty())){
            vertex_buffer[v].tangent = tangents[v];
            vertex_buffer[v].bitangent = bitangents[v];    
        }
        else if(!tangents.empty()){
            vertex_buffer[v].tangent = tangents[v];    
        }
        
    }
}

scm::gl::vertex_format Mesh::get_vertex_format() const
{
    if((!tangents.empty()) && (!bitangents.empty())){
        //std::cout << "Mesh::get_vertex_format() TANGETS + BITANGENTS" << std::endl;
        return scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC3F, sizeof(Vertex))(0, 1, scm::gl::TYPE_VEC2F, sizeof(Vertex))(0, 2, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
        0, 3, scm::gl::TYPE_VEC3F, sizeof(Vertex))(0, 4, scm::gl::TYPE_VEC3F, sizeof(Vertex));
    }
    else if(!tangents.empty()){
        //std::cout << "Mesh::get_vertex_format() TANGETS" << std::endl;
        return scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC3F, sizeof(Vertex))
                                     (0, 1, scm::gl::TYPE_VEC2F, sizeof(Vertex))
                                     (0, 2, scm::gl::TYPE_VEC3F, sizeof(Vertex))
                                     (0, 3, scm::gl::TYPE_VEC3F, sizeof(Vertex));
    }
    else{
        //std::cout << "Mesh::get_vertex_format()" << std::endl;
        return scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC3F, sizeof(Vertex))
                                     (0, 1, scm::gl::TYPE_VEC2F, sizeof(Vertex))
                                     (0, 2, scm::gl::TYPE_VEC3F, sizeof(Vertex));
    }
}


bool Mesh::save_to_binary(const char* filename, unsigned flags) const
{

    FILE* f = fopen( filename, "wb");
    if(nullptr == f){
        return false;
    }

    // header
    const unsigned int num_positions(positions.size());
    const unsigned int num_normals(normals.size());
    const unsigned int num_texCoords(texCoords.size());
    unsigned int num_tangents(tangents.size());
    unsigned int num_bitangents(bitangents.size());
    const unsigned int num_indices(indices.size());

    if(!(flags & Mesh::SAVE_TANGENTS)){
        num_tangents = 0;
        num_bitangents = 0; // BITANGENTS WITHOUT TANGENTS IS NOT SUPPORTED
    }

    if(!(flags & Mesh::SAVE_BITANGENTS)){
        num_bitangents = 0;
    }


    fwrite(&num_vertices, sizeof(unsigned), 1, f);
    fwrite(&num_triangles, sizeof(unsigned), 1, f);
    fwrite(&num_positions, sizeof(unsigned), 1, f);
    fwrite(&num_normals, sizeof(unsigned), 1, f);
    fwrite(&num_texCoords, sizeof(unsigned), 1, f);
    fwrite(&num_tangents, sizeof(unsigned), 1, f);
    fwrite(&num_bitangents, sizeof(unsigned), 1, f);
    fwrite(&num_indices, sizeof(unsigned), 1, f);

    // data
    fwrite(&positions[0], sizeof(scm::math::vec3f), num_positions, f);
    fwrite(&normals[0], sizeof(scm::math::vec3f), num_normals, f);
    fwrite(&texCoords[0], sizeof(scm::math::vec2f), num_texCoords, f);
    fwrite(&tangents[0], sizeof(scm::math::vec3f), num_tangents, f);        
    fwrite(&bitangents[0], sizeof(scm::math::vec3f), num_bitangents, f);
    fwrite(&indices[0], sizeof(unsigned), num_indices, f);

    fclose(f);

    return true;
}

} // namespace gua