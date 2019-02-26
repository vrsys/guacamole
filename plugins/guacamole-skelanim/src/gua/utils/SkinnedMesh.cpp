// class header
#include <gua/skelanim/utils/SkinnedMesh.hpp>

// guacamole headers
#include <gua/utils/Mesh.hpp>
#include <gua/utils/Logger.hpp>

// external headers
#ifdef GUACAMOLE_FBX
#include <fbxsdk.h>
#endif
#include <map>
#include <assimp/scene.h>

namespace gua
{
SkinnedMesh::SkinnedMesh() : Mesh{}, bone_ids{}, bone_weights{}, bone_counts{} {}

#ifdef GUACAMOLE_FBX
////////////////////////////////////////////////////////////////////////////////////
SkinnedMesh::SkinnedMesh(FbxMesh& mesh, Skeleton const& skeleton, unsigned const material_index)
{
    // weights are associated to controlpoints and there can be control points with
    // same positions after another
    // so its not possible to get the controlpoint associations just from positions
    std::vector<unsigned> point_indices{construct(mesh, material_index)};

    bool has_weights = false;
    std::vector<bone_influences> ctrlpt_weights{};
    // get weights of original control points
    if(skeleton.num_bones() > 0)
    {
        ctrlpt_weights = get_weights(mesh, skeleton);
        has_weights = ctrlpt_weights.size() > 0;
    }

    if(!has_weights)
    {
        // just map to first bone, with no mapping or weight 0 triangles wouldnt be
        // rendered
        bone_counts.resize(num_vertices, 1);
        bone_ids.resize(num_vertices, 0);
        bone_weights.resize(num_vertices, 1.0f);
    }
    else
    {
        bone_counts.reserve(num_vertices);
        bone_ids.reserve(num_vertices);
        bone_weights.reserve(num_vertices);

        // iterate over vertices
        for(unsigned index : point_indices)
        {
            bone_influences const& curr_influence{ctrlpt_weights[index]};
            // push all bone influences for current vert
            for(unsigned j = 0; j < curr_influence.weights.size(); ++j)
            {
                bone_ids.push_back(curr_influence.IDs[j]);
                bone_weights.push_back(curr_influence.weights[j]);
            }

            bone_counts.push_back(curr_influence.weights.size());
        }
    }
}

std::vector<SkinnedMesh::bone_influences> SkinnedMesh::get_weights(FbxMesh const& mesh, Skeleton const& skeleton)
{
    auto const& bone_mapping = skeleton.get_mapping();

    // check for skinning
    FbxSkin* skin = nullptr;
    for(size_t i = 0; i < size_t(mesh.GetDeformerCount()); ++i)
    {
        FbxDeformer* defPtr = {mesh.GetDeformer(i)};
        if(defPtr->GetDeformerType() == FbxDeformer::eSkin)
        {
            skin = static_cast<FbxSkin*>(defPtr);
            break;
        }
    }

    if(!skin)
    {
        Logger::LOG_WARNING << "Mesh does not contain skin deformer, ignoring weights" << std::endl;
        return std::vector<bone_influences>{};
    }
    // set up temporary weights, for control points, not actual vertices
    std::vector<bone_influences> temp_weights{unsigned(mesh.GetControlPointsCount())};
    // one cluster corresponds to one bone
    for(size_t i = 0; i < size_t(skin->GetClusterCount()); ++i)
    {
        FbxCluster* cluster = skin->GetCluster(i);
        FbxNode* bone = cluster->GetLink();

        if(!bone)
        {
            Logger::LOG_ERROR << "associated Bone does not exist!" << std::endl;
            assert(false);
        }

        std::string bone_name{bone->GetName()};
        unsigned bone_index;
        if(bone_mapping.find(bone_name) != bone_mapping.end())
        {
            bone_index = bone_mapping.at(bone_name);
        }
        else
        {
            Logger::LOG_ERROR << "Bone with name '" << bone_name << "' does not exist!, ignoring weights" << std::endl;
            return std::vector<bone_influences>{};
        }

        int* indices = cluster->GetControlPointIndices();
        double* weights = cluster->GetControlPointWeights();
        for(size_t i = 0; i < size_t(cluster->GetControlPointIndicesCount()); ++i)
        {
            // update mapping info of current control point
            temp_weights[indices[i]].add_bone(bone_index, weights[i]);
        }
    }

    return temp_weights;
}

#endif
///////////////////////////////////////////////////////////////////////////////

SkinnedMesh::SkinnedMesh(aiMesh const& mesh, Skeleton const& skeleton) : Mesh{mesh}
{
    // get weights and write them to vectors
    auto temp_weights = get_weights(mesh, skeleton);
    for(auto const& w : temp_weights)
    {
        for(unsigned i(0); i < w.weights.size(); ++i)
        {
            bone_ids.push_back(w.IDs[i]);
            bone_weights.push_back(w.weights[i]);
        }
        bone_counts.push_back(w.weights.size());
    }
}

std::vector<SkinnedMesh::bone_influences> SkinnedMesh::get_weights(aiMesh const& mesh, Skeleton const& skeleton)
{
    auto const& bone_mapping = skeleton.get_mapping();

    std::vector<bone_influences> temp_weights{mesh.mNumVertices};

    for(unsigned i = 0; i < mesh.mNumBones; i++)
    {
        std::string bone_name{mesh.mBones[i]->mName.data};
        unsigned bone_index = bone_mapping.at(bone_name);

        for(unsigned j = 0; j < mesh.mBones[i]->mNumWeights; j++)
        {
            unsigned vertex_index = mesh.mBones[i]->mWeights[j].mVertexId;
            float weight = mesh.mBones[i]->mWeights[j].mWeight;
            temp_weights[vertex_index].add_bone(bone_index, weight);
        }
    }

    return temp_weights;
}

///////////////////////////////////////////////////////////////////////////////
void SkinnedMesh::copy_to_buffer(Vertex* vertex_buffer, unsigned resource_offset_bytes) const
{
    unsigned bone_offset = resource_offset_bytes / sizeof(unsigned);

    for(unsigned v(0); v < num_vertices; ++v)
    {
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

scm::gl::vertex_format SkinnedMesh::get_vertex_format() const
{
    return scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC3F, sizeof(Vertex))(0, 1, scm::gl::TYPE_VEC2F, sizeof(Vertex))(0, 2, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
        0, 3, scm::gl::TYPE_VEC3F, sizeof(Vertex))(0, 4, scm::gl::TYPE_VEC3F, sizeof(Vertex))(0, 5, scm::gl::TYPE_UINT, sizeof(Vertex))(0, 6, scm::gl::TYPE_UINT, sizeof(Vertex));
}

std::vector<unsigned> const& SkinnedMesh::get_bone_ids() const { return bone_ids; }

std::vector<float> const& SkinnedMesh::get_bone_weights() const { return bone_weights; }

} // namespace gua
