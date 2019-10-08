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
#include <gua/skelanim/renderer/SkinnedMeshResource.hpp>

// guacamole headers
#include <gua/utils/Logger.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/math/BoundingBoxAlgo.hpp>
#include <gua/skelanim/renderer/SkeletalAnimationRenderer.hpp>

// external headers
#include <scm/gl_core/buffer_objects/scoped_buffer_map.h>

namespace gua
{
SkinnedMeshResource::SkinnedMeshResource() : mesh_() {}

////////////////////////////////////////////////////////////////////////////////

SkinnedMeshResource::SkinnedMeshResource(SkinnedMesh const& mesh, bool build_kd_tree) : mesh_(mesh)
{
    // TODO generate BBox and KDTree
    // if (mesh_->HasPositions()) {
    bounding_box_ = math::BoundingBox<math::vec3>();

    // without bone influence
    for(unsigned v(0); v < mesh_.num_vertices; ++v)
    {
        bounding_box_.expandBy(math::vec3{mesh_.positions[v]});
    }

    bone_boxes_ = std::vector<math::BoundingBox<math::vec3>>(100, math::BoundingBox<math::vec3>());

    // init non transformated/animated bone boxes
    init_bone_boxes();

    // TODO
    /*if (build_kd_tree) {
      kd_tree_.generate(mesh_);
    }
  //}*/
}

////////////////////////////////////////////////////////////////////////////////

void SkinnedMeshResource::upload_to(RenderContext& ctx, SharedSkinningResource& resource) /*const*/
{
    RenderContext::Mesh cmesh{};
    cmesh.indices_topology = scm::gl::PRIMITIVE_TRIANGLE_LIST;
    cmesh.indices_type = scm::gl::TYPE_UINT;
    cmesh.indices_count = mesh_.num_triangles * 3;
    if(mesh_.num_vertices <= 0)
    {
        Logger::LOG_WARNING << "Unable to load Mesh! Has no vertex data." << std::endl;
        return;
    }

    cmesh.vertices = ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER, scm::gl::USAGE_STATIC_DRAW, mesh_.num_vertices * sizeof(SkinnedMesh::Vertex), 0);

    SkinnedMesh::Vertex* data(static_cast<SkinnedMesh::Vertex*>(ctx.render_context->map_buffer(cmesh.vertices, scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER)));

    mesh_.copy_to_buffer(data, resource.offset_bytes);

    ctx.render_context->unmap_buffer(cmesh.vertices);

    cmesh.indices = ctx.render_device->create_buffer(scm::gl::BIND_INDEX_BUFFER, scm::gl::USAGE_STATIC_DRAW, mesh_.num_triangles * 3 * sizeof(unsigned), mesh_.indices.data());

    cmesh.vertex_array = ctx.render_device->create_vertex_array(mesh_.get_vertex_format(), {cmesh.vertices});
    ctx.meshes[uuid()] = cmesh;

    std::size_t new_id_bytes = mesh_.get_bone_ids().size() * sizeof(unsigned);
    std::size_t new_weight_bytes = mesh_.get_bone_weights().size() * sizeof(float);
    // init/reinit if necessary
    if(resource.offset_bytes == 0)
    {
        std::cout << "bone_ids" << mesh_.get_bone_ids().size() << std::endl;
        std::cout << "bone_weigths" << mesh_.get_bone_weights().size() << std::endl;

        // new storage buffer
        resource.bone_ids_ = ctx.render_device->create_buffer(scm::gl::BIND_STORAGE_BUFFER, scm::gl::USAGE_STREAM_COPY, new_id_bytes, mesh_.get_bone_ids().data());

        // new storage buffer
        resource.bone_weights_ = ctx.render_device->create_buffer(scm::gl::BIND_STORAGE_BUFFER, scm::gl::USAGE_STREAM_COPY, new_weight_bytes, mesh_.get_bone_weights().data());

        ctx.render_context->bind_storage_buffer(resource.bone_ids_, 2);
        ctx.render_context->bind_storage_buffer(resource.bone_weights_, 3);
    }
    else
    {
        // read old data
        std::vector<unsigned> old_ids(resource.offset_bytes, 0);
        {
            scm::gl::scoped_buffer_map read_ids_map(ctx.render_context, resource.bone_ids_, 0, resource.offset_bytes, scm::gl::ACCESS_READ_ONLY);
            memcpy(old_ids.data(), read_ids_map.data_ptr(), resource.offset_bytes);
        }

        // resize id buffer:
        ctx.render_device->resize_buffer(resource.bone_ids_, resource.offset_bytes + new_id_bytes);
        // write old and new data:
        {
            scm::gl::scoped_buffer_map write_ids_map(ctx.render_context, resource.bone_ids_, 0, resource.offset_bytes + new_id_bytes, scm::gl::ACCESS_WRITE_ONLY);
            memcpy(write_ids_map.data_ptr(), old_ids.data(), resource.offset_bytes);
            memcpy(write_ids_map.data_ptr() + resource.offset_bytes, mesh_.get_bone_ids().data(), new_id_bytes);
        }

        // read old data:
        std::vector<float> old_weights(resource.offset_bytes, 0.0f);
        {
            scm::gl::scoped_buffer_map read_weights_map(ctx.render_context, resource.bone_weights_, 0, resource.offset_bytes, scm::gl::ACCESS_READ_ONLY);
            memcpy(old_weights.data(), read_weights_map.data_ptr(), resource.offset_bytes);
        }
        // resize weight buffer:
        ctx.render_device->resize_buffer(resource.bone_weights_, resource.offset_bytes + new_weight_bytes);
        // write old and new data:
        {
            scm::gl::scoped_buffer_map write_weights_map(ctx.render_context, resource.bone_weights_, 0, resource.offset_bytes + new_weight_bytes, scm::gl::ACCESS_WRITE_ONLY);

            memcpy(write_weights_map.data_ptr(), old_weights.data(), resource.offset_bytes);
            memcpy(write_weights_map.data_ptr() + resource.offset_bytes, mesh_.get_bone_weights().data(), new_weight_bytes);
        }
    }

    resource.offset_bytes += new_weight_bytes;

    ctx.render_context->apply();
}

void SkinnedMeshResource::init_bone_boxes()
{
    // use every single vertex to be manipulated by a certain bone per bone box
    unsigned bone_offset = 0;
    for(unsigned v(0); v < mesh_.num_vertices; ++v)
    {
        auto final_pos = scm::math::vec4(mesh_.positions[v].x, mesh_.positions[v].y, mesh_.positions[v].z, 1.0);

        for(unsigned i(0); i < mesh_.bone_counts[v]; ++i)
        {
            bone_boxes_[mesh_.bone_ids[bone_offset + i]].expandBy(math::vec3{final_pos.x, final_pos.y, final_pos.z});
        }
        bone_offset += mesh_.bone_counts[v];
    }
}

////////////////////////////////////////////////////////////////////////////////

std::vector<math::BoundingBox<math::vec3>> SkinnedMeshResource::get_bone_boxes(std::vector<scm::math::mat4f> const& bone_transforms)
{
    auto tmp_boxes = std::vector<math::BoundingBox<math::vec3>>(100, math::BoundingBox<math::vec3>());

    for(unsigned b(0); b < bone_boxes_.size(); ++b)
    {
        if(!bone_boxes_[b].isEmpty() && b < bone_transforms.size())
        {
            tmp_boxes[b] = transform(bone_boxes_[b], scm::math::mat4d(bone_transforms[b]));
        }
    }
    return tmp_boxes;
}

////////////////////////////////////////////////////////////////////////////////

void SkinnedMeshResource::draw(RenderContext& ctx) /*const*/
{
    auto iter = ctx.meshes.find(uuid());
    if(iter != ctx.meshes.end())
    {
        ctx.render_context->bind_vertex_array(iter->second.vertex_array);
        ctx.render_context->bind_index_buffer(iter->second.indices, iter->second.indices_topology, iter->second.indices_type);
        ctx.render_context->apply_vertex_input();
        ctx.render_context->draw_elements(iter->second.indices_count);
    }
}

////////////////////////////////////////////////////////////////////////////////

void SkinnedMeshResource::ray_test(Ray const& ray, int options, node::Node* owner, std::set<PickResult>& hits)
{
    // TODO raycasting
    Logger::LOG_ERROR << "get_vertex() dynamic ray testing not supported " << std::endl;
    // kd_tree_.ray_test(ray, mesh_, options, owner, hits);
}

////////////////////////////////////////////////////////////////////////////////

unsigned int SkinnedMeshResource::num_vertices() const { return mesh_.num_vertices; }

////////////////////////////////////////////////////////////////////////////////

unsigned int SkinnedMeshResource::num_faces() const { return mesh_.num_triangles; }

////////////////////////////////////////////////////////////////////////////////

scm::math::vec3 SkinnedMeshResource::get_vertex(unsigned int i) const
{
    // TODO physics handling
    Logger::LOG_ERROR << "get_vertex() dynamic vertex positions not supported " << std::endl;
    return scm::math::vec3();
}

////////////////////////////////////////////////////////////////////////////////

std::vector<unsigned int> SkinnedMeshResource::get_face(unsigned int i) const
{
    // TODO cpu representation of mesh
    Logger::LOG_ERROR << "get_face() of merged neshes not supported " << std::endl;
    /*std::vector<unsigned int> face(mesh_->mFaces[i].mNumIndices);
    for (unsigned int j = 0; j < mesh_->mFaces[i].mNumIndices; ++j)
      face[j] = mesh_->mFaces[i].mIndices[j];
    return face;*/
    return std::vector<unsigned int>();
}
////////////////////////////////////////////////////////////////////////////////

SkinnedMesh const& SkinnedMeshResource::get_mesh() const { return mesh_; }

////////////////////////////////////////////////////////////////////////////////
} // namespace gua
