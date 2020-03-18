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
#include <gua/renderer/TriMeshRessource.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/node/TriMeshNode.hpp>
#include <gua/utils/Logger.hpp>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////

TriMeshRessource::TriMeshRessource() : kd_tree_(), mesh_() {}

////////////////////////////////////////////////////////////////////////////////

TriMeshRessource::TriMeshRessource(Mesh const& mesh, bool build_kd_tree) : kd_tree_(), mesh_(mesh)
{
    if(mesh_.num_vertices > 0)
    {
        bounding_box_ = math::BoundingBox<math::vec3>();

        for(unsigned v(0); v < mesh_.num_vertices; ++v)
        {
            bounding_box_.expandBy(math::vec3{mesh_.positions[v]});
        }

        if(build_kd_tree)
        {
            kd_tree_.generate(mesh);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

void TriMeshRessource::upload_to(RenderContext& ctx) const
{
    RenderContext::Mesh cmesh{};
    cmesh.indices_topology = scm::gl::PRIMITIVE_TRIANGLE_LIST;
    cmesh.indices_type = scm::gl::TYPE_UINT;
    cmesh.indices_count = mesh_.num_triangles * 3;

    if(!(mesh_.num_vertices > 0))
    {
        Logger::LOG_WARNING << "Unable to load Mesh! Has no vertex data." << std::endl;
        return;
    }

    cmesh.vertices = ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER, scm::gl::USAGE_STATIC_DRAW, mesh_.num_vertices * sizeof(Mesh::Vertex), 0);

    Mesh::Vertex* data(static_cast<Mesh::Vertex*>(ctx.render_context->map_buffer(cmesh.vertices, scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER)));

    mesh_.copy_to_buffer(data);

    ctx.render_context->unmap_buffer(cmesh.vertices);

    cmesh.indices = ctx.render_device->create_buffer(scm::gl::BIND_INDEX_BUFFER, scm::gl::USAGE_STATIC_DRAW, mesh_.num_triangles * 3 * sizeof(unsigned), mesh_.indices.data());

    cmesh.vertex_array = ctx.render_device->create_vertex_array(mesh_.get_vertex_format(), {cmesh.vertices});
    ctx.meshes[uuid()] = cmesh;

    ctx.render_context->apply();
}

////////////////////////////////////////////////////////////////////////////////

void TriMeshRessource::upload_kdtree_to(RenderContext& ctx) const
{
    RenderContext::BoundingBoxHierarchy cbounding_box_hierarchy{};
    cbounding_box_hierarchy.indices_topology = scm::gl::PRIMITIVE_TRIANGLE_STRIP;
    cbounding_box_hierarchy.indices_type = scm::gl::TYPE_UINT;
    //12 triangles per box, 3 vertices per triangle
    cbounding_box_hierarchy.indices_count =  kd_tree_.indices.size(); //render each box as triangle strip


    //should only happe if the KD-tree was not build in the first place
    if(!(kd_tree_.get_num_nodes() > 0))
    {
        Logger::LOG_WARNING << "Unable to load KD-Tree into GPU buffer! The KD-Tree does not contain nodes." << std::endl;
        return;
    }

    cbounding_box_hierarchy.vertices = ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER, scm::gl::USAGE_STATIC_DRAW, kd_tree_.get_num_nodes() * sizeof(float) * 3, 0);

    float* data(static_cast<float*>(ctx.render_context->map_buffer(cbounding_box_hierarchy.vertices, scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER)));

    //std::cout << "KD TREE COPY TO BUFFER!" << std::endl;
    kd_tree_.copy_to_buffer(data);

    //std::cout << "AFTER KD TREE COPY TO BUFFER!" << std::endl;
    ctx.render_context->unmap_buffer(cbounding_box_hierarchy.vertices);

    cbounding_box_hierarchy.indices = ctx.render_device->create_buffer(scm::gl::BIND_INDEX_BUFFER, scm::gl::USAGE_STATIC_DRAW, cbounding_box_hierarchy.indices_count * sizeof(unsigned), kd_tree_.indices.data());

    cbounding_box_hierarchy.vertex_array = ctx.render_device->create_vertex_array(scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC3F, sizeof(float)*3) , {cbounding_box_hierarchy.vertices});
    ctx.bounding_box_hierarchies[uuid()] = cbounding_box_hierarchy;

    ctx.render_context->apply();

}

////////////////////////////////////////////////////////////////////////////////

void TriMeshRessource::draw(RenderContext& ctx) const
{
    auto iter = ctx.meshes.find(uuid());
    if(iter == ctx.meshes.end())
    {
        // upload to GPU if neccessary
        upload_to(ctx);
        iter = ctx.meshes.find(uuid());
    }
    ctx.render_context->bind_vertex_array(iter->second.vertex_array);
    ctx.render_context->bind_index_buffer(iter->second.indices, iter->second.indices_topology, iter->second.indices_type);
    ctx.render_context->apply_vertex_input();
    ctx.render_context->draw_elements(iter->second.indices_count);
}

////////////////////////////////////////////////////////////////////////////////
void TriMeshRessource::draw_kdtree(RenderContext& ctx) const {
    auto iter = ctx.bounding_box_hierarchies.find(uuid());
    if(iter == ctx.bounding_box_hierarchies.end())
    {
        // upload to GPU if neccessary
        upload_kdtree_to(ctx);
        iter = ctx.bounding_box_hierarchies.find(uuid());
    }
    ctx.render_context->bind_vertex_array(iter->second.vertex_array);
    ctx.render_context->bind_index_buffer(iter->second.indices, iter->second.indices_topology, iter->second.indices_type);
    ctx.render_context->apply_vertex_input();
    ctx.render_context->draw_elements(iter->second.indices_count);
}

////////////////////////////////////////////////////////////////////////////////

void TriMeshRessource::ray_test(Ray const& ray, int options, node::Node* owner, std::set<PickResult>& hits) { kd_tree_.ray_test(ray, mesh_, options, owner, hits); }

////////////////////////////////////////////////////////////////////////////////

math::vec3 TriMeshRessource::get_vertex(unsigned int i) const { return math::vec3(mesh_.positions[i].x, mesh_.positions[i].y, mesh_.positions[i].z); }

////////////////////////////////////////////////////////////////////////////////

std::vector<unsigned int> TriMeshRessource::get_face(unsigned int i) const
{


    std::vector<unsigned int> face(3, 0);

    int64_t const face_base_index = 3 * i;


    face[0] = mesh_.indices[face_base_index + 0];
    face[1] = mesh_.indices[face_base_index + 1];   
    face[2] = mesh_.indices[face_base_index + 2];

    return face;
}

////////////////////////////////////////////////////////////////////////////////

bool TriMeshRessource::save_to_binary(const char* filename_gua_trimesh, const char* filename_gua_kdtree, unsigned flags)
{
    bool res = mesh_.save_to_binary(filename_gua_trimesh, flags);
    if( (nullptr != filename_gua_kdtree) && (0 < kd_tree_.get_num_nodes()) ){
        res = res && kd_tree_.save_to_binary(filename_gua_kdtree);
    }
    return res;
}

////////////////////////////////////////////////////////////////////////////////

} // namespace gua
