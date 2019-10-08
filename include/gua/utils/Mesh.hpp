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

#ifndef GUA_MESH_HPP
#define GUA_MESH_HPP

// guacamole headers
#include <gua/config.hpp>
#include <gua/platform.hpp>
#include <gua/utils/fbxfwd.hpp>

// external headers
#include <scm/gl_core.h>
#include <scm/core/math/quat.h>
#include <vector>

struct aiMesh;

namespace gua
{
/**
 * @brief holds vertex information of one mesh
 */
struct GUA_DLL Mesh
{
  public:
    Mesh();

    Mesh(aiMesh const& mesh);

#ifdef GUACAMOLE_FBX
    Mesh(FbxMesh& mesh, int material_index = -1);
#endif

    /**
     * @brief holds information of a vertex
     */
    struct Vertex
    {
        scm::math::vec3f pos;
        scm::math::vec2f tex;
        scm::math::vec3f normal;
        scm::math::vec3f tangent;
        scm::math::vec3f bitangent;
    };

    /**
     * @brief writes vertex info to given buffer
     *
     * @param vertex_buffer buffer to write to
     */
    void copy_to_buffer(Vertex* vertex_buffer) const;

    /**
     * @brief returns vertex layout for mesh vertex
     * @return schism vertex format
     */
    virtual scm::gl::vertex_format get_vertex_format() const;

    std::vector<scm::math::vec3f> positions;
    std::vector<scm::math::vec3f> normals;
    std::vector<scm::math::vec2f> texCoords;
    std::vector<scm::math::vec3f> tangents;
    std::vector<scm::math::vec3f> bitangents;
    std::vector<unsigned> indices;
    // TODO: flags, to specify if texcoors are present
    unsigned int num_vertices;
    unsigned int num_triangles;

  protected:
    /**
     * @brief stores a temporary vertex to be filtered and then written to buffers
     */
    struct temp_vert
    {
        temp_vert(unsigned oindex, unsigned pt, unsigned tr, unsigned ind) : old_index{oindex}, point{pt}, normal{}, tangent{}, bitangent{}, uv{}, tris{} { tris.push_back(std::make_pair(tr, ind)); }

        unsigned old_index;
        unsigned point;
        scm::math::vec3f normal;
        scm::math::vec3f tangent;
        scm::math::vec3f bitangent;
        scm::math::vec2f uv;
        std::vector<std::pair<unsigned, unsigned>> tris; // tris which share vertex
    };

    /**
     * @brief stores a temporary tri to be filtered and then written to buffers
     */
    struct temp_tri
    {
        temp_tri(unsigned a, unsigned b, unsigned c) : verts({a, b, c}) {}
        std::array<unsigned, 3> verts;
    };

    /**
     * @brief constructs this mesh from the given fbxmesh
     * @details if material index is given only triangles with this material are imported
     *
     * @param mesh mesh to convert
     * @param material_index material to filter with
     *
     * @return indices of triangles that were added to this mesh
     */
    std::vector<unsigned> construct(FbxMesh& mesh, int material_index);

    /**
     * @brief gets function to acces fbx vertex attribute
     * @details the returned function allows to acces a vertex attribute regardless of internal mapping
     *
     * @return access function
     */
    template <typename T>
    static std::function<unsigned(temp_vert const&)> get_access_function(FbxLayerElementTemplate<T> const& layer);
};

} // namespace gua

#endif // GUA_MESH_HPP
