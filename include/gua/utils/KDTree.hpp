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

#ifndef GUA_KD_TREE_HPP
#define GUA_KD_TREE_HPP

#include <gua/utils/Logger.hpp>
#include <gua/utils/string_utils.hpp>
#include <gua/math/BoundingBox.hpp>
#include <gua/utils/KDTreeUtils.hpp>
#include <gua/scenegraph/PickResult.hpp>
#include <gua/utils/Mesh.hpp>

#include <set>
#include <vector>
#include <iosfwd>
#include <algorithm>

namespace gua
{
/**
 * This class contains a simple KDTree implementation.
 */
class KDTree
{
  public:
    KDTree();

    //copies bounding box positions into the gpu buffer
    void copy_to_buffer(float* data) const;

    /**
     * Initializes the KDTree with the given triangles.
     *
     * \param triangles A vector of Triangle.
     */
    void generate(Mesh const& mesh);

    void generate(const char* filename);

    //returns the number of nodes in the tree
    int get_num_nodes() const;


    mutable std::vector<uint32_t> indices;
    /**
     * Checks for intersections with the KDTree.
     *
     * \param ray     The Ray which shall be tested against the tree.
     * \param options A bitwise combined set of options.
     * \param owner   The Node which will be written in the generated PickResults.
     * \param hits    A reference to the resulting set. Any contained data will be
     *                deleted depending on the supplied options.
     */
    void ray_test(Ray const& ray, Mesh const& mesh, int options, node::Node* owner, std::set<PickResult>& hits) const;

    bool save_to_binary(const char* filename) const;

  private:
    // a private struct used for triangle data storage in the leaves of the tree
    struct LeafData
    {
        struct Comparator
        {
            Comparator(unsigned dim);

            bool operator()(LeafData const& lhs, LeafData const& rhs) const;
            unsigned dim_;
        };

        LeafData();
        LeafData(Mesh const& mesh, Triangle const& triangle, unsigned id);
        LeafData(math::BoundingBox<math::vec3> const& bbox, unsigned id);

        unsigned id_;
        math::BoundingBox<math::vec3> bbox_;

        bool operator<(LeafData);
    };

    // a private struct which may be used for leaves and internal nodes
    struct KDNode
    {
        KDNode(std::vector<LeafData> const& data = std::vector<LeafData>());

        KDNode(KDNode* left_child, KDNode* right_child, unsigned splitting_dimension, float splitting_position, math::BoundingBox<math::vec3> const& bounds);

        void print(unsigned depth) const;
        
        size_t write(FILE* &f);
        size_t read(FILE* &f);

        std::vector<LeafData> data_;
        KDNode *left_child_, *right_child_;
        bool is_leaf_;
        unsigned splitting_dimension_;
        float splitting_position_;
        math::BoundingBox<math::vec3> bounds_;
    };

    // constructs the tree
    KDNode* build(std::vector<std::vector<LeafData>> const& sorted_triangles, math::BoundingBox<math::vec3> const& bounds );




    // ray test against the tree, returns upon the first intersection
    bool intersect_one(KDNode* node, Ray const& ray, Mesh const& mesh, int options, std::vector<Triangle> const& triangles, std::set<PickResult>& hits) const;

    // ray test against the tree, searches for all intersections
    void intersect_all(KDNode* node, Ray const& ray, Mesh const& mesh, int options, std::vector<Triangle> const& triangles, std::set<PickResult>& hits) const;


    KDNode* root_;
    std::vector<Triangle> triangles_;

    mutable node::Node* current_owner_;
    mutable int current_options_;
    mutable unsigned current_visit_flag_;
    mutable int num_nodes_in_tree_;


};

} // namespace gua

#endif // GUA_KD_TREE_HPP
