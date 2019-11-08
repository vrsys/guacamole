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

#include <gua/math/math.hpp>
#include <gua/utils/KDTree.hpp>

#include <gua/platform.hpp>

#include <iostream>
#include <queue>

namespace gua
{
KDTree::KDTree() : root_(nullptr), current_visit_flag_(0), num_nodes_in_tree_(0) {}

void KDTree::generate(Mesh const& mesh)
{
    triangles_.resize(mesh.num_triangles);

    for(unsigned int tri_idx = 0; tri_idx < mesh.num_triangles; ++tri_idx)
    {
        triangles_[tri_idx] = Triangle(tri_idx);
    }

    std::vector<std::vector<LeafData>> sorted_triangles(3);

    for(unsigned int dim_idx = 0; dim_idx < 3; ++dim_idx)
        sorted_triangles[dim_idx] = std::vector<LeafData>(triangles_.size());

    math::BoundingBox<math::vec3> root_bounds;

    for(unsigned int tri_idx = 0; tri_idx < triangles_.size(); ++tri_idx)
    {
        for(unsigned int dim_idx = 0; dim_idx < 3; ++dim_idx)
        {
            sorted_triangles[dim_idx][tri_idx] = LeafData(mesh, triangles_[tri_idx], tri_idx);
            root_bounds.expandBy(triangles_[tri_idx].get_vertex(mesh, dim_idx));
        }
    }

    //below allows axis-aligned tris to be picked.
    //axis-aligned tris cause unreliable picking when the boundary of
    //the kdtree coincides with the triangle. therefore, we expand
    //the box before building the tree. 
    auto dim = root_bounds.max - root_bounds.min;
    root_bounds.min -= 0.01f * dim;
    root_bounds.max += 0.01f * dim;

    for(unsigned int dim_idx = 0; dim_idx < 3; ++dim_idx)
    {
        LeafData::Comparator comp(dim_idx);
        std::sort(sorted_triangles[dim_idx].begin(), sorted_triangles[dim_idx].end(), comp);
    }



    root_ = build(sorted_triangles, root_bounds);


    //fill indices (14 indices for triangle strip rendering of cube per node)
    //https://stackoverflow.com/questions/28375338/cube-using-single-gl-triangle-strip
    indices.resize(14 * num_nodes_in_tree_);

    for(int32_t node_idx = 0; node_idx < num_nodes_in_tree_; ++node_idx) {
        int const& index_base_offset = node_idx * 14;
        int const& vertex_base_offset = node_idx * 8;
        // 3 2 6 7 4 2 0 3 1 6 5 4 1 0
        indices[index_base_offset +  0] = vertex_base_offset + 3;
        indices[index_base_offset +  1] = vertex_base_offset + 2;
        indices[index_base_offset +  2] = vertex_base_offset + 6;
        indices[index_base_offset +  3] = vertex_base_offset + 7;
        indices[index_base_offset +  4] = vertex_base_offset + 4;
        indices[index_base_offset +  5] = vertex_base_offset + 2;
        indices[index_base_offset +  6] = vertex_base_offset + 0;
        indices[index_base_offset +  7] = vertex_base_offset + 3;
        indices[index_base_offset +  8] = vertex_base_offset + 1;
        indices[index_base_offset +  9] = vertex_base_offset + 6;
        indices[index_base_offset + 10] = vertex_base_offset + 5;
        indices[index_base_offset + 11] = vertex_base_offset + 4;
        indices[index_base_offset + 12] = vertex_base_offset + 1;
        indices[index_base_offset + 13] = vertex_base_offset + 0;
    }
}

int KDTree::get_num_nodes() const {
    return num_nodes_in_tree_;
}

void KDTree::ray_test(Ray const& ray, Mesh const& mesh, int options, node::Node* owner, std::set<PickResult>& hits) const
{
    if(root_)
    {
        current_options_ = options;
        current_owner_ = owner;
        ++current_visit_flag_;

        if(options & PickResult::PICK_ONLY_FIRST_FACE && options & PickResult::PICK_ONLY_FIRST_OBJECT)
        {
            // override any existing intersection if it's closer
            intersect_one(root_, ray, mesh, options, triangles_, hits);
        }
        else if(options & PickResult::PICK_ONLY_FIRST_FACE)
        {
            // add newly found intersection to intersections list
            std::set<PickResult> new_hit;
            intersect_one(root_, ray, mesh, options, triangles_, new_hit);

            if(!new_hit.empty())
                hits.insert(*new_hit.begin());
        }
        else if(options & PickResult::PICK_ONLY_FIRST_OBJECT)
        {
            // override all existing intersections and replace 'em
            std::set<PickResult> new_hits;
            intersect_all(root_, ray, mesh, options, triangles_, new_hits);

            if(!new_hits.empty())
            {
                if(hits.empty() || new_hits.begin()->distance < hits.begin()->distance)
                {
                    hits = new_hits;
                }
            }
        }
        else
        {
            // add all intersections
            intersect_all(root_, ray, mesh, options, triangles_, hits);
        }
    }
}

KDTree::KDNode* KDTree::build(std::vector<std::vector<LeafData>> const& sorted_triangles, math::BoundingBox<math::vec3> const& bounds)
{
    ++num_nodes_in_tree_;

    // determine splitting dimension
    unsigned current_dim = 0;
    for(unsigned dim_idx = (1); dim_idx < 3; ++dim_idx)
    {
        if(bounds.size(dim_idx) > bounds.size(current_dim))
            current_dim = dim_idx;
    }

    // return empty leaf if there are no triangles left
    if(sorted_triangles[current_dim].size() == 0)
    {
        return new KDNode;
    }

    // return leaf with one triangle if there is only one left
    if(sorted_triangles[current_dim].size() == 1)
    {
        return new KDNode(std::vector<LeafData>({sorted_triangles[current_dim].front()}));
    }

    // select splitting position as minimum point of median triangles bounding
    // box
    float split_position(sorted_triangles[current_dim][sorted_triangles[current_dim].size() / 2].bbox_.min[current_dim]);

    // list for children
    std::vector<std::vector<LeafData>> left_list(3);
    std::vector<std::vector<LeafData>> right_list(3);

    // bounding boxes for children
    math::BoundingBox<math::vec3> left_bounds;
    math::BoundingBox<math::vec3> right_bounds;

    // put all triangles into the appropriate lists
    for(unsigned i(0); i < 3; ++i)
    {
        for(unsigned j(0); j < sorted_triangles[i].size(); ++j)
        {
            if(sorted_triangles[i][j].bbox_.max[current_dim] <= split_position)
            {
                // if triangle is entirely to the left
                left_list[i].push_back(sorted_triangles[i][j]);
                left_bounds.expandBy(sorted_triangles[i][j].bbox_);
            }
            else if(sorted_triangles[i][j].bbox_.min[current_dim] >= split_position)
            {
                // if triangle is entirely to the right
                right_list[i].push_back(sorted_triangles[i][j]);
                right_bounds.expandBy(sorted_triangles[i][j].bbox_);
            }
            else
            {
                // if triangle overlaps splitting point, split its bounding box
                // into two halfs
                math::vec3 left_max(sorted_triangles[i][j].bbox_.max);
                left_max[current_dim] = split_position;
                math::BoundingBox<math::vec3> left_box;
                left_box.expandBy(sorted_triangles[i][j].bbox_.min);
                left_box.expandBy(left_max);
                LeafData left_half(left_box, sorted_triangles[i][j].id_);

                math::vec3 right_min(sorted_triangles[i][j].bbox_.min);
                right_min[current_dim] = split_position;
                math::BoundingBox<math::vec3> right_box;
                right_box.expandBy(sorted_triangles[i][j].bbox_.max);
                right_box.expandBy(right_min);
                LeafData right_half(right_box, sorted_triangles[i][j].id_);

                left_list[i].push_back(left_half);
                right_list[i].push_back(right_half);
                left_bounds.expandBy(left_half.bbox_);
                right_bounds.expandBy(right_half.bbox_);
            }
        }
    }

    KDNode* left_child(nullptr);
    KDNode* right_child(nullptr);

    // return children with mutlitple triangles if there were no separations
    if(left_list[current_dim].empty())
    {
        right_child = new KDNode(right_list[current_dim]);
    }
    else if(right_list[current_dim].empty())
    {
        left_child = new KDNode(left_list[current_dim]);
    }
    else if(right_list[current_dim].size() == sorted_triangles[current_dim].size() || left_list[current_dim].size() == sorted_triangles[current_dim].size())
    {
        right_child = new KDNode(right_list[current_dim]);
        left_child = new KDNode(left_list[current_dim]);
    }
    else
    {
        // build children recursively
        left_child = build(left_list, left_bounds);
        right_child = build(right_list, right_bounds);
    }

    return new KDNode(left_child, right_child, current_dim, split_position, bounds);
}

void KDTree::copy_to_buffer(float* data) const {
    std::queue<KDNode*> node_traversal_queue;
    node_traversal_queue.push(root_);

    uint32_t current_bb_index_offset = 0;

    while(!node_traversal_queue.empty()) {
        auto current_node = node_traversal_queue.front();
        node_traversal_queue.pop();

        auto const& current_bb = current_node->bounds_;

        int const box_coords_base_offset = 6 * current_bb_index_offset;

        for(int32_t min_dim_idx = 0; min_dim_idx < 3; ++min_dim_idx) {
           // data[box_coords_base_offset + min_dim_idx] = current_bb.min[min_dim_idx];
        }
        for(int32_t max_dim_idx = 0; max_dim_idx < 3; ++max_dim_idx) {
            //data[box_coords_base_offset + 3 + max_dim_idx] = current_bb.max[max_dim_idx];
        }

        if(!current_node->is_leaf_) {
            node_traversal_queue.push(current_node->left_child_);
            node_traversal_queue.push(current_node->right_child_);
        }

        ++current_bb_index_offset;

    }
}



bool KDTree::intersect_one(KDNode* node, Ray const& ray, Mesh const& mesh, int options, std::vector<Triangle> const& triangles, std::set<PickResult>& hits) const
{
    // intersect with all triangles if leaf node
    if(node->is_leaf_)
    {
        bool intersected(false);
        for(auto const& triangle : node->data_)
        {
            if(triangles[triangle.id_].visit_flag_ != current_visit_flag_)
            {
                auto intersection(triangles[triangle.id_].intersect(mesh, ray));

                if(intersection < Ray::END)
                {
                    if(hits.empty() || intersection < hits.begin()->distance)
                    {
                        hits.clear();
                        float const inf(std::numeric_limits<float>::max());
                        math::vec3 position(inf, inf, inf), world_position(inf, inf, inf), normal(inf, inf, inf), world_normal(inf, inf, inf);
                        math::vec2 tex_coords;

                        if(options & PickResult::GET_POSITIONS || options & PickResult::GET_WORLD_POSITIONS || options & PickResult::INTERPOLATE_NORMALS || options & PickResult::GET_TEXTURE_COORDS)
                        {
                            position = ray.origin_ + intersection * ray.direction_;
                        }

                        if(options & PickResult::GET_NORMALS || options & PickResult::GET_WORLD_NORMALS)
                        {
                            if(options & PickResult::INTERPOLATE_NORMALS)
                            {
                                normal = triangles[triangle.id_].get_normal_interpolated(mesh, position);
                            }
                            else
                            {
                                normal = triangles[triangle.id_].get_normal(mesh);
                            }
                        }

                        if(options & PickResult::GET_TEXTURE_COORDS)
                        {
                            tex_coords = triangles[triangle.id_].get_texture_coords_interpolated(mesh, position);
                        }

                        hits.insert(PickResult(intersection, current_owner_, position, world_position, normal, world_normal, tex_coords));
                        intersected = true;
                    }
                }

                triangles[triangle.id_].visit_flag_ = current_visit_flag_;
            }
        }

        return intersected;
    }

    Ray traversing_ray(ray.intersection(node->bounds_));

    if(traversing_ray.t_max_ >= 0.f)
    {
        math::vec3 a(traversing_ray.origin_);
        math::vec3 b(traversing_ray.origin_ + traversing_ray.t_max_ * traversing_ray.direction_);

        // if ray is only to the left
        if(a[node->splitting_dimension_] <= node->splitting_position_ && b[node->splitting_dimension_] <= node->splitting_position_)
        {
            if(node->left_child_)
            {
                return intersect_one(node->left_child_, ray, mesh, options, triangles, hits);
            }

            // if ray is only to the right
        }
        else if(a[node->splitting_dimension_] >= node->splitting_position_ && b[node->splitting_dimension_] >= node->splitting_position_)
        {
            if(node->right_child_)
            {
                return intersect_one(node->right_child_, ray, mesh, options, triangles, hits);
            }

            // ray crosses splitting plane
        }
        else
        {
            // check left child first
            if(a[node->splitting_dimension_] < node->splitting_position_)
            {
                if(node->left_child_ && intersect_one(node->left_child_, ray, mesh, options, triangles, hits))
                {
                    return true;
                }

                if(node->right_child_ && intersect_one(node->right_child_, ray, mesh, options, triangles, hits))
                {
                    return true;
                }
            }
            else
            {
                if(node->right_child_ && intersect_one(node->right_child_, ray, mesh, options, triangles, hits))
                {
                    return true;
                }

                if(node->left_child_ && intersect_one(node->left_child_, ray, mesh, options, triangles, hits))
                {
                    return true;
                }
            }
        }
    }

    return false;
}

void KDTree::intersect_all(KDNode* node, Ray const& ray, Mesh const& mesh, int options, std::vector<Triangle> const& triangles, std::set<PickResult>& hits) const
{
    // intersect with all triangles if leaf node
    if(node->is_leaf_)
    {
        for(auto const& triangle : node->data_)
        {
            if(triangles[triangle.id_].visit_flag_ != current_visit_flag_)
            {
                auto intersection(triangles[triangle.id_].intersect(mesh, ray));

                if(intersection < Ray::END)
                {
                    float const inf(std::numeric_limits<float>::max());
                    math::vec3 position(inf, inf, inf), world_position(inf, inf, inf), normal(inf, inf, inf), world_normal(inf, inf, inf);
                    math::vec2 tex_coords;

                    if(options & PickResult::GET_POSITIONS || options & PickResult::GET_WORLD_POSITIONS || options & PickResult::INTERPOLATE_NORMALS || options & PickResult::GET_TEXTURE_COORDS)
                    {
                        position = ray.origin_ + intersection * ray.direction_;
                    }

                    if(options & PickResult::GET_NORMALS || options & PickResult::GET_WORLD_NORMALS)
                    {
                        if(options & PickResult::INTERPOLATE_NORMALS)
                        {
                            normal = triangles[triangle.id_].get_normal_interpolated(mesh, position);
                        }
                        else
                        {
                            normal = triangles[triangle.id_].get_normal(mesh);
                        }
                    }

                    if(options & PickResult::GET_TEXTURE_COORDS)
                    {
                        tex_coords = triangles[triangle.id_].get_texture_coords_interpolated(mesh, position);
                    }

                    hits.insert(PickResult(intersection, current_owner_, position, world_position, normal, world_normal, tex_coords));
                }

                triangles[triangle.id_].visit_flag_ = current_visit_flag_;
            }
        }
    }

    Ray traversing_ray(ray.intersection(node->bounds_));

    if(traversing_ray.t_max_ >= 0.f)
    {
        math::vec3 a(traversing_ray.origin_);
        math::vec3 b(traversing_ray.origin_ + traversing_ray.t_max_ * traversing_ray.direction_);

        // if ray is only to the left
        if(a[node->splitting_dimension_] <= node->splitting_position_ && b[node->splitting_dimension_] <= node->splitting_position_)
        {
            if(node->left_child_)
            {
                intersect_all(node->left_child_, ray, mesh, options, triangles, hits);
            }

            // if ray is only to the right
        }
        else if(a[node->splitting_dimension_] >= node->splitting_position_ && b[node->splitting_dimension_] >= node->splitting_position_)
        {
            if(node->right_child_)
            {
                intersect_all(node->right_child_, ray, mesh, options, triangles, hits);
            }

            // ray crosses splitting plane
        }
        else
        {
            if(node->left_child_)
                intersect_all(node->left_child_, ray, mesh, options, triangles, hits);

            if(node->right_child_)
                intersect_all(node->right_child_, ray, mesh, options, triangles, hits);
        }
    }
}

// LeafData --------------------------------------------------------------------

KDTree::LeafData::Comparator::Comparator(unsigned dim) : dim_(dim) {}

bool KDTree::LeafData::Comparator::operator()(LeafData const& lhs, LeafData const& rhs) const { return lhs.bbox_.min[dim_] < rhs.bbox_.min[dim_]; }

KDTree::LeafData::LeafData() : id_(-1), bbox_() {}

KDTree::LeafData::LeafData(Mesh const& mesh, Triangle const& triangle, unsigned id) : id_(id), bbox_()
{
    for(auto i(0); i < 3; ++i)
    {
        bbox_.expandBy(triangle.get_vertex(mesh, i));
    }
}

KDTree::LeafData::LeafData(math::BoundingBox<math::vec3> const& bbox, unsigned id) : id_(id), bbox_(bbox) {}

// KDNode ----------------------------------------------------------------------

KDTree::KDNode::KDNode(std::vector<LeafData> const& data) : data_(data), left_child_(nullptr), right_child_(nullptr), is_leaf_(true), splitting_dimension_(0), splitting_position_(0.f), bounds_() {}

KDTree::KDNode::KDNode(KDNode* left_child, KDNode* right_child, unsigned splitting_dimension, float splitting_position, math::BoundingBox<math::vec3> const& bounds)
    : data_(), left_child_(left_child), right_child_(right_child), is_leaf_(false), splitting_dimension_(splitting_dimension), splitting_position_(splitting_position), bounds_(bounds)
{
}

void KDTree::KDNode::print(unsigned depth) const
{
    std::string msg;
    for(unsigned i(0); i < depth; ++i)
        msg += " ";
    msg += is_leaf_ ? string_utils::to_string(data_.size()) + " leaf(s): " : "internal: ";

    if(is_leaf_)
        msg += "";
    else
        msg += "Split @ " + string_utils::to_string(splitting_position_) + " " + string_utils::to_string(bounds_.min) + " " + string_utils::to_string(bounds_.max);

    Logger::LOG_MESSAGE << msg << std::endl;

    if(left_child_)
        left_child_->print(depth + 1);
    if(right_child_)
        right_child_->print(depth + 1);
}

} // namespace gua
