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


#include <queue>

// class header
#include <gua/node/OcclusionCullingGroupNode.hpp>

// guacamole headers
#include <gua/scenegraph/NodeVisitor.hpp>
#include <gua/node/TransformNode.hpp>

namespace gua
{
namespace node
{
OcclusionCullingGroupNode::OcclusionCullingGroupNode(std::string const& name, math::mat4 const& transform) : GeometryNode(name, transform) {}

/* virtual */ void OcclusionCullingGroupNode::accept(NodeVisitor& visitor) { visitor.visit(this); }

std::shared_ptr<Node> OcclusionCullingGroupNode::copy() const { return std::make_shared<OcclusionCullingGroupNode>(*this); }


void OcclusionCullingGroupNode::regroup_children(){
    std::queue<gua::node::Node*> splitting_queue;

    //if we have less than 3 children, then the grouping is as good as it gets
    if( get_children().size() > 2) {
        splitting_queue.push(this);
    }

    std::array<std::vector<std::shared_ptr<gua::node::Node> >, 3> children_sorted_by_xyz;

    while(!splitting_queue.empty() ) {
        //get next node to split
        auto current_node_to_split = splitting_queue.front();
        splitting_queue.pop();

        current_node_to_split->update_cache();

        int best_splitting_axis = -1;
        double best_splitting_cost = std::numeric_limits<double>::max(); // get the maximal value for our type
        int best_candidate_index = 0;

        for(unsigned int dim_idx = 0; dim_idx < 3; ++dim_idx) {
            children_sorted_by_xyz[dim_idx] = current_node_to_split->get_children();
        }

        // handle case where there are less than 32 children in the vector
        unsigned int candidate_element_offset = 1;
        unsigned int num_elements_to_distribute = children_sorted_by_xyz[0].size();
        unsigned int num_candidates = num_elements_to_distribute;

        if(num_elements_to_distribute > 32) { 
            num_candidates = 32;
            candidate_element_offset = num_elements_to_distribute / 32;
        }

        for(unsigned int dim_idx = 0; dim_idx < 3; ++dim_idx) {
            for(unsigned int candidate_index = 1; candidate_index < num_candidates; ++candidate_index) { 
                sorting_based_on_axis(children_sorted_by_xyz[dim_idx], dim_idx);

                split_children(current_node_to_split, children_sorted_by_xyz[dim_idx], candidate_index, candidate_element_offset);
                
                double cost_for_current_axis = calculate_cost(current_node_to_split);

                if(cost_for_current_axis < best_splitting_cost) {
                    best_splitting_cost = cost_for_current_axis;
                    best_splitting_axis = dim_idx;
                    best_candidate_index = candidate_index;
                }

                cleanup_intermediate_nodes(current_node_to_split);
            }
        }

        // restore best candidate configuration by splitting once more
        split_children(current_node_to_split, children_sorted_by_xyz[best_splitting_axis], best_candidate_index, candidate_element_offset);

        auto children = current_node_to_split->get_children();

        for(unsigned int child_idx = 0; child_idx < 2; ++child_idx) {
            auto& current_child = children[child_idx];

            if(current_child->get_children().size() > 2) {
            	gua::node::Node* child_ptr = children[child_idx].get();
                splitting_queue.push(child_ptr);
            }

        }
    }

}

void OcclusionCullingGroupNode::sorting_based_on_axis(std::vector<std::shared_ptr<gua::node::Node>>& v, int axis){
    std::sort(v.begin(), v.end(), 
        [axis] (const std::shared_ptr<gua::node::Node> & a, const std::shared_ptr<gua::node::Node> & b) -> bool {
            return (a->get_world_position()[axis] < b->get_world_position()[axis]);
        });
}

void OcclusionCullingGroupNode::cleanup_intermediate_nodes(gua::node::Node* scene_occlusion_group_node) {
    for(auto& intermediate_node : scene_occlusion_group_node->get_children()) {
        intermediate_node->clear_children();
    }
}


void OcclusionCullingGroupNode::split_children(gua::node::Node* scene_occlusion_group_node, 
											   std::vector<std::shared_ptr<gua::node::Node>> & sorted_vector, 
                    unsigned int candidate_index, unsigned int candidate_element_offset) {
    scene_occlusion_group_node->clear_children();

    auto transform_node_L = scene_occlusion_group_node->add_child<gua::node::TransformNode>("transform_node_L");
    auto transform_node_R = scene_occlusion_group_node->add_child<gua::node::TransformNode>("transform_node_R");

    unsigned int index = 0;

    unsigned int pivot = candidate_index * candidate_element_offset;


    for(auto it = sorted_vector.begin(); it != sorted_vector.end(); ++it) {
        if(index < pivot) {
            transform_node_L->add_child(*it);
        } else {
            transform_node_R->add_child(*it);
        }
        ++index;
    }
}


double OcclusionCullingGroupNode::calculate_cost(gua::node::Node* node){
    node->update_cache();
    auto parent_surface_area = node->get_bounding_box().surface_area();

    auto children = node->get_children();

    float summed_weighted_surface_area = 0.0f;

    for(auto& child : children) {
        child->update_cache();
        summed_weighted_surface_area += child->get_bounding_box().surface_area() * (child->get_children()).size();;
    }

    return (summed_weighted_surface_area) / parent_surface_area;

}


} // namespace node
} // namespace gua
