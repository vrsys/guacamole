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

//global Variables

#define THRESHHOLD 1


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

/* virtual */ void OcclusionCullingGroupNode::accept(NodeVisitor& visitor) {
    visitor.visit(this);
}

std::shared_ptr<Node> OcclusionCullingGroupNode::copy() const {
    return std::make_shared<OcclusionCullingGroupNode>(*this);
}



void OcclusionCullingGroupNode::median_regroup_children() {
    std::queue<gua::node::Node*> splitting_queue;


    //if we have less than 3 children, then the grouping is as good as it gets
    std::vector<std::shared_ptr<Node>> children = get_children();

    if(children.size() > 2) {
        splitting_queue.push(this);
    }

    while(!splitting_queue.empty()) {

        std::array<std::vector<std::shared_ptr<gua::node::Node> >, 3> children_sorted_by_xyz;
        auto current_node_to_split = splitting_queue.front();
        splitting_queue.pop();

        current_node_to_split->update_cache();

        int best_splitting_axis = -1;
        double best_splitting_cost = std::numeric_limits<double>::max(); // get the maximal value for our type


        for(unsigned int dim_idx = 0; dim_idx < 3; ++dim_idx) {
            children_sorted_by_xyz[dim_idx] = current_node_to_split->get_children();
        }

        for(unsigned int dim_idx = 0; dim_idx < 3; ++dim_idx) {

            sorting_based_on_axis(children_sorted_by_xyz[dim_idx], dim_idx);
            unsigned int const median = children_sorted_by_xyz[dim_idx].size()/2;
            split_children(current_node_to_split, children_sorted_by_xyz[dim_idx],median, 1);

            double cost_for_current_axis = calculate_median_cost(current_node_to_split);

            if(cost_for_current_axis < best_splitting_cost) {
                best_splitting_cost = cost_for_current_axis;
                best_splitting_axis = dim_idx;
            }

            cleanup_intermediate_nodes(current_node_to_split);
        }

        unsigned int const best_median = children_sorted_by_xyz[best_splitting_axis].size()/2;

        split_children(current_node_to_split, children_sorted_by_xyz[best_splitting_axis],best_median, 1);
        auto children = current_node_to_split->get_children();

        for(unsigned int child_idx = 0; child_idx < 2; ++child_idx) {
            auto& current_child = children[child_idx];

            auto& current_grandchildren = current_child->get_children();

            if(current_grandchildren.size() > 2) {
                gua::node::Node* child_ptr = children[child_idx].get();
                splitting_queue.push(child_ptr);

            }

        }
    }
}

void OcclusionCullingGroupNode::regroup_children() {


    //this is only for renaming the children so we can access them via a unqiue path
    uint32_t global_node_renaming_index = 0;

    std::queue< std::shared_ptr<gua::node::Node> > renaming_queue;

    for(auto& child : get_children() ) {
        renaming_queue.push(child);
    }

    // node renaming
    while(!renaming_queue.empty()) {
        auto& current_node_to_rename = renaming_queue.front();
        renaming_queue.pop();

        std::string new_node_name = current_node_to_rename->get_name() + "_" + std::to_string(global_node_renaming_index);
        current_node_to_rename->set_name(new_node_name);
        ++global_node_renaming_index;

        for(auto& child : current_node_to_rename->get_children()) {
            renaming_queue.push(child);
        }
    }

    
    std::vector<std::shared_ptr<Node>> children = get_children();
    for(auto& child : get_children() ) {
        if (child->num_grouped_faces()>THRESHHOLD) {
            std::queue<gua::node::Node*> splitting_queue_multiple_nodes_for_object;
            std::vector<std::shared_ptr<Node>> grandchildren = child->get_children();
            if (grandchildren.size()>2) {
                splitting_queue_multiple_nodes_for_object.push(child.get());
            }
            determine_best_split(splitting_queue_multiple_nodes_for_object);
        }
    }


    std::queue<gua::node::Node*> splitting_queue;

    //if we have less than 3 children, then the grouping is as good as it gets
    if( get_children().size() > 2) {
        splitting_queue.push(this);
    }

    determine_best_split(splitting_queue);
}

//TODO: Still errors for single child for one Transform Node. Remove to optimize graph
//TODO: Include number of faces to stop splitting at reasonable depth
void OcclusionCullingGroupNode::determine_best_split(std::queue<gua::node::Node*> splitting_queue) {
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


        if(best_splitting_axis > -1) {
            // restore best candidate configuration by splitting once more
            split_children(current_node_to_split, children_sorted_by_xyz[best_splitting_axis], best_candidate_index, candidate_element_offset);

            auto children = current_node_to_split->get_children();

            for(unsigned int child_idx = 0; child_idx < 2; ++child_idx) {
                auto& current_child = children[child_idx];

                if(current_child->get_children().size() > 2 /*&& current_child->num_grouped_faces()>THRESHHOLD*/) {
                    gua::node::Node* child_ptr = children[child_idx].get();
                    splitting_queue.push(child_ptr);
                }

            }
        } else {
            current_node_to_split->clear_children();

            for(auto const& child : children_sorted_by_xyz[0]) {
                current_node_to_split->add_child(child);
            }

        }
    }
}

void OcclusionCullingGroupNode::sorting_based_on_axis(std::vector<std::shared_ptr<gua::node::Node>>& v, int axis) {
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

    bool remove_useless_split = false;
    unsigned int index = 0;

    unsigned int pivot = candidate_index * candidate_element_offset;


    std::vector<std::shared_ptr<gua::node::Node>> split_L (sorted_vector.begin(), sorted_vector.begin()+pivot);
    std::vector<std::shared_ptr<gua::node::Node>> split_R (sorted_vector.begin()+pivot, sorted_vector.end()) ;
    if (split_L.size() > 1 && split_R.size() == 1 && remove_useless_split)
    {
        auto transform_node_L = scene_occlusion_group_node->add_child<gua::node::TransformNode>("t_L");

        for(auto it = sorted_vector.begin(); it != sorted_vector.end(); ++it) {
            if(index < pivot) {
                transform_node_L->add_child(*it);
            } else {
                scene_occlusion_group_node->add_child(*it);
            }
            ++index;
        }


    } else if (split_R.size() > 1 && split_L.size() == 1 && remove_useless_split) {


        auto transform_node_R = scene_occlusion_group_node->add_child<gua::node::TransformNode>("t_R");
        for(auto it = sorted_vector.begin(); it != sorted_vector.end(); ++it) {
            if(index < pivot) {
                scene_occlusion_group_node->add_child(*it);
            } else {
                transform_node_R->add_child(*it);
            }
            ++index;
        }

    } else {
        auto transform_node_L = scene_occlusion_group_node->add_child<gua::node::TransformNode>("t_L");
        auto transform_node_R = scene_occlusion_group_node->add_child<gua::node::TransformNode>("t_R");

        for(auto it = sorted_vector.begin(); it != sorted_vector.end(); ++it) {
            if(index < pivot) {
                transform_node_L->add_child(*it);
            } else {
                transform_node_R->add_child(*it);
            }
            ++index;
        }
    }


    split_R.clear();
    split_L.clear();



}


double OcclusionCullingGroupNode::calculate_cost(gua::node::Node* node) {
    node->update_cache();
    auto parent_surface_area = node->get_bounding_box().surface_area();

    auto children = node->get_children();

    float summed_weighted_surface_area = 0.0f;

    float parent_cost = parent_surface_area * node->num_grouped_faces();



    for(auto& child : children) {
        child->update_cache();
        summed_weighted_surface_area += child->get_bounding_box().surface_area() * child->num_grouped_faces();//(child->get_children()).size();;
    }

    if(parent_cost < summed_weighted_surface_area * 1.5) {
        // std::cout << "PARENT COSTS ARE SMALLER: " << parent_cost << "    " << summed_weighted_surface_area << std::endl;

        return std::numeric_limits<double>::max();
    }

    return (summed_weighted_surface_area) / parent_surface_area;

}

double OcclusionCullingGroupNode::calculate_median_cost(gua::node::Node* node) {
    node->update_cache();
    auto parent_surface_area = node->get_bounding_box().surface_area();

#ifdef MAKE_OCCLUSION_CULLING_APP_VERBOSE
    std::cout << "Surface area Parent: " << parent_surface_area << std::endl;
#endif //MAKE_OCCLUSION_CULLING_APP_VERBOSE

    auto children = node->get_children();

    float summed_weighted_surface_area = 0.0f;

    for(auto& child : children) {
        child->update_cache();
        summed_weighted_surface_area += child->get_bounding_box().surface_area() * child->num_grouped_faces();
    }

    return (summed_weighted_surface_area) / parent_surface_area;

}



} // namespace node
} // namespace gua
