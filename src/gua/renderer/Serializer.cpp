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
#include <gua/renderer/Serializer.hpp>

// guacamole headers
#include <gua/platform.hpp>

#include <gua/databases/GeometryDatabase.hpp>

#include <gua/node/Node.hpp>
#include <gua/node/TransformNode.hpp>
#include <gua/node/LODNode.hpp>
#include <gua/node/SerializableNode.hpp>
#include <gua/scenegraph/SceneGraph.hpp>

// external headers
#include <stack>
#include <utility>
namespace gua
{
////////////////////////////////////////////////////////////////////////////////

Serializer::Serializer() : data_(nullptr), culling_frusta_(), rendering_frusta_(), enable_frustum_culling_(false), enable_alternative_frustum_culling_(false) {}


////////////////////////////////////////////////////////////////////////////////

void Serializer::check(SerializedScene& output, SceneGraph const& scene_graph, Mask const& mask, bool enable_frustum_culling, bool enable_mvr, int view_id)
{
    data_ = &output;
    data_->nodes.clear();
    data_->bounding_boxes.clear();
    data_->clipping_planes.clear();

    enable_frustum_culling_ = enable_frustum_culling;
    enable_alternative_frustum_culling_ = (output.rendering_frustum != output.culling_frustum) && enable_frustum_culling;

    render_mask_ = mask;

    rendering_frusta_.push_back(output.rendering_frustum);
    culling_frusta_.push_back(output.culling_frustum);

    //rendering_frustum_ = output.rendering_frustum;
    //culling_frustum_ = output.culling_frustum;
#ifdef GUACAMOLE_ENABLE_MULTI_VIEW_RENDERING
      if(enable_mvr){
        rendering_frusta_.push_back(output.secondary_rendering_frustum);
        culling_frusta_.push_back(output.secondary_culling_frustum);
      }
#endif //GUACAMOLE_ENABLE_MULTI_VIEW_RENDERING



    for(auto const& plane : scene_graph.get_clipping_plane_nodes())
    {
        if(plane->is_visible(view_id) && render_mask_.check(plane->get_tags()))
        {
            data_->clipping_planes.push_back(plane->get_component_vector());
        }
    }

    scene_graph.accept(*this);
}

////////////////////////////////////////////////////////////////////////////////

void Serializer::check(SerializedScene& output,
           SceneGraph const& scene_graph,
           Mask const& mask,
           bool enable_frustum_culling,
           int view_id){

  check(output, scene_graph, mask, enable_frustum_culling, false, view_id);
}


////////////////////////////////////////////////////////////////////////////////

/* virtual */ void Serializer::visit(node::Node* node)
{
    if(is_visible(node))
    {
        visit_children(node);
    }
}

////////////////////////////////////////////////////////////////////////////////

/* virtual */ void Serializer::visit(node::LODNode* node)
{
    if(is_visible(node))
    {
        float distance_to_camera(scm::math::length(node->get_world_position() - data_->reference_camera_position));

        unsigned child_index(0);

        if(!node->data.get_lod_distances().empty())
        {
            child_index = node->get_children().size();

            for(unsigned node_idx = 0; node_idx < node->data.get_lod_distances().size(); ++node_idx)
            {
                if(node->data.get_lod_distances()[node_idx] > distance_to_camera)
                {
                    child_index = node_idx;
                    break;
                }
            }
        }

        if(child_index < node->get_children().size())
        {
            node->get_children()[child_index]->accept(*this);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

/* virtual */ void Serializer::visit(node::SerializableNode* node)
{
    if(is_visible(node))
    {
        data_->nodes[std::type_index(typeid(*node))].push_back(node);

        visit_children(node);
    }
}

////////////////////////////////////////////////////////////////////////////////

bool Serializer::is_visible(node::Node* node) const
{
    bool is_visible(true);

    // check whether bounding box is (partially) within frustum
    if(enable_frustum_culling_)
    {
        is_visible = false;

        auto bbox(node->get_bounding_box());
        if(bbox != math::BoundingBox<math::vec3>())
        {

            for (auto const& rendering_frustum : rendering_frusta_) {
                if(rendering_frustum.intersects(bbox, data_->clipping_planes) ) {
                    is_visible = true;
                    break;
                }

            }

            if(is_visible && enable_alternative_frustum_culling_)
            {
                is_visible = false;
                for (auto const& culling_frustum : culling_frusta_) {
                    if( culling_frustum.intersects(bbox) ) {
                        is_visible = true;
                        break;
                    }

                }
            }
        }
    }

    // check whether mask allows rendering
    if(is_visible)
    {
        is_visible = render_mask_.check(node->get_tags());
    }

    if(is_visible && node->get_draw_bounding_box())
    {
        data_->bounding_boxes.push_back(node->get_bounding_box());
    }

    return is_visible;
}


////////////////////////////////////////////////////////////////////////////////

void Serializer::visit_children(node::Node* node)
{
    for(auto& c : node->children_)
    {
        c->accept(*this);
    }
}

} // namespace gua
