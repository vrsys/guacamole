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
#include <gua/node/Node.hpp>
#include <gua/math/BoundingBoxAlgo.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/WindowBase.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/utils/string_utils.hpp>
#include <gua/node/RayNode.hpp>
#include <gua/scenegraph/NodeVisitor.hpp>

namespace gua
{
namespace node
{
////////////////////////////////////////////////////////////////////////////////

Node::Node(std::string const& name, math::mat4 const& transform) : children_(), name_(name), transform_(transform), bounding_box_(), user_data_() {

    //std::cout << "Recreated node: " << name_ << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

#if 0
Node::~Node() {
  for (auto const& child : children_) {
    child->parent_ = nullptr;
    child->set_scenegraph(nullptr);
  }
}
#endif

std::size_t Node::num_grouped_faces() const {
    std::size_t accumulated_face_count = 0;

    for(auto const& child : children_) {
        accumulated_face_count += child->num_grouped_faces();
    }

    return accumulated_face_count;
} 

////////////////////////////////////////////////////////////////////////////////

void Node::update_cache()
{
    if(self_dirty_)
    {
        math::mat4 old_world_trans(world_transform_);

        if(is_root())
        {
            world_transform_ = get_transform(); // transform_;
        }
        else
        {
            world_transform_ = parent_->world_transform_ * get_transform(); // transform_;
        }

        if(world_transform_ != old_world_trans)
        {
            on_world_transform_changed.emit(world_transform_);
        }

        self_dirty_ = false;
    }

    if(child_dirty_)
    {
        for(auto const& child : children_)
        {
            child->update_cache();
        }

        update_bounding_box();

        child_dirty_ = false;
    }
}

////////////////////////////////////////////////////////////////////////////////

void Node::clear_children()
{
    if(children_.size() > 0)
    {
        for(auto const& child : children_)
        {
            child->parent_ = nullptr;
            child->set_scenegraph(nullptr);
        }

        set_dirty();

        children_.clear();
    }
}

////////////////////////////////////////////////////////////////////////////////

void Node::remove_child(std::shared_ptr<Node> const& child)
{
    for(auto c(children_.begin()); c != children_.end(); ++c)
    {
        if(*c == child)
        {
            children_.erase(c);
            child->parent_ = nullptr;
            set_dirty();
            child->set_scenegraph(nullptr);

            break;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

gua::utils::TagList const& Node::get_tags() const { return tags_; }

////////////////////////////////////////////////////////////////////////////////

gua::utils::TagList& Node::get_tags() { return tags_; }

////////////////////////////////////////////////////////////////////////////////

math::mat4 Node::get_world_transform() const
{
    if(parent_)
        return parent_->get_world_transform() * get_transform();

    return get_transform();
}

////////////////////////////////////////////////////////////////////////////////

math::mat4 Node::get_latest_world_transform(const WindowBase* w) const {

	int registered_node_id = w->is_node_registered(get_path());
	math::mat4 result;

	switch (registered_node_id) {
		case 0: // Vive HMD
			result = w->get_latest_matrices(4);
			break;
		case 1: // Vive CONTROLLER_0
			result = w->get_latest_matrices(5);
			break;
		case 11: // Vive CONTROLLER_0 (yaw only)
			result = gua::math::get_yaw_matrix(w->get_latest_matrices(5));
			break;
		case 2: // Vive CONTROLLER_1
			result = w->get_latest_matrices(6);
			break;
		case 21: // Vive CONTROLLER_1 (yaw only)
			result = gua::math::get_yaw_matrix(w->get_latest_matrices(6));
			break;
		default:
			result = get_transform();
			break;
	}

  if (parent_)
    return parent_->get_latest_world_transform(w) * result;

  return result;
}

////////////////////////////////////////////////////////////////////////////////

math::mat4 const& Node::get_cached_world_transform() const { return world_transform_; }

////////////////////////////////////////////////////////////////////////////////

math::mat4 Node::get_latest_cached_world_transform(const WindowBase* w) const {
  math::mat4 world_transform = world_transform_;
  world_transform_ = get_latest_world_transform(w);
  update_bounding_box();
  return world_transform;
}

////////////////////////////////////////////////////////////////////////////////

void Node::set_world_transform(math::mat4 const& transform)
{
    if(is_root())
    {
        transform_ = transform;
    }
    else
    {
        transform_ = scm::math::inverse(parent_->get_world_transform()) * transform;
    }

    set_dirty();
}

////////////////////////////////////////////////////////////////////////////////

math::vec3 Node::get_world_position() const { return gua::math::get_translation(get_world_transform()); }

////////////////////////////////////////////////////////////////////////////////

void Node::set_transform(math::mat4 const& transform)
{
    transform_ = transform;
    set_dirty();
}

////////////////////////////////////////////////////////////////////////////////

void Node::scale(math::float_t s) { scale(s, s, s); }

////////////////////////////////////////////////////////////////////////////////

void Node::scale(math::float_t x, math::float_t y, math::float_t z)
{
    transform_ = scm::math::make_scale(x, y, z) * transform_;
    set_dirty();
}

////////////////////////////////////////////////////////////////////////////////

void Node::scale(math::vec3 const& s) { scale(s.x, s.y, s.z); }

////////////////////////////////////////////////////////////////////////////////

void Node::rotate(math::float_t angle, math::float_t x, math::float_t y, math::float_t z)
{
    transform_ = scm::math::make_rotation(angle, x, y, z) * transform_;
    set_dirty();
}

////////////////////////////////////////////////////////////////////////////////

void Node::rotate(math::float_t angle, math::vec3 const& axis) { rotate(angle, axis.x, axis.y, axis.z); }

////////////////////////////////////////////////////////////////////////////////

void Node::translate(math::float_t x, math::float_t y, math::float_t z)
{
    transform_ = scm::math::make_translation(x, y, z) * transform_;
    set_dirty();
}

////////////////////////////////////////////////////////////////////////////////

void Node::translate(math::vec3 const& offset) { translate(offset.x, offset.y, offset.z); }

////////////////////////////////////////////////////////////////////////////////

int Node::get_depth() const
{
    if(!parent_)
    {
        return 0;
    }

    return parent_->get_depth() + 1;
}

////////////////////////////////////////////////////////////////////////////////

std::string Node::get_path() const
{
    if(!parent_)
    {
        return "/";
    }

    auto parent_path(parent_->get_path());
    if(parent_path != "/")
        return parent_path + "/" + name_;
    return parent_path + name_;
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Node> Node::get_parent_shared() const
{
    if(!parent_)
    {
        return nullptr;
    }

    for(auto const& child : parent_->get_children())
    {
        if(&*child == this)
        {
            return child;
        }
    }

    throw std::runtime_error("Node::get_parent_shared(): No such node.");
}

////////////////////////////////////////////////////////////////////////////////

void Node::update_bounding_box() const
{
    bounding_box_ = math::BoundingBox<math::vec3>();

    for(auto const& child : children_)
    {
        bounding_box_.expandBy(child->get_bounding_box());
    }
}

////////////////////////////////////////////////////////////////////////////////

void Node::set_draw_bounding_box(bool draw) { draw_bounding_box_ = draw; }

////////////////////////////////////////////////////////////////////////////////

bool Node::get_draw_bounding_box() const { return draw_bounding_box_; }

////////////////////////////////////////////////////////////////////////////////

std::set<PickResult> const Node::ray_test(RayNode const& ray, int options, Mask const& mask) { return ray_test(ray.get_world_ray(), options, mask); }

////////////////////////////////////////////////////////////////////////////////

std::set<PickResult> const Node::ray_test(Ray const& ray, int options, Mask const& mask)
{
    std::set<PickResult> hits;
    ray_test_impl(ray, options, mask, hits);
    return hits;
}
////////////////////////////////////////////////////////////////////////////////

void Node::ray_test_impl(Ray const& ray, int options, Mask const& mask, std::set<PickResult>& hits)
{
    auto box_hits(::gua::intersect(ray, bounding_box_));

    // ray did not intersect bbox -- therefore it wont intersect any child
    if(box_hits.first == Ray::END && box_hits.second == Ray::END)
    {
        return;
    }

    // return if only first object shall be returned and the current first hit
    // is in front of the bbox entry point and the ray does not start inside
    // the bbox
    if(options & PickResult::PICK_ONLY_FIRST_OBJECT && hits.size() > 0 && hits.begin()->distance < box_hits.first && box_hits.first != Ray::END)
    {
        return;
    }

    // check mask
    if(!mask.check(get_tags()))
    {
        return;
    }

    for(auto child : children_)
    {
        // test for intersection with each child
        child->ray_test_impl(ray, options, mask, hits);
    }
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Node> Node::deep_copy() const
{
    std::shared_ptr<Node> copied_node = copy();
    copied_node->tags_ = tags_;
    copied_node->draw_bounding_box_ = draw_bounding_box_;
    copied_node->scenegraph_ = scenegraph_;
    copied_node->bounding_box_ = bounding_box_;
    copied_node->user_data_ = user_data_;
    copied_node->world_transform_ = world_transform_;

    // the id should be unique, so we do not copy it
    //copied_node->uuid_ = boost::hash<boost::uuids::uuid>()(boost::uuids::random_generator()());

    //copying of uuid -> will not be unique anymore
    copied_node->uuid_=uuid_;

    for(int i(0); i < children_.size(); ++i)
    {
        copied_node->children_[i] = children_[i]->deep_copy();
        copied_node->children_[i]->parent_ = copied_node.get();
    }

    return copied_node;
}

////////////////////////////////////////////////////////////////////////////////

void* Node::get_user_data(unsigned handle) const
{
    if(user_data_.size() > handle)
        return user_data_[handle];
    else
        return nullptr;
}

////////////////////////////////////////////////////////////////////////////////

unsigned Node::add_user_data(void* data)
{
    user_data_.push_back(data);
    return unsigned(user_data_.size() - 1);
}

////////////////////////////////////////////////////////////////////////////////

void Node::set_dirty() const
{
    set_children_dirty();
    set_parent_dirty();
}

////////////////////////////////////////////////////////////////////////////////

void Node::set_parent_dirty() const
{
    if(!is_root() && !parent_->child_dirty_)
    {
        parent_->child_dirty_ = true;
        parent_->set_parent_dirty();
    }
}

////////////////////////////////////////////////////////////////////////////////

void Node::set_children_dirty() const
{
    self_dirty_ = true;
    child_dirty_ = true;
    for(auto const& child : children_)
    {
        child->set_children_dirty();
    }
}

////////////////////////////////////////////////////////////////////////////////

void Node::set_scenegraph(SceneGraph* scenegraph)
{
    scenegraph_ = scenegraph;

    for(auto const& child : children_)
    {
        child->set_scenegraph(scenegraph);
    }
}

////////////////////////////////////////////////////////////////////////////////

/*
bool Node::get_visibility(std::size_t in_camera_uuid) const {
    return is_visible_for_camera_[in_camera_uuid];
}

////////////////////////////////////////////////////////////////////////////////

void Node::set_visibility(std::size_t in_camera_uuid, bool is_visible) {
    is_visible_for_camera_[in_camera_uuid] = is_visible;
}

////////////////////////////////////////////////////////////////////////////////

int32_t Node::get_last_visibility_check_frame_id(std::size_t in_camera_uuid) const {
    return last_visibility_check_frame_id_[in_camera_uuid];
}

////////////////////////////////////////////////////////////////////////////////

void Node::set_last_visibility_check_frame_id(std::size_t in_camera_uuid, int32_t current_frame_id) {
    last_visibility_check_frame_id_[in_camera_uuid] = current_frame_id;
}

*/
////////////////////////////////////////////////////////////////////////////////

} // namespace node
} // namespace gua
