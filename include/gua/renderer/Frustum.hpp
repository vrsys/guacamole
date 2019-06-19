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

#ifndef GUA_FRUSTUM_HPP
#define GUA_FRUSTUM_HPP

// guacamole headers
#include <gua/math/BoundingBox.hpp>
#include <gua/math/math.hpp>
#include <gua/scenegraph/PickResult.hpp>

#include <set>

namespace gua
{
struct Ray;

namespace node
{
class RayNode;
}

/**
 *
 */
class GUA_DLL Frustum
{
  public:
    Frustum();

    static Frustum perspective(math::mat4 const& camera_transform, math::mat4 const& screen_transform, math::mat4::value_type clip_near, math::mat4::value_type clip_far);

    static Frustum orthographic(math::mat4 const& camera_transform, math::mat4 const& screen_transform, math::mat4::value_type clip_near, math::mat4::value_type clip_far);

    std::vector<math::vec3> get_corners() const;

    inline math::vec3 get_camera_position() const { return math::vec3(camera_transform_.column(3)[0], camera_transform_.column(3)[1], camera_transform_.column(3)[2]); }

    inline math::mat4 const& get_camera_transform() const { return camera_transform_; }
    inline math::mat4 const& get_screen_transform() const { return screen_transform_; }

    inline math::mat4 const& get_projection() const { return projection_; }
    inline math::mat4 const& get_view() const { return view_; }
	inline void set_view(math::mat4 const& view) const { view_ = view; }
    inline math::mat4::value_type get_clip_near() const { return clip_near_; }
    inline math::mat4::value_type get_clip_far() const { return clip_far_; }

    bool intersects(math::BoundingBox<math::vec3> const& bbox, std::vector<math::vec4> const& global_planes = {}) const;
    bool contains(math::vec3 const& point) const;

    std::set<PickResult> const ray_test(node::RayNode const& ray, int options = PickResult::PICK_ALL);

    std::set<PickResult> const ray_test(Ray const& ray, int options = PickResult::PICK_ALL);

    bool operator==(Frustum const& other) const { return projection_ == other.projection_ && clip_near_ == other.clip_near_ && clip_far_ == other.clip_far_; }
    bool operator!=(Frustum const& other) const { return !(*this == other); }

  private:
    static void init_frustum_members(math::mat4 const& camera_transform, math::mat4 const& screen_transform, Frustum& frustum);

    math::mat4 camera_transform_;
    math::mat4 screen_transform_;
    math::mat4 projection_;
    mutable math::mat4 view_;
    std::vector<math::vec4> planes_;
    math::mat4::value_type clip_near_;
    math::mat4::value_type clip_far_;
};

} // namespace gua

#endif // GUA_FRUSTUM_HPP
