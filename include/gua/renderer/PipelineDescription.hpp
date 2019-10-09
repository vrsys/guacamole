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

#ifndef GUA_PIPELINE_DESCRIPTION_HPP
#define GUA_PIPELINE_DESCRIPTION_HPP

#include <gua/renderer/PipelinePass.hpp>
#include <gua/math.hpp>

#include <memory>
#include <algorithm>

namespace gua
{
class TriMeshPassDescription;
class LineStripPassDescription;
class DynamicGeometryPassDescription;
class DynamicLinePassDescription;
class DynamicTrianglePassDescription;
// class SkeletalAnimationPassDescription;
class TexturedQuadPassDescription;
class LightVisibilityPassDescription;
class BBoxPassDescription;
class ResolvePassDescription;
class TexturedScreenSpaceQuadPassDescription;
class DebugViewPassDescription;
class SSAAPassDescription;

class GUA_DLL PipelineDescription
{
  public:
    static std::shared_ptr<PipelineDescription> make_default();

    PipelineDescription() {}
    PipelineDescription(PipelineDescription const& other);

    virtual ~PipelineDescription();

    void add_pass(std::shared_ptr<PipelinePassDescription> const& pass_desc);

    std::vector<std::shared_ptr<PipelinePassDescription>> const& get_passes() const;

    std::shared_ptr<PipelinePassDescription> const& get_pass(std::size_t index) const;

    std::shared_ptr<TriMeshPassDescription> const get_tri_mesh_pass() const;
    std::shared_ptr<LineStripPassDescription> const get_line_strip_pass() const;
    std::shared_ptr<DynamicGeometryPassDescription> const get_dynamic_geometry_pass() const;
    std::shared_ptr<DynamicLinePassDescription> const get_dynamic_line_pass() const;
    std::shared_ptr<DynamicTrianglePassDescription> const get_dynamic_triangle_pass() const;
    // std::shared_ptr<SkeletalAnimationPassDescription> const get_skel_anim_pass() const;
    std::shared_ptr<TexturedQuadPassDescription> const get_textured_quad_pass() const;
    std::shared_ptr<LightVisibilityPassDescription> const get_light_visibility_pass() const;
    std::shared_ptr<BBoxPassDescription> const get_bbox_pass() const;
    std::shared_ptr<ResolvePassDescription> const get_resolve_pass() const;
    std::shared_ptr<TexturedScreenSpaceQuadPassDescription> const get_textured_screen_space_quad_pass() const;
    std::shared_ptr<DebugViewPassDescription> const get_debug_view_pass() const;
    std::shared_ptr<SSAAPassDescription> const get_ssaa_pass() const;

    void set_enable_abuffer(bool value) { enable_abuffer_ = value; }

    bool get_enable_abuffer() const { return enable_abuffer_; }

    void set_abuffer_size(size_t value) { abuffer_size_ = value; }

    size_t get_abuffer_size() const { return abuffer_size_; }

    void set_blending_termination_threshold(float value) { blending_termination_threshold_ = std::max(std::min(value, 1.f), .5f); }

    float get_blending_termination_threshold() const { return blending_termination_threshold_; }

    void set_max_lights_count(int value) { max_lights_count_ = value; }

    int get_max_lights_count() const { return max_lights_count_; }

    void set_user_data(void* data) { user_data_ = data; }

    void* get_user_data() const { return user_data_; }

    bool operator==(PipelineDescription const& other) const;
    bool operator!=(PipelineDescription const& other) const;
    PipelineDescription& operator=(PipelineDescription const& other);

    template <typename T>
    std::shared_ptr<T> const get_pass_by_type() const
    {
        for(auto const& pass : passes_)
        {
            auto const& casted_pass = std::dynamic_pointer_cast<T>(pass);
            if(casted_pass)
            {
                return casted_pass;
            }
        }
        throw std::runtime_error("PipelinePassDescription::get_pass_by_type: No such pass in PipelineDescription");
    }

    inline void clear() { passes_.clear(); }

  private:
    std::vector<std::shared_ptr<PipelinePassDescription>> passes_;
    void* user_data_ = nullptr;
    bool enable_abuffer_ = false;
    size_t abuffer_size_ = 800; // in MiB
    float blending_termination_threshold_ = 0.99f;
    int max_lights_count_ = 128;
};

} // namespace gua

#endif // GUA_PIPELINE_DESCRIPTION_HPP
