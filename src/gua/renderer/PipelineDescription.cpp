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
#include <gua/renderer/PipelineDescription.hpp>

// guacamole headers
#include <gua/renderer/StencilPass.hpp>
#include <gua/renderer/TriMeshPass.hpp>
#include <gua/renderer/OcclusionCullingTriMeshPass.hpp>
#include <gua/renderer/LineStripPass.hpp>
#include <gua/renderer/LightVisibilityPass.hpp>
#include <gua/renderer/BBoxPass.hpp>
#include <gua/renderer/TexturedQuadPass.hpp>
#include <gua/renderer/TexturedScreenSpaceQuadPass.hpp>
#include <gua/renderer/ResolvePass.hpp>
#include <gua/renderer/FullscreenColorBufferViewPass.hpp>
#include <gua/renderer/DebugViewPass.hpp>
#include <gua/renderer/SSAAPass.hpp>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<PipelineDescription> PipelineDescription::make_default()
{
    auto pipe(std::make_shared<PipelineDescription>());

    pipe->add_pass(std::make_shared<TriMeshPassDescription>());
    pipe->add_pass(std::make_shared<LineStripPassDescription>());
    pipe->add_pass(std::make_shared<TexturedQuadPassDescription>());
    pipe->add_pass(std::make_shared<LightVisibilityPassDescription>());
    pipe->add_pass(std::make_shared<BBoxPassDescription>());
    pipe->add_pass(std::make_shared<ResolvePassDescription>());
    pipe->add_pass(std::make_shared<TexturedScreenSpaceQuadPassDescription>());

    pipe->set_enable_abuffer(false);

    return pipe;
}

////////////////////////////////////////////////////////////////////////////////

PipelineDescription::PipelineDescription(PipelineDescription const& other)
{
    for(auto pass : other.passes_)
    {
        passes_.push_back(pass->make_copy());
    }

    enable_abuffer_ = other.enable_abuffer_;
    abuffer_size_ = other.abuffer_size_;
    blending_termination_threshold_ = other.blending_termination_threshold_;
    max_lights_count_ = other.max_lights_count_;
}

////////////////////////////////////////////////////////////////////////////////

PipelineDescription::~PipelineDescription() {}

////////////////////////////////////////////////////////////////////////////////
void PipelineDescription::add_pass(std::shared_ptr<PipelinePassDescription> const& pass_desc) { passes_.push_back(pass_desc); }

////////////////////////////////////////////////////////////////////////////////

std::vector<std::shared_ptr<PipelinePassDescription>> const& PipelineDescription::get_passes() const { return passes_; }

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<PipelinePassDescription> const& PipelineDescription::get_pass(std::size_t index) const
{
    assert(index < passes_.size());
    return passes_[index];
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<TriMeshPassDescription> const PipelineDescription::get_tri_mesh_pass() const { return get_pass_by_type<TriMeshPassDescription>(); }

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<OcclusionCullingTriMeshPassDescription> const PipelineDescription::get_occlusion_culling_tri_mesh_pass() const { return get_pass_by_type<OcclusionCullingTriMeshPassDescription>(); }

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<LineStripPassDescription> const PipelineDescription::get_line_strip_pass() const { return get_pass_by_type<LineStripPassDescription>(); }

////////////////////////////////////////////////////////////////////////////////

// std::shared_ptr<SkeletalAnimationPassDescription> const PipelineDescription::get_skel_anim_pass() const
// {
//   return get_pass_by_type<SkeletalAnimationPassDescription>();
// }

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<TexturedQuadPassDescription> const PipelineDescription::get_textured_quad_pass() const { return get_pass_by_type<TexturedQuadPassDescription>(); }

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<LightVisibilityPassDescription> const PipelineDescription::get_light_visibility_pass() const { return get_pass_by_type<LightVisibilityPassDescription>(); }

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<BBoxPassDescription> const PipelineDescription::get_bbox_pass() const { return get_pass_by_type<BBoxPassDescription>(); }

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<ResolvePassDescription> const PipelineDescription::get_resolve_pass() const { return get_pass_by_type<ResolvePassDescription>(); }

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<TexturedScreenSpaceQuadPassDescription> const PipelineDescription::get_textured_screen_space_quad_pass() const { return get_pass_by_type<TexturedScreenSpaceQuadPassDescription>(); }

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<FullscreenColorBufferViewPassDescription> const PipelineDescription::get_full_screen_color_buffer_view_pass() const { return get_pass_by_type<FullscreenColorBufferViewPassDescription>(); }

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<DebugViewPassDescription> const PipelineDescription::get_debug_view_pass() const { return get_pass_by_type<DebugViewPassDescription>(); }

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<SSAAPassDescription> const PipelineDescription::get_ssaa_pass() const { return get_pass_by_type<SSAAPassDescription>(); }

////////////////////////////////////////////////////////////////////////////////

bool PipelineDescription::operator==(PipelineDescription const& other) const
{
    if(enable_abuffer_ != other.enable_abuffer_ || abuffer_size_ != other.abuffer_size_ || blending_termination_threshold_ != other.blending_termination_threshold_ ||
       max_lights_count_ != other.max_lights_count_ || passes_.size() != other.passes_.size())
    {
        return false;
    }

    for(unsigned int i = 0; i < passes_.size(); ++i)
    {
        if(typeid(*passes_[i]) != typeid(*other.passes_[i]))
        {
            return false;
        }
        if((*passes_[i]) != (*other.passes_[i]))
        {
            return false;
        }
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////

bool PipelineDescription::operator!=(PipelineDescription const& other) const { return !(*this == other); }

////////////////////////////////////////////////////////////////////////////////

PipelineDescription& PipelineDescription::operator=(PipelineDescription const& other)
{
    passes_.clear();

    for(auto const& pass : other.passes_)
    {
        passes_.push_back(pass->make_copy());
    }

    enable_abuffer_ = other.enable_abuffer_;
    abuffer_size_ = other.abuffer_size_;
    blending_termination_threshold_ = other.blending_termination_threshold_;
    max_lights_count_ = other.max_lights_count_;

    return *this;
}

////////////////////////////////////////////////////////////////////////////////

// getter and setter for occlusion culling render modes
OcclusionCullingMode PipelinePassDescription::get_occlusion_culling_mode() const {
    return occlusion_culling_mode_;
}

void PipelinePassDescription::set_occlusion_culling_mode(OcclusionCullingMode const& oc_mode) {
    occlusion_culling_mode_ = oc_mode;
}

} // namespace gua
