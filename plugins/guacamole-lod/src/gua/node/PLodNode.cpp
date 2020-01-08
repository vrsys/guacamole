#/******************************************************************************
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
#include "gua/node/PLodNode.hpp"

#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>
#include <gua/databases/TimeSeriesDataSetDatabase.hpp>

#include <gua/node/RayNode.hpp>
#include <gua/renderer/LodLoader.hpp>
#include <gua/renderer/LodResource.hpp>
#include <gua/renderer/Material.hpp>
#include <gua/renderer/ShaderProgram.hpp>

#include <gua/math/BoundingBoxAlgo.hpp>

#include <lamure/ren/policy.h>

// guacamole headers

namespace gua
{
namespace node
{
////////////////////////////////////////////////////////////////////////////////
PLodNode::PLodNode(std::string const& name,
                   std::string const& geometry_description,
                   std::string const& geometry_file_path,
                   std::shared_ptr<Material> const& material,
                   math::mat4 const& transform,
                   float const max_surfel_size,
                   float const scale,
                   float const threshold,
                   bool const enable_backface_culling_by_normal)
    : GeometryNode(name, transform), geometry_(nullptr), geometry_changed_(true), geometry_description_(geometry_description), geometry_file_path_(geometry_file_path), material_(material),
      radius_scale_(scale), max_surfel_size_(max_surfel_size), error_threshold_(threshold), enable_backface_culling_by_normal_(enable_backface_culling_by_normal)
{
}

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<LodResource> const& PLodNode::get_geometry() const { return geometry_; }

////////////////////////////////////////////////////////////////////////////////
math::mat4 PLodNode::get_world_transform() const
{
    if(!geometry_)
    {
        return Node::get_world_transform();
    }
    else
    {
        if(get_parent())
        {
            return get_parent()->get_world_transform() * get_transform() * geometry_->local_transform();
        }
        else
        {
            return get_transform() * geometry_->local_transform();
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
std::string const& PLodNode::get_geometry_file_path() const { return geometry_file_path_; }

////////////////////////////////////////////////////////////////////////////////
std::string const& PLodNode::get_geometry_description() const { return geometry_description_; }

////////////////////////////////////////////////////////////////////////////////
void PLodNode::set_geometry_description(std::string const& v)
{
    geometry_description_ = v;
    geometry_changed_ = self_dirty_ = true;
}

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<Material> const& PLodNode::get_material() const { return material_; }

////////////////////////////////////////////////////////////////////////////////
void PLodNode::set_material(std::shared_ptr<Material> const& material)
{
    material_ = material;
    material_changed_ = self_dirty_ = true;
}

////////////////////////////////////////////////////////////////////////////////
float PLodNode::get_radius_scale() const { return radius_scale_; }

////////////////////////////////////////////////////////////////////////////////
void PLodNode::set_radius_scale(float scale)
{
    radius_scale_ = scale;
    self_dirty_ = true;
}

////////////////////////////////////////////////////////////////////////////////
void PLodNode::set_enable_time_series_deformation(bool enable_deformation) {
    enable_time_series_deformation_ = enable_deformation;
}

////////////////////////////////////////////////////////////////////////////////
bool PLodNode::get_enable_time_series_deformation() const {
    return enable_time_series_deformation_;
}

////////////////////////////////////////////////////////////////////////////////
void PLodNode::set_enable_time_series_coloring(bool enable_coloring) {
    enable_time_series_coloring_ = enable_coloring;
}

////////////////////////////////////////////////////////////////////////////////
bool PLodNode::get_enable_time_series_coloring() const {
    return enable_time_series_coloring_;
}

////////////////////////////////////////////////////////////////////////////////
float PLodNode::get_max_surfel_radius() const { return max_surfel_size_; }

////////////////////////////////////////////////////////////////////////////////
void PLodNode::set_max_surfel_radius(float threshold)
{
    max_surfel_size_ = threshold;
    self_dirty_ = true;
}

////////////////////////////////////////////////////////////////////////////////
float PLodNode::get_error_threshold() const { return error_threshold_; }

////////////////////////////////////////////////////////////////////////////////
void PLodNode::set_error_threshold(float threshold)
{
    error_threshold_ = threshold;
    self_dirty_ = true;
}

////////////////////////////////////////////////////////////////////////////////
void PLodNode::set_enable_backface_culling_by_normal(bool const enable_backface_culling)
{
    enable_backface_culling_by_normal_ = enable_backface_culling;
    self_dirty_ = true;
}

////////////////////////////////////////////////////////////////////////////////
bool PLodNode::get_enable_backface_culling_by_normal() const { return enable_backface_culling_by_normal_; }

////////////////////////////////////////////////////////////////////////////////
void PLodNode::update_time_cursor(float elapsed_frame_time_seconds) {
  if(enable_automatic_playback_) {
       if( !associated_time_series_data_descriptions_.empty() ) {

            auto const& active_time_series_data_description = associated_time_series_data_descriptions_[active_time_series_data_description_index_];
            auto looked_up_time_series_data_item = TimeSeriesDataSetDatabase::instance()->lookup(active_time_series_data_description);

            if(looked_up_time_series_data_item) {
                if(1 != looked_up_time_series_data_item->num_timesteps) {
                    looked_up_time_series_data_item->time_cursor_position += time_series_playback_speed_ * elapsed_frame_time_seconds;
                    looked_up_time_series_data_item->time_cursor_position = std::fmod(looked_up_time_series_data_item->time_cursor_position, looked_up_time_series_data_item->sequence_length);
                } else {
                   looked_up_time_series_data_item->time_cursor_position = 0.0f; 
                }
            }
            //}
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
void PLodNode::set_attribute_to_visualize_index(int attribute_to_visualize_index) {
    attribute_to_visualize_index_ = attribute_to_visualize_index;  
}


////////////////////////////////////////////////////////////////////////////////
int PLodNode::get_attribute_to_visualize_index() const {
    return attribute_to_visualize_index_;   
}

////////////////////////////////////////////////////////////////////////////////
void PLodNode::set_time_cursor_position(float time_cursor) {
    if( !associated_time_series_data_descriptions_.empty() ) {
        auto const& active_time_series_data_description = associated_time_series_data_descriptions_[active_time_series_data_description_index_];

        auto looked_up_time_series_data_item = TimeSeriesDataSetDatabase::instance()->lookup(active_time_series_data_description);

        if(looked_up_time_series_data_item) {
            looked_up_time_series_data_item->time_cursor_position = time_cursor;
        }
        
    }
}

////////////////////////////////////////////////////////////////////////////////
float PLodNode::get_time_cursor_position() const {
    if( !associated_time_series_data_descriptions_.empty() ) {
        auto const& active_time_series_data_description = associated_time_series_data_descriptions_[active_time_series_data_description_index_];
        auto looked_up_time_series_data_item = TimeSeriesDataSetDatabase::instance()->lookup(active_time_series_data_description);

        if(looked_up_time_series_data_item) {
            return looked_up_time_series_data_item->time_cursor_position;
        }
        return -1.0f;
    } else {
        return -1.0f;
    }
}

////////////////////////////////////////////////////////////////////////////////
void PLodNode::set_enable_automatic_playback(bool enable_automatic_playback) {
    enable_automatic_playback_ = enable_automatic_playback;
}

////////////////////////////////////////////////////////////////////////////////
bool PLodNode::get_enable_automatic_playback() const {
    return enable_automatic_playback_;
}

////////////////////////////////////////////////////////////////////////////////
void PLodNode::set_enable_temporal_interpolation(bool enable_temporal_interpolation) {
    enable_temporal_interpolation_ = enable_temporal_interpolation;
}

////////////////////////////////////////////////////////////////////////////////
bool PLodNode::get_enable_temporal_interpolation() const {
    return enable_temporal_interpolation_;
}

////////////////////////////////////////////////////////////////////////////////
void PLodNode::set_time_series_playback_speed(float time_series_playback_speed) {
    time_series_playback_speed_ = std::max(0.0f, time_series_playback_speed);
}

////////////////////////////////////////////////////////////////////////////////
float PLodNode::get_time_series_playback_speed() const {
    return time_series_playback_speed_;
}

////////////////////////////////////////////////////////////////////////////////
void PLodNode::set_time_series_deform_factor(float time_series_deform_factor) {
    time_series_deform_factor_ = time_series_deform_factor;
}

////////////////////////////////////////////////////////////////////////////////
float PLodNode::get_time_series_deform_factor() const {
    return time_series_deform_factor_;
}

////////////////////////////////////////////////////////////////////////////////
void PLodNode::set_active_time_series_index(unsigned int time_series_index) {
    active_time_series_data_description_index_ = std::max(0u, std::min(time_series_index, unsigned(associated_time_series_data_descriptions_.size()) ) );
}

////////////////////////////////////////////////////////////////////////////////
int PLodNode::get_active_time_series_index() const {
    return active_time_series_data_description_index_;
}

////////////////////////////////////////////////////////////////////////////////
void PLodNode::bind_time_series_data_to(RenderContext& ctx, std::shared_ptr<ShaderProgram>& current_program) {

    if( !associated_time_series_data_descriptions_.empty() ) {
        auto const& active_time_series_description = associated_time_series_data_descriptions_[active_time_series_data_description_index_];

        //for(auto const& data_description : time_series_data_descriptions) {
        auto looked_up_time_series_data_item = TimeSeriesDataSetDatabase::instance()->lookup(active_time_series_description);


        float current_timecursor_position = get_time_cursor_position();
        current_timecursor_position = looked_up_time_series_data_item->calculate_active_cursor_position(current_timecursor_position);

        if(looked_up_time_series_data_item) {
            //std::cout << "FOUND DATA ITEM" << std::endl;
            //std::cout << looked_up_time_series_data_item->data.size() << std::endl;

            unsigned int timerange_to_upload_start = int(current_timecursor_position);
            unsigned int timerange_to_upload_end = timerange_to_upload_start + 1;
            if(looked_up_time_series_data_item->num_timesteps <= timerange_to_upload_end) {
                timerange_to_upload_end = timerange_to_upload_start;
            }
            looked_up_time_series_data_item->upload_time_range_to(ctx, enable_time_series_deformation_, enable_time_series_coloring_, attribute_to_visualize_index_, timerange_to_upload_start, timerange_to_upload_end);
        }

        looked_up_time_series_data_item->bind_to(ctx, 20, current_program, attribute_to_visualize_index_);

        current_program->set_uniform(ctx, current_timecursor_position, "current_timestep");
        current_program->set_uniform(ctx, enable_time_series_deformation_, "enable_time_series_deformation");
        current_program->set_uniform(ctx, enable_time_series_coloring_, "enable_time_series_coloring");
        current_program->set_uniform(ctx, time_series_deform_factor_, "deform_factor");               
        current_program->set_uniform(ctx, enable_temporal_interpolation_, "enable_linear_temporal_interpolation");
    }

}

////////////////////////////////////////////////////////////////////////////////
void PLodNode::ray_test_impl(Ray const& ray, int options, Mask const& mask, std::set<PickResult>& hits)
{
    // first of all, check bbox
    auto box_hits(::gua::intersect(ray, bounding_box_));

    // ray did not intersect bbox -- therefore it wont intersect
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

    // bbox is intersected, but check geometry only if mask tells us to check
    if(get_geometry_description() != "" && mask.check(get_tags()))
    {
        auto geometry(GeometryDatabase::instance()->lookup(get_geometry_description()));

        if(geometry)
        {
            Ray world_ray(ray);
            geometry->ray_test(world_ray, options, this, hits);
        }
    }

    for(auto child : get_children())
    {
        child->ray_test_impl(ray, options, mask, hits);
    }
}

////////////////////////////////////////////////////////////////////////////////
void PLodNode::update_bounding_box() const
{
    if(geometry_)
    {
        auto geometry_bbox(geometry_->get_bounding_box());
        bounding_box_ = transform(geometry_bbox, world_transform_);

        for(auto child : get_children())
        {
            bounding_box_.expandBy(child->get_bounding_box());
        }
    }
    else
    {
        Node::update_bounding_box();
    }
}

////////////////////////////////////////////////////////////////////////////////
void PLodNode::update_cache()
{
    if(geometry_changed_)
    {
        if(geometry_description_ != "")
        {
            if(!GeometryDatabase::instance()->contains(geometry_description_))
            {
                GeometryDescription desc(geometry_description_);
                try
                {
                    gua::LodLoader loader;
                    loader.load_lod_pointcloud(desc.filepath(), desc.flags());
                }
                catch(std::exception& e)
                {
                    Logger::LOG_WARNING << "PLodNode::update_cache(): Loading failed from " << desc.filepath() << " : " << e.what() << std::endl;
                }
            }
            geometry_ = std::dynamic_pointer_cast<LodResource>(GeometryDatabase::instance()->lookup(geometry_description_));

            if(!geometry_)
            {
                Logger::LOG_WARNING << "Failed to get LodResource for " << geometry_description_ << ": The data base entry is of wrong type!" << std::endl;
            }
        }

        geometry_changed_ = false;
    }

    // modified version of Node::update_cache -> add local transformation
    if(self_dirty_)
    {
        math::mat4 old_world_trans(world_transform_);

        if(is_root())
        {
            world_transform_ = transform_ * geometry_->local_transform();
        }
        else
        {
            world_transform_ = get_parent()->get_world_transform() * transform_ * geometry_->local_transform();
        }

        update_bounding_box();

        if(world_transform_ != old_world_trans)
        {
            on_world_transform_changed.emit(world_transform_);
        }

        self_dirty_ = false;
    }

    if(child_dirty_)
    {
        for(auto const& child : get_children())
        {
            child->update_cache();
        }

        update_bounding_box();

        child_dirty_ = false;
    }
}

////////////////////////////////////////////////////////////////////////////////
/* virtual */ void PLodNode::accept(NodeVisitor& visitor) { visitor.visit(this); }

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<Node> PLodNode::copy() const
{
    std::shared_ptr<PLodNode> result = std::make_shared<PLodNode>(*this);

    result->update_cache();

    result->shadow_mode_ = shadow_mode_;
    result->radius_scale_ = radius_scale_;
    result->max_surfel_size_ = max_surfel_size_;
    result->error_threshold_ = error_threshold_;
    result->enable_backface_culling_by_normal_ = enable_backface_culling_by_normal_;

    return result;
}


void PLodNode::set_time_series_data_descriptions(std::vector<std::string> const& time_series_data_descriptions) {
    associated_time_series_data_descriptions_ = time_series_data_descriptions;
}

std::vector<std::string> PLodNode::get_time_series_data_descriptions() const{
    return associated_time_series_data_descriptions_;
}

} // namespace node
} // namespace gua
