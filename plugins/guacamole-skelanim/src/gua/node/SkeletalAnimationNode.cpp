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
#include "gua/skelanim/node/SkeletalAnimationNode.hpp"

// guacamole headers
#include <gua/skelanim/utils/Bone.hpp>
#include <gua/skelanim/utils/SkeletalAnimation.hpp>
#include <gua/skelanim/utils/BoneAnimation.hpp>
#include <gua/skelanim/utils/SkeletalTransformation.hpp>
#include <gua/skelanim/renderer/SkeletalAnimationLoader.hpp>
#include <gua/skelanim/renderer/SkinnedMeshResource.hpp>
// #include <gua/node/RayNode.hpp>
#include <gua/math/BoundingBoxAlgo.hpp>
#include <gua/databases/GeometryDatabase.hpp>

namespace gua
{
namespace node
{
////////////////////////////////////////////////////////////////////////////////
SkeletalAnimationNode::SkeletalAnimationNode(
    std::string const& name, std::vector<std::string> const& geometry_descriptions, std::vector<std::shared_ptr<Material>> const& materials, Skeleton const& skeleton, math::mat4 const& transform)
    : GeometryNode(name, transform), geometries_(), geometry_descriptions_(geometry_descriptions), geometry_changed_(true), materials_(materials), render_to_gbuffer_(true),
      render_to_stencil_buffer_(false), skeleton_(skeleton), new_bones_{true}, has_anims_{false}, blend_factor_{1.0},
      anim_1_(SkeletalAnimationNode::none_loaded), // { } not allowed on msvc because of implicit conversion to initializer list
      anim_2_(SkeletalAnimationNode::none_loaded)
{
    geometries_.resize(geometry_descriptions_.size());
}

////////////////////////////////////////////////////////////////////////////////
Node* SkeletalAnimationNode::get()
{
    Node* base = this;
    return base;
}

////////////////////////////////////////////////////////////////////////////////
std::vector<std::string> const& SkeletalAnimationNode::get_geometry_descriptions() const { return geometry_descriptions_; }
////////////////////////////////////////////////////////////////////////////////
void SkeletalAnimationNode::set_geometry_descriptions(std::vector<std::string> const& v)
{
    geometry_descriptions_ = v;
    geometry_changed_ = self_dirty_ = true;
}

////////////////////////////////////////////////////////////////////////////////
void SkeletalAnimationNode::set_skeleton_description(std::string const& description)
{
    gua::SkeletalAnimationLoader loader;
    skeleton_ = loader.load_skeleton(description);
    new_bones_ = true;
}

////////////////////////////////////////////////////////////////////////////////
void SkeletalAnimationNode::add_material(std::shared_ptr<Material> const& material)
{
    // if (geometries_.size() > materials_.size()) {
    materials_.push_back(material);
    // } else {
    //   Logger::LOG_WARNING << "Cant have more materials than geometries"
    //                     << std::endl;
    // }
}

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<Material> const& SkeletalAnimationNode::get_material(unsigned index) const
{
    if(index < materials_.size())
    {
        return materials_[index];
    }
    else
    {
        Logger::LOG_ERROR << "Cant return material of invalid index!" << std::endl;
        return materials_[0];
    }
}

////////////////////////////////////////////////////////////////////////////////
std::vector<std::shared_ptr<Material>> const& SkeletalAnimationNode::get_materials() const { return materials_; }

////////////////////////////////////////////////////////////////////////////////
void SkeletalAnimationNode::set_material(std::shared_ptr<Material> material, unsigned index)
{
    if(index < materials_.size())
    {
        materials_[index] = material;
    }
    else
    {
        Logger::LOG_ERROR << "Cant set material of invalid index!" << std::endl;
    }
}

////////////////////////////////////////////////////////////////////////////////
void SkeletalAnimationNode::add_animations(std::string const& file_name, std::string const& name)
{
    std::vector<SkeletalAnimation> anims{SkeletalAnimationLoader{}.load_animation(file_name, name)};

    for(unsigned i = 0; i < anims.size(); ++i)
    {
        animations_.insert(std::make_pair(anims[i].get_name(), anims[i]));
    }

    has_anims_ = animations_.size() > 0;
}

////////////////////////////////////////////////////////////////////////////////
void SkeletalAnimationNode::update_bone_transforms()
{
    if(!has_anims_ && !new_bones_)
        return;
    // no animation and new bones -> generate transforms once
    if(!has_anims_)
    {
        new_bones_ = false;
        bone_transforms_ = SkeletalTransformation::from_hierarchy(skeleton_, 0);
    }
    // use first anim
    else if(blend_factor_ <= 0.0)
    {
        if(anim_1_ != SkeletalAnimationNode::none_loaded)
        {
            bone_transforms_ = SkeletalTransformation::from_anim(skeleton_, 0, anim_time_1_, animations_.at(anim_1_));
        }
        else
        {
            bone_transforms_ = SkeletalTransformation::from_hierarchy(skeleton_, 0);
        }
    }
    // use second anim
    else if(blend_factor_ >= 1.0)
    {
        if(anim_2_ != SkeletalAnimationNode::none_loaded)
        {
            SkeletalTransformation::from_anim(skeleton_, 0, anim_time_2_, animations_.at(anim_2_));
        }
        else
        {
            bone_transforms_ = SkeletalTransformation::from_hierarchy(skeleton_, 0);
        }
    }
    // use both anims
    else
    {
        bone_transforms_ = SkeletalTransformation::blend_anims(skeleton_, 0, blend_factor_, anim_time_1_, anim_time_2_, animations_.at(anim_1_), animations_.at(anim_2_));
    }
}
////////////////////////////////////////////////////////////////////////////////
bool SkeletalAnimationNode::get_render_to_gbuffer() const { return render_to_gbuffer_; }

////////////////////////////////////////////////////////////////////////////////
void SkeletalAnimationNode::set_render_to_gbuffer(bool enable) { render_to_gbuffer_ = enable; }

////////////////////////////////////////////////////////////////////////////////
bool SkeletalAnimationNode::get_render_to_stencil_buffer() const { return render_to_stencil_buffer_; }

////////////////////////////////////////////////////////////////////////////////
void SkeletalAnimationNode::set_render_to_stencil_buffer(bool enable) { render_to_stencil_buffer_ = enable; }

////////////////////////////////////////////////////////////////////////////////
void SkeletalAnimationNode::set_bones(std::vector<Bone> const& bones)
{
    skeleton_.set_bones(bones);
    new_bones_ = true;
}

////////////////////////////////////////////////////////////////////////////////
std::vector<gua::Bone> const& SkeletalAnimationNode::get_bones() const { return skeleton_.get_bones(); }

////////////////////////////////////////////////////////////////////////////////
void SkeletalAnimationNode::ray_test_impl(Ray const& ray, int options, Mask const& mask, std::set<PickResult>& hits)
{
    // TODO

    /*// first of all, check bbox
      auto box_hits(::gua::intersect(ray, bounding_box_));

      // ray did not intersect bbox -- therefore it wont intersect
      if (box_hits.first == Ray::END && box_hits.second == Ray::END) {
        return;
      }


      // return if only first object shall be returned and the current first hit
      // is in front of the bbox entry point and the ray does not start inside
      // the bbox
      if (options & PickResult::PICK_ONLY_FIRST_OBJECT
        && hits.size() > 0 && hits.begin()->distance < box_hits.first
        && box_hits.first != Ray::END) {

        return;
      }

      // bbox is intersected, but check geometry only if mask tells us to check
      if (get_geometry_description() != "" && mask.check(get_tags())) {

        auto
  geometry(GeometryDatabase::instance()->lookup(get_geometry_description()));

        if (geometry) {

          bool check_kd_tree(true);

          math::mat4 world_transform(get_world_transform());

          // check for bounding box intersection of contained geometry if node
          // has children (in this case, the bbox might be larger
          // than the actual geometry)
          if (has_children()) {
            auto geometry_bbox(geometry->get_bounding_box());

  #if 0
            auto inner_bbox = gua::math::transform(geometry_bbox,
  world_transform);
  #else
            math::BoundingBox<math::vec3> inner_bbox;
            inner_bbox.expandBy(world_transform * geometry_bbox.min);
            inner_bbox.expandBy(world_transform * geometry_bbox.max);
            inner_bbox.expandBy(world_transform *
              math::vec3(geometry_bbox.min.x,
              geometry_bbox.min.y,
              geometry_bbox.max.z));
            inner_bbox.expandBy(world_transform *
              math::vec3(geometry_bbox.min.x,
              geometry_bbox.max.y,
              geometry_bbox.min.z));
            inner_bbox.expandBy(world_transform *
              math::vec3(geometry_bbox.min.x,
              geometry_bbox.max.y,
              geometry_bbox.max.z));
            inner_bbox.expandBy(world_transform *
              math::vec3(geometry_bbox.max.x,
              geometry_bbox.min.y,
              geometry_bbox.max.z));
            inner_bbox.expandBy(world_transform *
              math::vec3(geometry_bbox.max.x,
              geometry_bbox.max.y,
              geometry_bbox.min.z));
            inner_bbox.expandBy(world_transform *
              math::vec3(geometry_bbox.max.x,
              geometry_bbox.min.y,
              geometry_bbox.min.z));
  #endif

            auto inner_hits(::gua::intersect(ray, inner_bbox));
            if (inner_hits.first == RayNode::END &&
              inner_hits.second == RayNode::END)
              check_kd_tree = false;
          }

          if (check_kd_tree) {
            Ray world_ray(ray);

            math::mat4 ori_transform(scm::math::inverse(world_transform));

            math::vec4 ori(world_ray.origin_[0],
              world_ray.origin_[1],
              world_ray.origin_[2],
              1.0);
            math::vec4 dir(world_ray.direction_[0],
              world_ray.direction_[1],
              world_ray.direction_[2],
              0.0);

            ori = ori_transform * ori;
            dir = ori_transform * dir;

            Ray object_ray(ori, dir, world_ray.t_max_);
            geometry->ray_test(object_ray, options, this, hits);

            float const inf(std::numeric_limits<float>::max());

            if (options & PickResult::GET_WORLD_POSITIONS) {

              for (auto& hit : hits) {
                if (hit.world_position == math::vec3(inf, inf, inf)) {
                  auto transformed(world_transform * math::vec4(hit.position.x,
  hit.position.y, hit.position.z, 0.0));
                  hit.world_position = scm::math::vec3(transformed.x,
  transformed.y, transformed.z);
                }
              }
            }

            if (options & PickResult::GET_WORLD_NORMALS) {

              math::mat4
  normal_matrix(scm::math::inverse(scm::math::transpose(world_transform)));
              for (auto& hit : hits) {
                if (hit.world_normal == math::vec3(inf, inf, inf)) {
                  auto transformed(normal_matrix * math::vec4(hit.normal.x,
  hit.normal.y, hit.normal.z, 0.0));
                  hit.world_normal =
  scm::math::normalize(scm::math::vec3(transformed.x, transformed.y,
  transformed.z));
                }
              }
            }
          }
        }
      }

      for (auto child : get_children()) {
        // test for intersection with each child
        child->ray_test_impl(ray, options, mask, hits);
      }*/
}

////////////////////////////////////////////////////////////////////////////////

void SkeletalAnimationNode::update_cache()
{
    // The code below auto-loads a geometry if it's not already supported by
    // the GeometryDatabase. Name is generated by GeometryDescription

    if(geometry_changed_)
    {
        geometries_.resize(geometry_descriptions_.size());

        for(unsigned i(0); i < geometry_descriptions_.size(); ++i)
        {
            if(geometry_descriptions_[i] != "")
            {
                if(!GeometryDatabase::instance()->contains(geometry_descriptions_[i]))
                {
                    GeometryDescription desc(geometry_descriptions_[i]);
                    try
                    {
                        gua::SkeletalAnimationLoader loader;
                        loader.create_geometry_from_file(get_name(), desc.filepath(), materials_[i], desc.flags());
                    }
                    catch(std::exception& e)
                    {
                        Logger::LOG_ERROR << "SkeletalAnimationNode::update_cache(): Loading failed from " << desc.filepath() << " : " << e.what() << std::endl;
                    }
                }

                geometries_[i] = std::dynamic_pointer_cast<SkinnedMeshResource>(GeometryDatabase::instance()->lookup(geometry_descriptions_[i]));

                if(!geometries_[i])
                {
                    Logger::LOG_ERROR << "Failed to get SkinnedMeshResource for " << geometry_descriptions_[i] << ": The data base entry is of wrong type!" << std::endl;
                }
            }
        }

        geometry_changed_ = false;
    }

    GeometryNode::update_cache();
}

////////////////////////////////////////////////////////////////////////////////
std::vector<std::shared_ptr<SkinnedMeshResource>> const& SkeletalAnimationNode::get_geometries() const { return geometries_; }

////////////////////////////////////////////////////////////////////////////////
/* virtual */ void SkeletalAnimationNode::accept(NodeVisitor& visitor) { visitor.visit(this); }

////////////////////////////////////////////////////////////////////////////////
std::vector<math::BoundingBox<math::vec3>> SkeletalAnimationNode::get_bone_boxes()
{
    auto tmp_boxes = std::vector<math::BoundingBox<math::vec3>>(100, math::BoundingBox<math::vec3>());

    for(unsigned i(0); i < geometries_.size(); ++i)
    {
        auto bone_boxes = geometries_[i]->get_bone_boxes(bone_transforms_);
        for(unsigned b(0); b < bone_boxes.size(); ++b)
        {
            if(!bone_boxes[b].isEmpty())
            {
                bone_boxes[b] = transform(bone_boxes[b], world_transform_);
                tmp_boxes[b].expandBy(bone_boxes[b]);
            }
        }
    }

    return tmp_boxes;
}

////////////////////////////////////////////////////////////////////////////////
void SkeletalAnimationNode::update_bounding_box() const
{
    if(geometries_.size() > 0)
    {
        auto geometry_bbox = math::BoundingBox<math::vec3>();

        for(unsigned i(0); i < geometries_.size(); ++i)
        {
            auto bone_boxes = geometries_[i]->get_bone_boxes(bone_transforms_);
            for(unsigned b(0); b < bone_boxes.size(); ++b)
            {
                if(!bone_boxes[b].isEmpty())
                {
                    bone_boxes[b] = transform(bone_boxes[b], world_transform_);
                    geometry_bbox.expandBy(bone_boxes[b]);
                }
            }
        }

        if(!geometry_bbox.isEmpty())
        {
            bounding_box_ = geometry_bbox;
        }
        else
        { // bbox out of bone boxes could not be computed yet....use initial
          // bbox
            for(unsigned i(0); i < geometries_.size(); ++i)
            {
                auto tmp_bbox = geometries_[i]->get_bounding_box();
                if(!tmp_bbox.isEmpty())
                {
                    geometry_bbox.expandBy(tmp_bbox);
                }
            }
            bounding_box_ = transform(geometry_bbox, world_transform_);
        }

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
std::shared_ptr<Node> SkeletalAnimationNode::copy() const
{
    auto result = std::make_shared<SkeletalAnimationNode>(*this);

    result->update_cache();

    result->geometries_ = geometries_;
    result->geometry_descriptions_ = geometry_descriptions_;
    result->geometry_changed_ = geometry_changed_;
    result->skeleton_ = skeleton_;
    result->animations_ = animations_;
    result->new_bones_ = new_bones_;
    result->has_anims_ = has_anims_;
    result->anim_1_ = anim_1_;
    result->anim_2_ = anim_2_;
    result->blend_factor_ = blend_factor_;
    result->anim_time_1_ = anim_time_1_;
    result->anim_time_2_ = anim_time_2_;
    result->bone_transforms_ = bone_transforms_;

    return result;
}

////////////////////////////////////////////////////////////////////////////////
std::string const& SkeletalAnimationNode::get_animation_1() const
{
    if(has_anims_ && animations_.find(anim_1_) != animations_.end())
    {
        return anim_1_;
    }
    else
    {
        return none_loaded;
    }
}

void SkeletalAnimationNode::set_animation_1(std::string const& animation_name)
{
    // if (animation_name == none_loaded || animations_.find(animation_name) != animations_.end()) {
    // }
    anim_1_ = animation_name;
    //  else {
    //   gua::Logger::LOG_ERROR << "No matching animation with name: '"
    //                            << animation_name << "' found!" << std::endl;
    // }
}
std::string const& SkeletalAnimationNode::get_animation_2() const
{
    if(has_anims_ && animations_.find(anim_2_) != animations_.end())
    {
        return anim_2_;
    }
    else
    {
        return none_loaded;
    }
}

void SkeletalAnimationNode::set_animation_2(std::string const& animation_name)
{
    // if (animation_name == none_loaded || animations_.find(animation_name) != animations_.end()) {
    // }
    anim_2_ = animation_name;
    //  else {
    //   gua::Logger::LOG_ERROR << "No matching animation with name: '"
    //                            << animation_name << "' found!" << std::endl;
    // }
}

float SkeletalAnimationNode::get_duration(std::string const& animation_name) const
{
    if(animations_.find(animation_name) != animations_.end())
    {
        return animations_.at(animation_name).get_duration();
    }
    else
    {
        gua::Logger::LOG_ERROR << "No matching animation with name: '" << animation_name << "' found!" << std::endl;
        return 0;
    }
}

float SkeletalAnimationNode::get_blend_factor() const { return blend_factor_; }

void SkeletalAnimationNode::set_blend_factor(float f) { blend_factor_ = f; }

void SkeletalAnimationNode::set_time_1(float time) { anim_time_1_ = time; }

float SkeletalAnimationNode::get_time_1() const { return anim_time_1_; }

void SkeletalAnimationNode::set_time_2(float time) { anim_time_2_ = time; }

float SkeletalAnimationNode::get_time_2() const { return anim_time_2_; }

bool SkeletalAnimationNode::has_anims() const { return has_anims_; }

std::vector<scm::math::mat4f> const& SkeletalAnimationNode::get_bone_transforms() const { return bone_transforms_; }

const std::string SkeletalAnimationNode::none_loaded = std::string("none");
;
} // namespace node
} // namespace gua
