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
#include "gua/node/SkeletalAnimationNode.hpp"

#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>
#include <gua/node/RayNode.hpp>
#include <gua/renderer/SkeletalAnimationLoader.hpp>
#include <gua/math/BoundingBoxAlgo.hpp>

// guacamole headers

namespace gua {
namespace node {

  ////////////////////////////////////////////////////////////////////////////////
  SkeletalAnimationNode::SkeletalAnimationNode(std::string const& name,
                           std::vector<std::string> const& geometry_descriptions,
                           std::vector<std::shared_ptr<Material>> const& materials,
                           std::shared_ptr<SkeletalAnimationDirector> animation_director,
                           math::mat4 const& transform)
    : GeometryNode(name, transform),
      geometries_(),
      geometry_descriptions_(geometry_descriptions),
      geometry_changed_(true),
      materials_(materials),
      material_changed_(true),
      animation_director_(animation_director),
      bone_transforms_block_{nullptr},
      first_run_{true}
  {
    geometries_.resize(geometry_descriptions_.size());
  }

  ////////////////////////////////////////////////////////////////////////////////
  Node* SkeletalAnimationNode::get(){
    Node* base = this;
    return base;
  }

  ////////////////////////////////////////////////////////////////////////////////
  std::vector<std::string> const& SkeletalAnimationNode::get_geometry_descriptions() const {
    return geometry_descriptions_;
  }

  ////////////////////////////////////////////////////////////////////////////////
  void SkeletalAnimationNode::set_geometry_description(std::string const& v,uint index) {
    if(index < geometry_descriptions_.size()){
      geometry_descriptions_[index] = v;
      geometry_changed_ = self_dirty_ = true;
    }
    else{
      Logger::LOG_WARNING << "Can't 'set_geometry_description()'! Index out of bounds! "<< std::endl;
    }
  }

  ////////////////////////////////////////////////////////////////////////////////
  std::shared_ptr<Material> SkeletalAnimationNode::get_material(uint index) const {
    if(index < materials_.size()){
      return materials_[index];
    }
    else{
      Logger::LOG_ERROR << "Cant return material of invalid index!" << std::endl;
      return materials_[0];
    }
  }

  ////////////////////////////////////////////////////////////////////////////////
  std::vector<std::shared_ptr<Material>>& SkeletalAnimationNode::get_materials() {
    return materials_;
  }

  ////////////////////////////////////////////////////////////////////////////////
  void SkeletalAnimationNode::set_material(std::shared_ptr<Material> material,uint index) {
    if(index < materials_.size()){
      materials_[index] = material;
      // material_changed_ = self_dirty_ = true;
    }
    else{
      Logger::LOG_WARNING << "Cant set material of invalid index!"<< std::endl;
    }
  }
  
  ////////////////////////////////////////////////////////////////////////////////
  std::string const& SkeletalAnimationNode::get_animation_1() const {
    if(animation_director_->has_anims_ && animation_director_->anim_num_1_< animation_director_->animations_.size()){
      return animation_director_->animations_[animation_director_->anim_num_1_].get_name();
    }
    else{
      return animation_director_->none_loaded;
    }
  }

  void SkeletalAnimationNode::set_animation_1(std::string const& animation_name) {
    if(animation_director_->animation_mapping_.find(animation_name) != animation_director_->animation_mapping_.end()) {
      animation_director_->anim_num_2_ = animation_director_->anim_num_1_;
      animation_director_->anim_num_1_ = animation_director_->animation_mapping_.at(animation_name);
    }
    else {
      gua::Logger::LOG_WARNING << "No matching animation with name: " << animation_name <<" found!" << std::endl;
    }
  }
  std::string const& SkeletalAnimationNode::get_animation_2() const {
    if(animation_director_->has_anims_ && animation_director_->anim_num_2_< animation_director_->animations_.size()){
      return animation_director_->animations_[animation_director_->anim_num_2_].get_name();
    }
    else{
      return SkeletalAnimationDirector::none_loaded;
    }
  }

  void SkeletalAnimationNode::set_animation_2(std::string const& animation_name) {
    if(animation_director_->animation_mapping_.find(animation_name) != animation_director_->animation_mapping_.end()) {
      animation_director_->anim_num_2_ = animation_director_->animation_mapping_.at(animation_name);
    }
    else {
      gua::Logger::LOG_WARNING << "No matching animation with name: " << animation_name <<" found!" << std::endl;
    }
  }

  float SkeletalAnimationNode::get_duration(std::string const& animation_name) const {
    if(animation_director_->animation_mapping_.find(animation_name) != animation_director_->animation_mapping_.end()) {
      return animation_director_->animations_.at(animation_director_->animation_mapping_.at(animation_name)).get_duration();
    }
    else {
      return 0;
      gua::Logger::LOG_WARNING << "No matching animation with name: " << animation_name <<" found!" << std::endl;
    }
  }

  float SkeletalAnimationNode::get_blending_factor() const{
    return animation_director_->blend_factor_;
  }

  void SkeletalAnimationNode::set_blending_factor(float f){
    animation_director_->blend_factor_ = f;
  }

  void SkeletalAnimationNode::set_time_1(float time){
    animation_director_->anim_time_1_ = time;
  }

  float SkeletalAnimationNode::get_time_1() const{
    return animation_director_->anim_time_1_;
  }

  void SkeletalAnimationNode::set_time_2(float time){
    animation_director_->anim_time_2_ = time;
  }

  float SkeletalAnimationNode::get_time_2() const{
    return animation_director_->anim_time_2_;
  }

  ////////////////////////////////////////////////////////////////////////////////
  void SkeletalAnimationNode::update_bone_transforms(RenderContext const& ctx) {
    if(!animation_director_->has_anims() && !first_run_) return;
    if(!animation_director_->has_anims()) first_run_ = false;

    if(!bone_transforms_block_) {
      //TODO one transform block per context
      bone_transforms_block_ = std::make_shared<BoneTransformUniformBlock>(ctx.render_device);
    }

    bone_transforms_block_->update(ctx.render_context, animation_director_->get_bone_transforms());
    ctx.render_context->bind_uniform_buffer( bone_transforms_block_->block().block_buffer(), 1 );
  }

  ////////////////////////////////////////////////////////////////////////////////
  void SkeletalAnimationNode::ray_test_impl(Ray const& ray, int options,
    Mask const& mask, std::set<PickResult>& hits) {

    //TODO

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

      auto geometry(GeometryDatabase::instance()->lookup(get_geometry_description()));

      if (geometry) {

        bool check_kd_tree(true);

        math::mat4 world_transform(get_world_transform());

        // check for bounding box intersection of contained geometry if node
        // has children (in this case, the bbox might be larger
        // than the actual geometry)
        if (has_children()) {
          auto geometry_bbox(geometry->get_bounding_box());

#if 0
          auto inner_bbox = gua::math::transform(geometry_bbox, world_transform);
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
                auto transformed(world_transform * math::vec4(hit.position.x, hit.position.y, hit.position.z, 0.0));
                hit.world_position = scm::math::vec3(transformed.x, transformed.y, transformed.z);
              }
            }
          }

          if (options & PickResult::GET_WORLD_NORMALS) {

            math::mat4 normal_matrix(scm::math::inverse(scm::math::transpose(world_transform)));
            for (auto& hit : hits) {
              if (hit.world_normal == math::vec3(inf, inf, inf)) {
                auto transformed(normal_matrix * math::vec4(hit.normal.x, hit.normal.y, hit.normal.z, 0.0));
                hit.world_normal = scm::math::normalize(scm::math::vec3(transformed.x, transformed.y, transformed.z));
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

  void SkeletalAnimationNode::update_cache() {

    // The code below auto-loads a geometry if it's not already supported by
    // the GeometryDatabase. Name is generated by GeometryDescription

    if (geometry_changed_)
    {
      for(uint i(0);i<geometry_descriptions_.size();++i){

        if (geometry_descriptions_[i] != "")
        {
          if (!GeometryDatabase::instance()->contains(geometry_descriptions_[i]))
          {
            GeometryDescription desc(geometry_descriptions_[i]);
            try {
              gua::SkeletalAnimationLoader loader;
              loader.create_geometry_from_file(get_name(), desc.filepath(), materials_[i], desc.flags());
            }
            catch ( std::exception& e ) {
              Logger::LOG_WARNING << "SkeletalAnimationNode::update_cache(): Loading failed from " << desc.filepath() << " : " << e.what() << std::endl;
            }
          }

          geometries_[i] = std::dynamic_pointer_cast<SkinnedMeshResource>(GeometryDatabase::instance()->lookup(geometry_descriptions_[i]));

          if (!geometries_[i]) {
            Logger::LOG_WARNING << "Failed to get SkinnedMeshResource for " << geometry_descriptions_[i] << ": The data base entry is of wrong type!" << std::endl;
          }
        }

      }

      geometry_changed_ = false;
    }

    // The code below auto-loads a material if it's not already supported by
    // the MaterialShaderDatabase. It expects a material name like
    //
    // data/materials/Stones.gmd

    /*if (material_changed_)
    {
      if (material_.get_shader_name() != "")
      {
        // if (!MaterialShaderDatabase::instance()->contains(material_))
        // {
        //   MaterialShaderDatabase::instance()->load_material(material_);
        // }
      }

      material_changed_ = false;
    }*/ // is not doing anything???????

    GeometryNode::update_cache();
  }

  ////////////////////////////////////////////////////////////////////////////////
  std::vector<std::shared_ptr<SkinnedMeshResource>> const& SkeletalAnimationNode::get_geometries() const {
    return geometries_;
  }

  ////////////////////////////////////////////////////////////////////////////////
  std::shared_ptr<SkeletalAnimationDirector> const& SkeletalAnimationNode::get_director() const {
    return animation_director_;
  }

  ////////////////////////////////////////////////////////////////////////////////
  /* virtual */ void SkeletalAnimationNode::accept(NodeVisitor& visitor) {
    visitor.visit(this);
  }


  ////////////////////////////////////////////////////////////////////////////////
  std::vector<math::BoundingBox<math::vec3>>
  SkeletalAnimationNode::get_bone_boxes(){

    auto tmp_boxes = std::vector<math::BoundingBox<math::vec3>>(100,math::BoundingBox<math::vec3>());

    // for(uint i(0);i<geometries_.size();++i){
    //   auto bone_boxes = geometries_[i]->get_bone_boxes();
    //   for(uint b(0);b<bone_boxes.size();++b){
    //     if(!bone_boxes[b].isEmpty()){
    //       bone_boxes[b] = transform(bone_boxes[b], world_transform_);
    //       tmp_boxes[b].expandBy(bone_boxes[b]);
    //     }
    //   }
    // }

    return tmp_boxes;

  }

  ////////////////////////////////////////////////////////////////////////////////
  void SkeletalAnimationNode::update_bounding_box() const {

    if (geometries_.size()>0) {

      auto geometry_bbox = math::BoundingBox<math::vec3>();

      // for(uint i(0);i<geometries_.size();++i){
      //   auto bone_boxes = geometries_[i]->get_bone_boxes();
      //   for(uint b(0);b<bone_boxes.size();++b){
      //     if(!bone_boxes[b].isEmpty()){
      //       bone_boxes[b] = transform(bone_boxes[b],world_transform_);
      //       geometry_bbox.expandBy(bone_boxes[b]);
      //     }
      //   }
      // }

      // if(!geometry_bbox.isEmpty()){
      //   bounding_box_ = geometry_bbox;
      // }
      // else{//bbox out of bone boxes could not be computed yet....use initial bbox
        for(uint i(0);i<geometries_.size();++i){
          auto tmp_bbox = geometries_[i]->get_bounding_box();
          if(!tmp_bbox.isEmpty()){
            geometry_bbox.expandBy(tmp_bbox);
          }
        }
        bounding_box_ = transform(geometry_bbox, world_transform_);
      // }


      for (auto child : get_children()) {
        bounding_box_.expandBy(child->get_bounding_box());
      }
    } else {
      Node::update_bounding_box();
    }
  }

  ////////////////////////////////////////////////////////////////////////////////
  std::shared_ptr<Node> SkeletalAnimationNode::copy() const {
    std::shared_ptr<SkeletalAnimationNode> result = std::make_shared<SkeletalAnimationNode>(get_name(), geometry_descriptions_, materials_, get_director(),get_transform());
    result->shadow_mode_ = shadow_mode_;
    result->geometries_ = geometries_;
    return result;
  }
}
}
