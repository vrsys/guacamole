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
#include <gua/renderer/SkeletalAnimationRessource.hpp>
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
      animation_director_(animation_director)
  {
    geometries_.resize(geometry_descriptions_.size());
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
  Material const& SkeletalAnimationNode::get_material(uint index) const {
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
  void SkeletalAnimationNode::set_material(Material const& material,uint index) {
    if(index < materials_.size()){
      materials_[index] = material;
      material_changed_ = self_dirty_ = true;
    }
    else{
      Logger::LOG_WARNING << "Cant set material of invalid index!"<< std::endl;
    }
  }


  ////////////////////////////////////////////////////////////////////////////////
  void SkeletalAnimationNode::set_fallback_materials(Material const& material) {
    for(auto & mat: materials_){
      if(mat.get_shader_name() == ""){
        mat = material;
        material_changed_= self_dirty_ = true;
      }
    }
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

          geometries_[i] = std::dynamic_pointer_cast<SkeletalAnimationRessource>(GeometryDatabase::instance()->lookup(geometry_descriptions_[i]));

          if (!geometries_[i]) {
            Logger::LOG_WARNING << "Failed to get SkeletalAnimationRessource for " << geometry_descriptions_[i] << ": The data base entry is of wrong type!" << std::endl;
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
  std::vector<std::shared_ptr<SkeletalAnimationRessource>> const& SkeletalAnimationNode::get_geometries() const {
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
  void SkeletalAnimationNode::update_bounding_box() const {

    if (geometries_.size()>0) {

      auto geometry_bbox(geometries_[0]->get_bounding_box());
      for(uint i(1);i<geometries_.size();++i){
        geometry_bbox.expandBy(geometries_[i]->get_bounding_box());
      }

      bounding_box_ = transform(geometry_bbox, world_transform_);


      for (auto child : get_children()) {
        bounding_box_.expandBy(child->get_bounding_box());
      }
    } else {
      Node::update_bounding_box();
    }
  }

  ////////////////////////////////////////////////////////////////////////////////
  std::shared_ptr<Node> SkeletalAnimationNode::copy() const {
    std::shared_ptr<SkeletalAnimationNode> result(new SkeletalAnimationNode(get_name(), geometry_descriptions_, materials_, get_director(),get_transform()));
    result->shadow_mode_ = shadow_mode_;
    result->geometries_ = geometries_;
    return result;
  }
}
}
