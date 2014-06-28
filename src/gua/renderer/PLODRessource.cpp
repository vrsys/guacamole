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
#include <gua/renderer/PLODRessource.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/utils/Logger.hpp>

// external headers

#include <iostream>


namespace gua {

////////////////////////////////////////////////////////////////////////////////

PLODRessource::PLODRessource()
    : upload_mutex_() {}

////////////////////////////////////////////////////////////////////////////////

PLODRessource::PLODRessource(const pbr::ren::LodPointCloud* point_cloud)
    : upload_mutex_() {


     //set already created BB
  	 scm::gl::boxf loaded_bb = point_cloud->kdn_tree()->bounding_boxes()[0];
  	 bounding_box_.min = loaded_bb.min_vertex();
 	   bounding_box_.max = loaded_bb.max_vertex();
}

////////////////////////////////////////////////////////////////////////////////

void PLODRessource::upload_to(RenderContext const& ctx) const {
}

////////////////////////////////////////////////////////////////////////////////
void PLODRessource::draw(RenderContext const& ctx) const
{/*dummy*/}
////////////////////////////////////////////////////////////////////////////////

void PLODRessource::draw(RenderContext const& ctx, pbr::context_t context_id, pbr::view_t view_id, pbr::model_t model_id, scm::gl::vertex_array_ptr const& vertex_array, std::vector<unsigned int> const& culling_results) const 
{

    pbr::ren::ModelDatabase* database = pbr::ren::ModelDatabase::GetInstance();
    pbr::ren::CutDatabase* cuts = pbr::ren::CutDatabase::GetInstance();
    
    pbr::ren::Cut& cut = cuts->GetCut(context_id, view_id, model_id);
    std::vector<pbr::ren::Cut::NodeSlotAggregate>& node_list = cut.complete_set();
    pbr::ren::KdnTree const *  kdn_tree = database->GetModel(model_id)->kdn_tree();

    uint32_t surfels_per_node = database->surfels_per_node();
    uint32_t surfels_per_node_of_model = kdn_tree->surfels_per_node();

    scm::gl::context_vertex_input_guard vig(ctx.render_context);

    ctx.render_context->bind_vertex_array(vertex_array);

    ctx.render_context->apply();
  
    pbr::node_t node_counter = 0;
  
    for(std::vector<pbr::ren::Cut::NodeSlotAggregate>::const_iterator k = node_list.begin(); k != node_list.end(); ++k, ++node_counter)
    {
        //0 = completely inside of frustum, 1 = completely outside of frustum, 2 = intersects frustum
        if(culling_results[node_counter] != 1)
        {
        

          ctx.render_context->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, (k->slot_id_) * surfels_per_node, surfels_per_node_of_model);
          
        }
    }
  
}

////////////////////////////////////////////////////////////////////////////////

void PLODRessource::ray_test(Ray const& ray, PickResult::Options options,
                    node::Node* owner, std::set<PickResult>& hits) {

 // scm::gl::boxf bb = pbr::ren::ModelDatabase::GetInstance()->GetModel(0)->kdn_tree()->bounding_boxes()[0];

//  auto k( intersection(ray, bb) );

  auto root_hit(ray.intersection(bounding_box_) );

  //std::cout << "origin of k: " << k.origin_ << "   & origin of lampidam: " << lampidam.origin_ << "\n";




  if(root_hit.t_max_ <= 0.0f)
  {
   //  std::cout << "NO INTERSECTION!!!!!!!!!!!!!!\n";  
  }
  else
  {
     hits.insert(PickResult(30.123, owner, math::vec3(99,87,66), math::vec3(3,4,5), math::vec3(0.0,0.0,1.0), math::vec3(0.0,0.0,2.0), math::vec2(0.5, 0.3) )  );
     //std::cout << "HIT!!!!\n";
  }

  return;
}


////////////////////////////////////////////////////////////////////////////////

/*virtual*/ std::shared_ptr<GeometryUberShader> PLODRessource::create_ubershader() const {
  return std::make_shared<PLODUberShader>();
}

/////////////////////////////////////////////////////////////////////////////////


//free functions to also support scm::gl::boxf of the pbr::kdn::tree as intersection object
std::pair<float, float> intersect(Ray const& ray,
    scm::gl::boxf const& box) {

  math::vec3 t1((box.min_vertex() - ray.origin_) / ray.direction_);
  math::vec3 t2((box.max_vertex() - ray.origin_) / ray.direction_);

  math::vec3 tmin1(
      std::min(t1[0], t2[0]), std::min(t1[1], t2[1]), std::min(t1[2], t2[2]));
  math::vec3 tmax1(
      std::max(t1[0], t2[0]), std::max(t1[1], t2[1]), std::max(t1[2], t2[2]));

  float tmin = std::max(std::max(tmin1[0], tmin1[1]), tmin1[2]);
  float tmax = std::min(std::min(tmax1[0], tmax1[1]), tmax1[2]);

  if (tmax >= tmin) {
    // there are two intersections
    if (tmin > 0.0 && tmax < ray.t_max_)
      return std::make_pair(tmin, tmax);

    // there is only one intersection, the ray ends inside the box
    else if (tmin > 0.0)
      return std::make_pair(tmin, Ray::END);

    // there is only one intersection, the ray starts inside the box
    else
      return std::make_pair(Ray::END, tmax);
  }

  // there is no intersection

  return std::make_pair(Ray::END, Ray::END);
}
///////////////////////////////////////////////////////////////////////////////

Ray const intersection(Ray const& ray, scm::gl::boxf const& box /*std::pair<float, float> const& hits*/)
{
  std::pair<float, float> const& hits( intersect(ray, box ) );
  // there are to hits -> clamp ray on both sides
  if (hits.first != gua::Ray::END && hits.first != gua::Ray::END)
    return Ray(ray.origin_ + ray.direction_ * hits.first,
               ray.direction_,
               hits.second - hits.first);

  // the ray ends inside the box -> clamp the origin
  if (hits.first != gua::Ray::END)
    return Ray(
        ray.origin_ + ray.direction_ * hits.first, ray.direction_, ray.t_max_ - hits.first);

  // the ray starts inside the box -> clamp the end
  if (hits.second != gua::Ray::END)
    return Ray(ray.origin_, ray.direction_, hits.second);

  // there is no intersection
  return Ray();
}


}
