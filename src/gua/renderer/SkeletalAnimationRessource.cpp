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
#include <gua/renderer/SkeletalAnimationRessource.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/node/SkeletalAnimationNode.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/math/BoundingBoxAlgo.hpp>


// external headers
#include <assimp/postprocess.h>
//#include <assimp/scene.h>

namespace gua {

SkeletalAnimationRessource::SkeletalAnimationRessource()
    : vertices_(), indices_(), vertex_array_(), upload_mutex_(), mesh_(){}

////////////////////////////////////////////////////////////////////////////////

SkeletalAnimationRessource::SkeletalAnimationRessource(Mesh const& mesh, std::shared_ptr<SkeletalAnimationDirector> animation_director, bool build_kd_tree)
: vertices_(),
  indices_(),
  vertex_array_(),
  upload_mutex_(),
  mesh_(mesh),
  animation_director_(animation_director)
{

  //TODO generate BBox and KDTree
  //if (mesh_->HasPositions()) {
  bounding_box_ = math::BoundingBox<math::vec3>();

  // without bone influence
  for (unsigned v(0); v < mesh_.num_vertices(); ++v) {
    bounding_box_.expandBy(scm::math::vec3(
        mesh_.get_position(v).x, mesh_.get_position(v).y, mesh_.get_position(v).z));
  }

  bone_boxes_ = std::vector<math::BoundingBox<math::vec3>>(100,math::BoundingBox<math::vec3>());

    // TODO
    /*if (build_kd_tree) {
      kd_tree_.generate(mesh_);
    }
  //}*/
}
SkeletalAnimationRessource::SkeletalAnimationRessource(Mesh const& mesh, std::shared_ptr<SkeletalAnimationDirector> animation_director, bool build_kd_tree)
: vertices_(),
  indices_(),
  vertex_array_(),
  upload_mutex_(),
  mesh_(mesh),
  animation_director_(animation_director)
{

  //TODO generate BBox and KDTree
  //if (mesh_->HasPositions()) {
  bounding_box_ = math::BoundingBox<math::vec3>();

  // without bone influence
  for (unsigned v(0); v < mesh_.num_vertices(); ++v) {
    bounding_box_.expandBy(scm::math::vec3(
        mesh_.get_position(v).x, mesh_.get_position(v).y, mesh_.get_position(v).z));
  }
  std::cout << "box dims" << bounding_box_.corners().first << " and " << bounding_box_.corners().second << std::endl;

  bone_boxes_ = std::vector<math::BoundingBox<math::vec3>>(100,math::BoundingBox<math::vec3>());

    // TODO
    /*if (build_kd_tree) {
      kd_tree_.generate(mesh_);
    }
  //}*/
}

////////////////////////////////////////////////////////////////////////////////

void SkeletalAnimationRessource::upload_to(RenderContext const& ctx) /*const*/{

  if (vertices_.size() <= ctx.id || vertices_[ctx.id] == nullptr) {

    // mesh = Mesh{};
    // InitMesh(mesh);

    std::unique_lock<std::mutex> lock(upload_mutex_);

    if (vertices_.size() <= ctx.id) {
      vertices_.resize(ctx.id + 1);
      indices_.resize(ctx.id + 1);
      vertex_array_.resize(ctx.id + 1);
    }

    vertices_[ctx.id] =
        ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                                         scm::gl::USAGE_STATIC_DRAW,
                                         mesh_.num_vertices() * sizeof(Vertex),
                                         0);


    Vertex* data(static_cast<Vertex*>(ctx.render_context->map_buffer(
        vertices_[ctx.id], scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER)));

    mesh_.copy_to_buffer(data);

    ctx.render_context->unmap_buffer(vertices_[ctx.id]);

    indices_[ctx.id] =
        ctx.render_device->create_buffer(scm::gl::BIND_INDEX_BUFFER,
                                         scm::gl::USAGE_STATIC_DRAW,
                                         mesh_.num_triangles() * 3 * sizeof(unsigned),
                                         &mesh_.indices[0]);

    vertex_array_[ctx.id] = ctx.render_device->create_vertex_array(
        scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
            0, 1, scm::gl::TYPE_VEC2F, sizeof(Vertex))(
            0, 2, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
            0, 3, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
            0, 4, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
            0, 5, scm::gl::TYPE_VEC4F, sizeof(Vertex))(
            0, 6, scm::gl::TYPE_VEC4I, sizeof(Vertex)),
        {vertices_[ctx.id]});
    
    // init non transformated/animated bone boxes
    // use every single vertex to be manipulated by a certain bone per bone box
    for (unsigned v(0); v < mesh_.num_vertices(); ++v) {
      auto final_pos  = scm::math::vec4(mesh_.get_position(v).x, mesh_.get_position(v).y, mesh_.get_position(v).z, 1.0);
      for(unsigned i(0); i<4; ++i){
        std::cout << mesh_.get_weight(v).IDs[i] << std::endl;
        bone_boxes_[mesh_.get_weight(v).IDs[i]].expandBy(scm::math::vec3(final_pos.x,final_pos.y,final_pos.z));
      }
    }

    ctx.render_context->apply();
  }

}

////////////////////////////////////////////////////////////////////////////////

std::vector<math::BoundingBox<math::vec3>>
SkeletalAnimationRessource::get_bone_boxes(){
  
  auto tmp_boxes = std::vector<math::BoundingBox<math::vec3>>(100,math::BoundingBox<math::vec3>());

  auto bone_transformation = animation_director_->get_bone_transforms();

  for(uint b(0);b<bone_boxes_.size();++b){

    if(!bone_boxes_[b].isEmpty()){
      tmp_boxes[b] = transform(bone_boxes_[b], bone_transformation[b]);
    }
  }
  return tmp_boxes;
}


////////////////////////////////////////////////////////////////////////////////

void SkeletalAnimationRessource::draw(RenderContext const& ctx) /*const*/ {

  // upload to GPU if neccessary
  upload_to(ctx);

  ctx.render_context->bind_vertex_array(vertex_array_[ctx.id]);
  ctx.render_context->bind_index_buffer(indices_[ctx.id], scm::gl::PRIMITIVE_TRIANGLE_LIST, scm::gl::TYPE_UINT);
  ctx.render_context->apply_vertex_input();
  ctx.render_context->draw_elements(mesh_.num_triangles() * 3);
}

////////////////////////////////////////////////////////////////////////////////

void SkeletalAnimationRessource::ray_test(Ray const& ray, int options,
                    node::Node* owner, std::set<PickResult>& hits) {
  //TODO raycasting
  Logger::LOG_ERROR << "get_vertex() dynamic ray testing not supported " << std::endl;
  //kd_tree_.ray_test(ray, mesh_, options, owner, hits);
}

////////////////////////////////////////////////////////////////////////////////

unsigned int SkeletalAnimationRessource::num_vertices() const { return mesh_.num_vertices(); }

////////////////////////////////////////////////////////////////////////////////

unsigned int SkeletalAnimationRessource::num_faces() const { return mesh_.num_triangles(); }

////////////////////////////////////////////////////////////////////////////////

scm::math::vec3 SkeletalAnimationRessource::get_vertex(unsigned int i) const {

  //TODO physics handling
  Logger::LOG_ERROR << "get_vertex() dynamic vertex positions not supported " << std::endl;
  return scm::math::vec3();
}

////////////////////////////////////////////////////////////////////////////////

std::vector<unsigned int> SkeletalAnimationRessource::get_face(unsigned int i) const {

  //TODO cpu representation of mesh
  Logger::LOG_ERROR << "get_face() of merged neshes not supported " << std::endl;
  /*std::vector<unsigned int> face(mesh_->mFaces[i].mNumIndices);
  for (unsigned int j = 0; j < mesh_->mFaces[i].mNumIndices; ++j)
    face[j] = mesh_->mFaces[i].mIndices[j];
  return face;*/
  return std::vector<unsigned int>();
}

////////////////////////////////////////////////////////////////////////////////
}