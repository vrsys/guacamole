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

#ifndef GUA_SKELETAL_ANIMATION_UTILS_HPP
#define GUA_SKELETAL_ANIMATION_UTILS_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/utils/Mesh.hpp>
#include <gua/utils/Bone.hpp>

// external headers
#include <scm/gl_core.h>
#include <scm/core/math/quat.h>

#include <vector>
#include <map>
#include <assimp/scene.h>       // Output data structure
#include <fbxsdk.h>

namespace gua {
class Pose;

struct Transformation {
 public:
  Transformation();

  Transformation(scm::math::vec3 const& scale, scm::math::quatf const& rotate, scm::math::vec3 const& translate);

  ~Transformation();

  scm::math::mat4f to_matrix() const;

  Transformation blend(Transformation const& t, float const factor) const;

  Transformation operator+(Transformation const& t) const;
  Transformation& operator+=(Transformation const& t);

  Transformation operator*(float const factor) const;
  Transformation& operator*=(float const f);

 private:
  scm::math::vec3 scaling;
  scm::math::quatf rotation;
  scm::math::vec3 translation;
};

template<class T>
struct Key {

  Key(double time, T const& value):
    time{time},
    value{value}
  {}

  ~Key(){};

  double time;
  T value;
};

class BoneAnimation {
 public:
  BoneAnimation();

  ~BoneAnimation();

  BoneAnimation(aiNodeAnim* anim);
  BoneAnimation(FbxTakeInfo const& take, FbxNode& node);

  Transformation calculate_transform(float time) const;

  std::string const& get_name() const;

 private:

  scm::math::vec3 interpolate(scm::math::vec3 val1, scm::math::vec3 val2, float factor) const;

  scm::math::quatf interpolate(scm::math::quatf val1, scm::math::quatf val2, float factor) const;

  template<class T> 
  uint find_key(float animationTime, std::vector<Key<T>> keys) const;

  template<class T> 
  T calculate_value(float time, std::vector<Key<T>> keys) const;

  std::string name;

  std::vector<Key<scm::math::vec3>> scalingKeys;
  std::vector<Key<scm::math::quatf>> rotationKeys;
  std::vector<Key<scm::math::vec3>> translationKeys;
};

class Pose {
 public:
  Pose();

  ~Pose();

  bool contains(std::string const& name ) const;

  Transformation const& get_transform(std::string const& name) const;

  void set_transform(std::string const& name, Transformation const& value);

  void blend(Pose const& pose2, float blendFactor);

  Pose& operator+=(Pose const& pose2);
  Pose operator+(Pose const& p2) const;

  Pose& operator*=(float const factor);
  Pose operator*(float const factor) const;

  void partial_replace(Pose const& pose2, std::shared_ptr<Bone> const& pNode);

 private: 
  std::map<std::string, Transformation> transforms;
};

class SkeletalAnimation {
 public:
  SkeletalAnimation();

<<<<<<< HEAD
  SkeletalAnimation(aiAnimation const& anim);
<<<<<<< HEAD
=======
  SkeletalAnimation(aiAnimation* anim, std::string const& file_name);
>>>>>>> add animation name interface for animation control in avango
=======
  SkeletalAnimation(FbxAnimStack* anim, std::vector<FbxNode*> const& bones);
>>>>>>> animation sampling theoretically works for translation

  ~SkeletalAnimation();

  Pose calculate_pose(float time) const;

  double get_duration() const;
  std::string const& get_name() const;

  std::string get_name() const;

 private:
  std::string name;
  unsigned numFrames;
  double numFPS;
  double duration;
  unsigned numBoneAnims;

  std::vector<BoneAnimation> boneAnims;
};

namespace blend {
  float cos(float x); 
  float swap(float x); 
  float linear(float x);
  float smoothstep(float x);
};

<<<<<<< HEAD
#define ZERO_MEM(a) memset(a, 0, sizeof(a))
struct weight_map
{        
  uint IDs[4];
  float weights[4];

  weight_map()
  {
      Reset();
  };
  
  void Reset()
  {
      ZERO_MEM(IDs);
      ZERO_MEM(weights);        
  }
  
  void AddBoneData(uint bone_ID, float weight)
  {
    uint num_weights = (sizeof(IDs)/sizeof(IDs[0]));
    for (uint i = 0 ; i <  num_weights; i++) {
        if (weights[i] == 0.0) {
            IDs[i]     = bone_ID;
            weights[i] = weight;
            return;
        }        
    }
    // should never get here - more bones than we have space for
    Logger::LOG_WARNING << "Warning: Ignoring bone associated to vertex (more than " << num_weights << ")" << std::endl;
    //assert(false);
  }
};

<<<<<<< HEAD

<<<<<<< HEAD:include/gua/renderer/SkeletalAnimationUtils.hpp
struct Vertex {
  scm::math::vec3 pos;
=======
struct SkinnedVertex {
  scm::math::vec3f pos;
>>>>>>> seperate file for Mesh class, moved to utils:plugins/guacamole-skelanim/include/gua/renderer/SkeletalAnimationUtils.hpp
  scm::math::vec2f tex;
  scm::math::vec3 normal;
  scm::math::vec3 tangent;
  scm::math::vec3 bitangent;
  scm::math::vec4f bone_weights;
  scm::math::vec4i bone_ids;
};

struct SkinnedMesh {
 public:
  SkinnedMesh();

  SkinnedMesh(aiMesh const& mesh, Node const& root = Node{});
  SkinnedMesh(FbxMesh& mesh, Node const& root = Node{});

  void copy_to_buffer(SkinnedVertex* vertex_buffer)  const;

  // std::vector<Vertex> vertices;
  std::vector<scm::math::vec3> positions;
  std::vector<scm::math::vec3> normals;
  std::vector<scm::math::vec2> texCoords;
  std::vector<scm::math::vec3> tangents;
  std::vector<scm::math::vec3> bitangents;
  std::vector<weight_map> weights;
  std::vector<unsigned> indices;

  unsigned int num_vertices;
  unsigned int num_triangles;

 private:
  void init_weights(aiMesh const& mesh, Node const& root);
  std::vector<weight_map> get_weights(FbxMesh const& mesh, Node const& root);
};

=======
>>>>>>> extra file for skinned mesh
=======
>>>>>>> Node renamed to Bone and moved to seperate file
class SkeletalAnimationUtils {
 public:

<<<<<<< HEAD
  static std::vector<std::shared_ptr<SkeletalAnimation>> load_animations(aiScene const&);
<<<<<<< HEAD
=======
  static std::vector<std::shared_ptr<SkeletalAnimation>> load_animations(aiScene const*, std::string const& file_name);
  static std::shared_ptr<Node> load_hierarchy(aiScene const* scene);
>>>>>>> add animation name interface for animation control in avango
=======
  static std::vector<std::shared_ptr<SkeletalAnimation>> load_animations(FbxScene const&);
>>>>>>> animation sampling theoretically works for translation

  static void calculate_matrices(float TimeInSeconds, Bone const& root, SkeletalAnimation const& pAnim, std::vector<scm::math::mat4f>& Transforms);
  static void calculate_matrices(Bone const& root, std::vector<scm::math::mat4f>& Transforms);

 private:
  SkeletalAnimationUtils() = delete;
};

}

#endif //GUA_SKELETAL_ANIMATION_UTILS_HPP
