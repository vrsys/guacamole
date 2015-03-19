// class header
#include <gua/renderer/SkeletalAnimationDirector.hpp>


// guacamole headers
#include <gua/utils/Logger.hpp>
#include <gua/utils/Blending.hpp>

//external headers
#include <iostream>
#include <queue>
#include <fbxsdk.h>


namespace gua 
{

const std::string SkeletalAnimationDirector::none_loaded{"none loaded"};

SkeletalAnimationDirector::SkeletalAnimationDirector(std::shared_ptr<Bone> const& root):
    num_bones_{0},
    has_anims_{false},
    firstRun_{true},
    animations_{},
    anim_num_1_{0},
    anim_num_2_{0},
    bone_mapping_{},
    anim_start_node_{},
    animation_mapping_{},
    blend_factor_{0.0},
    root_{root} {

  root_->collect_indices(bone_mapping_);
  num_bones_ = bone_mapping_.size();
  //prevent doom models from moving during anim
  if(root_->name == "<MD5_Root>") {
    anim_start_node_ = root_->children[1]->children[0]->children[0];
  }
  else {
    anim_start_node_ = root_;
  }
}

void SkeletalAnimationDirector::add_animations(aiScene const& scene, std::string const& name) {
  if(!scene.HasAnimations()) Logger::LOG_WARNING << "scene contains no animations!" << std::endl;
 
  for(uint i = 0; i < scene.mNumAnimations; ++i) {
    std::string new_name = (scene.mNumAnimations == 1) ? name : name + std::to_string(i);
    animations_.push_back(SkeletalAnimation{*scene.mAnimations[i], new_name});
    animation_mapping_.insert(std::make_pair(new_name, animations_.size() -1));
  }

  has_anims_ = animations_.size() > 0;
}

void SkeletalAnimationDirector::add_animations(FbxScene& scene, std::string const& name) {
  std::vector<std::shared_ptr<SkeletalAnimation>> animations{};
  unsigned num_anims = scene.GetSrcObjectCount<FbxAnimStack>();
  if(num_anims <= 0) Logger::LOG_WARNING << "scene contains no animations!" << std::endl;
 
  std::vector<FbxNode*> nodes{};
  // traverse hierarchy and collect pointers to all nodes
  std::queue<FbxNode*> node_stack{};
  node_stack.push(scene.GetRootNode());
  while(!node_stack.empty()) {
    FbxNode* curr_node = node_stack.front(); 
    nodes.push_back(curr_node);

    for(int i = 0; i < curr_node->GetChildCount(); ++i) {
      FbxSkeleton const* skelnode{curr_node->GetChild(i)->GetSkeleton()};
      if(skelnode && skelnode->GetSkeletonType() == FbxSkeleton::eEffector && curr_node->GetChild(i)->GetChildCount() == 0) {
        Logger::LOG_DEBUG << curr_node->GetChild(i)->GetName() << " is effector, ignoring it" << std::endl;
      }
      else {
        node_stack.push(curr_node->GetChild(i));
      }
    }
    node_stack.pop();
  }

  for(uint i = 0; i < num_anims; ++i) {
    std::string new_name = (num_anims == 1) ? name : name + std::to_string(i);
    animations_.push_back(SkeletalAnimation{scene.GetSrcObject<FbxAnimStack>(i), nodes, new_name});
    animation_mapping_.insert(std::make_pair(new_name, animations_.size() -1));
  }

  has_anims_ = animations_.size() > 0;
}

void SkeletalAnimationDirector::blend_pose(float timeInSeconds1, float timeInSeconds2, SkeletalAnimation const& pAnim1, SkeletalAnimation const& pAnim2, std::vector<scm::math::mat4f>& transforms) {
  float time_normalized1 = timeInSeconds1 / pAnim1.get_duration();
  time_normalized1 = scm::math::fract(time_normalized1);

  float time_normalized2 = timeInSeconds2 / pAnim2.get_duration();
  time_normalized2 = scm::math::fract(time_normalized2);

  scm::math::mat4f identity = scm::math::mat4f::identity();

  Pose pose1{pAnim1.calculate_pose(time_normalized1)};
  Pose pose2{pAnim2.calculate_pose(time_normalized2)};
  
  pose1.blend(pose2, blend_factor_);

  anim_start_node_->accumulate_matrices(transforms, pose1, identity);
}

void SkeletalAnimationDirector::partial_blend(float timeInSeconds, SkeletalAnimation const& pAnim1, SkeletalAnimation const& pAnim2, std::string const& nodeName, std::vector<scm::math::mat4f>& transforms) {
  std::shared_ptr<Bone> start{anim_start_node_->find(nodeName)};
  
  if(!start) {
    Logger::LOG_WARNING << "node '"<< nodeName << "' not found" << std::endl; 
    return;
  }

  float time_normalized1 = timeInSeconds / pAnim1.get_duration();
  time_normalized1 = scm::math::fract(time_normalized1);

  float time_normalized2 = timeInSeconds / pAnim2.get_duration();
  time_normalized2 = scm::math::fract(time_normalized2);

  scm::math::mat4f identity = scm::math::mat4f::identity();

  Pose full_body{pAnim1.calculate_pose(time_normalized1)};
  Pose upper_body{pAnim2.calculate_pose(time_normalized2)};

  full_body.partial_replace(upper_body, start);

  anim_start_node_->accumulate_matrices(transforms, full_body, identity);
}

std::vector<scm::math::mat4f> SkeletalAnimationDirector::get_bone_transforms() {
  //reserve vector for transforms
  std::vector<scm::math::mat4f> transforms{num_bones_, scm::math::mat4f::identity()};

  if(!has_anims_) {
    calculate_matrices(transforms);
    return transforms;  
  }

  if(blend_factor_ <= 0) {
    calculate_matrices(anim_time_2_, animations_[anim_num_2_], transforms);
  } 
  else if(blend_factor_ >= 1) {
    calculate_matrices(anim_time_1_, animations_[anim_num_1_], transforms);
  }
  else {
    blend_pose(anim_time_2_, anim_time_1_, animations_[anim_num_2_], animations_[anim_num_1_], transforms);    
  }
  
  return transforms;
}

void SkeletalAnimationDirector::calculate_matrices(float timeInSeconds, SkeletalAnimation const& pAnim, std::vector<scm::math::mat4f>& transforms) {
 
  float time_normalized = 0;
  Pose pose{};

  time_normalized = timeInSeconds / pAnim.get_duration();
  time_normalized = scm::math::fract(time_normalized);

  pose = pAnim.calculate_pose(time_normalized);

  scm::math::mat4f identity = scm::math::mat4f::identity();
  anim_start_node_->accumulate_matrices(transforms, pose, identity);
}

void SkeletalAnimationDirector::calculate_matrices(std::vector<scm::math::mat4f>& transforms) {

  Pose pose{};

  scm::math::mat4f identity = scm::math::mat4f::identity();
  anim_start_node_->accumulate_matrices(transforms, pose, identity);
}

bool SkeletalAnimationDirector::has_anims() const {
  return has_anims_;
}

} // namespace gua