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
    curr_anim_num_{0},
    last_anim_num_{0},
    bone_mapping_{},
    anim_start_node_{},
    state_{Playback::play},
    animation_mapping_{},
    blending_state_{Blending::linear},
    blendFactor_{0.0},
    blend_start_{0},
    blend_end_{0},
    blend_duration_{0.5f},
    root_{root},
    timer_{} {
  timer_.start();

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

  float timeNormalized1 = timeInSeconds1 / pAnim1.get_duration();
  timeNormalized1 = scm::math::fract(timeNormalized1);

  float timeNormalized2 = timeInSeconds2 / pAnim2.get_duration();
  timeNormalized2 = scm::math::fract(timeNormalized2);

  scm::math::mat4f identity = scm::math::mat4f::identity();

  Pose pose1{pAnim1.calculate_pose(timeNormalized1)};
  Pose pose2{pAnim2.calculate_pose(timeNormalized2)};
  
  pose1.blend(pose2, blendFactor_);

  anim_start_node_->accumulate_matrices(transforms, pose1, identity);
}

void SkeletalAnimationDirector::partial_blend(float timeInSeconds, SkeletalAnimation const& pAnim1, SkeletalAnimation const& pAnim2, std::string const& nodeName, std::vector<scm::math::mat4f>& transforms) {
  
  std::shared_ptr<Bone> start{anim_start_node_->find(nodeName)};
  
  if(!start) {
    Logger::LOG_WARNING << "node '"<< nodeName << "' not found" << std::endl; 
    return;
  }

  float timeNormalized1 = timeInSeconds / pAnim1.get_duration();
  timeNormalized1 = scm::math::fract(timeNormalized1);

  float timeNormalized2 = timeInSeconds / pAnim2.get_duration();
  timeNormalized2 = scm::math::fract(timeNormalized2);

  scm::math::mat4f identity = scm::math::mat4f::identity();

  Pose full_body{pAnim1.calculate_pose(timeNormalized1)};
  Pose upper_body{pAnim2.calculate_pose(timeNormalized2)};

  full_body.partial_replace(upper_body, start);

  anim_start_node_->accumulate_matrices(transforms, full_body, identity);
}

std::vector<scm::math::mat4f> SkeletalAnimationDirector::get_bone_transforms()
{
  //reserve vector for transforms
  std::vector<scm::math::mat4f> transforms{num_bones_, scm::math::mat4f::identity()};
  
  float currentTime = timer_.get_elapsed();

  if(!has_anims_) {
    calculate_matrices(transforms);
    return transforms;  
  }

  float anim_time_1 = currentTime - blend_start_;
  float anim_time_2 = currentTime;
  switch(state_) {
    //crossfade two anims
    case Playback::crossfade: {
      float time = 1.0f-((blend_end_ - currentTime) / blend_duration_);
      
      switch(blending_state_){
        case Blending::swap : blendFactor_ = blend::swap(time);break;
        case Blending::linear : blendFactor_ = blend::linear(time);break;
        case Blending::smoothstep : blendFactor_ = blend::smoothstep(time);break;
        case Blending::cosinus : blendFactor_ = blend::cos(time);break;
        default: blendFactor_ = blend::linear(time);
      }
      
      blend_pose(anim_time_2, anim_time_1, animations_[last_anim_num_], animations_[curr_anim_num_], transforms);
      //if the blend duration is over, switch to normal playback
      if(currentTime > blend_end_) {
        state_ = Playback::play;
        blendFactor_ = 1.0;
        std::cout << "finished blending at " << timer_.get_elapsed() << std::endl;
      }
      break;
    }
    case Playback::play: {
      calculate_matrices(anim_time_1, animations_[curr_anim_num_], transforms);  
      break;
    }
    //blend two anims
    case Playback::partial: {
      partial_blend(currentTime, animations_[0], animations_[1], "Waist", transforms);
      break;
    }
    default: Logger::LOG_WARNING << "playback mode not found" << std::endl; break;
  }
  
  return transforms;
}

void SkeletalAnimationDirector::calculate_matrices(float timeInSeconds, SkeletalAnimation const& pAnim, std::vector<scm::math::mat4f>& transforms) {
 
  float timeNormalized = 0;
  Pose pose{};

  timeNormalized = timeInSeconds / pAnim.get_duration();
  timeNormalized = scm::math::fract(timeNormalized);

  pose = pAnim.calculate_pose(timeNormalized);

  scm::math::mat4f identity = scm::math::mat4f::identity();
  anim_start_node_->accumulate_matrices(transforms, pose, identity);
}

void SkeletalAnimationDirector::calculate_matrices(std::vector<scm::math::mat4f>& transforms) {

  Pose pose{};

  scm::math::mat4f identity = scm::math::mat4f::identity();
  anim_start_node_->accumulate_matrices(transforms, pose, identity);
}

void SkeletalAnimationDirector::fade_to(std::string const& name, float duration, bool loop) {
  set_animation(name);
  blend_start_ = timer_.get_elapsed();
  blend_end_ = blend_start_ + duration;
  blend_duration_ = duration;
  state_ = Playback::crossfade;
  std::cout << "fading to " << name << " at " << blend_start_ << std::endl;

}

void SkeletalAnimationDirector::set_playback_mode(uint mode) {
  switch(mode) {
    case 0 : state_ = Playback::partial; break;
    case 1 : state_ = Playback::crossfade; break;
    case 2 : state_ = Playback::play; break;
    default: state_ = Playback::partial;
  }
}

uint SkeletalAnimationDirector::get_playback_mode() const {
  return state_;
}

void SkeletalAnimationDirector::set_blending_mode(uint mode) {
  switch(mode) {
    case 0 : blending_state_ = Blending::swap; break;
    case 1 : blending_state_ = Blending::linear; break;
    case 2 : blending_state_ = Blending::smoothstep; break;
    case 3 : blending_state_ = Blending::cosinus; break;
    default: blending_state_ = Blending::linear;;
  }
}

uint SkeletalAnimationDirector::get_blending_mode() const {
  return blending_state_;
}

std::string const& SkeletalAnimationDirector::get_animation() const {
  if(has_anims_ && curr_anim_num_< animations_.size()){
    return animations_[curr_anim_num_].get_name();
  }
  else{
    return none_loaded;
  }
}

void SkeletalAnimationDirector::set_animation(std::string const& animation_name) {
  if(animation_mapping_.find(animation_name) != animation_mapping_.end()) {
    last_anim_num_ = curr_anim_num_;
    curr_anim_num_ = animation_mapping_.at(animation_name);
  }
  else {
    gua::Logger::LOG_WARNING << "No matching animation with name: " << animation_name <<" found!" << std::endl;
  }
}

float SkeletalAnimationDirector::get_blending_factor() const{
  return blendFactor_;
}

void SkeletalAnimationDirector::set_blending_factor(float f){
  blendFactor_ = f;
}

float SkeletalAnimationDirector::get_blending_duration() const{
  return blend_duration_;
}

void SkeletalAnimationDirector::set_blending_duration(float duration){
  blend_duration_ = duration;
}

bool SkeletalAnimationDirector::has_anims() const {
  return has_anims_;
}

} // namespace gua