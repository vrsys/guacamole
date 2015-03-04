// class header
#include <gua/utils/Pose.hpp>
#include <gua/utils/Bone.hpp>
//external headers
#include <iostream>
#include <queue>

namespace gua {

Pose::Pose():
  transforms{}
{}

Pose::~Pose()
{}

bool Pose::contains(std::string const& name ) const {
  return transforms.find(name) != transforms.end();
}

Transformation const& Pose::get_transform(std::string const& name) const{
  try {
    return transforms.at(name);
  }
  catch(std::exception const& e) {
    Logger::LOG_ERROR << "bone '" << name << "' not contained in pose" << std::endl;
    return transforms.begin()->second;
  }
}

void Pose::set_transform(std::string const& name, Transformation const& value) {
  transforms[name] = value;
}

void Pose::blend(Pose const& pose2, float blendFactor) {
  for_each(pose2.transforms.cbegin(), pose2.transforms.cend(), [this, &blendFactor](std::pair<std::string, Transformation> const& p) {
    if(contains(p.first)) {
      set_transform(p.first, get_transform(p.first).blend(p.second, blendFactor));
    }
    else {
      set_transform(p.first, p.second);
    }
  });
  // *this = *this * (1 - blendFactor) + pose2 * blendFactor;
}

Pose& Pose::operator+=(Pose const& pose2) {
  for_each(pose2.transforms.cbegin(), pose2.transforms.cend(), [this](std::pair<std::string, Transformation> const& p) {
    if(contains(p.first)) {
      set_transform(p.first, get_transform(p.first) + p.second);
    }
    else {
      set_transform(p.first, p.second);
    }
  });
  return *this;
}
Pose Pose::operator+(Pose const& p2) const {
  Pose temp{*this};
  temp += p2;
  return temp;
}

Pose& Pose::operator*=(float const factor) {
  for(auto& p : transforms)
  {
    p.second *=factor;
  }
  return *this;
}
Pose Pose::operator*(float const factor) const {
  Pose temp{*this};
  temp *= factor;
  return temp;
}

void Pose::partial_replace(Pose const& pose2, std::shared_ptr<Bone> const& pNode) {
  if(pose2.contains(pNode->name)) {
    set_transform(pNode->name, pose2.get_transform(pNode->name));
  }

  for(std::shared_ptr<Bone>& child : pNode->children) {
    partial_replace(pose2, child);
  }
}

} // namespace gua