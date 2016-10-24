// class header
#include <gua/utils/Skeleton.hpp>

// guacamole headers
#include <gua/utils/ToGua.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/utils/SkeletalPose.hpp>
#include <gua/utils/BonePose.hpp>
#include <gua/utils/Bone.hpp>

//external headers
#ifdef GUACAMOLE_FBX
  #include <fbxsdk.h>
#endif
#include <assimp/scene.h>

namespace gua {

unsigned Skeleton::addBone(aiNode const& node) {
  unsigned index = m_bones.size();
  m_bones.emplace_back(node);
  m_bones[index].index = index;

  for (unsigned i = 0; i < node.mNumChildren; ++i) {
    unsigned child_index = addBone(*(node.mChildren[i]));
    m_bones[index].children2.push_back(child_index);
  }

  return index;
}

Skeleton::Skeleton(aiScene const& scene) {
  //construct hierarchy
  addBone(*(scene.mRootNode));

  std::map<std::string, std::pair<unsigned int, scm::math::mat4f> > bone_info {};
  unsigned num_bones = 0;
  for (unsigned int i = 0; i < scene.mNumMeshes; i++) {
    for (unsigned int b = 0; b < scene.mMeshes[i]->mNumBones; ++b) {

      std::string BoneName(scene.mMeshes[i]->mBones[b]->mName.data);
      if (bone_info.find(BoneName) == bone_info.end()) {

        bone_info[BoneName] = std::make_pair(
            num_bones,
            to_gua::mat4f(scene.mMeshes[i]->mBones[b]->mOffsetMatrix));
        ++num_bones;
      }
    }
  }
  set_properties(bone_info);

  store_mapping();
}

#ifdef GUACAMOLE_FBX
unsigned Skeleton::addBone(FbxNode& node) {
  unsigned index = m_bones.size();
  m_bones.emplace_back(node);
  m_bones[index].index = index;
  m_bones[index].children2.resize(node.GetChildCount());

  for (int i = 0; i < node.GetChildCount(); ++i) {
    FbxSkeleton const* skelnode { node.GetChild(i)->GetSkeleton() };
    if (skelnode && skelnode->GetSkeletonType() == FbxSkeleton::eEffector &&
        node.GetChild(i)->GetChildCount() == 0) {
      Logger::LOG_DEBUG << node.GetChild(i)->GetName()
                        << " is effector, ignoring it" << std::endl;
    } else {
      unsigned child_index = addBone(*(node.GetChild(i)));
      m_bones[index].children2.at(i) = child_index;
    }
  }

  return index;
}

Skeleton::Skeleton(FbxScene& scene) {
  //construct hierarchy
  addBone(*(scene.GetRootNode()));

  std::map<std::string, std::pair<unsigned int, scm::math::mat4f> > bone_info{}
  ;
  unsigned num_bones = 0;
  for (size_t i = 0; i < size_t(scene.GetGeometryCount()); i++) {
    FbxGeometry* geo = scene.GetGeometry(i);
    if (geo->GetAttributeType() == FbxNodeAttribute::eMesh) {

      //check for skinning, use first skin deformer
      FbxSkin* skin = nullptr;
      for (size_t i = 0; i < size_t(geo->GetDeformerCount()); ++i) {
        FbxDeformer* defPtr = { geo->GetDeformer(i) };
        if (defPtr->GetDeformerType() == FbxDeformer::eSkin) {
          skin = static_cast<FbxSkin*>(defPtr);
          break;
        }
      }

      if (!skin) {
        Logger::LOG_ERROR << "Mesh does not contain skin deformer" << std::endl;
        assert(false);
      }

      //one cluster corresponds to one bone
      for (size_t i = 0; i < size_t(skin->GetClusterCount()); ++i) {
        FbxCluster* cluster = skin->GetCluster(i);
        FbxNode* node = cluster->GetLink();

        if (!node) {
          Logger::LOG_ERROR << "associated node does not exist!" << std::endl;
          assert(false);
        }

        std::string bone_name(node->GetName());
        //helper to check for matrix magnitude
        auto magnitude = [](scm::math::mat4f const & mat)->float {
          float mag = 0.0f;
          for (unsigned i = 0; i < 16; ++i) {
            mag += mat[i] * mat[i];
          }
          return mag;
        }
        ;

        //reference pose of bone
        FbxAMatrix bind_transform;
        cluster->GetTransformLinkMatrix(bind_transform);
        //check if the clusters have an extra transformation
        FbxAMatrix cluster_transform;
        cluster->GetTransformMatrix(cluster_transform);
        if (magnitude(to_gua::mat4f(cluster_transform) -
                      scm::math::mat4f::identity()) > 0.000000001f) {
          Logger::LOG_WARNING
              << "weight cluster of bone '" << bone_name
              << "' has transformation, animation will be skewed" << std::endl;
        }
        //add bone to list if it is not already included
        if (bone_info.find(bone_name) == bone_info.end()) {
          bone_info[bone_name] = std::make_pair(
              num_bones,
              to_gua::mat4f(bind_transform.Inverse() * cluster_transform));
          ++num_bones;
        }
      }

      // traverse hierarchy and set accumulated values in the bone
      set_properties(bone_info);
    }
  }
  store_mapping();
}

#endif

gua::Bone const* Skeleton::find(std::string const& name) const {
  for (auto const& bone : m_bones) {
    if (bone.name == name)
      return &bone;
  }
  return nullptr;
}

void Skeleton::collect_indices(std::map<std::string, int>& ids) const {
  ids = m_mapping;
}

void Skeleton::store_mapping() {
  for (std::size_t i = 0; i < m_bones.size(); ++i) {
    m_mapping[m_bones[i].name] = i;
  }
}

void Skeleton::set_properties(
    std::map<std::string, std::pair<unsigned, scm::math::mat4f> > const& infos) {
  for (auto& bone : m_bones) {
    if (infos.find(bone.name) != infos.end()) {
      bone.offsetMatrix = infos.at(bone.name).second;
      // bone.index = infos.at(bone.name).first;
    }
  }
}

void Skeleton::accumulate_matrices(unsigned start_node, std::vector<scm::math::mat4f>& transformMat4s,
                               SkeletalPose const& pose,
                                scm::math::mat4f const& parentTransform) const {
  auto const& bone = m_bones[start_node];
  // initialize with idle transform
  scm::math::mat4f nodeTransformation{bone.transformation};
  if (pose.contains(bone.name)) {
    nodeTransformation = pose.get_transform(bone.name).to_matrix();
  }

  scm::math::mat4f finalTransformation = parentTransform * nodeTransformation;

  //update transform if bone is mapped
  if (bone.index >= 0) {
    transformMat4s[bone.index] = finalTransformation * bone.offsetMatrix;
  }

  for (auto const& child : bone.children2) {
    accumulate_matrices(child, transformMat4s, pose, finalTransformation);
  }
}

std::size_t Skeleton::num_bones() const {
  return m_bones.size();
}


}  // namespace gua