// class header
#include <gua/utils/Bone.hpp>

// guacamole headers
#include <gua/utils/ToGua.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/utils/SkeletalPose.hpp>
#include <gua/utils/BonePose.hpp>

//external headers
#ifdef GUACAMOLE_FBX
  #include <fbxsdk.h>
#endif
#include <assimp/scene.h>

namespace gua {

Bone::Bone()
  : name( "none" ),
    index (-1),
    parentName ( "none" ),
    numChildren ( 0 ),
    transformation ( scm::math::mat4f::identity() ),
    offsetMatrix ( scm::math::mat4f::identity() )
{}

Bone::Bone(aiNode const& node)
  : name ( node.mName.C_Str() ),
    index ( -1 ),
    parentName ( node.mParent != nullptr ? node.mParent->mName.C_Str() : "none" ),
    numChildren ( node.mNumChildren ),
    transformation ( to_gua::mat4f(node.mTransformation) ),
    offsetMatrix ( scm::math::mat4f::identity() )
{
  for (unsigned i = 0; i < node.mNumChildren; ++i) {
    auto child = std::make_shared<Bone>(*(node.mChildren[i]));
    children.push_back(child);
  }
}

Bone::Bone(aiScene const& scene) {
  //construct hierarchy
  *this = Bone { *(scene.mRootNode) };

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
  this->set_properties(bone_info);
}

#ifdef GUACAMOLE_FBX
Bone::Bone(FbxScene& scene) {
  //construct hierarchy
  *this = Bone { *(scene.GetRootNode()) };

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
      this->set_properties(bone_info);
    }
  }
}

Bone::Bone(FbxNode& node)
  : name ( node.GetName() )
  , index ( -1 )
  , parentName ( node.GetParent() != nullptr ? node.GetParent()->GetName() : "none" )
  , numChildren ( unsigned(node.GetChildCount()) )
  , transformation ( to_gua::mat4f(node.EvaluateLocalTransform()) )
  , offsetMatrix ( scm::math::mat4f::identity() )
{
  for (int i = 0; i < node.GetChildCount(); ++i) {
    FbxSkeleton const* skelnode { node.GetChild(i)->GetSkeleton() };
    if (skelnode && skelnode->GetSkeletonType() == FbxSkeleton::eEffector &&
        node.GetChild(i)->GetChildCount() == 0) {
      Logger::LOG_DEBUG << node.GetChild(i)->GetName()
                        << " is effector, ignoring it" << std::endl;
    } else {
      auto child = std::make_shared<Bone>(*(node.GetChild(i)));
      children.push_back(child);
    }
  }
}
#endif

Bone::~Bone() {}

std::shared_ptr<Bone> Bone::find(std::string const& name) const {

  for (auto const& child : children) {
    if (child->name == name)
      return child;

    std::shared_ptr<Bone> found = child->find(name);
    if (found)
      return found;
  }

  return nullptr;
}

void Bone::collect_indices(std::map<std::string, int>& ids) const {
  ids[name] = index;

  for (auto const& child : children) {
    child->collect_indices(ids);
  }
}

void Bone::set_properties(
    std::map<std::string, std::pair<unsigned, scm::math::mat4f> > const& infos) {
  if (infos.find(name) != infos.end()) {
    offsetMatrix = infos.at(name).second;
    index = infos.at(name).first;
  }

  for (std::shared_ptr<Bone>& child : children) {
    child->set_properties(infos);
  }
}

void Bone::accumulate_matrices(std::vector<scm::math::mat4f>& transformMat4s,
                               SkeletalPose const& pose,
                               scm::math::mat4f const& parentTransform) const {
  scm::math::mat4f nodeTransformation{transformation};
  if (pose.contains(name)) {
    nodeTransformation = pose.get_transform(name).to_matrix();
  }

  scm::math::mat4f finalTransformation = parentTransform * nodeTransformation;

  //update transform if bone is mapped
  if (index >= 0) {
    transformMat4s[index] = finalTransformation * offsetMatrix;
  }

  for (auto const& child : children) {
    child->accumulate_matrices(transformMat4s, pose, finalTransformation);
  }
}

}  // namespace gua
