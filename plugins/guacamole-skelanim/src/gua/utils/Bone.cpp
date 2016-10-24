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
{}

#ifdef GUACAMOLE_FBX
Bone::Bone(FbxNode& node)
  : name ( node.GetName() )
  , index ( -1 )
  , parentName ( node.GetParent() != nullptr ? node.GetParent()->GetName() : "none" )
  , numChildren ( unsigned(node.GetChildCount()) )
  , transformation ( to_gua::mat4f(node.EvaluateLocalTransform()) )
  , offsetMatrix ( scm::math::mat4f::identity() )
{}
#endif

}  // namespace gua
