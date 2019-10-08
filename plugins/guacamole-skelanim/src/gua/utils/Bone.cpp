// class header
#include <gua/skelanim/utils/Bone.hpp>

// guacamole headers
#include <gua/utils/ToGua.hpp>

// external headers
#ifdef GUACAMOLE_FBX
#include <fbxsdk.h>
#endif
#include <assimp/scene.h>

namespace gua
{
Bone::Bone() : Bone{"none", scm::math::mat4f::identity()} {}

Bone::Bone(std::string const& nm, scm::math::mat4f const& idle, scm::math::mat4f const& offset, std::vector<unsigned> childs) : name{nm}, children{childs}, idle_matrix{idle}, offset_matrix{offset} {}

Bone::Bone(aiNode const& node) : Bone{node.mName.C_Str(), to_gua::mat4f(node.mTransformation)} {}

#ifdef GUACAMOLE_FBX
Bone::Bone(FbxNode& node) : Bone{node.GetName(), to_gua::mat4f(node.EvaluateLocalTransform())} {}
#endif

} // namespace gua
