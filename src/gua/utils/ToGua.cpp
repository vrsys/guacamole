// class header
#include <gua/utils/ToGua.hpp>

// external headers
#ifdef GUACAMOLE_FBX
#include <fbxsdk.h>
#endif // GUACAMOLE_FBX

namespace to_gua
{
scm::math::mat4f mat4f(aiMatrix4x4 const& m)
{
    scm::math::mat4f res(m.a1, m.b1, m.c1, m.d1, m.a2, m.b2, m.c2, m.d2, m.a3, m.b3, m.c3, m.d3, m.a4, m.b4, m.c4, m.d4);
    return res;
}
scm::math::quatf quatf(aiQuaternion const& q)
{
    scm::math::quatf res(q.w, q.x, q.y, q.z);
    return res;
}
#ifdef GUACAMOLE_FBX
scm::math::mat4f mat4f(FbxAMatrix const& m)
{
    scm::math::mat4f res(m[0][0], m[0][1], m[0][2], m[0][3], m[1][0], m[1][1], m[1][2], m[1][3], m[2][0], m[2][1], m[2][2], m[2][3], m[3][0], m[3][1], m[3][2], m[3][3]);
    return res;
}
scm::math::mat4d mat4d(FbxAMatrix const& m)
{
    scm::math::mat4d res(m[0][0], m[0][1], m[0][2], m[0][3], m[1][0], m[1][1], m[1][2], m[1][3], m[2][0], m[2][1], m[2][2], m[2][3], m[3][0], m[3][1], m[3][2], m[3][3]);
    return res;
}
scm::math::quatf quatf(FbxQuaternion const& q)
{
    scm::math::quatf res(q[3], q[0], q[1], q[2]);
    return res;
}
#endif
} // namespace to_gua