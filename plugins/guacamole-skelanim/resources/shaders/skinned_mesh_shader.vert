@include "common/header.glsl"

layout(location=0) in vec3 gua_in_position;
layout(location=1) in vec2 gua_in_texcoords;
layout(location=2) in vec3 gua_in_normal;
layout(location=3) in vec3 gua_in_tangent;
layout(location=4) in vec3 gua_in_bitangent;
layout(location=5) in uint gua_in_bone_id_offset;
layout(location=6) in uint gua_in_nr_of_bones;

layout (std140, binding=2) uniform boneBlock {
  mat4 bone_transformation[100];
};

layout (std430, binding=2) buffer boneIDs {
  uint ids[];
};

layout (std430, binding=3) buffer boneWeights {
  float weights[];
};

@include "common/gua_camera_uniforms.glsl"

@material_uniforms@

@include "common/gua_vertex_shader_output.glsl"

@include "common/gua_global_variable_declaration.glsl"

@material_method_declarations_vert@

void main() {

  mat4 BoneTransform = mat4(0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0);

  for(uint i = 0; i < gua_in_nr_of_bones; ++i){
    uint bone_id = ids[gua_in_bone_id_offset+i];
    BoneTransform+= bone_transformation[bone_id] * weights[gua_in_bone_id_offset+i];
  }

  @material_input@
  mat4 normalBoneTransform = inverse(transpose(BoneTransform));

  gua_position  = (gua_model_matrix * BoneTransform * vec4(gua_in_position, 1.0)).xyz;
  gua_normal    = (gua_normal_matrix * normalBoneTransform * vec4(gua_in_normal, 0.0)).xyz;
  gua_normal    = normalize(gua_normal);
  gua_tangent   = (gua_normal_matrix * normalBoneTransform * vec4(gua_in_tangent, 0.0)).xyz;
  gua_bitangent = (gua_normal_matrix * normalBoneTransform * vec4(gua_in_bitangent, 0.0)).xyz;
  gua_texcoords = gua_in_texcoords;
  gua_metalness  = 0.5;
  gua_roughness  = 0.5;
  gua_emissivity  = 0;

  @material_method_calls_vert@

  @include "common/gua_varyings_assignment.glsl"

  // gua_varying_color = gua_normal;

  gl_Position = gua_projection_matrix * gua_view_matrix * vec4(gua_position, 1.0);
}
