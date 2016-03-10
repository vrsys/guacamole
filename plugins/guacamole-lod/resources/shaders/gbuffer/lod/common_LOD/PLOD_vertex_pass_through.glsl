@include "PLOD_calculate_tangents.glsl"

@include "PLOD_assign_tangents.glsl"

  VertexOut.pass_normal = normalize((gua_normal_matrix * vec4(in_normal, 0.0)).xyz);

  gl_Position = vec4(in_position, 1.0);
