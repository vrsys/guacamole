@include "SPOINTS_calculate_tangents.glsl"

@include "SPOINTS_assign_tangents.glsl"

gl_Position = vec4(unquantized_pos, 1.0);
