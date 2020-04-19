// header
#extension GL_NV_bindless_texture  : require
#extension GL_ARB_bindless_texture : enable
#extension GL_NV_gpu_shader5       : enable

#extension GL_ARB_shader_storage_buffer_object : enable
#extension GL_ARB_separate_shader_objects : enable

#ifdef GL_NV_shader_atomic_int64
#extension GL_NV_shader_atomic_int64 : enable
#endif

#extension GL_ARB_derivative_control : enable

// single pass instanced rendering or multi view rendering
#extension GL_ARB_shader_viewport_layer_array: enable