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


#extension GL_ARB_shader_viewport_layer_array: enable


#if @get_enable_hardware_mvr@
#extension GL_OVR_multiview2 : require
#endif

#if @get_enable_multi_view_rendering@
#if @get_enable_hardware_mvr@
#define LAYER_ID gl_ViewID_OVR
#else
#define LAYER_ID gl_InstanceID
#endif
#endif

//#extension GL_NV_viewport_array2: require

//#extension GL_OVR_multiview2 : enable