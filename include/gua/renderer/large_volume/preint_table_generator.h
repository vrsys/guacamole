
#ifndef SCM_LARGE_DATA_PREINT_TABLE_GENERATOR_H_INCLUDED
#define SCM_LARGE_DATA_PREINT_TABLE_GENERATOR_H_INCLUDED

#include <scm/gl_core/frame_buffer_objects/frame_buffer_objects_fwd.h>
#include <scm/gl_core/render_device/render_device_fwd.h>
#include <scm/gl_core/shader_objects/shader_objects_fwd.h>
#include <scm/gl_core/state_objects/state_objects_fwd.h>
#include <scm/gl_core/texture_objects/texture_objects_fwd.h>

#include <scm/gl_util/primitives/primitives_fwd.h>

namespace scm {
namespace data {

class preint_table_generator
{
public:
    preint_table_generator(const gl::render_device_ptr& device);
    virtual ~preint_table_generator();

    void                        generate_table(const gl::render_context_ptr& context,
                                               const gl::texture_2d_ptr&     ca_map,
                                               const gl::texture_2d_ptr&     preint_map,
                                                     int                     preint_steps,
                                                     float                   cur_sdist,
                                                     float                   ref_sdist);

    bool                        reload_shader_resources(const gl::render_device_ptr& device);

protected:
    gl::program_ptr             _preint_program;

    gl::depth_stencil_state_ptr _dstate;
    gl::rasterizer_state_ptr    _rstate;
    gl::blend_state_ptr         _bstate;
    gl::sampler_state_ptr       _sstate_linear;

    gl::fullscreen_triangle_ptr _fs_geom;

    gl::frame_buffer_ptr        _fbo;

}; // class preint_table_generator

} // namespace data
} // namespace scm

#endif // SCM_LARGE_DATA_PREINT_TABLE_GENERATOR_H_INCLUDED
