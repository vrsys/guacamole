
#include "gauss_table_generator.h"

#include <exception>
#include <sstream>
#include <stdexcept>

#include <boost/assign/list_of.hpp>

#include <scm/gl_core/math.h>
#include <scm/gl_core/frame_buffer_objects.h>
#include <scm/gl_core/render_device.h>
#include <scm/gl_core/shader_objects.h>
#include <scm/gl_core/state_objects.h>
#include <scm/gl_core/texture_objects.h>

#include <scm/gl_util/primitives/fullscreen_triangle.h>

namespace scm {
namespace data {

gauss_table_generator::gauss_table_generator(const gl::render_device_ptr& device)
{
    using namespace scm;
    using namespace scm::gl;
    using namespace scm::math;
    using boost::assign::list_of;

    // program objects ////////////////////////////////////////////////////////////////////////////
    if (!reload_shader_resources(device)) {
        throw std::runtime_error("gauss_table_generator::gauss_table_generator(): error creating gauss generator progam.");
    }

    // state objects //////////////////////////////////////////////////////////////////////////////
    _bstate = device->create_blend_state(false, FUNC_ONE, FUNC_ZERO, FUNC_ONE, FUNC_ZERO);
    _dstate = device->create_depth_stencil_state(false, false);
    _rstate = device->create_rasterizer_state(FILL_SOLID, CULL_BACK, ORIENT_CCW);
    _sstate_linear = device->create_sampler_state(FILTER_MIN_MAG_LINEAR, WRAP_CLAMP_TO_EDGE);

    // framebuffer objects ////////////////////////////////////////////////////////////////////////
    _fbo = device->create_frame_buffer();

    if (!_fbo) {
        throw std::runtime_error("gauss_table_generator::gauss_table_generator(): error creating framebuffer object.");
    }

    // geometry ///////////////////////////////////////////////////////////////////////////////////
    try {
        _fs_geom = make_shared<fullscreen_triangle>(device);
    }
    catch (const std::exception& e) {
        throw std::runtime_error(std::string("gauss_table_generator::gauss_table_generator(): error creating fs geometry objects: ") + e.what());
    }
}

gauss_table_generator::~gauss_table_generator()
{
    _fs_geom.reset();
    _fbo.reset();

    _gauss_program.reset();
    
    _dstate.reset();
    _rstate.reset();
    _bstate.reset();
    _sstate_linear.reset();
}

void
gauss_table_generator::generate_table(const gl::render_context_ptr& context,
                                       const gl::texture_2d_ptr&     ca_map,
                                       const gl::texture_2d_ptr&     gauss_map,
                                             int                     gauss_steps)
{
    using namespace scm::gl;
    using namespace scm::math;

    _fbo->attach_color_buffer(0, gauss_map);

    // !! gauss_steps = gauss_map->descriptor()._size.x;

    _gauss_program->uniform("gauss_steps", gauss_steps);
    
    {
        context_framebuffer_guard   fbg(context);
        context_state_objects_guard csg(context);
        context_program_guard       prg(context);
        context_texture_units_guard tug(context);

        context->set_depth_stencil_state(_dstate);
        context->set_rasterizer_state(_rstate);
        context->set_blend_state(_bstate);

        context->set_frame_buffer(_fbo);
        context->set_viewport(viewport(vec2ui::zero(), gauss_map->descriptor()._size));

        context->bind_texture(ca_map, _sstate_linear, 0);
        context->bind_program(_gauss_program);

        _fs_geom->draw(context, geometry::MODE_SOLID);
    }
}

bool
gauss_table_generator::reload_shader_resources(const gl::render_device_ptr& device)
{
    using namespace scm;
    using namespace scm::gl;
    using namespace scm::math;
    using boost::assign::list_of;

    program_ptr p = device->create_program(list_of(device->create_shader_from_file(STAGE_VERTEX_SHADER,   "../../../src/renderer/shaders/volume/gauss_generator.glslv"))
                                                  (device->create_shader_from_file(STAGE_FRAGMENT_SHADER, "../../../src/renderer/shaders/volume/gauss_generator.glslf")),
                                           "gauss_table_generator::gauss_program");
    if (!p) {
        return false;
    }
    else {
        _gauss_program = p;
        _gauss_program->uniform("mvp_matrix", make_ortho_matrix(0.0f, 1.0f, 0.0f, 1.0f, -1.0f, 1.0f));
    }

    return true;
}

} // namespace data
} // namespace scm
