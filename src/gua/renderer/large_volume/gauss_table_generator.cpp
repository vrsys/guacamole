
#include <gua/renderer/large_volume/gauss_table_generator.h>

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

#include <scm/gl_util/primitives/quad.h>

#include <gua/databases.hpp>

namespace scm {
namespace data {

	gauss_table_generator::gauss_table_generator(const gua::RenderContext& ctx)
        :_gauss_program(new gua::ShaderProgram)
{
    using namespace scm;
    using namespace scm::gl;
    using namespace scm::math;
    using boost::assign::list_of;


    // state objects //////////////////////////////////////////////////////////////////////////////
    _bstate = ctx.render_device->create_blend_state(false, FUNC_ONE, FUNC_ZERO, FUNC_ONE, FUNC_ZERO);
    _dstate = ctx.render_device->create_depth_stencil_state(false, false);
    _rstate = ctx.render_device->create_rasterizer_state(FILL_SOLID, CULL_BACK, ORIENT_CCW);
    _sstate_linear = ctx.render_device->create_sampler_state(FILTER_MIN_MAG_LINEAR, WRAP_CLAMP_TO_EDGE);

    // framebuffer objects ////////////////////////////////////////////////////////////////////////	
    _fbo = std::shared_ptr<gua::FrameBufferObject>(new gua::FrameBufferObject());
    
    if (!_fbo) {
        throw std::runtime_error("gauss_table_generator::gauss_table_generator(): error creating framebuffer object.");
    }

    // program objects ////////////////////////////////////////////////////////////////////////////
#if 1
	_gauss_program->create_from_files("H:\\guacamole\\git_gua\\guacamole\\resources\\shaders\\volume\\gauss_generator.glslv",
									 "H:\\guacamole\\git_gua\\guacamole\\resources\\shaders\\volume\\gauss_generator.glslf");
#else
	std::string gauss_vertex_shader(gua::Resources::lookup_shader(gua::Resources::shaders_volume_gauss_generator_glslv));
	std::string gauss_fragment_shader(gua::Resources::lookup_shader(gua::Resources::shaders_volume_gauss_generator_glslf));

	_gauss_program->create_from_sources(gauss_vertex_shader, gauss_fragment_shader);
#endif

    // geometry ///////////////////////////////////////////////////////////////////////////////////
    try {
        _fullscreen_quad = scm::gl::quad_geometry_ptr(new scm::gl::quad_geometry(ctx.render_device, math::vec2(0.f, 0.f), math::vec2(1.f, 1.f)));
    }
    catch (const std::exception& e) {
        throw std::runtime_error(std::string("gauss_table_generator::gauss_table_generator(): error creating fs geometry objects: ") + e.what());
    }
}

gauss_table_generator::~gauss_table_generator()
{
    //_fs_geom.reset();
    _fbo.reset();

    delete _gauss_program;
    
    _dstate.reset();
    _rstate.reset();
    _bstate.reset();
    _sstate_linear.reset();
}

void
gauss_table_generator::generate_table(const gua::RenderContext& ctx,
										const std::shared_ptr<gua::Texture2D>     ca_map,
										const std::shared_ptr<gua::Texture2D>     gauss_map,
										int                                         gauss_steps)
{
    using namespace scm::gl;
    using namespace scm::math;

    context_framebuffer_guard   fbg(ctx.render_context);
    context_state_objects_guard csg(ctx.render_context);
    context_program_guard       prg(ctx.render_context);
    context_texture_units_guard tug(ctx.render_context);

    ctx.render_context->set_depth_stencil_state(_dstate);
    ctx.render_context->set_rasterizer_state(_rstate);
    ctx.render_context->set_blend_state(_bstate);

	_fbo->attach_color_buffer(ctx, 0, gauss_map);
    
    _fbo->bind(ctx);    
    {        
        ctx.render_context->set_viewport(viewport(vec2ui::zero(), vec2ui(gauss_map->width(), gauss_map->height())));
        //_fbo->clear_color_buffers(ctx, gua::utils::Color3f(0.0f, 0.0f, 0.0f));

        _gauss_program->set_uniform(ctx, ca_map, "color_map");
        _gauss_program->set_uniform(ctx, gauss_steps, "gauss_steps");
        _gauss_program->set_uniform(ctx, scm::math::make_ortho_matrix(0.0f, 1.0f, 0.0f, 1.0f, -1.0f, 1.0f), "mvp_matrix");
		
		_gauss_program->use(ctx);
		    _fullscreen_quad->draw(ctx.render_context);
		_gauss_program->unuse(ctx);        
    }

	_fbo->unbind(ctx);	
}

//bool
//gauss_table_generator::reload_shader_resources(const gl::render_device_ptr& device)
//{
//    using namespace scm;
//    using namespace scm::gl;
//    using namespace scm::math;
//    using boost::assign::list_of;
//
//    program_ptr p = device->create_program(list_of(device->create_shader_from_file(STAGE_VERTEX_SHADER,   "../../../src/renderer/shaders/volume/gauss_generator.glslv"))
//                                                  (device->create_shader_from_file(STAGE_FRAGMENT_SHADER, "../../../src/renderer/shaders/volume/gauss_generator.glslf")),
//                                           "gauss_table_generator::gauss_program");
//    if (!p) {
//        return false;
//    }
//    else {
//        _gauss_program = p;
//        _gauss_program->uniform("mvp_matrix", make_ortho_matrix(0.0f, 1.0f, 0.0f, 1.0f, -1.0f, 1.0f));
//    }
//
//    return true;
//}

} // namespace data
} // namespace scm
