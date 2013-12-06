
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

namespace scm {
namespace data {

	gauss_table_generator::gauss_table_generator(const gua::RenderContext& ctx)
{
    using namespace scm;
    using namespace scm::gl;
    using namespace scm::math;
    using boost::assign::list_of;

    // program objects ////////////////////////////////////////////////////////////////////////////
    //if (!reload_shader_resources(device)) {
    //    throw std::runtime_error("gauss_table_generator::gauss_table_generator(): error creating gauss generator progam.");
    //}

    // state objects //////////////////////////////////////////////////////////////////////////////
    //_bstate = device->create_blend_state(false, FUNC_ONE, FUNC_ZERO, FUNC_ONE, FUNC_ZERO);
    //_dstate = device->create_depth_stencil_state(false, false);
    //_rstate = device->create_rasterizer_state(FILL_SOLID, CULL_BACK, ORIENT_CCW);
    //_sstate_linear = device->create_sampler_state(FILTER_MIN_MAG_LINEAR, WRAP_CLAMP_TO_EDGE);

    // framebuffer objects ////////////////////////////////////////////////////////////////////////
	//FrameBufferObject* fbo(gbuffer_->get_eye_buffers()[i]);
	//_fbo = gua::FrameBufferObject();

	_gauss_program.create_from_files("H:\\guacamole\\git_gua\\guacamole\\resources\\shaders\\volume\\gauss_generator.glslv",
									 "H:\\guacamole\\git_gua\\guacamole\\resources\\shaders\\volume\\gauss_generator.glslf");

    //if (!_fbo) {
    //    throw std::runtime_error("gauss_table_generator::gauss_table_generator(): error creating framebuffer object.");
    //}

    // geometry ///////////////////////////////////////////////////////////////////////////////////
    //try {
    //    _fs_geom = make_shared<fullscreen_triangle>(device);
    //}
    //catch (const std::exception& e) {
    //    throw std::runtime_error(std::string("gauss_table_generator::gauss_table_generator(): error creating fs geometry objects: ") + e.what());
    //}
}

gauss_table_generator::~gauss_table_generator()
{
    //_fs_geom.reset();
    //_fbo.reset();

    //_gauss_program.reset();
    
    _dstate.reset();
    _rstate.reset();
    _bstate.reset();
    _sstate_linear.reset();
}

void
gauss_table_generator::generate_table(const gua::RenderContext& ctx,
										const std::shared_ptr<gua::Texture2D>     ca_map,
										const std::shared_ptr<gua::Texture2D>     gauss_map,
										int                     gauss_steps)
{
    using namespace scm::gl;
    using namespace scm::math;

	_fbo.attach_color_buffer(ctx, 0, gauss_map, 1, 0);
	_fbo.bind(ctx);
	
    // !! gauss_steps = gauss_map->descriptor()._size.x;

    //_gauss_program->uniform("gauss_steps", gauss_steps);
	_gauss_program.set_uniform(ctx, gauss_steps, "gauss_steps");
	_gauss_program.set_uniform(ctx, ca_map, "color_map");
    
    {
        context_framebuffer_guard   fbg(ctx.render_context);
        context_state_objects_guard csg(ctx.render_context);
        context_program_guard       prg(ctx.render_context);
        context_texture_units_guard tug(ctx.render_context);

		ctx.render_context->set_depth_stencil_state(_dstate);
		ctx.render_context->set_rasterizer_state(_rstate);
		ctx.render_context->set_blend_state(_bstate);
		
		//ctx.render_context->set_frame_buffer(_fbo);
		
		ctx.render_context->set_viewport(viewport(vec2ui::zero(), vec2ui(gauss_map->width(), gauss_map->height())));
		//ctx.render_context->bind_texture(ca_map, _sstate_linear, 0);
		//ctx.render_context->bind_program(_gauss_program);
		_gauss_program.use(ctx);
		_fullscreen_quad->draw(ctx.render_context);
		_gauss_program.unuse(ctx);
        //_fs_geom->draw(context, geometry::MODE_SOLID);
    }
	_fbo.unbind(ctx);

	std::cout << "blabla" << std::endl;

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
