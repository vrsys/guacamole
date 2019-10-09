#include <gua/renderer/GBufferResponsibility.hpp>

#ifdef GUACAMOLE_ENABLE_VIRTUAL_TEXTURING

namespace gua
{
GBufferPreResponsibilityDescription::GBufferPreResponsibilityDescription()
{
    private_.name_ = "gbuffer_pre_responsibility";
    private_.type_ = PipelineResponsibilityPrivate::TYPE::PRE_RENDER;
    private_.fulfil_ = [](Pipeline& pipe) {
        RenderContext const& ctx(pipe.get_context());
        auto& target = *pipe.current_viewstate().target;
        bool write_depth = true;
        target.bind(ctx, write_depth);
        target.set_viewport(ctx);
    };
}
GBufferPostResponsibilityDescription::GBufferPostResponsibilityDescription()
{
    private_.name_ = "gbuffer_post_responsibility";
    private_.type_ = PipelineResponsibilityPrivate::TYPE::POST_RENDER;
    private_.fulfil_ = [](Pipeline& pipe) {
        RenderContext const& ctx(pipe.get_context());
        auto& target = *pipe.current_viewstate().target;
        target.unbind(ctx);
    };
}
} // namespace gua

#endif