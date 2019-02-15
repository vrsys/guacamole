#include <gua/nrp/nrp_pass.hpp>

namespace gua
{
namespace nrp
{
NRPPassDescription::NRPPassDescription() : PipelinePassDescription()
{
    needs_color_buffer_as_input_ = false;
    writes_only_color_buffer_ = false;
    enable_for_shadows_ = true;
    rendermode_ = RenderMode::Custom;
}
std::shared_ptr<PipelinePassDescription> NRPPassDescription::make_copy() const { return std::make_shared<NRPPassDescription>(*this); }
PipelinePass NRPPassDescription::make_pass(const RenderContext &ctx, SubstitutionMap &substitution_map)
{
    PipelinePass pass{*this, ctx, substitution_map};

    pass.process_ = [](PipelinePass &pass, PipelinePassDescription const &desc, Pipeline &pipe) { NRPBinder::get_instance().pre_render(); };

    return pass;
}
}
}