#include <gua/nrp/nrp_pass.hpp>

namespace gua
{
namespace nrp
{
NRPPassDescription::NRPPassDescription() : PipelinePassDescription()
{
    private_.needs_color_buffer_as_input_ = false;
    private_.writes_only_color_buffer_ = false;
    private_.enable_for_shadows_ = true;
    private_.rendermode_ = RenderMode::Custom;
}
std::shared_ptr<PipelinePassDescription> NRPPassDescription::make_copy() const { return std::make_shared<NRPPassDescription>(*this); }
PipelinePass NRPPassDescription::make_pass(const RenderContext &ctx, SubstitutionMap &substitution_map)
{
    private_.process_ = [](PipelinePass &pass, PipelinePassDescription const &desc, Pipeline &pipe) { NRPBinder::get_instance().pre_render(); };

    PipelinePass pass{*this, ctx, substitution_map};
    return pass;
}
} // namespace nrp
} // namespace gua