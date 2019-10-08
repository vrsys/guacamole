#ifndef GUACAMOLE_NRP_PASS_HPP
#define GUACAMOLE_NRP_PASS_HPP

#include <gua/nrp/nrp_binder.hpp>
#include <gua/renderer/PipelinePass.hpp>

namespace gua
{
namespace nrp
{
class GUA_NRP_DLL NRPPassDescription : public PipelinePassDescription
{
  public:
    NRPPassDescription();

    std::shared_ptr<PipelinePassDescription> make_copy() const override;
    PipelinePass make_pass(RenderContext const &, SubstitutionMap &) override;

    friend class Pipeline;
};
} // namespace nrp
} // namespace gua

#endif // GUACAMOLE_NRP_PASS_HPP
