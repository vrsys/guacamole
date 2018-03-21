#ifndef GUACAMOLE_NRP_PASS_HPP
#define GUACAMOLE_NRP_PASS_HPP

#include <gua/renderer/Lod.hpp>
#include <gua/renderer/PipelinePass.hpp>

namespace gua
{
class GUA_NRP_DLL NRPPassDescription : public PipelinePassDescription
{
  public:
    NRPPassDescription();

    std::shared_ptr<PipelinePassDescription> make_copy() const override;
    PipelinePass make_pass(RenderContext const &, SubstitutionMap &) override;

    friend class Pipeline;
};
}

#endif // GUACAMOLE_NRP_PASS_HPP
