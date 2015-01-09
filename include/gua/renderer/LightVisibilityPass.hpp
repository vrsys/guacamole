#ifndef GUA_LIGHT_VISIBILITY_PASS_HPP
#define GUA_LIGHT_VISIBILITY_PASS_HPP

#include <gua/renderer/PipelinePass.hpp>
#include <gua/renderer/TriMeshRessource.hpp>
#include <gua/renderer/ShadowMap.hpp>

#include <memory>

namespace gua {

class Pipeline;

class GUA_DLL LightVisibilityPassDescription : public PipelinePassDescription {
 public:
  LightVisibilityPassDescription();
  PipelinePassDescription* make_copy() const override;
  friend class Pipeline;
 protected:
  PipelinePass make_pass(RenderContext const&, SubstitutionMap const&) override;
};

}

#endif  // GUA_LIGHT_VISIBILITY_PASS_HPP
