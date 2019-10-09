#ifndef GUACAMOLE_GBUFFERRESPONSIBILITY_H
#define GUACAMOLE_GBUFFERRESPONSIBILITY_H

#include <gua/platform.hpp>
#include <gua/config.hpp>

#ifdef GUACAMOLE_ENABLE_VIRTUAL_TEXTURING

#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/PipelinePass.hpp>

#include <gua/renderer/VTRenderer.hpp>

#include <gua/virtual_texturing/VTBackend.hpp>

namespace gua
{
class GUA_DLL GBufferPreResponsibilityDescription : public PipelineResponsibilityDescription
{
  public:
    GBufferPreResponsibilityDescription();
};

class GUA_DLL GBufferPostResponsibilityDescription : public PipelineResponsibilityDescription
{
  public:
    GBufferPostResponsibilityDescription();
};

} // namespace gua

#endif

#endif // GUACAMOLE_GBUFFERRESPONSIBILITY_H
