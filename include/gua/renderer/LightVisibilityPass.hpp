#ifndef GUA_LIGHT_VISIBILITY_PASS_HPP
#define GUA_LIGHT_VISIBILITY_PASS_HPP

#include <gua/renderer/PipelinePass.hpp>

#include <memory>

namespace gua
{
class Pipeline;

class GUA_DLL LightVisibilityPassDescription : public PipelinePassDescription
{
  public:
    enum RasterizationMode
    {
        AUTO = 0,
        SIMPLE = 1,
        CONSERVATIVE = 2,
        MULTISAMPLED_2 = 3,
        MULTISAMPLED_4 = 4,
        MULTISAMPLED_8 = 5,
        MULTISAMPLED_16 = 6,
        FULLSCREEN_FALLBACK = 7,
    };

    LightVisibilityPassDescription();

    LightVisibilityPassDescription& rasterization_mode(RasterizationMode const& mode)
    {
        rasterization_mode_ = mode;
        return *this;
    }
    RasterizationMode rasterization_mode() const { return rasterization_mode_; }

    LightVisibilityPassDescription& tile_power(int power)
    {
        tile_power_ = std::max(std::min(power, 7), 0);
        return *this;
    }
    unsigned tile_power() const { return tile_power_; }

    std::shared_ptr<PipelinePassDescription> make_copy() const override;
    friend class Pipeline;

  protected:
    PipelinePass make_pass(RenderContext const&, SubstitutionMap&) override;
    RasterizationMode rasterization_mode_ = AUTO;
    int tile_power_ = 2;
};

} // namespace gua

#endif // GUA_LIGHT_VISIBILITY_PASS_HPP
