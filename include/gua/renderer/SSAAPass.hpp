/******************************************************************************
 * guacamole - delicious VR                                                   *
 *                                                                            *
 * Copyright: (c) 2011-2013 Bauhaus-Universität Weimar                        *
 * Contact:   felix.lauer@uni-weimar.de / simon.schneegans@uni-weimar.de      *
 *                                                                            *
 * This program is free software: you can redistribute it and/or modify it    *
 * under the terms of the GNU General Public License as published by the Free *
 * Software Foundation, either version 3 of the License, or (at your option)  *
 * any later version.                                                         *
 *                                                                            *
 * This program is distributed in the hope that it will be useful, but        *
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY *
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License   *
 * for more details.                                                          *
 *                                                                            *
 * You should have received a copy of the GNU General Public License along    *
 * with this program. If not, see <http://www.gnu.org/licenses/>.             *
 *                                                                            *
 ******************************************************************************/

#ifndef GUA_SSAA_PASS_HPP
#define GUA_SSAA_PASS_HPP

#include <gua/renderer/PipelinePass.hpp>

#include <memory>

namespace gua
{
class Pipeline;

class GUA_DLL SSAAPassDescription : public PipelinePassDescription
{
  public:
    enum class SSAAMode
    {
        FAST_FXAA = 0,
        FXAA311 = 1,
        DISABLED = 2
    };

    SSAAPassDescription();

    /////////////////////////////////////////////////////////////////////////////
    // SSAA mode selection
    /////////////////////////////////////////////////////////////////////////////
    SSAAPassDescription& mode(SSAAMode mode);
    SSAAMode mode() const;

    /////////////////////////////////////////////////////////////////////////////
    // FXAA 3.11 configuration
    /////////////////////////////////////////////////////////////////////////////

    // This can effect sharpness.
    // 1.00 - upper limit (softer)
    // 0.75 - default amount of filtering
    // 0.50 - lower limit (sharper, less sub-pixel aliasing removal)
    // 0.25 - almost off
    // 0.00 - completely off
    SSAAPassDescription& fxaa_quality_subpix(float intensity);
    float fxaa_quality_subpix() const;

    // The minimum amount of local contrast required to apply algorithm.
    // 0.333 - too little (faster)
    // 0.250 - low quality
    // 0.166 - default
    // 0.125 - high quality
    // 0.063 - overkill (slower)
    SSAAPassDescription& fxaa_edge_threshold(float intensity);
    float fxaa_edge_threshold() const;

    // Trims the algorithm from processing darks.
    // 0.0833 - upper limit (default, the start of visible unfiltered edges)
    // 0.0625 - high quality (faster)
    // 0.0312 - visible limit (slower)

    SSAAPassDescription& fxaa_threshold_min(float intensity);
    float fxaa_threshold_min() const;

    /////////////////////////////////////////////////////////////////////////////
    SSAAPassDescription& enable_pinhole_correction(bool enable);
    bool enable_pinhole_correction() const;

    std::shared_ptr<PipelinePassDescription> make_copy() const override;
    friend class Pipeline;

  protected:
    PipelinePass make_pass(RenderContext const&, SubstitutionMap&) override;
};

} // namespace gua

#endif // GUA_RESOLVE_PASS_HPP
