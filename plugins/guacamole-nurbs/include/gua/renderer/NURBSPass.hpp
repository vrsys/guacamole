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
#ifndef GUA_NURBSPASS_HPP
#define GUA_NURBSPASS_HPP

// guacamole headers
#include <gua/renderer/NURBS.hpp>
#include <gua/renderer/PipelinePass.hpp>

namespace gua
{
class GUA_NURBS_DLL NURBSPassDescription : public PipelinePassDescription
{
  public: // typedefs, enums
    friend class Pipeline;

    void enable_pretessellation(bool enable);
    bool enable_pretessellation() const;

  public:
    NURBSPassDescription();
    std::shared_ptr<PipelinePassDescription> make_copy() const override;
    PipelinePass make_pass(RenderContext const&, SubstitutionMap& substitution_map) override;

  private:
    bool _enable_pretessellation = true;
};

} // namespace gua

#endif // GUA_NURBSPASS_HPP
