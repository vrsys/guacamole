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

// class header
#include <gua/renderer/PipelineDescription.hpp>

// guacamole headers
#include <gua/renderer/TriMeshPass.hpp>
#include <gua/renderer/EmissivePass.hpp>
#include <gua/renderer/PhysicallyBasedShadingPass.hpp>
#include <gua/renderer/LightVisibilityPass.hpp>
#include <gua/renderer/SSAOPass.hpp>
#include <gua/renderer/BBoxPass.hpp>
#include <gua/renderer/TexturedQuadPass.hpp>
#include <gua/renderer/TexturedScreenSpaceQuadPass.hpp>
#include <gua/renderer/TexturedScreenSpaceQuadPass.hpp>
#include <gua/renderer/ToneMappingPass.hpp>
#include <gua/renderer/BackgroundPass.hpp>
#include <gua/renderer/ResolvePass.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<PipelineDescription> PipelineDescription::make_default() {
  auto pipe(std::make_shared<PipelineDescription>());
  /*
  pipe->add_pass<TriMeshPassDescription>();
  pipe->add_pass<TexturedQuadPassDescription>();
  pipe->add_pass<EmissivePassDescription>();
  pipe->add_pass<PhysicallyBasedShadingPassDescription>();
  pipe->add_pass<BBoxPassDescription>();
  pipe->add_pass<BackgroundPassDescription>();
  pipe->add_pass<TexturedScreenSpaceQuadPassDescription>();
  pipe->add_pass<ToneMappingPassDescription>();
  */
  pipe->add_pass<TriMeshPassDescription>();
  pipe->add_pass<TexturedQuadPassDescription>();
  pipe->add_pass<LightVisibilityPassDescription>();
  pipe->add_pass<BBoxPassDescription>();
  pipe->add_pass<ResolvePassDescription>();
  pipe->add_pass<TexturedScreenSpaceQuadPassDescription>();
  pipe->add_pass<ToneMappingPassDescription>();

  return pipe;
}

////////////////////////////////////////////////////////////////////////////////

PipelineDescription::PipelineDescription(PipelineDescription const& other) {
  for (auto pass: other.passes_) {
    passes_.push_back(pass->make_copy());
  }
}

////////////////////////////////////////////////////////////////////////////////

PipelineDescription::~PipelineDescription() {
  for (auto pass: passes_) {
    delete pass;
  }
}

////////////////////////////////////////////////////////////////////////////////

std::vector<PipelinePassDescription*> const& PipelineDescription::get_all_passes() const {
  return passes_;
}

////////////////////////////////////////////////////////////////////////////////

bool PipelineDescription::operator==(PipelineDescription const& other) const {
  boost::shared_lock<boost::shared_mutex> lock(mutex_);
  
  if (passes_.size() != other.passes_.size()) {
    return false;
  }

  for (int i(0); i<passes_.size(); ++i) {
    if (typeid(*passes_[i]) != typeid(*other.passes_[i])) {
      return false;
    }
    if ((*passes_[i]) != (*other.passes_[i])) {
      return false;
    }
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////

bool PipelineDescription::operator!=(PipelineDescription const& other) const {
  return !(*this == other);
}

////////////////////////////////////////////////////////////////////////////////

PipelineDescription& PipelineDescription::operator=(PipelineDescription const& other) {
  for (auto pass: passes_) {
    delete pass;
  }

  passes_.clear();

  for (auto pass: other.passes_) {
    passes_.push_back(pass->make_copy());
  }

  return *this;
}

////////////////////////////////////////////////////////////////////////////////

}

