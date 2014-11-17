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

#ifndef GUA_PIPELINE_DESCRIPTION_HPP
#define GUA_PIPELINE_DESCRIPTION_HPP

#include <gua/renderer/PipelinePass.hpp>
#include <gua/math.hpp>

#include <memory>

namespace gua {

class PipelineDescription {
 public:

  static PipelineDescription make_default();

  PipelineDescription() {}
  PipelineDescription(PipelineDescription const& other);
  
  virtual ~PipelineDescription();

  template<class T>
  T& add_pass() {
    T* t = new T();
    passes_.push_back(t);
    return *t;
  }

  std::vector<PipelinePassDescription*> const& get_all_passes() const;

  template<class T>
  std::vector<T*> get_passes() const {
    std::vector<T*> result;

    for (auto pass: passes_) {
      T* casted(dynamic_cast<T*>(pass));

      if (casted) {
        result.push_back(casted);
      }
    }

    return result;
  }

  template<class T>
  T& get_pass() const {

    for (auto pass: passes_) {
      T* casted(dynamic_cast<T*>(pass));

      if (casted) {
        return *casted;
      }
    }
  }

  bool operator==(PipelineDescription const& other) const;
  bool operator!=(PipelineDescription const& other) const;
  PipelineDescription& operator=(PipelineDescription const& other);
 
 private:
  std::vector<PipelinePassDescription*> passes_;
};

}

#endif  // GUA_PIPELINE_DESCRIPTION_HPP
