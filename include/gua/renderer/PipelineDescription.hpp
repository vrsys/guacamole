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

class GUA_DLL PipelineDescription {
 public:

  static std::shared_ptr<PipelineDescription> make_default();

  PipelineDescription() {}
  PipelineDescription(PipelineDescription const& other);

  virtual ~PipelineDescription();

  template<class T>
  T& add_pass() {
    boost::upgrade_lock<boost::shared_mutex> lock(mutex_);
    boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
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
    throw std::runtime_error("PipelineDescription::get_pass(): pass not valid");
  }

  void set_enable_abuffer(bool value) {
    enable_abuffer_ = value;
  }

  bool get_enable_abuffer() const {
    return enable_abuffer_;
  }

  void set_abuffer_size(size_t value) {
    abuffer_size_ = value;
  }

  size_t get_abuffer_size() const {
    return abuffer_size_;
  }

  void set_blending_termination_threshold(float value) {
    blending_termination_threshold_ = std::max(std::min(value, 1.f), .5f);
  }

  float get_blending_termination_threshold() const {
    return blending_termination_threshold_;
  }

  void set_max_lights_count(int value) {
    max_lights_count_ = value;
  }

  int get_max_lights_count() const {
    return max_lights_count_;
  }

  void set_user_data(void* data) {
    user_data_ = data;
  }

  void* get_user_data() const {
    return user_data_;
  }

  bool operator==(PipelineDescription const& other) const;
  bool operator!=(PipelineDescription const& other) const;
  PipelineDescription& operator=(PipelineDescription const& other);

 private:
  std::vector<PipelinePassDescription*> passes_;
  void*  user_data_ = nullptr;
  bool   enable_abuffer_ = false;
  size_t abuffer_size_ = 800; // in MiB
  float  blending_termination_threshold_ = 0.99;
  int    max_lights_count_ = 128;
  mutable boost::shared_mutex mutex_;
};

}

#endif  // GUA_PIPELINE_DESCRIPTION_HPP
