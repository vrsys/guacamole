/********************************************************************************
*
* Copyright (C) 2007-2010 Bauhaus-Universitaet Weimar
*
*********************************************************************************
*
*  module     : copy_constructor_adapter.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_COPY_CONSTRUCTOR_ADAPTER_HPP
#define TML_COPY_CONSTRUCTOR_ADAPTER_HPP

// header, system

namespace tml {
namespace util {

template <typename source_type, typename target_type>
class copy_constructor_adapter {
 public:
  target_type operator()(source_type const& s) { return target_type(s); }
};

}  // namespace util
}  // namespace tml

#endif  // TML_COPY_CONSTRUCTOR_ADAPTER_HPP
