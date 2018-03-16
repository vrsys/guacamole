#ifndef GUA_INCLUDE_NRP_HPP
#define GUA_INCLUDE_NRP_HPP

#include <gua/nrp/pagoda_binder.hpp>

#if defined (_MSC_VER)
#if defined (GUA_NRP_LIBRARY)
#define GUA_NRP_DLL __declspec( dllexport )
#else
#define GUA_NRP_DLL __declspec( dllimport )
#endif
#else
#define GUA_NRP_DLL
#endif // #if defined(_MSC_VER)

#endif  // GUA_INCLUDE_NRP_HPP
