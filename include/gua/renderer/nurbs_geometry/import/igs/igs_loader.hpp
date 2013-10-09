/*******************************************************************************
*
* Copyright (C) 2007-2010 Bauhaus-Universitaet Weimar
*
*********************************************************************************
*
*  module     : igs_loader.hpp
*  project    : gua
*  description:
*
********************************************************************************/
#ifndef gua_IGS_LOADER_HPP
#define gua_IGS_LOADER_HPP

// header, system
#include <string>
#include <vector>
#include <memory>

// header, project

namespace gua {

// fwd decl
class TrimmedNurbsSurfaceObject;

///////////////////////////////////////////////////////////////////////////////
class igs_loader {
 public:  // c'tor / d'tor

  igs_loader();
  ~igs_loader();

 private:  // noncopyable

  igs_loader(igs_loader const& cpy);
  igs_loader& operator=(igs_loader const& cpy);

 public:  // methods

  bool load(std::string const& file,
            std::shared_ptr<TrimmedNurbsSurfaceObject> no);
  bool load(std::istream& is, std::shared_ptr<TrimmedNurbsSurfaceObject> no);
  //TrimmedNurbsSurfaceObject*          get     ( );

 private:  // members

  bool set_target_(std::shared_ptr<TrimmedNurbsSurfaceObject> no);

  std::string error_;
  std::shared_ptr<TrimmedNurbsSurfaceObject> result_;
};

}  // namespace gua

#endif  // gua_IGS_LOADER_HPP
