/********************************************************************************
*
* Copyright (C) 2007-2010 Bauhaus-Universitaet Weimar
*
*********************************************************************************
*
*  module     : igs_loader.cpp
*  project    : gpucast
*  description: part of inventor fileloader
*
********************************************************************************/

// header i/f
#include <gua/renderer/nurbs_geometry/import/igs/igs_loader.hpp>

// header, system
#include <cassert>
#include <fstream>
#include <stdexcept>

// header, project
#include <gua/renderer/nurbs_geometry/import/igs/igs_grammar.hpp>
#include <gua/renderer/nurbs_geometry/import/igs/igs_actor.hpp>
#include <gua/renderer/nurbs_geometry/import/igs/igs_stack.hpp>
#include <gua/renderer/nurbs_geometry/TrimmedNurbsSurfaceObject.hpp>

namespace gua {

// helper functor
class spacer {
 public:
  spacer(std::vector<char>& v);
  char operator()(char const& c);

 private:
  std::vector<char>& v_;
  std::size_t i_;
};

spacer::spacer(std::vector<char>& v) : v_(v), i_(0) {}

char spacer::operator()(char const& c) {
  if (i_ == 72) {
    v_.insert(v_.end() - 72, c);
    v_.insert(v_.end() - 72, ' ');

    v_.push_back(' ');
    v_.push_back('^');
    v_.push_back(' ');

    switch (c) {
      case 'D':

        // separate columns
        for (int i = 67; i > 0; i -= 8) {
          v_.insert(v_.end() - i, ' ');
        }
        break;
      case 'P':
        // separate last col
        v_.insert(v_.end() - 10, ' ');
        v_.insert(v_.end() - 10, ' ');
        v_.insert(v_.end() - 10, '^');
        v_.insert(v_.end() - 10, '^');
        break;
      default:
        break;
        // do nothing
    }
  }

  // secure space
  if (i_ == 73) {
    v_.push_back(' ');
  }

  if (c == '\n') {
    i_ = 0;
  } else {
    ++i_;
  }
  return c;
}

////////////////////////////////////////////////////////////////////////////////
igs_loader::igs_loader() : error_(), result_() {}

////////////////////////////////////////////////////////////////////////////////
igs_loader::~igs_loader() {}

////////////////////////////////////////////////////////////////////////////////
bool igs_loader::load(std::string const& file,
                      std::shared_ptr<TrimmedNurbsSurfaceObject> no) {
  // first check if target is allocated and set result pointer
  if (!set_target_(no)) {
    return false;
  }

  // try to open file
  std::ifstream fstr;
  fstr.open(file.c_str(), std::ios::in);

  // if file good parse stream
  if (fstr.good()) {
    return load(fstr, no);
  } else {
    return false;
  }
}

////////////////////////////////////////////////////////////////////////////////
bool igs_loader::load(std::istream& istr,
                      std::shared_ptr<TrimmedNurbsSurfaceObject> no) {
  istr.unsetf(std::ios::skipws);

  std::vector<char> tmp;
  std::transform(std::istream_iterator<char>(istr),
                 std::istream_iterator<char>(),
                 std::back_inserter(tmp),
                 spacer(tmp));

  // instance grammar and parse stream
  boost::spirit::classic::parse_info<std::vector<char>::iterator> info;
  info = boost::spirit::classic::parse(
      tmp.begin(), tmp.end(), igs_grammar(result_));

  error_ = std::string(info.stop, tmp.end());

  return (error_.length() > 1) ? false : true;
}

////////////////////////////////////////////////////////////////////////////////
bool igs_loader::set_target_(std::shared_ptr<TrimmedNurbsSurfaceObject> no) {
  if (no) {
    result_ = no;
    return true;
  } else {
    return false;
  }
}

////////////////////////////////////////////////////////////////////////////////
/*nurbssurfaceobject*
igs_loader::get()
{
  return result_;
}*/

}  // namespace gua
