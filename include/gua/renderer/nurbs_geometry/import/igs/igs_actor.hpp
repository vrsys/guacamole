/********************************************************************************
*
* Copyright (C) 2007-2010 Bauhaus-Universitaet Weimar
*
*********************************************************************************
*
*  module     : igs_actor.hpp
*  project    : gua
*  description:
*
********************************************************************************/
#ifndef gua_IGS_ACTOR_HPP
#define gua_IGS_ACTOR_HPP

// header, system
#include <functional>  // std::unary_function<>
#include <string>      // std::string
#include <iostream>    // std::cout
#include <stdexcept>

// header, dependencies
#include <boost/spirit/include/classic_core.hpp>

// header, project
#include <gua/renderer/nurbs_geometry/import/igs/igs_stack.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////
// parametric inserters : double and std::string
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
class FloatInserter : public std::unary_function<void, void> {
 private:
  igs_stack& stack_;

 public:
  FloatInserter(igs_stack& s) : stack_(s) {}

  template <typename T> void operator()(T value) const {
    stack_.float_action(value);
  }
};

////////////////////////////////////////////////////////////////////////////////
class StringInserter : public std::unary_function<void, void> {
 private:
  igs_stack& stack_;

 public:
  StringInserter(igs_stack& s) : stack_(s) {}

 public:
  template <typename iteratorT>
  void operator()(iteratorT begin, iteratorT end) const {
    std::string str(begin, end);
    stack_.string_action(str);
  }
};

////////////////////////////////////////////////////////////////////////////////
class ParameterStringInserter : public std::unary_function<void, void> {
 private:
  igs_stack& stack_;

 public:
  ParameterStringInserter(igs_stack& s) : stack_(s) {}

 public:
  template <typename iteratorT>
  void operator()(iteratorT begin, iteratorT end) const {
    if (std::count(begin, end, ' ') != std::distance(begin, end)) {
      std::string str(begin, end);
      stack_.pstring_action(str);
    }
  }
};

////////////////////////////////////////////////////////////////////////////////
class IntInserter : public std::unary_function<void, void> {
 private:
  igs_stack& stack_;

 public:
  IntInserter(igs_stack& s) : stack_(s) {}

 public:
  template <typename T> void operator()(T value) const {
    stack_.int_action(value);
  }
};

////////////////////////////////////////////////////////////////////////////////
class SLineActor : public std::unary_function<void, void> {
 private:
  igs_stack& stack_;

 public:
  SLineActor(igs_stack& s) : stack_(s) {}

 public:
  template <typename iteratorT>
  void operator()(iteratorT /*begin*/, iteratorT /*end*/) const {
    stack_.sline_action();
  }
};

////////////////////////////////////////////////////////////////////////////////
class GLineActor : public std::unary_function<void, void> {
 private:
  igs_stack& stack_;

 public:
  GLineActor(igs_stack& s) : stack_(s) {}

 public:
  template <typename iteratorT>
  void operator()(iteratorT /*begin*/, iteratorT /*end*/) const {
    stack_.gline_action();
  }
};

////////////////////////////////////////////////////////////////////////////////
class DLineActor : public std::unary_function<void, void> {
 private:
  igs_stack& stack_;

 public:
  DLineActor(igs_stack& s) : stack_(s) {}

 public:
  template <typename iteratorT>
  void operator()(iteratorT begin, iteratorT end) const {
    std::string str(begin, end);
    stack_.dline_action(str);
  }
};

////////////////////////////////////////////////////////////////////////////////
class PLineActor : public std::unary_function<void, void> {
 private:
  igs_stack& stack_;

 public:
  PLineActor(igs_stack& s) : stack_(s) {}

 public:
  template <typename iteratorT>
  void operator()(iteratorT begin, iteratorT end) const {
    std::string str(begin, end);
    stack_.pline_action(str);
  }
};

////////////////////////////////////////////////////////////////////////////////
class TLineActor : public std::unary_function<void, void> {
 private:
  igs_stack& stack_;

 public:
  TLineActor(igs_stack& s) : stack_(s) {}

 public:
  template <typename iteratorT>
  void operator()(iteratorT /*begin*/, iteratorT /* end*/) const {
    stack_.tline_action();
  }
};

}  // namespace gua

#endif  // gua_IGS_ACTOR_HPP
