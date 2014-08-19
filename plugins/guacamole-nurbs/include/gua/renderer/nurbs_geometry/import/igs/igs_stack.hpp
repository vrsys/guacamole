/********************************************************************************
*
* Copyright (C) 2007-2010 Bauhaus-Universitaet Weimar
*
*********************************************************************************
*
*  module     : igs_stack.hpp
*  project    : gua
*  description:
*
********************************************************************************/
#ifndef gua_IGS_STACK_HPP
#define gua_IGS_STACK_HPP

#ifdef _MSC_VER
#pragma warning(disable : 4251)
#endif

// std includes
#include <vector>
#include <string>
#include <map>
#include <memory>

#include <boost/noncopyable.hpp>

// header, project

namespace gua {

// fwd decl
class TrimmedNurbsSurfaceObject;

////////////////////////////////////////////////////////////////////////////////
struct igs_globals {
  igs_globals();

  std::string para_delim;
  std::string record_delim;
  std::string sender_id;
  std::string file_name;
  std::string system_id;
  std::string preprocessor_version;
  int num_bits_integer;
  int magnitude_float;
  int signif_float;
  int magnitude_double;
  int signif_double;
  std::string receiver_id;
  double scale;
  int units_flag;
  std::string units_name;
  int line_weight;
  double line_units;
  std::string date;
  double min_resolution;
  double max_coord;
  std::string author;
  std::string organization;
  int version;
  int draft;
  std::string creation_time;
  std::string protocol;
};

////////////////////////////////////////////////////////////////////////////////
struct igs_data {
  // first line
  int entity_type;
  int parameter;
  int structure;
  int line_font;
  int level;
  int view;
  int matrix;
  int label;
  int status;

  // second line
  int entity_type2;
  int line_weight;
  int color;
  int parameter_line;
  int form;
  std::string reserved1;
  std::string reserved2;
  std::string entity_label;
  int entity_subscript;
};

////////////////////////////////////////////////////////////////////////////////
struct igs_parameter {
  igs_parameter();
  int index;
  std::map<int, double> numeric_values;
  std::map<int, std::string> string_values;
};

////////////////////////////////////////////////////////////////////////////////
struct igs_curve_on_surface {
  int surface;
  int bmap;       // curve or curve composite
  int cmap;       // curve or curve composite
  int preferred;  // preferred mapping 1 = bmap, 2 = cmap
};

struct igs_trim {
  int surface;
  bool inner;
  int outer_boundary;
  std::vector<int> composites;
};

////////////////////////////////////////////////////////////////////////////////
struct map_adapter {
  double operator()(std::pair<int, double> const& entry) {
    return entry.second;
  }
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
class igs_stack : public boost::noncopyable {
 public:

  igs_stack(std::shared_ptr<TrimmedNurbsSurfaceObject> target);
  ~igs_stack();

 private:

  igs_stack(igs_stack const& igs_stack);
  igs_stack& operator=(igs_stack const& rhs);

 public:  // methods

  // global evaluation
  void evaluateGlobals();
  int evalInt();
  double evalDouble();
  std::string evalString();
  void evalDelimiter();

  // data evaluation
  void insertDouble(std::string& s);
  void insertString(std::string& s);
  void insertInt(std::string& s);

  // clear stack
  void clear();

  // finish parameter entry
  void addParameterRecord();

  // finish data entry and put on data entry stack
  void addData();

  // finally create entities
  void createDataEntities();

  // entity creator's
  void createArc(igs_data const&, std::size_t index);
  void createCurve(igs_data const& data, std::size_t index);
  void createCurveComposite(igs_data const& data, std::size_t index);
  void createCurveOnSurface(igs_data const& data, std::size_t index);
  void createMaterial(igs_data const&, std::size_t index);
  void createMatrix(igs_data const& data, std::size_t index);
  void createLine(igs_data const& data, std::size_t index);
  void createSurface(igs_data const& data, std::size_t index);
  void createTrimmedSurface(igs_data const& data, std::size_t index);

  double convertString(double d, std::string& s);

  void sline_action();
  void gline_action();
  void dline_action(std::string const& str);
  void pline_action(std::string const& str);
  void tline_action();

  void float_action(double const& value);
  void string_action(std::string const& str);
  void pstring_action(std::string const& str);
  void int_action(int value);

 private:  // data member

  struct impl_t;
  impl_t* impl_;
};

}  // namespace gua

#endif  // gua_IGS_STACK_HPP
