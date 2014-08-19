/********************************************************************************
*
* Copyright (C) 2007-2010 Bauhaus-Universitaet Weimar
*
*********************************************************************************
*
*  module     : igs_stack.cpp
*  project    : gpucast
*  description: part of igs fileloader
*
********************************************************************************/

// header i/f
#include <gua/renderer/nurbs_geometry/import/igs/igs_stack.hpp>

// header, system
#include <cassert>
#include <iostream>
#include <stdexcept>
#include <sstream>
#include <vector>

#include <boost/shared_ptr.hpp>

// header, project
#include <gua/renderer/nurbs_geometry/tml/parametric/nurbscurve.hpp>
#include <gua/renderer/nurbs_geometry/tml/parametric/point.hpp>

#include <gua/renderer/nurbs_geometry/import/igs/igs_loader.hpp>
#include <gua/renderer/nurbs_geometry/TrimmedNurbsSurfaceObject.hpp>
#include <gua/renderer/nurbs_geometry/TrimmedNurbsSurface.hpp>

using namespace tml;

namespace gua {

///////////////////////////////////////////////////////////////////
struct igs_stack::impl_t {
  impl_t(std::shared_ptr<TrimmedNurbsSurfaceObject> no) : target_(no) {}

  // typedefs
  typedef TrimmedNurbsSurface surface_t;
  typedef TrimmedNurbsSurface::curve_type curve_t;

  // attributes
  std::shared_ptr<TrimmedNurbsSurfaceObject> target_;

  short dataline_;
  int para_startindex_;
  int para_linecount_;

  int current_paraline_;
  int current_parapos_;

  igs_parameter current_para_;

  // parametrics
  std::vector<double> double_;
  std::vector<std::string> string_;
  std::vector<int> int_;

  // data
  std::vector<igs_data> data_;
  std::map<int, igs_parameter> parameter_;

  std::map<int, surface_t> surfaces_;
  std::map<int, curve_t> curves_;
  std::map<int, std::vector<curve_t> > composites_;
  std::map<int, igs_curve_on_surface> curve_surfs_;
  std::map<int, igs_trim> trims_;

  // globals
  std::string globalstring_;
  igs_globals globals_;
};

igs_parameter::igs_parameter() : index(), numeric_values(), string_values() {}

igs_globals::igs_globals()
    : para_delim(","),
      record_delim(";"),
      sender_id(""),
      file_name(""),
      system_id(""),
      preprocessor_version(""),
      num_bits_integer(0),
      magnitude_float(0),
      signif_float(0),
      magnitude_double(0),
      signif_double(0),
      receiver_id(""),
      scale(1.0),
      units_flag(1),
      units_name(""),
      line_weight(1),
      line_units(),
      date(""),
      min_resolution(),
      max_coord(0.0),
      author("nullptr"),
      organization("nullptr"),
      version(3),
      draft(0),
      creation_time("nullptr"),
      protocol("nullptr") {}

////////////////////////////////////////////////////////////////////////////////
/* private */ igs_stack::igs_stack(
    std::shared_ptr<TrimmedNurbsSurfaceObject> target)
    : impl_(new impl_t(target)) {
  impl_->dataline_ = 0;
  impl_->para_startindex_ = 1;
  impl_->para_linecount_ = 0;
  impl_->current_paraline_ = 1;
  impl_->current_parapos_ = 0;
}

////////////////////////////////////////////////////////////////////////////////
igs_stack::~igs_stack() { delete impl_; }

////////////////////////////////////////////////////////////////////////////////
void igs_stack::evaluateGlobals() {
  try {
    if (isdigit(impl_->globalstring_[0])) {
      std::string delim = evalString();
      if (delim.length() == 1) {
        impl_->globals_.para_delim = delim;
      } else {
        std::cerr << "igs_stack::evaluateGlobals() : Non supported delimiter"
                  << std::endl;
        exit(0);
      }
    }
    evalDelimiter();

    if (impl_->globalstring_[0] != impl_->globals_.para_delim[0]) {
      std::string delim = evalString();
      if (delim.length() == 1) {
        impl_->globals_.record_delim = delim;
      } else {
        std::cerr << "igs_stack::evaluateGlobals() : Non supported delimiter"
                  << std::endl;
        exit(0);
      }
    }
    evalDelimiter();

    if (impl_->globalstring_[0] != impl_->globals_.para_delim[0]) {
      std::string str = evalString();
      impl_->globals_.sender_id = str;
      impl_->globals_.receiver_id = str;
    }
    evalDelimiter();

    if (impl_->globalstring_[0] != impl_->globals_.para_delim[0]) {
      impl_->globals_.file_name = evalString();
    }
    evalDelimiter();

    if (impl_->globalstring_[0] != impl_->globals_.para_delim[0]) {
      impl_->globals_.system_id = evalString();
    }
    evalDelimiter();

    if (impl_->globalstring_[0] != impl_->globals_.para_delim[0]) {
      impl_->globals_.preprocessor_version = evalString();
    }
    evalDelimiter();

    if (impl_->globalstring_[0] != impl_->globals_.para_delim[0]) {
      impl_->globals_.num_bits_integer = evalInt();
    }
    evalDelimiter();

    if (impl_->globalstring_[0] != impl_->globals_.para_delim[0]) {
      impl_->globals_.magnitude_float = evalInt();
    }
    evalDelimiter();

    if (impl_->globalstring_[0] != impl_->globals_.para_delim[0]) {
      impl_->globals_.signif_float = evalInt();
    }
    evalDelimiter();

    if (impl_->globalstring_[0] != impl_->globals_.para_delim[0]) {
      impl_->globals_.magnitude_double = evalInt();
    }
    evalDelimiter();

    if (impl_->globalstring_[0] != impl_->globals_.para_delim[0]) {
      impl_->globals_.signif_double = evalInt();
    }
    evalDelimiter();

    if (impl_->globalstring_[0] != impl_->globals_.para_delim[0]) {
      impl_->globals_.receiver_id = evalString();
    }
    evalDelimiter();

    if (impl_->globalstring_[0] != impl_->globals_.para_delim[0]) {
      impl_->globals_.scale = evalDouble();
    }
    evalDelimiter();

    if (impl_->globalstring_[0] != impl_->globals_.para_delim[0]) {
      impl_->globals_.units_flag = evalInt();
    }
    evalDelimiter();

    if (impl_->globalstring_[0] != impl_->globals_.para_delim[0]) {
      impl_->globals_.units_name = evalString();
    }
    evalDelimiter();

    if (impl_->globalstring_[0] != impl_->globals_.para_delim[0]) {
      impl_->globals_.line_weight = evalInt();
    }
    evalDelimiter();

    if (impl_->globalstring_[0] != impl_->globals_.para_delim[0]) {
      impl_->globals_.line_units = evalDouble();
    }
    evalDelimiter();

    if (impl_->globalstring_[0] != impl_->globals_.para_delim[0]) {
      impl_->globals_.date = evalString();
    }
    evalDelimiter();

    if (impl_->globalstring_[0] != impl_->globals_.para_delim[0]) {
      impl_->globals_.min_resolution = evalDouble();
    }
    evalDelimiter();

    if (impl_->globalstring_[0] != impl_->globals_.para_delim[0]) {
      impl_->globals_.max_coord = evalDouble();
    }
    evalDelimiter();

    if (impl_->globalstring_[0] != impl_->globals_.para_delim[0]) {
      impl_->globals_.author = evalString();
    }
    evalDelimiter();

    if (impl_->globalstring_[0] != impl_->globals_.para_delim[0]) {
      impl_->globals_.organization = evalString();
    }
    evalDelimiter();

    if (impl_->globalstring_[0] != impl_->globals_.para_delim[0]) {
      impl_->globals_.version = evalInt();
    }
    evalDelimiter();

    if (impl_->globalstring_[0] != impl_->globals_.para_delim[0]) {
      impl_->globals_.draft = evalInt();
    }
    evalDelimiter();

    if (impl_->globalstring_[0] != impl_->globals_.para_delim[0] &&
        impl_->globalstring_[0] != impl_->globals_.record_delim[0]) {
      impl_->globals_.creation_time = evalString();
    }
    evalDelimiter();

    if (impl_->globalstring_[0] != impl_->globals_.para_delim[0] &&
        impl_->globalstring_[0] != impl_->globals_.record_delim[0]) {
      impl_->globals_.protocol = evalString();
    }
    // cleanup
    impl_->string_.clear();
    impl_->int_.clear();
    impl_->double_.clear();

  }
  catch (...) {
    std::cout << "Warning: Invalid file header" << std::endl;
  }
}

////////////////////////////////////////////////////////////////////////////////
int igs_stack::evalInt() {
  std::size_t len_offset(0);
  while (isdigit(impl_->globalstring_[len_offset]) ||
         isspace(impl_->globalstring_[len_offset])) {
    ++len_offset;
  }

  int res = 0;
  if (len_offset > 0) {
    std::string nr(impl_->globalstring_.begin(),
                   impl_->globalstring_.begin() + len_offset);
    res = atoi(nr.c_str());
    impl_->globalstring_.erase(0, len_offset);
  } else {
    std::cerr << "igs_stack::evalInt(): Invalid string in file" << std::endl;
    exit(0);
  }

  return res;
}

////////////////////////////////////////////////////////////////////////////////
double igs_stack::evalDouble() {
  std::size_t len_offset(0);
  while (
      (impl_->globalstring_[len_offset] != impl_->globals_.para_delim[0]) &&
      (impl_->globalstring_[len_offset] != impl_->globals_.record_delim[0])) {
    ++len_offset;
  }

  double res;
  if (len_offset > 0) {
    std::string nr(impl_->globalstring_.begin(),
                   impl_->globalstring_.begin() + len_offset);
    res = atof(nr.c_str());
    impl_->globalstring_.erase(0, len_offset);
  } else {
    //std::cerr << "Error: igs_stack::evalDouble(): Invalid string in file\n";
    throw std::runtime_error("Invalid string in file");
  }

  return res;
}

////////////////////////////////////////////////////////////////////////////////
std::string igs_stack::evalString() {
  std::size_t char_count = evalInt();

  if (impl_->globalstring_[0] != 'H') {
    //std::cerr << "Error: igs_stack::evalDouble(): Invalid string format\n";
    throw std::runtime_error("Invalid string format");
  } else {
    impl_->globalstring_.erase(0, 1);  // erase 'H' delimiter
    std::string res(impl_->globalstring_.begin(),
                    impl_->globalstring_.begin() + char_count);
    impl_->globalstring_.erase(0, char_count);
    return res;
  }
}

////////////////////////////////////////////////////////////////////////////////
void igs_stack::evalDelimiter() {
  while (isspace(impl_->globalstring_[0])) {
    impl_->globalstring_.erase(0, 1);  // erase delimiter
  }

  if (impl_->globalstring_[0] != impl_->globals_.para_delim[0] &&
      impl_->globalstring_[0] != impl_->globals_.record_delim[0]) {
    //std::cerr << "Error: igs_stack::evalDouble(): Unsupported delimiter\n";
    throw std::runtime_error("Unsupported delimiter");
  } else {
    impl_->globalstring_.erase(0, 1);  // erase delimiter
  }
}

////////////////////////////////////////////////////////////////////////////////
void igs_stack::insertDouble(std::string& s) {
  impl_->double_.push_back(atof(s.c_str()));
}

////////////////////////////////////////////////////////////////////////////////
void igs_stack::insertString(std::string& s) { impl_->string_.push_back(s); }

////////////////////////////////////////////////////////////////////////////////
void igs_stack::insertInt(std::string& s) {
  // correct invalid data fields
  for (std::size_t i = 0; i < s.length(); ++i) {
    if (!isdigit(s[i])) {
      s[i] = '0';
    }
  }

  impl_->int_.push_back(atoi(s.c_str()));
}

////////////////////////////////////////////////////////////////////////////////
void igs_stack::addData() {
  // check stack

  if (impl_->string_.size() != 3 || impl_->int_.size() != 15) {
    throw std::runtime_error("Invalid data field");
  }

  igs_data igs;

  igs.entity_type = impl_->int_[0];
  igs.parameter = impl_->int_[1];
  igs.structure = impl_->int_[2];
  igs.line_font = impl_->int_[3];
  igs.level = impl_->int_[4];
  igs.view = impl_->int_[5];
  igs.matrix = impl_->int_[6];
  igs.label = impl_->int_[7];
  igs.status = impl_->int_[8];

  igs.entity_type2 = impl_->int_[9];
  igs.line_weight = impl_->int_[10];
  igs.color = impl_->int_[11];
  igs.parameter_line = impl_->int_[12];
  igs.form = impl_->int_[13];
  igs.reserved1 = impl_->string_[0];
  igs.reserved2 = impl_->string_[1];
  igs.entity_label = impl_->string_[2];
  igs.entity_subscript = impl_->int_[13];

  impl_->int_.clear();
  impl_->string_.clear();
  impl_->double_.clear();

  impl_->data_.push_back(igs);
}

////////////////////////////////////////////////////////////////////////////////
void igs_stack::clear() {
  // set defaults
  impl_->dataline_ = 0;
  impl_->para_startindex_ = 1;
  impl_->para_linecount_ = 0;
  impl_->current_paraline_ = 1;
  impl_->current_parapos_ = 0;
  impl_->globalstring_ = "";
  impl_->globals_ = igs_globals();

  impl_->current_para_ = igs_parameter();

  // clear stack
  impl_->double_.clear();
  impl_->string_.clear();
  impl_->int_.clear();
  impl_->data_.clear();
  impl_->parameter_.clear();

  // data
  impl_->surfaces_.clear();
  impl_->curves_.clear();
  impl_->composites_.clear();
  impl_->curve_surfs_.clear();
  impl_->trims_.clear();
}

////////////////////////////////////////////////////////////////////////////////
void igs_stack::addParameterRecord() {
  // get current line
  impl_->current_paraline_ = impl_->int_.back();
  impl_->current_para_.index = impl_->current_paraline_;

  impl_->parameter_
      .insert(std::make_pair(impl_->para_startindex_, impl_->current_para_));

  // clear current parameter data
  impl_->current_para_.numeric_values.clear();
  impl_->current_para_.string_values.clear();

  // reset pos counter
  impl_->current_parapos_ = 0;
  impl_->int_.clear();
  impl_->double_.clear();
  impl_->string_.clear();
}

////////////////////////////////////////////////////////////////////////////////
void igs_stack::createDataEntities() {
  std::size_t index = 0;

  for (std::vector<igs_data>::const_iterator de = impl_->data_.begin();
       de != impl_->data_.end();
       ++de, ++index) {

    // re-create line number of data entry
    std::size_t de_nr = 1 + index * 2;

    switch (de->entity_type) {
      case 100:
        createArc(*de, de_nr);  // Circular Arc
        break;
      case 102:
        createCurveComposite(*de, de_nr);  // Composite Curve Entity
        break;
      case 108:
        //std::cout << "Plane entity" << std::endl;
        break;
      case 110:
        createLine(*de, de_nr);  // create line, but as nurbscurve
        break;
      case 116:
        //std::cout << "Point entity" << std::endl;
        break;
      case 120:
        //std::cout << "Warning: Surface of Revolution not implemented yet" <<
        //std::endl;
        break;
      case 124:
        createMatrix(*de, de_nr);  // Transformation Matrix
        break;
      case 126:
        createCurve(*de, de_nr);  // Rational Curve
        break;
      case 128:
        createSurface(*de, de_nr);  // Rational Surface
        break;
      case 142:
        createCurveOnSurface(*de, de_nr);  // Curve on surface entity
        break;
      case 144:
        createTrimmedSurface(*de, de_nr);  // Trimmed surface entity
        break;
      case 314:
        createMaterial(*de, de_nr);  // Color entity
        break;
      case 404:
        //std::cout << "Drawing entity" << std::endl;
        break;
      case 406:
        //std::cout << "Property entity" << std::endl;
        break;
      case 410:
        //std::cout << "View entity" << std::endl;
        break;
      default:
        std::cout << "Warning: Non-implemented entity type" << de->entity_type
                  << std::endl;
    }
  }

  try {
    // apply trims
    for (std::map<int, igs_trim>::const_iterator i = impl_->trims_.begin();
         i != impl_->trims_.end();
         ++i) {

      if (impl_->surfaces_.count(i->second.surface)) {
        // set trim type
        if (i->second.inner) {
          impl_->surfaces_[i->second.surface].trimtype(true);
        } else {
          impl_->surfaces_[i->second.surface].trimtype(false);
        }
        // outer boundary
        std::size_t id = i->second.outer_boundary;

        if (id > 0) {
          // outer boundary is composite
          std::vector<nurbscurve2d> const& outer = impl_->composites_[(int) id];
          if (outer.size() > 0) {
            impl_->surfaces_[i->second.surface].add(outer);
          } else {
            // outer boundary is curve
            if (impl_->curves_.count((int) id) > 0) {
              std::vector<nurbscurve2d> loop;
              loop.push_back(impl_->curves_[(int) id]);
              impl_->surfaces_[i->second.surface].add(loop);
            } else {
              throw std::runtime_error(
                  "Warning: Non-supported type for outer boundary");
            }
          }
        }

        // inner boundaries
        for (std::size_t t = 0; t < i->second.composites.size(); ++t) {
          std::size_t inner_id = i->second.composites[t];

          // inner loop is curve
          if (impl_->curves_.count((int) inner_id)) {
            std::vector<nurbscurve2d> loop;
            loop.push_back(impl_->curves_[(int) inner_id]);
            impl_->surfaces_[i->second.surface].add(loop);
          } else {

            // inner loop is curve composite
            if (impl_->composites_.count((int) inner_id)) {
              impl_->surfaces_[i->second.surface]
                  .add(impl_->composites_[int(inner_id)]);
            } else {
              throw std::runtime_error(
                  "Warning: Trim parametric not implemented or not defined");
            }
          }
        }
      } else {
        throw std::runtime_error(
            "Warning: Surface to be trimmed not implemented or incorrect");
      }
    }
  }  // try
  catch (std::runtime_error const & s) {
    std::cout << s.what() << std::endl;
  }

  // insert all geometry into the scene
  for (std::map<int, TrimmedNurbsSurface>::const_iterator i =
           impl_->surfaces_.begin();
       i != impl_->surfaces_.end();
       ++i) {
    impl_->target_->add(i->second);
  }
}

////////////////////////////////////////////////////////////////////////////////
void igs_stack::createArc(igs_data const& data, std::size_t /*index*/) {
  // convert circular arc into rational curve
  boost::shared_ptr<nurbscurve2d> nc(new nurbscurve2d);
  igs_parameter& p = impl_->parameter_[data.parameter];

  // z offset
  // double zt = p.numeric_values[1];

  // center point
  point2d center(p.numeric_values[2], p.numeric_values[3], 1.0);
  point2d arcbeg(p.numeric_values[4], p.numeric_values[5], 1.0);
  point2d arcend(p.numeric_values[6], p.numeric_values[7], 1.0);

  // TODO: implement correct!
  std::vector<point2d> points;
  points.push_back(arcbeg);
  points.push_back(arcend);
  std::vector<double> kv(2, 0.0);
  kv.insert(kv.end(), 2, 1.0);

  nc->degree(1);
  nc->set_knotvector(kv.begin(), kv.end());
  nc->set_points(points.begin(), points.end());

}

////////////////////////////////////////////////////////////////////////////////
void igs_stack::createCurve(igs_data const& data, std::size_t index) {
  nurbscurve2d nc;
  igs_parameter& p = impl_->parameter_[data.parameter];

  // # of points
  std::size_t nr_cp = (std::size_t) p.numeric_values[1] + 1;

  // degree in u,v
  nc.degree((std::size_t) p.numeric_values[2]);

  // 3 : planar/non-planar -> not necessary

  // 4 : open/closed -> not necessary

  bool rational = bool(p.numeric_values[5] > 0.5);
  std::size_t coords = 3;  // default
  switch (rational) {
    case 0:
      coords = 2;
      break;
    case 1:
      coords = 3;
      break;
  }
  // 6 : periodic -> not necessary

  // knots
  std::size_t os = 7;  // data offset
  std::size_t kv = nc.order() + nr_cp;
  std::map<int, double>::iterator first = p.numeric_values.begin();
  std::map<int, double>::iterator last = p.numeric_values.begin();

  std::advance(first, os);
  std::advance(last, os + kv);
  std::vector<double> tmp;

  for (std::map<int, double>::iterator it = first; it != last; ++it) {
    if (p.string_values.count(it->first)) {
      tmp.push_back(convertString(it->second, p.string_values[it->first]));
    } else {
      tmp.push_back(it->second);
    }
  }
  nc.set_knotvector(tmp.begin(), tmp.end());

  // weights
  std::advance(first, kv);
  std::advance(last, nr_cp);
  std::vector<point2d> points(nr_cp);
  std::vector<point2d>::iterator ins(points.begin());
  for (std::map<int, double>::const_iterator i = first; i != last; ++i, ++ins) {
    ins->weight(i->second);
  }

  // control points
  std::size_t idx1 = os + kv + nr_cp;
  std::size_t idx2 = os + kv + nr_cp + nr_cp * coords;
  std::size_t coord = 0;
  ins = points.begin();

  for (std::size_t i = idx1; i < idx2; ++i, ++coord) {
    // go to next point
    if (coord > 0 && coord % coords == 0) {
      ++ins;
    }

    // if there's a string in value
    if (p.string_values.count((int) i)) {
      if (coord % coords != 2) {
        // try to recover string to double
        (*ins)[coord % coords] =
            convertString(p.numeric_values[(int) i], p.string_values[(int) i]);
      }
    } else {
      if (coord % coords != 2) {
        (*ins)[coord % coords] = p.numeric_values[(int) i];
      }
    }
  }

  nc.set_points(points.begin(), points.end());
  impl_->curves_.insert(std::make_pair(int(index), nc));
}

////////////////////////////////////////////////////////////////////////////////
void igs_stack::createCurveComposite(igs_data const& data, std::size_t index) {
  std::vector<nurbscurve2d> cc;
  igs_parameter& p = impl_->parameter_[data.parameter];

  std::size_t curve_count = (std::size_t) p.numeric_values[1];
  for (std::size_t i = 2; i < 2 + curve_count; ++i) {
    std::size_t curve_index = (std::size_t) p.numeric_values[(int) i];
    if (impl_->curves_.count((int) curve_index)) {
      cc.push_back(impl_->curves_[int(curve_index)]);
    } else {
      throw std::runtime_error("Curve entity not implemented yet");
    }
  }

  impl_->composites_.insert(std::make_pair(int(index), cc));
}

////////////////////////////////////////////////////////////////////////////////
void igs_stack::createCurveOnSurface(igs_data const& data, std::size_t index) {
  igs_curve_on_surface cos;
  igs_parameter& p = impl_->parameter_[data.parameter];

  cos.surface = (int) p.numeric_values[2];
  cos.bmap = (int) p.numeric_values[3];
  cos.cmap = (int) p.numeric_values[4];
  cos.preferred = (int) p.numeric_values[5];

  //std::cout << "cos : " << cos.bmap << " - " << cos.cmap << " - "  <<
  //cos.preferred << std::endl;

  impl_->curve_surfs_.insert(std::make_pair(int(index), cos));
}

////////////////////////////////////////////////////////////////////////////////
void igs_stack::createLine(igs_data const& data, std::size_t index) {
  nurbscurve2d nc;
  igs_parameter& p = impl_->parameter_[data.parameter];

  std::vector<point2d> points(2);
  for (std::size_t i = 1; i < 7; ++i) {
    std::size_t pt_index = i / 4;
    std::size_t coord = (i - 1) % 3;

    if (coord <= 1)  // only x/y or u/v coordinate
        {
      // if there's a string in value
      if (p.string_values.count((int) i)) {
        // try to recover string to double
        points[pt_index][coord] =
            convertString(p.numeric_values[(int) i], p.string_values[(int) i]);
      } else {
        points[pt_index][coord] = p.numeric_values[(int) i];
      }
    }

    // check weight
    if (points[pt_index].weight() <= 0.0) {
      points[pt_index].weight(1.0);
    }
  }

  std::vector<double> kv;
  kv.insert(kv.end(), 2, 0.0);
  kv.insert(kv.end(), 2, 1.0);
  nc.degree(1);
  nc.set_knotvector(kv.begin(), kv.end());
  nc.set_points(points.begin(), points.end());

  impl_->curves_.insert(std::make_pair(int(index), nc));
}

////////////////////////////////////////////////////////////////////////////////
void igs_stack::createMaterial(igs_data const& data, std::size_t index) {
  /*
  glpp::material m;
  igs_parameter& p = impl_->parameter_[data.parameter];

  glpp::vec3d color(p.numeric_values[1], p.numeric_values[2],
p.numeric_values[3]);
  m.diffuse = glpp::vec3f(float(color[0]), float(color[1]), float(color[2]));
  m.ambient = glpp::vec3f(float(color[0]), float(color[1]), float(color[2]));

  impl_->materials_.insert(std::make_pair(int(index), m));
  */
}

////////////////////////////////////////////////////////////////////////////////
void igs_stack::createMatrix(igs_data const& data, std::size_t index) {
  /*
  //nurbscurve<double3_t> nc;
  igs_parameter& p = impl_->parameter_[data.parameter];

  glpp::matrix4x4<double> m;
  // scale and rotation
  m[0] = p.numeric_values[1];
  m[4] = p.numeric_values[2];
  m[8] = p.numeric_values[3];

  m[1] = p.numeric_values[5];
  m[5] = p.numeric_values[6];
  m[9] = p.numeric_values[7];

  m[2] = p.numeric_values[9];
  m[6] = p.numeric_values[10];
  m[10] = p.numeric_values[11];

  m[15] = 1.0;

  // translation
  glpp::vec3d trans(p.numeric_values[4], p.numeric_values[8],
p.numeric_values[12]);
  glpp::matrix4x4<double> result = m * glpp::make_translation(trans[0],
trans[1], trans[2]);

  impl_->matrices_.insert(std::make_pair(int(index), result));
  */
}

////////////////////////////////////////////////////////////////////////////////
void igs_stack::createSurface(igs_data const& data, std::size_t index) {
  TrimmedNurbsSurface ns;
  igs_parameter& p = impl_->parameter_[data.parameter];

  // # of points in u,v
  ns.numberofpoints_u(std::size_t(p.numeric_values[1] + 1));
  ns.numberofpoints_v(std::size_t(p.numeric_values[2] + 1));

  // degree in u,v
  ns.degree_u((std::size_t) p.numeric_values[3]);
  ns.degree_v((std::size_t) p.numeric_values[4]);

  // 5-6 : closed in u,v -> not necessary yet

  // 7 : rational or not -> not explicitly necessary

  // 8-9 : periodic in u,v -> not necessary

  // knots u,v
  std::size_t os = 10;  // data offset
  std::size_t kv_u = ns.order_u() + ns.numberofpoints_u();
  std::size_t kv_v = ns.order_v() + ns.numberofpoints_v();
  std::map<int, double>::iterator first = p.numeric_values.begin();
  std::map<int, double>::iterator last = p.numeric_values.begin();

  std::advance(first, os);
  std::advance(last, os + kv_u);
  std::vector<double> tmp;
  for (std::map<int, double>::iterator it = first; it != last; ++it) {
    if (p.string_values.count(it->first)) {
      tmp.push_back(convertString(it->second, p.string_values[it->first]));
    } else {
      tmp.push_back(it->second);
    }
  }
  ns.knotvector_u(tmp.begin(), tmp.end());

  std::advance(first, kv_u);
  std::advance(last, kv_v);
  tmp.clear();

  for (std::map<int, double>::iterator it = first; it != last; ++it) {
    if (p.string_values.count(it->first)) {
      tmp.push_back(convertString(it->second, p.string_values[it->first]));
    } else {
      tmp.push_back(it->second);
    }
  }
  ns.knotvector_v(tmp.begin(), tmp.end());

  // weights
  std::advance(first, kv_v);
  std::advance(last, ns.numberofpoints_u() * ns.numberofpoints_v());
  std::vector<point3d> points(ns.numberofpoints_u() * ns.numberofpoints_v());
  std::vector<point3d>::iterator ins(points.begin());

  // control points
  std::size_t coords = 3;  // x,y,z
  std::size_t nr_cp = ns.numberofpoints_u() * ns.numberofpoints_v();
  std::size_t idx1 = os + kv_u + kv_v + nr_cp;
  std::size_t idx2 = os + kv_u + kv_v + nr_cp + nr_cp * coords;
  std::size_t coord = 0;
  ins = points.begin();

  for (std::size_t i = idx1; i < idx2 && first != last; ++i, ++coord) {
    // go to next point
    if (coord > 0 && coord % coords == 0) {
      ++ins;
      ++first;
    }

    // if entry in numeric values exists
    if (!p.string_values.count((int) i)) {
      (*ins)[coord % coords] = p.numeric_values[(int) i];
    } else {
      (*ins)[coord % coords] =
          convertString(p.numeric_values[(int) i], p.string_values[(int) i]);
    }
    // set weight
    ins->weight(first->second);
  }

  // parameter space limits -> not necessary yet (already given by knots)
  ns.set_points(points.begin(), points.end());
  impl_->surfaces_.insert(std::make_pair(int(index), ns));
}

////////////////////////////////////////////////////////////////////////////////
void igs_stack::createTrimmedSurface(igs_data const& data, std::size_t index) {
  igs_trim trim;
  igs_parameter& p = impl_->parameter_[data.parameter];

  try {
    trim.surface = int(p.numeric_values[1]);
    if (impl_->surfaces_.count(trim.surface)) {

      trim.inner = bool(p.numeric_values[2] > 0.5);
      std::size_t cos_cnt = (std::size_t) p.numeric_values[3];

      // outer boundary
      if (0 != (std::size_t) p.numeric_values[4]) {
        igs_curve_on_surface const& cos =
            impl_->curve_surfs_[int(p.numeric_values[4])];

        // get composite
        std::size_t composite_index;
        switch (cos.preferred) {
          case 1:
            composite_index = cos.bmap;
            break;
          case 2:
            composite_index = cos.cmap;
            break;
          case 3:
            composite_index = cos.bmap;
            break;
          case 0:
            composite_index = cos.bmap;
            break;
          default:
            throw std::runtime_error("Warning: Unsupported trimming type");
        }
        trim.outer_boundary = (int) composite_index;
      }

      // all boundaries
      for (std::size_t i = 5; i < 5 + cos_cnt; ++i) {
        //std::cout << "inner bounds : " << cos_cnt << std::endl;
        if (impl_->curve_surfs_.count((int) p.numeric_values[(int) i])) {
          igs_curve_on_surface const& cos =
              impl_->curve_surfs_[(int) p.numeric_values[(int) i]];

          // get composite
          std::size_t composite_index;
          switch (cos.preferred) {
            case 1:
              composite_index = cos.bmap;
              break;
            case 2:
              composite_index = cos.cmap;
              break;
            case 3:
              composite_index = cos.bmap;
              break;
            case 0:
              composite_index = cos.bmap;
              break;
            default:
              throw std::runtime_error("Warning: Unsupported trimming type");
          }
          trim.composites.push_back((int) composite_index);
        } else {
          throw std::runtime_error("Warning: Curve on Surface cannot be found");
        }
      }

    } else {
      throw std::runtime_error(
          "Warning: Surface to be trimmed is not supported yet");
    }

    impl_->trims_.insert(std::make_pair(int(index), trim));
  }
  catch (std::runtime_error const & s) {
    std::cout << s.what() << std::endl;
  }
}

////////////////////////////////////////////////////////////////////////////////
double igs_stack::convertString(double d, std::string& s) {
  std::size_t pos = s.find('D');
  if (pos < s.length()) {
    s[pos] = 'E';
  }

  std::stringstream ss;
  ss << d << s;
  std::string tmp;
  ss >> tmp;

  return atof(tmp.c_str());
}

////////////////////////////////////////////////////////////////////////////////
void igs_stack::sline_action() {
  if (impl_->string_.back().size() != 73) {
    throw std::runtime_error("Invalid file format");
  } else {
    impl_->string_.back().erase(73, 1);
  }
  impl_->string_.clear();
}

////////////////////////////////////////////////////////////////////////////////
void igs_stack::gline_action() {
  std::string line = impl_->string_.back();

  if (line.length() != 73) {
    throw std::runtime_error("Invalid file format");
  } else {
    line.erase(72, 1);

    if (line.size() > 0) {
      if (line.at(line.size() - 1) != ';') {
        impl_->globalstring_ += line;
      } else {
        impl_->globalstring_ += line;
        evaluateGlobals();
      }
    }
    // evaluate string
  }
  impl_->string_.clear();
}

////////////////////////////////////////////////////////////////////////////////
void igs_stack::dline_action(std::string const& str) {
  const std::size_t line_offset = 1;  // D_
  const std::size_t width = 9;

  std::string::const_iterator begin = str.begin();
  //  std::string::const_iterator end = str.end();

  switch (impl_->dataline_) {

    // first data line
    case 0:
      for (unsigned i = 0; i < 9; ++i) {
        std::string s(line_offset + begin + i * width,
                      line_offset + begin + (i + 1) * width);
        insertInt(s);
      }

      impl_->dataline_ = 1;
      break;

    // second data line
    case 1:

      for (unsigned i = 0; i < 5; ++i) {
        std::string s(line_offset + begin + i * width,
                      line_offset + begin + (i + 1) * width);
        insertInt(s);
      }

      for (unsigned i = 5; i < 8; ++i) {
        std::string s(line_offset + begin + i * width,
                      line_offset + begin + (i + 1) * width);
        insertString(s);
      }

      for (unsigned i = 8; i < 9; ++i) {
        std::string s(line_offset + begin + i * width,
                      line_offset + begin + (i + 1) * width);
        insertInt(s);
      }

      addData();

      impl_->dataline_ = 0;
      break;
  }
  impl_->double_.clear();
}

////////////////////////////////////////////////////////////////////////////////
void igs_stack::pline_action(std::string const& str) {
  ++impl_->para_linecount_;

  if (str.find(impl_->globals_.record_delim) < str.size()) {
    addParameterRecord();
    impl_->para_startindex_ += impl_->para_linecount_;
    impl_->para_linecount_ = 0;
  }
}

////////////////////////////////////////////////////////////////////////////////
void igs_stack::tline_action() { createDataEntities(); }

////////////////////////////////////////////////////////////////////////////////
void igs_stack::float_action(double const& value) {
  impl_->current_para_.numeric_values
      .insert(std::make_pair(impl_->current_parapos_, value));
  ++impl_->current_parapos_;
}

////////////////////////////////////////////////////////////////////////////////
void igs_stack::string_action(std::string const& str) {
  impl_->string_.push_back(str);  // pushback on string stack
}

////////////////////////////////////////////////////////////////////////////////
void igs_stack::pstring_action(std::string const& str) {
  // set back because of string length value
  --impl_->current_parapos_;

  // add string for current index
  impl_->current_para_.string_values
      .insert(std::make_pair(impl_->current_parapos_, str));

  // increase pos pointer
  ++impl_->current_parapos_;
}

////////////////////////////////////////////////////////////////////////////////
void igs_stack::int_action(int value) { impl_->int_.push_back(value); }

}  // namespace gua
