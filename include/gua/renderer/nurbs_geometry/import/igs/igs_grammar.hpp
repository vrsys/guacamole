/********************************************************************************
*
* Copyright (C) 2007-2010 Bauhaus-Universitaet Weimar
*
*********************************************************************************
*
*  module     : igs_grammar.hpp
*  project    : gua
*  description:
*
********************************************************************************/
#ifndef gua_IGS_GRAMMAR_HPP
#define gua_IGS_GRAMMAR_HPP

// header, system
#include <string>      // std::string
#include <iostream>    // std::cout
#include <functional>  // std::unary_function
#include <sstream>     // std::istringstream
#include <memory>

// header, dependencies
#include <boost/spirit/include/classic_core.hpp>

// header, project
#include <gua/renderer/nurbs_geometry/import/igs/igs_actor.hpp>
#include <gua/renderer/nurbs_geometry/import/igs/igs_loader.hpp>
#include <gua/renderer/nurbs_geometry/import/igs/igs_stack.hpp>

namespace gua {

class TrimmedNurbsSurfaceObject;

class igs_grammar : public boost::spirit::classic::grammar<igs_grammar> {
 public:  // typedefs, namespaces, enums

  std::shared_ptr<TrimmedNurbsSurfaceObject> target_;

 public:  // c'tor / d'tor

  igs_grammar(std::shared_ptr<TrimmedNurbsSurfaceObject> no) : target_(no) {}

  ~igs_grammar() {}

 public:  // definition

  template <typename ScannerT> class definition {
   public:
    definition(igs_grammar const& self) : stack_(self.target_) {
      using namespace boost::spirit::classic;
      // types
      int1_p = *blank_p >> int_p >> *blank_p;

      ////////////////////////////////////////////////////////////////////////////////
      string_p =
          +((digit_p | alpha_p | ch_p('_') | ch_p('.') | ch_p('-') | ch_p('/') |
             ch_p(',') | ch_p(';') | ch_p('*') | ch_p('=') | ch_p(':') |
             ch_p('>') | ch_p('<') | ch_p('+') | ch_p('(') | ch_p(')') |
             ch_p(39) | ch_p('ö') | ch_p('ü') | ch_p('ä') | ch_p('#') |
             ch_p('!') | ch_p('$') | ch_p('%') | ch_p('&') | ch_p('@') |
             ch_p('~') | ch_p('`') | ch_p('\\') | ch_p('~')));

      string_wo_comma_p =
          +((digit_p | alpha_p | ch_p('_') | ch_p('.') | ch_p('-') | ch_p('/') |
             ch_p('*') | ch_p('=') | ch_p(':') | ch_p('>') | ch_p('<') |
             ch_p('+') | ch_p('(') | ch_p(')') | ch_p(39) | ch_p('ö') |
             ch_p('ü') | ch_p('ä') | ch_p('#') | ch_p('!') | ch_p('$') |
             ch_p('%') | ch_p('&') | ch_p('@') | ch_p('~') | ch_p('`') |
             ch_p('\\') | ch_p('~')));

      variable_p = +(string_p | blank_p);

      /////////////////////////////////////////////////////////////////////////////////
      lineS_p =
          ch_p('S') >> blank_p >> variable_p[StringInserter(stack_)] >>
          ch_p('^') >> +blank_p >> ch_p('S') >> +blank_p >> int_p >> *space_p;

      lineG_p =
          ch_p('G') >> blank_p >> +variable_p[StringInserter(stack_)] >>
          ch_p('^') >> +blank_p >> ch_p('G') >> +blank_p >> int_p >> *space_p;

      lineT_p = ch_p('T') >> +(string_p | blank_p) >> ch_p('^') >> +blank_p >>
                ch_p('T') >> +blank_p >> int_p >> *space_p;
      ///////////////////////////////////////////////////////////////////////////////
      lineD_p = dataline_p >> +blank_p >> int_p >> *space_p;
      dataline_p = ch_p('D') >> blank_p >> +(int1_p | (string_p >> blank_p)) >>
                   ch_p('^') >> +blank_p >> ch_p('D');

      ////////////////////////////////////////////////////////////////////////////////
      lineP_p = paraline_p >> +blank_p >> int_p >> *space_p;

      paraterm_p = +(string_wo_comma_p | +blank_p);
      paraline_p =
          ch_p('P') >> +blank_p >>
          +((real_p[FloatInserter(stack_)] |
             paraterm_p[ParameterStringInserter(stack_)]) >>
            (ch_p(',') | ch_p(';') | ch_p('H') | *blank_p)) >> !ch_p('^') >>
          ch_p('^') >> *blank_p >> int_p[IntInserter(stack_)] >> +blank_p >>
          ch_p('^') >> +blank_p >> ch_p('P');
      // complete file
      igsfile_p =
          +lineS_p[SLineActor(stack_)] >> +lineG_p[GLineActor(stack_)] >>
          +lineD_p[DLineActor(stack_)] >> +lineP_p[PLineActor(stack_)] >>
          +lineT_p[TLineActor(stack_)];
    }

    // double types
    boost::spirit::classic::rule<ScannerT> int1_p, uint_p;
    boost::spirit::classic::rule<ScannerT> string_p,
        variable_p,
        string_wo_comma_p;

    boost::spirit::classic::rule<ScannerT> igsfile_p;  // complete file

    // lines
    boost::spirit::classic::rule<ScannerT> paraline_p, paraterm_p;
    boost::spirit::classic::rule<ScannerT> dataline_p;
    boost::spirit::classic::rule<ScannerT> lineS_p,
        lineG_p,
        lineD_p,
        lineP_p,
        lineT_p,
        line_p;

    const boost::spirit::classic::rule<ScannerT>& start() const {
      return igsfile_p;
    }

   private:

    igs_stack stack_;

  };
};

}  // namespace gua

#endif  // gua_IGS_GRAMMAR_HPP
