/********************************************************************************
*
* Copyright (C) 2013 Bauhaus University Weimar
*
*********************************************************************************
*
*  module     : contour_map_kd.hpp
*  project    : tml
*  description:
*
********************************************************************************/
#ifndef TML_CONTOUR_MAP_KD_HPP
#define TML_CONTOUR_MAP_KD_HPP

// includes, system

// includes, project
#include <gua/renderer/nurbs_geometry/tml/parametric/domain/contour_map_base.hpp>

namespace tml {

template <typename value_t>
class contour_map_kd : public contour_map_base<value_t> {
 public:  // typedef / enums

  typedef contour_map_base<value_t> base_type;

  struct contour_cell {
    void print(std::ostream& os) const;

    contour_segment_container overlapping_segments;
    interval_type interval_u;
    interval_type interval_v;
    bool inside;
  };

  struct domain_start_event;
  struct domain_end_event;
  struct contour_start_event;
  struct contour_end_event;

  struct min_interval_compare {
    bool operator()(interval_type const& lhs, interval_type const& rhs) const {
      return lhs.minimum() < rhs.minimum();
    }
  };

  struct status_queue {
    void process(domain_start_event const&);
    void process(domain_end_event const&);
    void process(contour_start_event const&);
    void process(contour_end_event const&);
    void update(value_type const& u);

    std::set<value_type> splits;
    std::set<contour_segment_ptr> segments;
    interval_type interval_v;
    std::map<interval_type, value_type, min_interval_compare> intervals;
    std::vector<contour_cell> result;
  };

  struct contour_event {
    contour_event(value_type const& u) : value_u(u) {}
    ;
    value_type value_u;
    virtual unsigned priority() const = 0;
    virtual void visit(status_queue& s) const = 0;
  };

  struct contour_start_event : public contour_event {
    contour_start_event(value_type const& u, contour_segment_ptr const& s)
        : contour_event(u), segment(s) {}
    ;
    contour_segment_ptr segment;
    virtual unsigned priority() const { return 1; }
    ;
    virtual void visit(status_queue& s) const { s.process(*this); }
    ;
  };

  struct contour_end_event : public contour_event {
    contour_end_event(value_type const& u, contour_segment_ptr const& s)
        : contour_event(u), segment(s) {}
    ;
    virtual unsigned priority() const { return 2; }
    ;
    virtual void visit(status_queue& s) const { s.process(*this); }
    ;
    contour_segment_ptr segment;
  };

  struct domain_start_event : public contour_event {
    domain_start_event(value_type const& u, interval_type const& v)
        : contour_event(u), interval_v(v) {}
    ;
    virtual unsigned priority() const { return 3; }
    ;
    virtual void visit(status_queue& s) const { s.process(*this); }
    ;
    interval_type interval_v;
  };

  struct domain_end_event : public contour_event {
    domain_end_event(value_type const& u, interval_type const& v)
        : contour_event(u), interval_v(v) {}
    ;
    virtual unsigned priority() const { return 0; }
    ;
    virtual void visit(status_queue& s) const { s.process(*this); }
    ;
    interval_type interval_v;
  };

  struct contour_compare {
    bool operator()(contour_event* lhs, contour_event* rhs) const {
      if (lhs->value_u == rhs->value_u) {
        return lhs->priority() > rhs->priority();
      } else {
        return lhs->value_u < rhs->value_u;
      }
    }
  };

  struct event_queue {
    std::set<contour_event*, contour_compare> events;
  };

  struct kdnode {
    kdnode(value_type const& s,
           typename point_type::coordinate_type const& d,
           contour_cell* c,
           kdnode* l,
           kdnode* m)
        : split_value(s), split_dimension(d), cell(c), less(l), more(m) {}
    ;

    bool is_child() const { return cell != nullptr; }
    ;

    value_type split_value;
    typename point_type::coordinate_type split_dimension;
    contour_cell* cell;
    kdnode* less;
    kdnode* more;
  };

 public:  // c'tor / d'tor

  contour_map_kd();
  virtual ~contour_map_kd();

 public:  // methods

  /* virtual */ void initialize();

  void destroy(kdnode* n);

  kdnode* create(bbox_type const& bounds,
                 std::vector<contour_cell> const& cells);

  kdnode* split(bbox_type const& bounds,
                typename point_type::coordinate_type const& dim,
                std::set<value_type> const& candidates,
                std::vector<contour_cell> const& cells);

  std::set<value_type> split_candidates(
      bbox_type const& bounds,
      typename point_type::coordinate_type const& dim,
      std::vector<contour_cell> const& cells);

  // stream output of domain
  virtual void print(std::ostream& os) const;

 protected:  // methods

 private:  // internal/auxilliary methods

 protected:  // attributes

  std::vector<contour_cell> _cells;

  kdnode* _root;
};

template <typename value_t>
    std::ostream& operator<<(std::ostream& os,
                             tml::contour_map_kd<value_t> const& rhs);

}  // namespace tml

#include <gua/contour_map_kd_impl.hpp>

#endif  // TML_RECTANGULAR_MAP_HPP
