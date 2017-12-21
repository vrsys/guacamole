#ifndef GUACAMOLE_PAGODA_JOINT_VISUAL_H
#define GUACAMOLE_PAGODA_JOINT_VISUAL_H

#include "pagoda_visual.hpp"

class PagodaJointVisual;

typedef std::shared_ptr<PagodaJointVisual> ptr_joint_visual;

class PagodaJointVisual : public PagodaVisual
{
  public:
    PagodaJointVisual(const std::string &name, ptr_visual parent);
    ~PagodaJointVisual();
};
#endif // GUACAMOLE_PAGODA_JOINT_VISUAL_H
