#include "GuaInteractiveModelPlugin.hpp"

void gazebo::GuaInteractiveModelPlugin::on_update() { this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0)); }
