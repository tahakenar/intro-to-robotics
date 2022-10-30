#include <iostream>

#include "forward_kinematics.h"
#include "inverse_kinematics.h"
#include "transforms.h"

#define UR5E_DOF 6  // degree of freedom

int main() {
  Eigen::Matrix4d m;

  AxisDHParam joint0 = {0, M_PI / 2, 0.1625, 0};
  AxisDHParam joint1 = {-0.4250, 0, 0, 0};
  AxisDHParam joint2 = {-0.3922, 0, 0, 0};
  AxisDHParam joint3 = {0, M_PI / 2, 0.1333, 0};
  AxisDHParam joint4 = {0, -M_PI / 2, 0.0997, 0};
  AxisDHParam joint5 = {0, 0, 0.0996, 0};

  std::vector<AxisDHParam> dh_param_vec = {joint0, joint1, joint2,
                                           joint3, joint4, joint5};

  IKModel ur5e_ik_model;
  ur5e_ik_model.initializeIKModel(UR5E_DOF);

  return 0;
}
