#include <iostream>

#include "forward_kinematics.h"
#include "transforms.h"

#define UR5E_DOF 6  // degree of freedom

int main() {
  Eigen::Matrix4d m;

  AxisDHParam joint0 = {0, M_PI / 2, 0.1625, 5.270894341};
  AxisDHParam joint1 = {0, 0, 0, 3.316125579};
  AxisDHParam joint2 = {-0.3922, 0, 0, 1.029744259};
  AxisDHParam joint3 = {0, M_PI / 2, 0.1333, 3.473205211};
  AxisDHParam joint4 = {0, -M_PI / 2, 0.0997, 2.094395102};
  AxisDHParam joint5 = {0, 0, 0.0996, 1.570796327};

  FKModel ur5e_fk_model = FKModel(UR5E_DOF);
  ur5e_fk_model.assignDHParameters(0, joint0);
  ur5e_fk_model.assignDHParameters(1, joint1);
  ur5e_fk_model.assignDHParameters(2, joint2);
  ur5e_fk_model.assignDHParameters(3, joint3);
  ur5e_fk_model.assignDHParameters(4, joint4);
  ur5e_fk_model.assignDHParameters(5, joint5);
  ur5e_fk_model.assignTransformationMatrices();
  ur5e_fk_model.calculateFKModel();
  m = ur5e_fk_model.getFKModel();

  std::cout << m << std::endl;

  return 0;
}
