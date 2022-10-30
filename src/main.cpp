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

  IKModel ur5e_ik_model = IKModel(UR5E_DOF);
  ur5e_ik_model.assignTransformationMatrices(
      ur5e_fk_model.getTransformationMatrices());

  m = ur5e_fk_model.getFKModel();
  std::cout << "FK model" << std::endl;
  std::cout << m << std::endl;

  Eigen::Vector4d initial_pos = m.block<4, 1>(0, 3);

  ur5e_ik_model.assignCurrentPosition(initial_pos);

  Eigen::MatrixXd j = ur5e_ik_model.getJacobianMatrix();

  std::cout << "Jacobian: " << std::endl;
  std::cout << j << std::endl;

  return 0;
}
