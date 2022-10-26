#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include <vector>
//#include <eigen3/Eigen/Core>

struct VectorSpaceVel {
  double x_dot;
  double y_dot;
  double z_dot;
  double w_x;
  double w_y;
  double w_z;
};

class IKModel {
 private:
  int dof_;  // degree of freedom
  std::vector<Eigen::Matrix4d> transformation_matrices_;
  Eigen::Matrix4d homogeneous_trans_matrix_;
  Eigen::MatrixXd total_tool_displacement_;
  std::vector<double> joint_space_variables_;
  std::vector<double> joint_space_output_;

  Eigen::MatrixXd jacobian_;

 public:
  IKModel(const int& number_of_joints);
  void getJacobianMatrix();
  Eigen::MatrixXd getJacobianColumn(const int& joint_idx);
  Eigen::Matrix4d getHomogeneousTransUntil(const int& joint_idx);
};

#endif  // INVERSE_KINEMATICS_H
