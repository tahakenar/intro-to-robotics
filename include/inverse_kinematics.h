#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <vector>

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
  Eigen::Vector4d current_tool_position_;
  Eigen::Vector4d goal_tool_position_;
  Eigen::Vector4d delta_x_;

  std::vector<Eigen::Matrix4d> transformation_matrices_;
  Eigen::Matrix4d homogeneous_trans_matrix_;

  std::vector<double> joint_space_variables_;
  std::vector<double> joint_space_output_;

  Eigen::MatrixXd jacobian_;

 public:
  IKModel(const int& number_of_joints);
  void assignTransformationMatrices(std::vector<Eigen::Matrix4d> tf_matrices);
  void assignCurrentPosition(const Eigen::Vector4d& curr_vec);
  Eigen::Vector4d getCurrentPosition();
  void assignGoalPosition(const Eigen::Vector4d& goal_vec);
  Eigen::Vector4d getGoalPosition();
  Eigen::MatrixXd getJacobianMatrix();
  Eigen::MatrixXd getJacobianColumn(const int& joint_idx);
  Eigen::Matrix4d getHomogeneousTransUntil(const int& joint_idx);

  void calculateDeltaX();
  void minifyDeltaX(double fraction);
};

#endif  // INVERSE_KINEMATICS_H
