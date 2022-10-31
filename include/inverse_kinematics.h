#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include <Eigen/Dense>
#include <Eigen/QR>
#include <cmath>
#include <iostream>
#include <vector>

#include "forward_kinematics.h"

/*
Step1: Find DX
Step2: Get dX
Step3: Get Jacobian
Step4: Get PseudoInverse
Step5: Calculate deltaQ
Step6: Add deltaQ to Q
Step7: repeat until DX ~= 0
*/

class IKModel {
 private:
  int dof_;  // degree of freedom

  FKModel fk_model_;

  Eigen::Vector4d current_tool_position_;
  Eigen::Vector4d goal_tool_position_;
  Eigen::Vector4d delta_x_;
  Eigen::Vector4d minified_delta_x_;

  std::vector<Eigen::Matrix4d> transformation_matrices_;
  Eigen::Matrix4d homogeneous_trans_matrix_;

  std::vector<double> joint_space_variables_;
  std::vector<double> joint_space_output_;

  Eigen::MatrixXd jacobian_;
  Eigen::MatrixXd pinv_jacobian_;

 public:
  void initializeIKModel(const int& number_of_joints);
  void assignTransformationMatrices();
  void assignCurrentPosition(const Eigen::Vector4d& curr_vec);
  void updateCurrentPosition();
  Eigen::Vector4d getCurrentPosition();
  void assignGoalPosition(const Eigen::Vector4d& goal_vec);
  Eigen::Vector4d getGoalPosition();

  void calculateJointSpaceOutput();
  Eigen::VectorXd getDeltaJointSpace();
  void calculatePseudoInvJacob();
  Eigen::MatrixXd getJacobianMatrix();
  Eigen::MatrixXd getJacobianColumn(const int& joint_idx);
  Eigen::Matrix4d getHomogeneousTransUntil(const int& joint_idx);

  void calculateDeltaX();
  void minifyDeltaX(double fraction);
  void assignDHParams(const std::vector<AxisDHParam>& dh_params);
  void assignJointSpaceVariables();

  std::vector<double> solveInverseKinematics();
};

#endif  // INVERSE_KINEMATICS_H
