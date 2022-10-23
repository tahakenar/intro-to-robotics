#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H

#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include <vector>
// #include <eigen3/Eigen/Core>

struct AxisDHParam {
  double a;      // link length
  double alpha;  // link twist
  double d;      // link offset
  double theta;  // link angle (variable for revolute joints)
};

class FKModel {
 private:
  int dof_;  // degree-of-freedom
  std::vector<AxisDHParam> dh_params_;
  std::vector<double> variable_joint_states_;
  std::vector<Eigen::Matrix4d> transformation_matrices_;
  Eigen::Matrix4d fk_model_;

 public:
  FKModel(const int& number_of_joints);
  void assignDHParameters(const int& joint_idx, const AxisDHParam& params);
  void assignTransformationMatrices();
  Eigen::Matrix4d getHomogeneousTransMatrix(const AxisDHParam& dh_params);
  std::vector<Eigen::Matrix4d> getTransformationMatrices();
  void calculateFKModel();
  Eigen::Matrix4d getFKModel();
};

#endif  // FORWARD_KINEMATICS_H
