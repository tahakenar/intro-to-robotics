#include "inverse_kinematics.h"

/*
Step1: Find DX
Step2: Get dX
Step3: Get Jacobian
Step4: Get PseudoInverse
Step5: Calculate deltaQ
Step6: Add deltaQ to Q
Step7: repeat until DX ~= 0
*/

void IKModel::initializeIKModel(const int& number_of_joints) {
  dof_ = number_of_joints;
  transformation_matrices_.resize(dof_);
  joint_space_variables_.resize(dof_);
  joint_space_output_.resize(dof_);

  fk_model_.initializeFKModel(dof_);
}

void IKModel::assignTransformationMatrices() {
  std::vector<Eigen::Matrix4d> tf_matrices =
      fk_model_.getTransformationMatrices();
  for (int i = 0; i < tf_matrices.size(); i++) {
    transformation_matrices_.at(i) = tf_matrices.at(i);
  }
}

void IKModel::assignCurrentPosition(const Eigen::Vector4d& curr_vec) {
  current_tool_position_ = curr_vec;
}

Eigen::Vector4d IKModel::getCurrentPosition() { return current_tool_position_; }

void IKModel::assignGoalPosition(const Eigen::Vector4d& goal_vec) {
  goal_tool_position_ = goal_vec;
}

Eigen::Vector4d IKModel::getGoalPosition() { return goal_tool_position_; }

Eigen::MatrixXd IKModel::getJacobianMatrix() {
  Eigen::MatrixXd jacob(6, dof_);

  for (int i = 0; i < dof_; i++) {
    jacob.block<6, 1>(0, i) = getJacobianColumn(i);
  }

  jacobian_ = jacob;
  return jacobian_;
}

Eigen::MatrixXd IKModel::getJacobianColumn(const int& joint_idx) {
  Eigen::MatrixXd m(6, 1);
  Eigen::Matrix4d homogeneous_m = this->getHomogeneousTransUntil(joint_idx);

  Eigen::Vector3d z = homogeneous_m.block<3, 1>(0, 2);
  Eigen::Vector3d t = homogeneous_m.block<3, 1>(0, 3);
  Eigen::Vector3d t_ultimate =
      this->getHomogeneousTransUntil(dof_ - 1).block<3, 1>(0, 3);

  Eigen::Vector3d t_diff = t_ultimate - t;
  m.block<3, 1>(0, 0) = z.cross(t_diff);
  m.block<3, 1>(3, 0) = z;

  return m;
}

Eigen::Matrix4d IKModel::getHomogeneousTransUntil(const int& joint_idx) {
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity(4, 4);

  for (int i = 0; i <= joint_idx; i++) {
    m = m * transformation_matrices_.at(i);
  }
  return m;
}

void IKModel::calculateDeltaX() {
  delta_x_ = goal_tool_position_ - current_tool_position_;
}

void IKModel::minifyDeltaX(double fraction) { delta_x_ *= fraction; }

void IKModel::assignDHParams(const std::vector<AxisDHParam>& dh_params) {
  for (int i = 0; i < dof_; i++) {
    fk_model_.assignDHParameters(i, dh_params.at(i));
  }
}

void IKModel::assignJointSpaceVariables() {
  std::vector<AxisDHParam> dh_params = fk_model_.getDHParams();

  for (int i = 0; i < dof_; i++) {
    joint_space_variables_.at(i) = dh_params.at(i).theta;
  }
}
