#include "inverse_kinematics.h"

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

void IKModel::updateCurrentPosition() {
  fk_model_.calculateFKModel();
  current_tool_position_ = fk_model_.getFKModel().block<4, 1>(3, 0);
}

Eigen::Vector4d IKModel::getGoalPosition() { return goal_tool_position_; }

void IKModel::calculateJointSpaceOutput() {
  Eigen::VectorXd delta_q = this->getDeltaJointSpace();

  for (int i = 0; i < dof_; i++) {
    joint_space_output_.at(i) = joint_space_variables_.at(i) + delta_q[i];
  }
}

Eigen::VectorXd IKModel::getDeltaJointSpace() {
  Eigen::VectorXd delta_q = pinv_jacobian_ * delta_x_;
  return delta_q;
}

void IKModel::calculatePseudoInvJacob() {
  pinv_jacobian_ = this->getJacobianMatrix()
                       .completeOrthogonalDecomposition()
                       .pseudoInverse();
}

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

void IKModel::minifyDeltaX(double fraction) {
  minified_delta_x_ = delta_x_ * fraction;
}

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

std::vector<double> IKModel::solveInverseKinematics() {
  this->calculateDeltaX();

  // TODO: get rid of magic number below
  while (delta_x_.norm() > 0.002) {
    this->calculateDeltaX();
    this->minifyDeltaX(0.002);
    this->getJacobianMatrix();
    this->calculateJointSpaceOutput();

    // TODO: update the joint states and ultimate homogeneous tf.
    fk_model_.updateVariableDHParams(joint_space_output_);
    fk_model_.assignTransformationMatrices();
    fk_model_.calculateFKModel();

    // TODO: feed ik model using fk model
    this->assignTransformationMatrices();
    this->updateCurrentPosition();
  }

  return joint_space_output_;
}
