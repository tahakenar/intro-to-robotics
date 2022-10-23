#include "transforms.h"

Transform::Transform() { this->resetHomogeneousTrans(); }

void Transform::resetHomogeneousTrans() {
  homogeneous_transform_ = Eigen::Matrix4d::Identity(4, 4);
}

void Transform::clearTransformQuery() { transform_query_.clear(); }

void Transform::printQuery() {
  std::cout << "\n\nQuery: " << std::endl;
  for (auto m : transform_query_) {
    std::cout << " ------ " << std::endl;
    std::cout << m << std::endl;
  }
  std::cout << "---- end of query ---- " << std::endl;
}

void Transform::addTransform(const int& frame_type,
                             const Eigen::Matrix4d& tf_matrix) {
  if (frame_type == transform_enums::INITIAL && transform_query_.size() == 0)
    transform_query_.push_back(tf_matrix);

  // POST-MULTIPLICATION
  else if (frame_type == transform_enums::CURRENT_FRAME)
    transform_query_.push_back(tf_matrix);

  // PRE-MULTIPLICATION
  else if (frame_type == transform_enums::FIXED_FRAME)
    transform_query_.push_front(tf_matrix);

  else
    std::cout << "Invalid frame type!" << std::endl;
}

Eigen::Matrix4d Transform::getHomogeneousTransform() {
  homogeneous_transform_ = Eigen::Matrix4d::Identity(4, 4);
  for (const auto m : transform_query_) {
    homogeneous_transform_ = homogeneous_transform_ * m;
  }
  return homogeneous_transform_;
}

void Transform::getXRotation(Eigen::Matrix4d& m,
                             const std::pair<double, double>& trig_results) {
  // first: cosine, second: sine
  m(1, 1) = trig_results.first;
  m(1, 2) = -trig_results.second;
  m(2, 1) = trig_results.second;
  m(2, 2) = trig_results.first;
}

void Transform::getYRotation(Eigen::Matrix4d& m,
                             const std::pair<double, double>& trig_results) {
  // first: cosine, second: sine
  m(0, 0) = trig_results.first;
  m(0, 2) = trig_results.second;
  m(2, 0) = -trig_results.second;
  m(2, 2) = trig_results.first;
}

void Transform::getZRotation(Eigen::Matrix4d& m,
                             const std::pair<double, double>& trig_results) {
  // first: cosine, second: sine
  m(0, 0) = trig_results.first;
  m(0, 1) = -trig_results.second;
  m(0, 1) = trig_results.second;
  m(1, 1) = trig_results.first;
}

Eigen::Matrix4d Transform::getRotation(const int& axis, const double& radians) {
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity(4, 4);
  std::pair<double, double> trig_results(std::cos(radians), std::sin(radians));
  if (axis == transform_enums::X)
    this->getXRotation(m, trig_results);
  else if (axis == transform_enums::Y)
    this->getYRotation(m, trig_results);
  else if (axis == transform_enums::Z)
    this->getZRotation(m, trig_results);
  else
    std::cout << "Invalid axis! Returning identity matrix!" << std::endl;
  return m;
}

Eigen::Matrix4d Transform::getTranslation(const int& axis,
                                          const double& displacement) {
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity(4, 4);
  if (axis == transform_enums::X)
    this->getXTranslation(m, displacement);
  else if (axis == transform_enums::Y)
    this->getYTranslation(m, displacement);
  else if (axis == transform_enums::Z)
    this->getZTranslation(m, displacement);
  else
    std::cout << "Invalid axis! Returning identity matrix!" << std::endl;
  return m;
}

void Transform::getXTranslation(Eigen::Matrix4d& m,
                                const double& displacement) {
  m(0, 3) = displacement;
}

void Transform::getYTranslation(Eigen::Matrix4d& m,
                                const double& displacement) {
  m(1, 3) = displacement;
}

void Transform::getZTranslation(Eigen::Matrix4d& m,
                                const double& displacement) {
  m(2, 3) = displacement;
}
