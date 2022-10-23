#include <gtest/gtest.h>

#include <Eigen/Core>

#include "transforms.h"

void testTranslations(Transform tf_object, const double& displacement) {
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity(4, 4);
  Eigen::Matrix4d expected_m = Eigen::Matrix4d::Identity(4, 4);

  std::vector<int> axis_vec = {transform_enums::X, transform_enums::Y,
                               transform_enums::Z};

  for (auto axis : axis_vec) {
    // Test X translation
    tf_object.addTransform(transform_enums::INITIAL,
                           tf_object.getTranslation(axis, displacement));
    expected_m(axis, 3) = displacement;
    m = tf_object.getHomogeneousTransform();
    EXPECT_EQ(expected_m, m);

    // Clear
    tf_object.clearTransformQuery();
    expected_m = Eigen::Matrix4d::Identity(4, 4);
    m = Eigen::Matrix4d::Identity(4, 4);
  }
}

TEST(robotics_tests, TranslationTests) {
  Transform tf = Transform();
  double displacement = 2.0;

  testTranslations(tf, displacement);
}

void testRotations(Transform tf_object, const double& rotation) {
  std::vector<Eigen::MatrixXd> rotation_matrices;
  Eigen::MatrixXd x_rot(3, 3);
  x_rot << 1, 0, 0, 0, std::cos(rotation), -std::sin(rotation), 0,
      std::sin(rotation), std::cos(rotation);
  rotation_matrices.push_back(x_rot);

  Eigen::MatrixXd y_rot(3, 3);
  y_rot << std::cos(rotation), 0, sin(rotation), 0, 1, 0, -std::sin(rotation),
      0, std::cos(rotation);
  rotation_matrices.push_back(y_rot);

  Eigen::MatrixXd z_rot(3, 3);
  z_rot << std::cos(rotation), -std::sin(rotation), 0, std::sin(rotation),
      std::cos(rotation), 0, 0, 0, 1;
  rotation_matrices.push_back(z_rot);

  Eigen::Matrix4d m = Eigen::Matrix4d::Identity(4, 4);
  Eigen::Matrix4d expected_m = Eigen::Matrix4d::Identity(4, 4);

  for (int i = 0; i < rotation_matrices.size(); i++) {
    tf_object.addTransform(transform_enums::INITIAL,
                           tf_object.getRotation(i, rotation));
    expected_m.block<3, 3>(0, 0) = rotation_matrices.at(i);
    m = tf_object.getHomogeneousTransform();
    ASSERT_EQ(expected_m, m) << "Fault in the idx: " << i << std::endl;

    // Clear
    tf_object.clearTransformQuery();
    expected_m = Eigen::Matrix4d::Identity(4, 4);
    m = Eigen::Matrix4d::Identity(4, 4);
  }
}

TEST(robotics_tests, RotationTests) {
  Transform tf = Transform();
  double rotation = M_PI / 2;

  testRotations(tf, rotation);
}
