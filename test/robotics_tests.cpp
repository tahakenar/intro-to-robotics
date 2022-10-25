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

struct TranslationFixture : public testing::TestWithParam<double> {};

TEST_P(TranslationFixture, TranslationTest) {
  Transform tf = Transform();
  double displacement = GetParam();
  testTranslations(tf, displacement);
}

INSTANTIATE_TEST_SUITE_P(TranslationTests, TranslationFixture,
                         testing::Values(1.9923, 2.13, -5.21, 3.235));

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

struct RotationFixture : public testing::TestWithParam<double> {};

TEST_P(RotationFixture, RotationTest) {
  Transform tf = Transform();
  double rotation = GetParam();
  testRotations(tf, rotation);
}

INSTANTIATE_TEST_SUITE_P(RotationTests, RotationFixture,
                         testing::Values(M_PI, M_PI / 2, -M_PI / 3, -M_PI / 7));

void testCompareMatrices(Eigen::Matrix4d m1, Eigen::Matrix4d m2,
                         double abs_err) {
  Eigen::MatrixXd m1_reshaped = m1.reshaped();
  Eigen::MatrixXd m2_reshaped = m2.reshaped();

  for (int i = 0; i < m1.size(); i++) {
    EXPECT_NEAR(m1(i), m2(i), abs_err);
  }
}

TEST(robotics_tests, HomogeneousTransformationTest) {
  //    The Homogeneous Transformation H represents a rotation by angle 'alpha'
  //    about the current x-axis. Then :
  // 1. Followed by a translation of 'a' along the current x-axis
  // 2. Followed by a translation of 'd' along the current z-axis
  // 3. Followed by a rotation of 'theta' along the current z-axis

  Transform tf = Transform();

  Eigen::Matrix4d m = Eigen::Matrix4d::Identity(4, 4);
  Eigen::Matrix4d expected_m(4, 4);

  double alpha = M_PI / 2;
  double b = 5.0;
  double d = 1.0;
  double theta = M_PI / 2;

  tf.addTransform(transform_enums::INITIAL,
                  tf.getRotation(transform_enums::X, alpha));
  tf.addTransform(transform_enums::CURRENT_FRAME,
                  tf.getTranslation(transform_enums::X, b));
  tf.addTransform(transform_enums::CURRENT_FRAME,
                  tf.getTranslation(transform_enums::Z, d));
  tf.addTransform(transform_enums::CURRENT_FRAME,
                  tf.getRotation(transform_enums::Z, theta));
  m = tf.getHomogeneousTransform();

  expected_m << std::cos(theta), -std::sin(theta), 0, b,
      std::cos(alpha) * std::sin(theta), std::cos(alpha) * std::cos(theta),
      -std::sin(alpha), -d * std::sin(alpha), std::sin(alpha) * std::sin(theta),
      std::sin(alpha) * std::cos(theta), std::cos(alpha), -d * std::cos(alpha),
      0, 0, 0, 1;

  testCompareMatrices(expected_m, m, 0.0002);

  // Clear
  tf.clearTransformQuery();
  expected_m = Eigen::Matrix4d::Identity(4, 4);
  m = Eigen::Matrix4d::Identity(4, 4);
}
