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
