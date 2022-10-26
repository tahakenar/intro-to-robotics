#include <gtest/gtest.h>

#include <Eigen/Core>
#include <iostream>

#include "forward_kinematics.h"

#define UR5E_DOF 6  // degree of freedom

void testCompareMatrices(Eigen::Matrix4d m1, Eigen::Matrix4d m2,
                         double abs_err) {
  Eigen::MatrixXd m1_reshaped = m1.reshaped();
  Eigen::MatrixXd m2_reshaped = m2.reshaped();
  for (int i = 0; i < m1.size(); i++) {
    ASSERT_NEAR(m1(i), m2(i), abs_err);
  }
}

AxisDHParam joint0 = {0, M_PI / 2, 0.1625, 5.270894341};
AxisDHParam joint1 = {0, 0, 0, 3.316125579};
AxisDHParam joint2 = {-0.3922, 0, 0, 1.029744259};
AxisDHParam joint3 = {0, M_PI / 2, 0.1333, 3.473205211};
AxisDHParam joint4 = {0, -M_PI / 2, 0.0997, 2.094395102};
AxisDHParam joint5 = {0, 0, 0.0996, 1.570796327};

std::array<AxisDHParam, UR5E_DOF> dh_list1 = {joint0, joint1, joint2,
                                              joint3, joint4, joint5};

struct FKFixture
    : public testing::TestWithParam<std::array<AxisDHParam, UR5E_DOF>> {};

INSTANTIATE_TEST_SUITE_P(ForwardKinematicTests, FKFixture,
                         testing::Values(dh_list1));

TEST_P(FKFixture, ForwardKinematicTest) {
  FKModel ur5e_fk_model = FKModel(UR5E_DOF);
  ur5e_fk_model.assignDHParameters(0, std::get<0>(GetParam()));
  ur5e_fk_model.assignDHParameters(1, std::get<1>(GetParam()));
  ur5e_fk_model.assignDHParameters(2, std::get<2>(GetParam()));
  ur5e_fk_model.assignDHParameters(3, std::get<3>(GetParam()));
  ur5e_fk_model.assignDHParameters(4, std::get<4>(GetParam()));
  ur5e_fk_model.assignDHParameters(5, std::get<5>(GetParam()));
  ur5e_fk_model.assignTransformationMatrices();
  ur5e_fk_model.calculateFKModel();

  Eigen::Matrix4d m = ur5e_fk_model.getFKModel();

  // TODO: will be separeted. Having compiler errors if defined globally dafuq?
  Eigen::Matrix4d result_matrix(4, 4);
  result_matrix << -0.52960, 0.74368, 0.40801, 0.05487, 0.84753, 0.44413,
      0.29059, -0.24539, 0.03490, 0.49970, -0.86550, 0.43897, 0.00000, 0.00000,
      0.00000, 1.00000;

  testCompareMatrices(result_matrix, m, 0.0002);
}
