#include "forward_kinematics.h"

FKModel::FKModel(const int& number_of_joints) : dof_(number_of_joints)
{
    dh_params_.reserve(dof_);
    variable_joint_states_.reserve(dof_);
    fk_model_ = Eigen::Matrix4d::Identity(4,4);
}

void FKModel::assignDHParameters(const int& joint_idx, const AxisDHParam& params)
{
    dh_params_.at(joint_idx) = params;
}

void FKModel::calculateFKModel()
{
    fk_model_ = Eigen::Matrix4d::Identity(4,4);
    for (auto joint: dh_params_)
    {
        fk_model_ *= this->getHomogeneousTransMatrix(joint);
    }
}

Eigen::Matrix4d FKModel::getHomogeneousTransMatrix(const AxisDHParam& dh_param)
{
    Eigen::Matrix4d A = Eigen::Matrix4d::Identity(4,4);
    A(0,0) = std::cos(dh_param.theta);
    A(0,1) = -std::sin(dh_param.theta)*std::cos(dh_param.alpha);
    A(0,2) = std::sin(dh_param.theta)*std::sin(dh_param.alpha);
    A(0,3) = dh_param.a * std::cos(dh_param.theta);

    A(1,0) = std::sin(dh_param.theta);
    A(1,1) = std::cos(dh_param.theta)*std::cos(dh_param.alpha);
    A(1,2) = -std::cos(dh_param.theta)*std::sin(dh_param.alpha);
    A(1,3) = dh_param.a * std::sin(dh_param.theta);

    A(2,1) = std::sin(dh_param.alpha);
    A(2,2) = std::cos(dh_param.alpha);
    A(2,3) = dh_param.d;

    return A;
}

Eigen::Matrix4d FKModel::getFKModel()
{
    return fk_model_;
}
