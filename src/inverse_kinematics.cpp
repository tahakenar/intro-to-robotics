#include "inverse_kinematics.h"

IKModel::IKModel(const int& number_of_joints) : dof_(number_of_joints)
{
    transformation_matrices_.reserve(dof_);
    joint_space_variables_.reserve(dof_);
    joint_space_output_.reserve(dof_);
}

void IKModel::getJacobianMatrix()
{
    Eigen::MatrixXd jacob(6,dof_);

    for (int i = 0; i < dof_; i++)
    {
        jacob.block<6,1>(0,i) = getJacobianColumn(i);
    }

    jacobian_ = jacob;
}

Eigen::MatrixXd IKModel::getJacobianColumn(const int& joint_idx)
{
    Eigen::MatrixXd m(6,1);
    Eigen::Matrix4d homogeneous_m = this->getHomogeneousTransUntil(joint_idx);

    m.block<3,1>(0,0) = homogeneous_m.block<3,1>(0,2).cross(total_tool_displacement_ - homogeneous_m.block<3,1>(0,3));
    m.block<3,1>(3,0) = homogeneous_m.block<3,1>(0,2);

    return m;

}


Eigen::Matrix4d IKModel::getHomogeneousTransUntil(const int& joint_idx)
{
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity(4,4);

    for (int i = 0; i < joint_idx; i++)
    {
        m = m*transformation_matrices_(i);
    }

    return m;
}