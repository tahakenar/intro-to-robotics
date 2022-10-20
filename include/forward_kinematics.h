#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H

#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Core>

struct AxisDHParam
{
    double a;       // link length
    double alpha;   // link twist
    double d;       // link offset
    double theta;   // link angle (variable for revolute joints)
};

class FKModel
{
    private:
        int dof_;   // degree-of-freedom
        std::vector<AxisDHParam> dh_params_;
        std::vector<double> variable_joint_states_;
        Eigen::Matrix4d fk_model_;

    public:
        FKModel(const int& number_of_joints);
        void assignDHParameters(const int& joint_idx, const AxisDHParam& params);
        Eigen::Matrix4d getHomogeneousTransMatrix(const AxisDHParam& dh_params);
        void calculateFKModel();
        Eigen::Matrix4d getFKModel();

};


#endif // FORWARD_KINEMATICS_H
