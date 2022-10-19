#ifndef TRANSFORMS_H
#define TRANSFORMS_H

#include <iostream>
#include <cmath>
#include <list>
//#include <eigen3/Eigen/Core>
#include <Eigen/Core>

namespace transform_enums
{
    enum Axis
    {
        X = 0,
        Y = 1,
        Z = 2
    };

    enum Frame
    {
        INITIAL = 0,
        CURRENT_FRAME = 1,
        FIXED_FRAME = 2
    };
}

class Transform
{
    private:
        std::list<Eigen::Matrix4d> transform_query_;
        Eigen::Matrix4d homogeneous_transform_;
    public:
        Transform();

        void resetHomogeneousTrans();
        void clearTransformQuery();
        void printQuery();

        void addTransform(const int& frame_type, const Eigen::Matrix4d& tf_matrix);

        Eigen::Matrix4d getHomogeneousTransform();

        Eigen::Matrix4d getRotation(const int& axis, const double& radians);
        void getXRotation(Eigen::Matrix4d& m, const std::pair<double,double>& trig_results);
        void getYRotation(Eigen::Matrix4d& m, const std::pair<double,double>& trig_results);
        void getZRotation(Eigen::Matrix4d& m, const std::pair<double,double>& trig_results);

        Eigen::Matrix4d getTranslation(const int& axis, const double& displacement);
        void getXTranslation(Eigen::Matrix4d& m, const double& displacement);
        void getYTranslation(Eigen::Matrix4d& m, const double& displacement);
        void getZTranslation(Eigen::Matrix4d& m, const double& displacement);
  
};

#endif // TRANSFORMS_H