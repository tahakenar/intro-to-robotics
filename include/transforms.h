#ifndef TRANSFORMS_H
#define TRANSFORMS_H

#include <iostream>
#include <cmath>
#include <list>
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

/**
 * @brief A class to perform basic homogeneous transformation stuff. 
 * 
 * 
 */
class Transform
{
    private:
        std::list<Eigen::Matrix4d> transform_query_;
        Eigen::Matrix4d homogeneous_transform_;

    public:
        Transform();

        /**
         * @brief Resets the resulting homogeneous_transform to the identity matrix
         * 
         */
        void resetHomogeneousTrans();

        /**
         * @brief Clears the transformation query
         * 
         */
        void clearTransformQuery();

        /**
         * @brief Prints the transformation query in order
         * 
         */
        void printQuery();

        /**
         * @brief Adds homogeneous transform to the query. Takes the frame type (FIXED/CURRENT) into account
         * 
         * @param frame_type FIXED/CURRENT or INITIAL if it is the first matrix
         * @param tf_matrix Homogeneous transform matrix
         */
        void addTransform(const int& frame_type, const Eigen::Matrix4d& tf_matrix);

        /**
         * @brief Get the resulting homogeneous transform
         * 
         * @return Eigen::Matrix4d Resulting homogeneous transform
         */
        Eigen::Matrix4d getHomogeneousTransform();

        /**
         * @brief Get the rotation of the object in 3-dimensional space
         * 
         * @param axis The axis which rotation occurs
         * @param displacement The amount of rotation (in radians)
         * @return Eigen::Matrix4d Resulting homogeneous transform
         */
        Eigen::Matrix4d getRotation(const int& axis, const double& radians);

        /**
         * @brief Changes the matrix to represent rotation around x-axis
         * 
         * @param m Matrix to be changed
         * @param trig_results Pair of cos(theta) (first) and sin(theta) (second) to be used to represent rotation
         */
        void getXRotation(Eigen::Matrix4d& m, const std::pair<double,double>& trig_results);

        /**
         * @brief Changes the matrix to represent rotation around y-axis
         * 
         * @param m Matrix to be changed
         * @param trig_results Pair of cos(theta) (first) and sin(theta) (second) to be used to represent rotation
         */
        void getYRotation(Eigen::Matrix4d& m, const std::pair<double,double>& trig_results);

        /**
         * @brief Changes the matrix to represent rotation around z-axis
         * 
         * @param m Matrix to be changed
         * @param trig_results Pair of cos(theta) (first) and sin(theta) (second) to be used to represent rotation
         */
        void getZRotation(Eigen::Matrix4d& m, const std::pair<double,double>& trig_results);

        /**
         * @brief Get the translation of the object in 3-dimensional space
         * 
         * @param axis The axis of translation
         * @param displacement The amount of displacement
         * @return Eigen::Matrix4d Resulting homogeneous transform
         */
        Eigen::Matrix4d getTranslation(const int& axis, const double& displacement);

        /**
         * @brief Changes the matrix to represent translation on x-axis
         * 
         * @param m Matrix to be changed
         * @param displacement The amount of displacement
         */
        void getXTranslation(Eigen::Matrix4d& m, const double& displacement);

        /**
         * @brief Changes the matrix to represent translation on y-axis
         * 
         * @param m Matrix to be changed
         * @param displacement The amount of displacement
         */
        void getYTranslation(Eigen::Matrix4d& m, const double& displacement);

        /**
         * @brief Changes the matrix to represent translation on z-axis
         * 
         * @param m Matrix to be changed
         * @param displacement The amount of displacement
         */
        void getZTranslation(Eigen::Matrix4d& m, const double& displacement);
  
};

#endif // TRANSFORMS_H
