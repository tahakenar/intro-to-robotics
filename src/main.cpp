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

Transform::Transform()
{
    this->resetHomogeneousTrans();
}

void Transform::resetHomogeneousTrans()
{
    homogeneous_transform_ = Eigen::Matrix4d::Identity(4,4);
}

void Transform::clearTransformQuery()
{
    transform_query_.clear();
}

void Transform::printQuery()
{
    std::cout << "\n\nQuery: " << std::endl;
    for (auto m: transform_query_)
    {
        std::cout << " ------ " << std::endl;
        std::cout << m << std::endl;
    }
    std::cout << "---- end of query ---- " << std::endl;
}

void Transform::addTransform(const int& frame_type, const Eigen::Matrix4d& tf_matrix)
{
    std::cout << "query size " << transform_query_.size() << std::endl;
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

Eigen::Matrix4d Transform::getHomogeneousTransform()
{
    homogeneous_transform_ = Eigen::Matrix4d::Identity(4,4);
    for (const auto m: transform_query_)
    {
        homogeneous_transform_ = homogeneous_transform_*m;
    }
    return homogeneous_transform_;
}

void Transform::getXRotation(Eigen::Matrix4d& m, const std::pair<double,double>& trig_results)
{
    // first: cosine, second: sine
    m(1,1) = trig_results.first;
    m(1,2) = -trig_results.second;
    m(2,1) = trig_results.second;
    m(2,2) = trig_results.first;
}

void Transform::getYRotation(Eigen::Matrix4d& m, const std::pair<double,double>& trig_results)
{
    // first: cosine, second: sine
    m(0,0) = trig_results.first;
    m(0,2) = trig_results.second;
    m(2,0) = -trig_results.second;
    m(2,2) = trig_results.first;
}

void Transform::getZRotation(Eigen::Matrix4d& m, const std::pair<double,double>& trig_results)
{
    // first: cosine, second: sine
    m(0,0) = trig_results.first;
    m(0,1) = -trig_results.second;
    m(0,1) = trig_results.second;
    m(1,1) = trig_results.first;
}

Eigen::Matrix4d Transform::getRotation(const int& axis, const double& radians)
{
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity(4,4);
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

Eigen::Matrix4d Transform::getTranslation(const int& axis, const double& displacement)
{
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity(4,4);
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

void Transform::getXTranslation(Eigen::Matrix4d& m, const double& displacement)
{
    m(0,3) = displacement;
}

void Transform::getYTranslation(Eigen::Matrix4d& m, const double& displacement)
{
    m(1,3) = displacement;
}

void Transform::getZTranslation(Eigen::Matrix4d& m, const double& displacement)
{
    m(2,3) = displacement;
}

int main(){

    Eigen::Matrix4d m;
    Transform tf = Transform();

    // m = tf.getHomogeneousTransform();

    // std::cout << "Initial homogeneous transform\n";
    // std::cout << m << std::endl;

    // m = tf.getTranslation(transform_enums::X, 1.1);

    // std::cout << "X translation\n";
    // std::cout << m << std::endl;

    // m = tf.getTranslation(transform_enums::Y, 2.2);

    // std::cout << "Y translation\n";
    // std::cout << m << std::endl;

    // m = tf.getTranslation(transform_enums::Z, 3.3);

    // std::cout << "Z translation\n";
    // std::cout << m << std::endl;

    // m = tf.getRotation(transform_enums::X, M_PI/2);

    // std::cout << "X rotation\n";
    // std::cout << m << std::endl;

    // m = tf.getRotation(transform_enums::Y, M_PI/2);

    // std::cout << "Y rotation\n";
    // std::cout << m << std::endl;

    // m = tf.getRotation(transform_enums::Z, M_PI/2);

    // std::cout << "Z rotation\n";
    // std::cout << m << std::endl;

    tf.addTransform(transform_enums::INITIAL, Eigen::Matrix4d::Identity(4,4));
    tf.addTransform(transform_enums::CURRENT_FRAME, tf.getTranslation(transform_enums::X,1.1));
    tf.addTransform(transform_enums::FIXED_FRAME, tf.getTranslation(transform_enums::Y,2.2));
    tf.addTransform(transform_enums::CURRENT_FRAME, tf.getTranslation(transform_enums::Z,3.3));
    tf.addTransform(transform_enums::CURRENT_FRAME, tf.getRotation(transform_enums::X, M_PI/2));
    tf.addTransform(transform_enums::FIXED_FRAME, tf.getRotation(transform_enums::Y, M_PI/2));
    tf.addTransform(transform_enums::FIXED_FRAME, tf.getRotation(transform_enums::Y, M_PI/2));
    tf.printQuery();
    m = tf.getHomogeneousTransform();
    std::cout << "Homogeneous transform: " << std::endl;
    std::cout << m << std::endl;


    tf.clearTransformQuery();
    tf.printQuery();
    m = tf.getHomogeneousTransform();

    
    return 0;
}