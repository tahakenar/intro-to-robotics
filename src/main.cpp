#include <iostream>
#include "transforms.h"

int main(){

    Eigen::Matrix4d m;
    Transform tf = Transform();

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
