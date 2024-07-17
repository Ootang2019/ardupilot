// util.h
#ifndef UTIL_H
#define UTIL_H

#include <eigen3/Eigen/Dense>

// Declaration of the softmax function
Eigen::ArrayXf softmax(const Eigen::ArrayXf& logits);

std::string matrixToString(const Eigen::MatrixXf& matrix);

#endif 