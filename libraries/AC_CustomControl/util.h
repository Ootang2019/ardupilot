// util.h
#ifndef UTIL_H
#define UTIL_H

#include <iostream>
#include <vector>
#include <cassert>
#include <cmath>
#include <algorithm> // For std::max and std::min

std::vector<double> getLastColumn(const std::vector<std::vector<double>>& matrix);

void clampToRange(std::vector<double>& vec, double min_val, double max_val);

std::vector<double> vecCat(const std::vector<double>& vec1, const std::vector<double>& vec2);
std::vector<double> vecAdd(const std::vector<double>& vec1, const std::vector<double>& vec2);
std::vector<std::vector<double>> vec2DAdd(const std::vector<std::vector<double>>& c1, const std::vector<std::vector<double>>& buf);

std::vector<double> relu(const std::vector<double>& x);
std::vector<std::vector<double>> relu2D(const std::vector<std::vector<double>>& input);
std::vector<double> softmax(const std::vector<double>& x);

std::vector<std::vector<double>> chomp1d(const std::vector<std::vector<double>>& input, int padding);

std::vector<std::vector<double>> applyPadding(const std::vector<std::vector<double>>& input, int padding);

std::vector<std::vector<double>> conv1d(
    const std::vector<std::vector<double>>& input,
    const std::vector<std::vector<std::vector<double>>>& weight,
    const std::vector<double>& bias,
    int stride, int padding, int dilation);

double vecDot(const std::vector<double>& vec1, const std::vector<double>& vec2);
std::vector<double> matVecMul(const std::vector<std::vector<double>>& mat, const std::vector<double>& vec);
std::vector<double> linear_layer(const std::vector<double>& bias, const std::vector<std::vector<double>>& weight, const std::vector<double>& input, bool activation);
std::vector<double> composition_layer(
    const std::vector<std::vector<std::vector<double>>>& weight, // [contextdim, outdim, indim]
    const std::vector<std::vector<double>>& bias,               // [contextdim, outdim]
    const std::vector<double>& comp_weight,                     // [contextdim]
    const std::vector<double>& input,                           // [indim]
    bool activation);
#endif 