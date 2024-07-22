#include "util.h"
// #include <iostream>
#include <vector>
#include <algorithm> // For std::max and std::min

// Function to get the last column of a 2D vector
std::vector<double> getLastColumn(const std::vector<std::vector<double>>& matrix) {
    std::vector<double> last_column;
    for (const auto& row : matrix) {
        if (!row.empty()) {
            last_column.push_back(row.back());  // Get the last element of the row
        }
    }
    return last_column;
}

// Function to clamp elements of a vector to the range [-1, 1]
void clampToRange(std::vector<double>& vec, double min_val = -1.0, double max_val = 1.0) {
    for (double& element : vec) {
        element = std::max(min_val, std::min(element, max_val));
    }
}

std::vector<double> vecCat(const std::vector<double>& vec1, const std::vector<double>& vec2) {
    std::vector<double> result = vec1; // Copy the first vector into the result
    result.insert(result.end(), vec2.begin(), vec2.end()); // Insert the second vector at the end of the result
    return result;
}
// Function to perform vector addition
std::vector<double> vecAdd(const std::vector<double>& vec1, const std::vector<double>& vec2) {
    assert(vec1.size() == vec2.size());
    std::vector<double> result(vec1.size(), 0.0);
    for (size_t i = 0; i < vec1.size(); ++i) {
        result[i] = vec1[i] + vec2[i];
    }
    return result;
}
std::vector<std::vector<double>> vec2DAdd(const std::vector<std::vector<double>>& c1, const std::vector<std::vector<double>>& buf) {
    // Ensure that the dimensions match
    assert(c1.size() == buf.size() && c1[0].size() == buf[0].size());

    // Create a result vector with the same dimensions
    std::vector<std::vector<double>> result(c1.size(), std::vector<double>(c1[0].size(), 0.0));

    for (size_t i = 0; i < c1.size(); ++i) {
        for (size_t j = 0; j < c1[i].size(); ++j) {
            result[i][j] = c1[i][j] + buf[i][j];
        }
    }

    return result;
}

// Function to apply the ReLU activation
std::vector<double> relu(const std::vector<double>& x) {
    std::vector<double> result(x.size());
    for (size_t i = 0; i < x.size(); ++i) {
        result[i] = std::max(double(0.0), x[i]);
    }
    return result;
}

// Function to apply ReLU to a 2D vector
std::vector<std::vector<double>> relu2D(const std::vector<std::vector<double>>& input) {
    std::vector<std::vector<double>> result = input;  // Create a copy of the input

    for (auto& row : result) {
        for (auto& element : row) {
            element = std::max(double(0.0), element);  // Apply ReLU
        }
    }

    return result;
}
// Function to apply the softmax activation
std::vector<double> softmax(const std::vector<double>& x) {
    std::vector<double> exp_x(x.size());
    double sum_exp = 0.0;
    for (size_t i = 0; i < x.size(); ++i) {
        exp_x[i] = std::exp(x[i]);
        sum_exp += exp_x[i];
    }
    for (size_t i = 0; i < x.size(); ++i) {
        exp_x[i] /= sum_exp;
    }
    return exp_x;
}

std::vector<std::vector<double>> chomp1d(const std::vector<std::vector<double>>& input, int padding) {
    std::vector<std::vector<double>> result;

    for (const auto& row : input) {
        // Create a new row with the required number of elements removed from the end
        std::vector<double> new_row(row.begin(), row.end() - padding);
        result.push_back(new_row);
    }

    return result;
}

// Helper function to apply padding
std::vector<std::vector<double>> applyPadding(const std::vector<std::vector<double>>& input, int padding) {
    int C_in = input.size();
    int L_in = input[0].size();
    int padded_L_in = L_in + 2 * padding;

    std::vector<std::vector<double>> padded_input(C_in, std::vector<double>(padded_L_in, 0.0));

    for (int c_in = 0; c_in < C_in; ++c_in) {
        for (int l_in = 0; l_in < L_in; ++l_in) {
            padded_input[c_in][l_in + padding] = input[c_in][l_in];
        }
    }

    return padded_input;
}

std::vector<std::vector<double>> conv1d(
    const std::vector<std::vector<double>>& input,
    const std::vector<std::vector<std::vector<double>>>& weight,
    const std::vector<double>& bias,
    int stride = 1,
    int padding = 0,
    int dilation = 1) {

    int C_in = input.size();
    int L_in = input[0].size();
    int C_out = weight.size();
    int K = weight[0][0].size();

    // Calculate the output length
    int L_out = (L_in + 2 * padding - dilation * (K - 1) - 1) / stride + 1;

    // Initialize the output tensor
    std::vector<std::vector<double>> output(C_out, std::vector<double>(L_out, 0.0));

    // Apply padding to the input
    std::vector<std::vector<double>> input_padded = applyPadding(input, padding);

    // Perform the convolution
    for (int c_out = 0; c_out < C_out; ++c_out) {
        for (int l_out = 0; l_out < L_out; ++l_out) {
            for (int c_in = 0; c_in < C_in; ++c_in) {
                for (int k = 0; k < K; ++k) {
                    int l_in = l_out * stride + k * dilation;
                    output[c_out][l_out] += input_padded[c_in][l_in] * weight[c_out][c_in][k];
                }
            }
            if (!bias.empty()) {
                output[c_out][l_out] += bias[c_out];
            }
        }
    }

    return output;
}

// Function to perform vector dot product
double vecDot(const std::vector<double>& vec1, const std::vector<double>& vec2) {
    assert(vec1.size() == vec2.size());
    double result = 0.0;
    for (size_t i = 0; i < vec1.size(); ++i) {
        result += vec1[i] * vec2[i];
    }
    return result;
}

// Function to perform matrix-vector multiplication
std::vector<double> matVecMul(const std::vector<std::vector<double>>& mat, const std::vector<double>& vec) {
    assert(mat[0].size() == vec.size());
    std::vector<double> result(mat.size(), 0.0);
    for (size_t i = 0; i < mat.size(); ++i) {
        for (size_t j = 0; j < vec.size(); ++j) {
            result[i] += mat[i][j] * vec[j];
        }
    }
    return result;
}


// Function to apply the linear layer with activation
std::vector<double> linear_layer(const std::vector<double>& bias, const std::vector<std::vector<double>>& weight, const std::vector<double>& input, bool activation) {
    // Compute the linear transformation
    std::vector<double> output = vecAdd(bias, matVecMul(weight, input));

    // Apply the activation function
    if (activation) {
        return relu(output);
    }
    return output;
}

// Function to apply the composition layer
std::vector<double> composition_layer(
    const std::vector<std::vector<std::vector<double>>>& weight, // [contextdim][outdim][indim]
    const std::vector<std::vector<double>>& bias,               // [contextdim][outdim]
    const std::vector<double>& comp_weight,                     // [contextdim]
    const std::vector<double>& input,                           // [indim]
    bool activation = true) {
    // Ensure dimensions are correct
    assert(weight.size() == bias.size());
    assert(weight[0].size() == bias[0].size());
    assert(weight[0][0].size() == input.size());
    assert(comp_weight.size() == weight.size());

    size_t contextdim = weight.size();
    size_t outdim = weight[0].size();
    
    // Compute x = bias + weight * input for each context dimension
    std::vector<std::vector<double>> x(contextdim, std::vector<double>(outdim));
    for (size_t i = 0; i < contextdim; ++i) {
        x[i] = vecAdd(bias[i], matVecMul(weight[i], input));
    }

    // Compute output = comp_weight * sum(x)
    std::vector<double> output(outdim, 0.0);
    for (size_t i = 0; i < contextdim; ++i) {
        for (size_t j = 0; j < outdim; ++j) {
            output[j] += comp_weight[i] * x[i][j];
        }
    }

    // Apply activation function
    if (activation) {
        return relu(output);
    } else {
        return output;
    }
}