// #pragma once

// #include <vector>
// #include <stdexcept>
// #include <initializer_list>

// template<typename T>
// class Matrix
// {
// public:
//     Matrix() : rows_(0), cols_(0) {}

//     Matrix(size_t rows, size_t cols, const T& value = T())
//         : rows_(rows), cols_(cols), data_(rows * cols, value)
//     {
//     }

//     Matrix(size_t rows, size_t cols, std::initializer_list<T> values)
//         : rows_(rows), cols_(cols), data_(values)
//     {
//         if (values.size() != rows * cols)
//             throw std::runtime_error("Initializer size mismatch");
//     }

//     size_t rows() const { return rows_; }
//     size_t cols() const { return cols_; }

//     T& operator()(size_t r, size_t c)
//     {
//         return data_[r * cols_ + c];
//     }

//     const T& operator()(size_t r, size_t c) const
//     {
//         return data_[r * cols_ + c];
//     }

//     // Matrix addition
//     Matrix operator+(const Matrix& rhs) const
//     {
//         check_same_size(rhs);
//         Matrix result(rows_, cols_);
//         for (size_t i = 0; i < data_.size(); ++i)
//             result.data_[i] = data_[i] + rhs.data_[i];
//         return result;
//     }

//     // Matrix subtraction
//     Matrix operator-(const Matrix& rhs) const
//     {
//         check_same_size(rhs);
//         Matrix result(rows_, cols_);
//         for (size_t i = 0; i < data_.size(); ++i)
//             result.data_[i] = data_[i] - rhs.data_[i];
//         return result;
//     }

//     Matrix& operator+=(const Matrix& rhs)
//     {
//         check_same_size(rhs);
//         for (size_t i = 0; i < data_.size(); ++i)
//             data_[i] += rhs.data_[i];
//         return *this;
//     }

//     Matrix& operator-=(const Matrix& rhs)
//     {
//         check_same_size(rhs);
//         for (size_t i = 0; i < data_.size(); ++i)
//             data_[i] -= rhs.data_[i];
//         return *this;
//     }

//     // Matrix-matrix multiplication
//     Matrix operator*(const Matrix& rhs) const
//     {
//         if (cols_ != rhs.rows_)
//             throw std::runtime_error("Dimension mismatch in multiplication");

//         Matrix result(rows_, rhs.cols_, T());

//         for (size_t i = 0; i < rows_; ++i)
//         {
//             for (size_t k = 0; k < cols_; ++k)
//             {
//                 T temp = (*this)(i, k);
//                 for (size_t j = 0; j < rhs.cols_; ++j)
//                 {
//                     result(i, j) += temp * rhs(k, j);
//                 }
//             }
//         }

//         return result;
//     }

//     // Matrix-vector multiplication
//     std::vector<T> operator*(const std::vector<T>& vec) const
//     {
//         if (cols_ != vec.size())
//             throw std::runtime_error("Dimension mismatch in matrix-vector multiplication");

//         std::vector<T> result(rows_, T());

//         for (size_t i = 0; i < rows_; ++i)
//         {
//             for (size_t j = 0; j < cols_; ++j)
//             {
//                 result[i] += (*this)(i, j) * vec[j];
//             }
//         }

//         return result;
//     }

// private:
//     size_t rows_;
//     size_t cols_;
//     std::vector<T> data_;

//     void check_same_size(const Matrix& rhs) const
//     {
//         if (rows_ != rhs.rows_ || cols_ != rhs.cols_)
//             throw std::runtime_error("Matrix size mismatch");
//     }
// };









#pragma once

#include <cstddef>

template<typename T, std::size_t Rows, std::size_t Cols>
class Matrix
{
public:
    // Storage (row-major)
    T data[Rows][Cols] = {};

    // Element access
    constexpr T& operator()(std::size_t r, std::size_t c)
    {
        return data[r][c];
    }

    constexpr const T& operator()(std::size_t r, std::size_t c) const
    {
        return data[r][c];
    }

    // Fill constructor (optional)
    constexpr explicit Matrix(const T& value)
    {
        for (std::size_t i = 0; i < Rows; ++i)
            for (std::size_t j = 0; j < Cols; ++j)
                data[i][j] = value;
    }

    constexpr Matrix() = default;

    // Addition
    constexpr Matrix operator+(const Matrix& rhs) const
    {
        Matrix result;
        for (std::size_t i = 0; i < Rows; ++i)
            for (std::size_t j = 0; j < Cols; ++j)
                result(i,j) = data[i][j] + rhs(i,j);
        return result;
    }

    // Subtraction
    constexpr Matrix operator-(const Matrix& rhs) const
    {
        Matrix result;
        for (std::size_t i = 0; i < Rows; ++i)
            for (std::size_t j = 0; j < Cols; ++j)
                result(i,j) = data[i][j] - rhs(i,j);
        return result;
    }

    constexpr Matrix& operator+=(const Matrix& rhs)
    {
        for (std::size_t i = 0; i < Rows; ++i)
            for (std::size_t j = 0; j < Cols; ++j)
                data[i][j] += rhs(i,j);
        return *this;
    }

    constexpr Matrix& operator-=(const Matrix& rhs)
    {
        for (std::size_t i = 0; i < Rows; ++i)
            for (std::size_t j = 0; j < Cols; ++j)
                data[i][j] -= rhs(i,j);
        return *this;
    }

    // Scalar multiply
    constexpr Matrix operator*(const T& scalar) const
    {
        Matrix result;
        for (std::size_t i = 0; i < Rows; ++i)
            for (std::size_t j = 0; j < Cols; ++j)
                result(i,j) = data[i][j] * scalar;
        return result;
    }

    constexpr Matrix& operator*=(const T& scalar)
    {
        for (std::size_t i = 0; i < Rows; ++i)
            for (std::size_t j = 0; j < Cols; ++j)
                data[i][j] *= scalar;
        return *this;
    }

    // Matrix × Matrix
    template<std::size_t OtherCols>
    constexpr Matrix<T, Rows, OtherCols>
    operator*(const Matrix<T, Cols, OtherCols>& rhs) const
    {
        Matrix<T, Rows, OtherCols> result{};

        // Loop order optimized for cache:
        for (std::size_t i = 0; i < Rows; ++i)
        {
            for (std::size_t k = 0; k < Cols; ++k)
            {
                const T temp = data[i][k];
                for (std::size_t j = 0; j < OtherCols; ++j)
                {
                    result(i,j) += temp * rhs(k,j);
                }
            }
        }

        return result;
    }

    // Matrix × Vector (as fixed-size array)
    constexpr T* multiply(const T (&vec)[Cols], T (&result)[Rows]) const
    {
        for (std::size_t i = 0; i < Rows; ++i)
        {
            result[i] = T{};
            for (std::size_t j = 0; j < Cols; ++j)
            {
                result[i] += data[i][j] * vec[j];
            }
        }
        return result;
    }
};