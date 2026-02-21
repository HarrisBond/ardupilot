#pragma once
#include <cstddef>

template<typename T, std::size_t Rows, std::size_t Cols>
class Matrix
{
public:
    T data[Rows][Cols] = {};

    Matrix() noexcept = default;

    explicit Matrix(const T& value) noexcept
    {
        for (std::size_t i = 0; i < Rows; ++i)
            for (std::size_t j = 0; j < Cols; ++j)
                data[i][j] = value;
    }

    // Non-const access
    T& operator()(std::size_t r, std::size_t c) noexcept
    {
        return data[r][c];
    }

    // Const access  ← REQUIRED
    const T& operator()(std::size_t r, std::size_t c) const noexcept
    {
        return data[r][c];
    }

    // Addition
    Matrix operator+(const Matrix& rhs) const noexcept
    {
        Matrix result;
        for (std::size_t i = 0; i < Rows; ++i)
            for (std::size_t j = 0; j < Cols; ++j)
                result(i,j) = data[i][j] + rhs(i,j);
        return result;
    }

    // Subtraction
    Matrix operator-(const Matrix& rhs) const noexcept
    {
        Matrix result;
        for (std::size_t i = 0; i < Rows; ++i)
            for (std::size_t j = 0; j < Cols; ++j)
                result(i,j) = data[i][j] - rhs(i,j);
        return result;
    }

    // Matrix multiplication
    template<std::size_t OtherCols>
    Matrix<T, Rows, OtherCols>
    operator*(const Matrix<T, Cols, OtherCols>& rhs) const noexcept
    {
        Matrix<T, Rows, OtherCols> result{};

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
};

// operator*(T scalar, const Matrix<T, Rows, Cols>& mat) noexcept
// {
//     Matrix<T, Rows, Cols> result;

//     for (std::size_t i = 0; i < Rows; ++i)
//         for (std::size_t j = 0; j < Cols; ++j)
//             result(i,j) = scalar * mat(i,j);

//     return result;
// }