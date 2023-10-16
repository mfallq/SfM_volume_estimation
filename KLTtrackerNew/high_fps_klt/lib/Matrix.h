#pragma once
#include <initializer_list>
#include <algorithm>

namespace cvl {

/// A matrix class for small matrices and vectors with real-valued scalars,
/// primarily for geometry operations.
template<typename T, int Rows, int Cols>
class Matrix
{
public:
    T _data[Rows * Cols];

    //// Element access ////////////

    /// Get a pointer to the matrix or vector elements. The elements are stored in row-major order.
    T* data()
    {
        return _data;
    }

    /// Get a const pointer to the matrix or vector elements. The elements are stored in row-major order.
    const T* data() const
    {
        return _data;
    }

    /// Access element (row, col)
    T& operator()(unsigned int row, unsigned int col)
    {
        return _data[row * Cols + col];
    }

    /// Const access to element (row, col)
    const T& operator()(unsigned int row, unsigned int col) const
    {
        return _data[row * Cols + col];
    }

    /// Access element (i). Useful for vectors and row-major iteration over matrices.
    T& operator()(unsigned int i)
    {
        return _data[i];
    }

    /// Const access to element (i). Useful for vectors and row-major iteration over matrices.
    const T& operator()(unsigned int i) const
    {
        return _data[i];
    }

    const T *cbegin() const
    {
        return _data;
    }

    T *begin()
    {
        return _data;
    }

    const T *end() const
    {
        return _data + Rows * Cols;
    }

    T *end()
    {
        return _data + Rows * Cols;
    }

    unsigned int size() const
    {
        return Rows * Cols;
    }

    unsigned int cols() const
    {
        return Cols;
    }

    unsigned int rows() const
    {
        return Rows;
    }

    /// Default constructor
    Matrix()
    {
    }

    /// n-coefficient constructor, e.g Matrix2f m(1,2,3,4);
    template<typename... S>
    Matrix(T first_val, S... remaining_vals)
    {
        Matrix& a = *this;
        if (sizeof...(S) == 0) {
            for (unsigned int i = 0; i < Rows * Cols; ++i) {
                a(i) = first_val;
            }
        }
        else {
            static_assert(sizeof...(S) == Rows * Cols - 1, "Incorrect number of elements given.");
            T b[] = { first_val, T(remaining_vals)... };

            for (unsigned int i = 0; i < Rows * Cols; i++) {
                a(i) = b[i];
            }
        }
    }

    Matrix(T scalar)
    {
        Matrix& a = *this;
        for (unsigned int i = 0; i < Cols*Rows; ++i) {
            a(i) = scalar;
        }
    }

    Matrix(const T *coeffs)
    {
        Matrix& a = *this;
        for (unsigned int i = 0; i < Rows * Cols; i++) {
            a(i) = *coeffs++;
        }
    }

    template<class S>
    Matrix(const Matrix<S, Rows, Cols>& b)
    {
        Matrix& a = *this;
        for (int i = 0; i < Rows * Cols; i++) {
            a(i) = S(b(i));
        }
    }

    //// Elementwise arithmetic operations ////////////

    /// Add the elements of another matrix (elementwise addition)
    Matrix& operator+=(const Matrix& b)
    {
        Matrix& a = *this;
        for (unsigned int i = 0; i < Rows * Cols; i++) {
            a(i) += b(i);
        }
        return a;
    }

    /// Subtract the elements of another matrix (elementwise subtraction)
    Matrix& operator-=(const Matrix& b)
    {
        Matrix& a = *this;
        for (unsigned int i = 0; i < Rows * Cols; i++) {
            a(i) -= b(i);
        }
        return a;
    }

    /// Multiply by a scalar
    Matrix& operator*=(const T& s)
    {
        Matrix& a = *this;
        for (unsigned int i = 0; i < Rows * Cols; i++) {
            a(i) *= s;
        }
        return a;
    }

    /// Divide by a scalar
    Matrix& operator/=(const T& s)
    {
        Matrix& a = *this;
        for (unsigned int i = 0; i < Rows * Cols; i++) {
            a(i) /= s;
        }
        return a;
    }

    /// Elementwise matrix addtion
    Matrix operator+(const Matrix& b) const
    {
        Matrix c;
        const Matrix& a = *this;
        for (unsigned int i = 0; i < Rows * Cols; i++) {
            c(i) = a(i) + b(i);
        }
        return c;
    }

    /// Elementwise matrix subtraction
    Matrix operator-(const Matrix& b) const
    {
        Matrix c;
        const Matrix& a = *this;
        for (unsigned int i = 0; i < Rows * Cols; i++) {
            c(i) = a(i) - b(i);
        }
        return c;
    }

    /// Element negation
    Matrix operator-() const
    {
        Matrix b;
        const Matrix& a = *this;
        for (unsigned int i = 0; i < Rows * Cols; i++) {
            b(i) = -a(i);
        }
        return b;
    }

    /// Multiply by a scalar
    Matrix operator*(const T& s) const
    {
        Matrix b;
        const Matrix& a = *this;
        for (unsigned int i = 0; i < Rows * Cols; i++) {
            b(i) = a(i) * s;
        }
        return b;
    }

    /// Divide by a scalar
    Matrix operator/(const T& s) const
    {
        Matrix b;
        const Matrix& a = *this;
        for (unsigned int i = 0; i < Rows * Cols; i++) {
            b(i) = a(i) / s;
        }
        return b;
    }

    //// Constant initializers /////////////

    /// Return a matrix or vector with all elements set to zero.
    static Matrix Zero()
    {
        Matrix a;
        a.setZero();
        return a;
    }

    /// Return a matrix or vector with all elements set to one.
    static Matrix Ones()
    {
        Matrix a;
        a.setOnes();
        return a;
    }

    /// Return an identity matrix.
    static Matrix Identity()
    {
        Matrix a;
        a.setIdentity();
        return a;
    }

    /// Set all elements to zero.
    Matrix& setZero()
    {
        Matrix& a = *this;
        for (unsigned int i = 0; i < Rows * Cols; i++) {
            a(i) = T(0);
        }
        return *this;
    }

    /// Set all elements to one.
    Matrix& setOnes()
    {
        Matrix& a = *this;
        for (unsigned int i = 0; i < Rows * Cols; i++) {
            a(i) = T(1);
        }
        return *this;
    }

    /// Set a matrix to identity.
    Matrix& setIdentity()
    {
        Matrix& a = *this;

        for (unsigned int row = 0; row < Rows; row++) {
            for (unsigned int col = 0; col < Cols; col++) {
                a(row, col) = (row == col) ? T(1) : T(0);
            }
        }
        return *this;
    }

    //// Various matrix operations ///////////////////////

    /// Matrix transpose
    Matrix<T, Cols, Rows> transpose() const
    {
        Matrix<T, Cols, Rows> b;

        const Matrix& a = *this;
        for (unsigned int row = 0; row < Rows; row++) {
            for (unsigned int col = 0; col < Cols; col++) {
                b(col, row) = a(row, col);
            }
        }
        return b;
    }

    /// Matrix determinant
    T determinant() const
    {
        static_assert(Rows == Cols, "Matrix must be square.");
        static_assert(Rows <= 4, "determinant() is not implemented for this matrix size.");

        const Matrix& a = *this;

        if (Rows == 1) {
            return a(0, 0);
        }
        else if (Rows == 2) {
            return a(0, 0) * a(1, 1) - a(0, 1) * a(1, 0);
        }
        else if (Rows == 3) {

            T M00 = a(1, 1) * a(2, 2) - a(1, 2) * a(2, 1);
            T M10 = a(1, 2) * a(2, 0) - a(1, 0) * a(2, 2);
            T M20 = a(1, 0) * a(2, 1) - a(1, 1) * a(2, 0);

            return a(0, 0) * M00 + a(0, 1) * M10 + a(0, 2) * M20;
        }
        else if (Rows == 4) {

            T a00 = a(0, 0), a01 = a(0, 1), a02 = a(0, 2), a03 = a(0, 3);
            T a10 = a(1, 0), a11 = a(1, 1), a12 = a(1, 2), a13 = a(1, 3);
            T a20 = a(2, 0), a21 = a(2, 1), a22 = a(2, 2), a23 = a(2, 3);
            T a30 = a(3, 0), a31 = a(3, 1), a32 = a(3, 2), a33 = a(3, 3);

            return a00 * (a11*(a22*a33 - a23*a32) + a12*(a23*a31 - a21*a33) + a13*(a21*a32 - a22*a31))
                 + a01 * (a10*(a23*a32 - a22*a33) + a12*(a20*a33 - a23*a30) + a13*(a22*a30 - a20*a32))
                 + a02 * (a10*(a21*a33 - a23*a31) + a11*(a23*a30 - a20*a33) + a13*(a20*a31 - a21*a30))
                 + a03 * (a10*(a22*a31 - a21*a32) + a11*(a20*a32 - a22*a30) + a12*(a21*a30 - a20*a31));
        }
    }

    /// Matrix inverse
    Matrix inverse() const
    {
        static_assert(Rows == Cols, "Matrix must be square.");
        static_assert(Rows <= 4, "inverse() is not implemented for matrices with more than 4 rows.");

        const Matrix& a = *this;
        Matrix b;

        if (Rows == 1) {
            b(0) = T(1) / a(0);
        }
        else if (Rows == 2) {

            T idet = T(1) / determinant();

            b(0, 0) = a(1, 1) * idet;
            b(0, 1) = -a(0, 1) * idet;
            b(1, 0) = -a(1, 0) * idet;
            b(1, 1) = a(0, 0) * idet;
        }
        else if (Rows == 3) {

            Matrix M; // Minors
            T idet; // Determinant

            M(0, 0) = a(1, 1) * a(2, 2) - a(1, 2) * a(2, 1);
            M(0, 1) = a(0, 2) * a(2, 1) - a(0, 1) * a(2, 2);
            M(0, 2) = a(0, 1) * a(1, 2) - a(0, 2) * a(1, 1);

            M(1, 0) = a(1, 2) * a(2, 0) - a(1, 0) * a(2, 2);
            M(1, 1) = a(0, 0) * a(2, 2) - a(0, 2) * a(2, 0);
            M(1, 2) = a(0, 2) * a(1, 0) - a(0, 0) * a(1, 2);

            M(2, 0) = a(1, 0) * a(2, 1) - a(1, 1) * a(2, 0);
            M(2, 1) = a(0, 1) * a(2, 0) - a(0, 0) * a(2, 1);
            M(2, 2) = a(0, 0) * a(1, 1) - a(0, 1) * a(1, 0);

            idet = T(1) / (a(0, 0) * M(0, 0) + a(0, 1) * M(1, 0) + a(0, 2) * M(2, 0));

            return (M * idet);
        }
        else {

            T idet = T(1) / determinant();

            T a00 = a(0, 0), a01 = a(0, 1), a02 = a(0, 2), a03 = a(0, 3);
            T a10 = a(1, 0), a11 = a(1, 1), a12 = a(1, 2), a13 = a(1, 3);
            T a20 = a(2, 0), a21 = a(2, 1), a22 = a(2, 2), a23 = a(2, 3);
            T a30 = a(3, 0), a31 = a(3, 1), a32 = a(3, 2), a33 = a(3, 3);

            b(0, 0) = (a11*a22*a33 - a11*a23*a32 - a12*a21*a33 + a12*a23*a31 + a13*a21*a32 - a13*a22*a31) * idet;
            b(0, 1) = (a01*a23*a32 - a01*a22*a33 + a02*a21*a33 - a02*a23*a31 - a03*a21*a32 + a03*a22*a31) * idet;
            b(0, 2) = (a01*a12*a33 - a01*a13*a32 - a02*a11*a33 + a02*a13*a31 + a03*a11*a32 - a03*a12*a31) * idet;
            b(0, 3) = (a01*a13*a22 - a01*a12*a23 + a02*a11*a23 - a02*a13*a21 - a03*a11*a22 + a03*a12*a21) * idet;

            b(1, 0) = (a10*a23*a32 - a10*a22*a33 + a12*a20*a33 - a12*a23*a30 - a13*a20*a32 + a13*a22*a30) * idet;
            b(1, 1) = (a00*a22*a33 - a00*a23*a32 - a02*a20*a33 + a02*a23*a30 + a03*a20*a32 - a03*a22*a30) * idet;
            b(1, 2) = (a00*a13*a32 - a00*a12*a33 + a02*a10*a33 - a02*a13*a30 - a03*a10*a32 + a03*a12*a30) * idet;
            b(1, 3) = (a00*a12*a23 - a00*a13*a22 - a02*a10*a23 + a02*a13*a20 + a03*a10*a22 - a03*a12*a20) * idet;

            b(2, 0) = (a10*a21*a33 - a10*a23*a31 - a11*a20*a33 + a11*a23*a30 + a13*a20*a31 - a13*a21*a30) * idet;
            b(2, 1) = (a00*a23*a31 - a00*a21*a33 + a01*a20*a33 - a01*a23*a30 - a03*a20*a31 + a03*a21*a30) * idet;
            b(2, 2) = (a00*a11*a33 - a00*a13*a31 - a01*a10*a33 + a01*a13*a30 + a03*a10*a31 - a03*a11*a30) * idet;
            b(2, 3) = (a00*a13*a21 - a00*a11*a23 + a01*a10*a23 - a01*a13*a20 - a03*a10*a21 + a03*a11*a20) * idet;

            b(3, 0) = (a10*a22*a31 - a10*a21*a32 + a11*a20*a32 - a11*a22*a30 - a12*a20*a31 + a12*a21*a30) * idet;
            b(3, 1) = (a00*a21*a32 - a00*a22*a31 - a01*a20*a32 + a01*a22*a30 + a02*a20*a31 - a02*a21*a30) * idet;
            b(3, 2) = (a00*a12*a31 - a00*a11*a32 + a01*a10*a32 - a01*a12*a30 - a02*a10*a31 + a02*a11*a30) * idet;
            b(3, 3) = (a00*a11*a22 - a00*a12*a21 - a01*a10*a22 + a01*a12*a20 + a02*a10*a21 - a02*a11*a20) * idet;
        }

        return b;
    }

    /// Matrix multiplication
    template<int N>
    Matrix<T, Rows, N> operator*(const Matrix<T, Cols, N>& b) const
    {
        Matrix<T, Rows, N> c;
        const Matrix& a = *this;
        for (unsigned int row = 0; row < Rows; row++) {
            for (unsigned int col = 0; col < N; col++) {

                T sum = T(0);
                for (unsigned int i = 0; i < Cols; i++) {
                    sum += a(row, i) * b(i, col);
                }
                c(row, col) = sum;
            }
        }
        return c;
    }

    /// Compute the inner product of this and another vector
    template<int Rows2, int Cols2>
    T dot(const Matrix<T, Rows2, Cols2>& b) const
    {
        static_assert((Cols == 1 || Rows == 1),
            "The dot product is only defined for vectors.");
        static_assert((Cols2 == 1 || Rows2 == 1),
            "The dot product is only defined for vectors.");
        static_assert(Rows * Cols == Rows2 * Cols2,
            "The vectors in a dot product must have the same number of elements.");

        T sum = T(0);
        const Matrix& a = *this;
        for (unsigned int i = 0; i < Rows * Cols; i++) {
            sum += a(i) * b(i);
        }
        return sum;
    }

    /// Compute the cross product of this and another vector
    Matrix cross(const Matrix& b) const
    {
        static_assert((Rows == 3 && Cols == 1) || (Rows == 1 && Cols == 3),
            "The cross product is only defined for vectors of length 3.");

        const Matrix& a = *this;
        Matrix c(
            a(1) * b(2) - a(2) * b(1),
            a(2) * b(0) - a(0) * b(2),
            a(0) * b(1) - a(1) * b(0));
        return c;
    }

    /// Return the 3-element vector as cross product matrix
    Matrix<T, 3, 3> crossMatrix() const
    {
        static_assert((Rows == 3 && Cols == 1) || (Rows == 1 && Cols == 3),
            "The cross product matrix is only defined for vectors of length 3.");

        const Matrix& a = *this;
        Matrix<T, 3, 3> b(
            0, -a(2), a(1),
            a(2), 0, -a(0),
            -a(1), a(0), 0);
        return b;
    }

    ///Matrix trace
    T trace() const
    {
        static_assert(Rows == Cols, "The matrix trace is only defined for square matrices.");

        const Matrix& a = *this;
        T tr = T(0);
        for (unsigned int i = 0; i < std::min(Rows, Cols); i++) {
            tr += a(i, i);
        }
        return tr;
    }

    /// The sum of all elements
    T sum() const
    {
        const Matrix& a = *this;
        T sum = T(0);
        for (unsigned int i = 0; i < Rows * Cols; i++) {
            sum += a(i);
        }
        return sum;
    }

    /// The sum of the squared elements
    T squaredNorm() const
    {
        const Matrix& a = *this;
        T sum = T(0);
        for (unsigned int i = 0; i < Rows * Cols; i++) {
            sum += a(i) * a(i); // does not do the right thing for complex values
        }
        return sum;
    }

    /// The squared vector L2 norm, with a different name
    T squaredLength() const
    {
        static_assert(Cols == 1 || Rows == 1,
            "length() is only defined for vectors. Use norm() with matrices.");
        return squaredNorm();
    }

    /// The L2 norm
    T norm() const
    {
        return std::sqrt(squaredNorm());
    }

    /// The vector L2 norm, with a different name
    T length() const
    {
        static_assert(Cols == 1 || Rows == 1,
            "length() is only defined for vectors. Use norm() with matrices.");
        return norm();
    }

    /// Elementwise absolute value
    Matrix cwiseAbs() const
    {
        Matrix<T, Rows, Cols> b;

        const Matrix& a = *this;
        for (unsigned int i = 0; i < Rows * Cols; i++) {
            b(i) = fabs(a(i));
        }
        return b;
    }

    /// Maximum coefficient
    T maxCoeff() const
    {
        const Matrix& a = *this;

        T v = a(0);
        for (unsigned int i = 1; i < Rows * Cols; i++) {
            if (v < a(i)) {
                v = a(i);
            }
        }
        return v;
    }

    /// Minimum coefficient
    T minCoeff() const
    {
        const Matrix& a = *this;

        T v = a(0);
        for (unsigned int i = 1; i < Rows * Cols; i++) {
            if (v > a(i)) {
                v = a(i);
            }
        }
        return v;
    }

    // Normalize the matrix with its own L2-norm
    void normalize()
    {
        (*this) *= (T(1) / norm());
    }

    // Return the matrix normalized with its own L2-norm
    Matrix normalized() const
    {
        return (*this) * (T(1) / norm());
    }

    Matrix<T, Rows - 1, Cols> hnormalized() const
    {
        Matrix<T, Rows - 1, Cols> b;
        const Matrix& a = *this;
        for (unsigned int col = 0; col < Cols; col++) {
            for (unsigned int row = 0; row < Rows - 1; row++) {
                b(row, col) = a(row, col) / a(Rows - 1, col);
            }
        }
        return b;
    }

    Matrix<T, Rows + 1, Cols> homogeneous() const
    {
        Matrix<T, Rows + 1, Cols> b;
        const Matrix& a = *this;
        for (unsigned int col = 0; col < Cols; col++) {
            for (unsigned int row = 0; row < Rows; row++) {
                b(row, col) = a(row, col);
            }
            b(Rows, col) = T(1.0);
        }
        return b;
    }

    /// Test whether the matrix or vector has at least one NaN element
    bool hasNaN() const
    {
        const Matrix& a = *this;
        for (int i = 0; i < Rows * Cols; i++) {
            if (a(i) != a(i)) return true;
        }
        return false;
    }

    bool allFinite() const
    {
        const Matrix& a = *this;
        for (int i = 0; i < Rows * Cols; i++) {
            if (std::isinf(a(i)) || std::isnan(a(i))) return false;
        }
        return true;
    }

    bool operator==(const Matrix& b) const
    {
        const Matrix& a = *this;

        for (unsigned int i = 0; i < Rows * Cols; i++) {
            if (a(i) != b(i)) return false;
        }
        return true;
    }

    bool isApprox(const Matrix& B, double prec) const
    {
        const Matrix& A = *this;

        double sum_a = abs(A).sum();
        double sum_b = abs(B).sum();
        double min_ab = (sum_a < sum_b ? sum_a : sum_b);

        return abs(B - A).sum() < (prec * prec) * min_ab; // same as Eigen::Matrix::isApprox()
    }

    /// Test whether the L2 norm of the difference between two matrices or vectors is less than d.
    bool isAlmost(const Matrix& B, double d) const
    {
        const Matrix& A = *this;
        return (B - A).squaredNorm() < d * d;
    }

    Matrix<T, Rows, 1> col(int i) const
    {
        Matrix<T, Rows, 1> b;
        const Matrix& a = *this;
        for (unsigned int j = 0; j < Rows; j++) {
            b(j) = a(j, i);
        }
        return b;
    }

    template<int Height, int Width>
    Matrix<T, Height, Width> topLeftCorner() const
    {
        static_assert((Height <= Rows) && (Width <= Cols), "Matrix block size is out of bounds.");

        Matrix<T, Height, Width> b;

        const Matrix& a = *this;
        for (unsigned int row = 0; row < Height; row++) {
            for (unsigned int col = 0; col < Width; col++) {
                b(row, col) = a(row, col);
            }
        }
        return b;
    }
};


/// Free scalar-matrix multiplication s * matrix
template<typename T, int Rows, int Cols>
Matrix<T, Rows, Cols> operator*(const T& s, const Matrix<T, Rows, Cols>& a)
{
    Matrix<T, Rows, Cols> b;
    for (unsigned int i = 0; i < Rows * Cols; i++) {
        b(i) = a(i) * s;
    }
    return b;
}

/// Compute the inner product of this and another vector
template<class T, int Rows, int Cols, int Rows2, int Cols2>
T dot(const Matrix<T, Rows, Cols>& a, const Matrix<T, Rows2, Cols2>& b)
{
    return a.dot(b);
}

/// Elementwise absolute value
template<typename T, int Rows, int Cols>
Matrix<T, Rows, Cols> abs(const Matrix<T, Rows, Cols>& a)
{
    return a.cwiseAbs();
}

/// Elementwise square root
template<typename T, int Rows, int Cols>
Matrix<T, Rows, Cols> sqrt(const Matrix<T, Rows, Cols>& a)
{
    Matrix<T, Rows, Cols> b;
    for (unsigned int i = 0; i < Rows * Cols; i++) {
        b(i) = sqrt(a(i));
    }
    return b;
}

using Vector2f = Matrix<float, 2, 1>;
using Vector3f = Matrix<float, 3, 1>;
using Vector4f = Matrix<float, 4, 1>;

using Vector2d = Matrix<double, 2, 1>;
using Vector3d = Matrix<double, 3, 1>;
using Vector4d = Matrix<double, 4, 1>;

using Matrix2f = Matrix<float, 2, 2>;
using Matrix3f = Matrix<float, 3, 3>;
using Matrix34f = Matrix<float, 3, 4>;
using Matrix4f = Matrix<float, 4, 4>;

using Matrix2d = Matrix<double, 2, 2>;
using Matrix3d = Matrix<double, 3, 3>;
using Matrix34d = Matrix<double, 3, 4>;
using Matrix4d = Matrix<double, 4, 4>;

} // cvl::
