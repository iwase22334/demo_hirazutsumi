//------------------------------------------------------------
// @author: Iwase Hajime
// @date : 2016.02.09
// @Description : This software is Calculation library of linear algebra.
//	It has been created in order to calculate,
//  such as coordinate transformation and the rotation matrix of the robot .
// @Note : Inverse matrix is currently unavailable.
//------------------------------------------------------------

#ifndef LINEAR_H
#define LINEAR_H

#include <iostream>
#include <algorithm>
#include <functional>
#include <cmath>
#include <cassert>

namespace la{

using std::min;
using std::cout;
using std::endl;
using std::ostream;
using std::sin;
using std::cos;
using std::asin;
using std::acos;
using std::atan2;
using std::sqrt;
using std::abs;

//----------------------------------------------------------
// Forward declaration
//----------------------------------------------------------
// Matrix
template<typename Tp, int M, int N> class Matrix;
// Operations for Matrix
template<typename Tp, int N> struct DetOperation;
template<typename Tp, int N> double det(const Matrix<Tp, N, N>&);
template<typename Tp, int L, int M, int N> void svd(const Matrix<Tp, M, N>&, Matrix<Tp, M, L>&, Matrix<Tp, L, L>&, Matrix<Tp, L, N>&);
template<typename Tp, int N> Matrix<Tp, N, 1> eigenvalue(const Matrix<Tp, N, N>&);
template<typename Tp, int N> Matrix<Tp, N, 1> jacobi_method(const Matrix<Tp, N, N>&);
template<typename Tp, int N> Matrix<Tp, N, 1> eigenvector(const Matrix<Tp, N, N>&, const Tp&);
template<typename Tp, int N> struct LU_decomposition;
template<typename Tp, int N> struct JacobiMethod;

template<typename Tp, int M, int N> void swap_row(Matrix<Tp, M, N>&, const int, const int);
template<typename Tp, int M, int N> int max_index(const Matrix<Tp, M, N>& );
template<typename Tp, int M, int N> Tp max_value(const Matrix<Tp, M, N>& );
template<typename Tp, int M, int N> int max_abs_index(const Matrix<Tp, M, N>& );
template<typename Tp, int M, int N> Tp max_abs_value(const Matrix<Tp, M, N>& );
template<typename Tp, int M, int N> Matrix<Tp, M, N> operator + (const Matrix<Tp, M, N>&, const Matrix<Tp, M, N>&);
template<typename Tp, int M, int N> Matrix<Tp, M, N> operator - (const Matrix<Tp, M, N>&, const Matrix<Tp, M, N>&);
template<typename Tp, int L, int M, int N> Matrix<Tp, M, L> operator * (const Matrix<Tp, M, N>&, const Matrix<Tp, N, L>&);
template<typename Tp, int M, int N> Matrix<Tp, M, N> operator * (const int, const Matrix<Tp, M, N>&);
template<typename Tp, int M, int N> Matrix<Tp, M, N> operator * (const Matrix<Tp, M, N>&, const int);	
template<typename Tp, int M, int N> Matrix<Tp, M, N> operator * (const float, const Matrix<Tp, M, N>&);
template<typename Tp, int M, int N> Matrix<Tp, M, N> operator * (const Matrix<Tp, M, N>&, const float);
template<typename Tp, int M, int N> Matrix<Tp, M, N> operator * (const double, const Matrix<Tp, M, N>&);
template<typename Tp, int M, int N> Matrix<Tp, M, N> operator * (const Matrix<Tp, M, N>&, const double);

template<typename Tp, int M, int N> Matrix<Tp, M, N> operator / (const Matrix<Tp, M, N>&, const int);	
template<typename Tp, int M, int N> Matrix<Tp, M, N> operator / (const Matrix<Tp, M, N>&, const float);
template<typename Tp, int M, int N> Matrix<Tp, M, N> operator / (const Matrix<Tp, M, N>&, const double);
template<typename Tp, int M, int N> ostream& operator << (ostream &, Matrix<Tp, M, N>&);

template<typename Tp, int M, int N> Tp norm(const Matrix<Tp, M, N>&);

// Vector
template<typename Tp, int N> class Vector;
// Operations for Vector
template<typename Tp, int N> Tp norm(const Vector<Tp, N>&);
template<typename Tp, int N> Tp angle(const Vector<Tp, N>&, const Vector<Tp, N>&);

// RoateMatrix
template<typename Tp> class RotateMatrix;
// Operations for RotateMatrix
template<typename Tp> Tp alpha(const RotateMatrix<Tp>&);
template<typename Tp> Tp beta(const RotateMatrix<Tp>&);
template<typename Tp> Tp gamma(const RotateMatrix<Tp>&);

// Objects to identify operations
struct AddOperation {};
struct SubOperation {};
struct ScaleOperation {};
struct MulOperation {};
struct MatMulOperation {};
struct TransOperation {};

//----------------------------------------------------------
// Matrix class for numerical computation of linear algebra
//----------------------------------------------------------
template<typename Tp, int M, int N>
class Matrix{
public:
	typedef Matrix<Tp, (M < N ? M : N), 1> diag_type;
	enum { E_Rows = M, E_Cols = N, E_Channels = E_Rows * E_Cols, E_Diags = (M < N ? M : N)};

	static Matrix all(Tp);
	static Matrix zeros();
	static Matrix ones();
	static Matrix eye();
	static Matrix diag(const diag_type&);

protected:
	Tp val[M * N];

public:
	Matrix();
	Matrix(Tp);
	Matrix(Tp, Tp); 
	Matrix(Tp, Tp, Tp); 
	Matrix(Tp, Tp, Tp, Tp); 
	Matrix(Tp, Tp, Tp, Tp, Tp);
	Matrix(Tp, Tp, Tp, Tp, Tp, Tp);
	Matrix(Tp, Tp, Tp, Tp, Tp, Tp, Tp);
	Matrix(Tp, Tp, Tp, Tp, Tp, Tp, Tp, Tp);
	Matrix(Tp, Tp, Tp, Tp, Tp, Tp, Tp, Tp, Tp);
	// 1*16, 16*1, 4*4 Matrix
	Matrix(Tp, Tp, Tp, Tp, Tp, Tp, Tp, Tp, Tp, Tp, Tp, Tp, Tp, Tp, Tp, Tp);
	explicit Matrix(const Tp*);
	Matrix(const Matrix<Tp, M, N> &);

	// Add operation Constructor
	Matrix(const Matrix<Tp, M, N>&, const Matrix<Tp, M, N>&, AddOperation);
	// Sub operation Constructor
	Matrix(const Matrix<Tp, M, N>&, const Matrix<Tp, M, N>&, SubOperation);
	// Scale operation Constructor
	template<typename Tp2> Matrix(const Matrix<Tp, M, N>& , Tp2, ScaleOperation);
	// Mul operation Constructor
	Matrix(const Matrix<Tp, M, N>& , const Matrix<Tp, M, N>& , MulOperation);
	// Matrix multiple operation Constructor
	template<int L> Matrix(const Matrix<Tp, M, L>&, const Matrix<Tp, L, N>&, MatMulOperation);
	// Transposition operation Constructor
	Matrix(const Matrix<Tp, N, M>&, TransOperation);
	
	// Row vector accessor 
	Matrix<Tp, 1, N> row(const int) const;
	// Column vector accessor
	Matrix<Tp, M, 1> col(const int) const;
	// diag vector accessor
	diag_type diag() const;

    // Extract part of the matrix
    template<int K, int L> Matrix<Tp, K, L> get_minor(int, int ) const;

	// Transpose the matrix
	Matrix<Tp, N, M> t() const;

	// Invert matrix the matrix
	virtual Matrix<Tp, M, N> inv() const;

	// Multiply two matrices element-wise
	Matrix<Tp, M, N> mul(const Matrix<Tp, M, N>&) const;

	// Accessor
	const Tp& operator ()(int, int) const;
	Tp& operator ()(int, int);

	// 1D accessor
	const Tp& operator ()(int) const;
	Tp& operator ()(int);

	Matrix<Tp, M, N> operator - (void);
	Matrix<Tp, M, N>& operator = (const Matrix<Tp, M, N>&);
	Matrix<Tp, M, N>& operator = (const Matrix<Tp, M, N>&&);

};

//----------------------------------------------------------
// Vector class 
//----------------------------------------------------------
template<typename Tp, int N>
class Vector : public Matrix<Tp, N, 1>{
public:
	Vector();
	Vector(Tp);
	Vector(Tp, Tp); 
	Vector(Tp, Tp, Tp); 
	Vector(Tp, Tp, Tp, Tp); 
	Vector(Tp, Tp, Tp, Tp, Tp);
	explicit Vector(const Tp *);

	Vector(const Vector &);
	Vector(const Matrix<Tp, N, 1> &);

	// Cross product
	Vector<Tp, N> cross(const Vector<Tp, N>&) const;
	// Inner product
	Tp dot(const Vector<Tp, N>&) const;

	// Accessor
	const Tp& operator [](const int) const;
	Tp& operator[](const int);
	const Tp& operator ()(const int) const;
	Tp& operator ()(const int);

	Vector<Tp, N>& operator = (const Matrix<Tp, N, 1>&);
	Vector<Tp, N>& operator = (const Matrix<Tp, 1, N>&);
	Vector<Tp, N>& operator = (const Matrix<Tp, N, 1>&&);
	Vector<Tp, N>& operator = (const Matrix<Tp, 1, N>&&);

	// Add operation Constructor
	//Vector(const Matrix<Tp, N, 1>&, const Matrix<Tp, N, 1>&, AddOperation);
	// Sub operation Constructor
	//Vector(const Matrix<Tp, N, 1>&, const Matrix<Tp, N, 1>&, SubOperation);
	// Scale operation Constructor
	//template<typename Tp2> Vector(const Matrix<Tp, N, 1>&, Tp2, ScaleOperation);

};

//----------------------------------------------------------
// Rotate Matrix
//----------------------------------------------------------
template<typename Tp>
class RotateMatrix : public Matrix<Tp, 3, 3>{
public:
	RotateMatrix();
	// alpha, beta, gamma
	RotateMatrix(Tp, Tp, Tp);
	// Axis, angle
	RotateMatrix(const Tp, const Matrix<Tp, 3, 1> &);
	RotateMatrix(const RotateMatrix<Tp> &);
	RotateMatrix(const Matrix<Tp, 3, 3> &);

	RotateMatrix<Tp>& operator = (const Matrix<Tp, 3 ,3>&);
	RotateMatrix<Tp>& operator = (const Matrix<Tp, 3 ,3>&&);
	RotateMatrix<Tp>& operator = (const RotateMatrix<Tp>&);
	RotateMatrix<Tp>& operator = (const RotateMatrix<Tp>&&);
};



//----------------------------------------------------------
// 
//----------------------------------------------------------
#include "matrix.hpp"
#include "vector.hpp"
#include "RotateMatrix.hpp"

typedef Matrix<double, 1, 2> Mat12d;
typedef Matrix<double, 1, 3> Mat13d;
typedef Matrix<double, 1, 4> Mat14d;
typedef Matrix<double, 1, 6> Mat16d;
typedef Matrix<double, 2, 1> Mat21d;
typedef Matrix<double, 3, 1> Mat31d;
typedef Matrix<double, 4, 1> Mat41d;
typedef Matrix<double, 6, 1> Mat61d;
typedef Matrix<double, 2, 2> Mat22d;
typedef Matrix<double, 2, 3> Mat23d;
typedef Matrix<double, 3, 2> Mat32d;
typedef Matrix<double, 3, 3> Mat33d;
typedef Matrix<double, 3, 4> Mat34d;
typedef Matrix<double, 4, 3> Mat43d;
typedef Matrix<double, 4, 4> Mat44d;

typedef Vector<double, 1> Vec1d;
typedef Vector<double, 2> Vec2d;
typedef Vector<double, 3> Vec3d;
typedef Vector<double, 4> Vec4d;

typedef RotateMatrix<double> RMatd; 

};
#endif