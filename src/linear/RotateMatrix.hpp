//------------------------------------------------------------
// @author: Iwase Hajime
// @date : 2016.02.09
// @Description : Implementation of the class RotateMatrix
//		This class can handle Rotation around any axis.
//------------------------------------------------------------
#ifndef ROTATE_MATRIX_H
#define ROTATE_MATRIX_H
//----------------------------------------------------------
// Constructor
//----------------------------------------------------------
// Default constructor
template<typename Tp>
inline RotateMatrix<Tp>::RotateMatrix() : 
	Matrix<Tp, 3, 3>(){};

// Make RotateMatrix from alpha, beta, gamma
template<typename Tp>
inline RotateMatrix<Tp>::RotateMatrix(Tp a, Tp b, Tp c){
	Matrix<Tp, 3, 3> A1(1,      0,       0,
						0, cos(a), -sin(a),
						0, sin(a),  cos(a));

	Matrix<Tp, 3, 3> A2( cos(b), 0, sin(b),
						      0, 1,      0,
						-sin(b), 0, cos(b));

	Matrix<Tp, 3, 3> A3( cos(c), -sin(c), 0,
						 sin(c),  cos(c), 0,
						      0,       0, 1);

	Matrix<Tp, 3, 3> mat = (A3 * A1) * A2;
	for(int i = 0; i < 3 * 3; ++ i)
		this->val[i] = mat(i);
};

// Rotation around any axis
template<typename Tp>
inline RotateMatrix<Tp>::RotateMatrix(const Tp th, const Matrix<Tp, 3, 1> & a){
	Tp c = cos(th);
	Tp s = sin(th);
	Matrix<Tp, 3, 3> mat(c + a(0) * a(0) * (1 - c), a(0) * a(1) * (1 - c) - a(2) * s, a(0) * a(2) * (1 - c) + a(1) * s,
						  a(1) * a(0) * (1 - c) + a(2) * s, c + a(1) * a(1) * ( 1 - c ), a(1) * a(2) * (1 - c) - a(0) * s,
						  a(2) * a(0) * (1 - c) - a(1) * s, a(2) * a(1) * (1 - c) + a(0) * s, c + a(2) * a(2) * ( 1 - c ));
	for(int i = 0; i < 3 * 3; ++ i)
		this->val[i] = mat(i);
};

template<typename Tp>
inline RotateMatrix<Tp>::RotateMatrix(const RotateMatrix<Tp> & a){
	for(int i = 0; i < 3 * 3; ++ i)
		this->val[i] = a.val(i);
};

template<typename Tp>
inline RotateMatrix<Tp>::RotateMatrix(const Matrix<Tp, 3, 3> & a){
	for(int i = 0; i < 3 * 3; ++ i)
		this->val[i] = a(i);
};

template<typename Tp>
inline RotateMatrix<Tp>& RotateMatrix<Tp>::operator = (const Matrix<Tp, 3 ,3>& a){
	for(int i = 0; i < 3 * 3; ++ i)
		*this->val[i] = a(i);
	return (*this);
};

template<typename Tp>
inline RotateMatrix<Tp>& RotateMatrix<Tp>::operator = (const Matrix<Tp, 3 ,3>&& a){
	for(int i = 0; i < 3 * 3; ++ i)
		*this->val[i] = a(i);
	return (*this);
};

template<typename Tp>
inline RotateMatrix<Tp>& RotateMatrix<Tp>::operator = (const RotateMatrix<Tp>& a){
	for(int i = 0; i < 3 * 3; ++ i)
		*this->val[i] = a(i);
	return (*this);
};

template<typename Tp>
inline RotateMatrix<Tp>& RotateMatrix<Tp>::operator = (const RotateMatrix<Tp>&& a){
	for(int i = 0; i < 3 * 3; ++ i)
		*this->val[i] = a(i);
	return (*this);
};

//----------------------------------------------------------
// Global functions
//----------------------------------------------------------
// Functions for parameter extraction
/*template<typename Tp> 
static inline Tp alpha(const RotateMatrix<Tp>& a){
	return (fabs(a(0, 2)) >= 1 ? 0 : atan2(-a(1, 2), a(2, 2)));
};

template<typename Tp>
static inline  Tp beta(const RotateMatrix<Tp>& a){
	if(fabs(a(0, 2)) >= 1.0) 
		return M_PI / 2.0;
	else if(fabs(a(0, 2)) <= -1.0) 
		return - M_PI / 2.0;
	else
		return asin(-a(0, 2)); 
};

template<typename Tp>
static inline  Tp gamma(const RotateMatrix<Tp>& a){
	if(fabs(a(0, 2)) >= 1.0) 
		return atan2(a(2, 1), a(1, 1));
	else if(fabs(a(0, 2)) <= -1.0) 
		return atan2(a(1, 0), a(2, 0));
	else
		return atan2(-a(0, 1), a(0, 0)); 
};*/
template<typename Tp> 
static inline Tp alpha(const RotateMatrix<Tp>& a){	
	if(a(2, 1) >= 1.0) 
		return M_PI / 2.0;
	else if(a(2, 1) <= -1.0) 
		return - M_PI / 2.0;
	else
		return asin(a(2, 1)); 
};
template<typename Tp>
static inline  Tp beta(const RotateMatrix<Tp>& a){
	return (fabs(a(2, 1)) >= 1 ? 0 : atan2(-a(2, 0), a(2, 2)));
};
template<typename Tp>
static inline  Tp gamma(const RotateMatrix<Tp>& a){
	if(fabs(a(2, 1)) >= 1.0) 
		return atan2(a(1, 0), a(0, 0));
	else
		return atan2(-a(0, 1), a(1, 1)); 
};

#endif