//------------------------------------------------------------
// @author: Iwase Hajime
// @date : 2016.02.09
// @Description : Implementation of the class Vector.
//		Outer product and the Inner product are defined 
//		in this derived class.
//------------------------------------------------------------
#ifndef VECTOR_HPP
#define VECTOR_HPP
//----------------------------------------------------------
// Constructor
//----------------------------------------------------------
template<typename Tp, int N> 
inline Vector<Tp, N>::Vector(){};
template<typename Tp, int N> 
inline Vector<Tp, N>::Vector(Tp v0) 
	: Matrix<Tp, N, 1>(v0){};
template<typename Tp, int N> 
inline Vector<Tp, N>::Vector(Tp v0, Tp v1) 
	: Matrix<Tp, N, 1>(v0, v1){}; 
template<typename Tp, int N> 
inline Vector<Tp, N>::Vector(Tp v0, Tp v1, Tp v2)
	: Matrix<Tp, N, 1>(v0, v1, v2){}; 
template<typename Tp, int N> 
inline Vector<Tp, N>::Vector(Tp v0, Tp v1, Tp v2, Tp v3)
	: Matrix<Tp, N, 1>(v0, v1, v2, v3){};
template<typename Tp, int N> 
inline Vector<Tp, N>::Vector(Tp v0, Tp v1, Tp v2, Tp v3, Tp v4)
	: Matrix<Tp, N, 1>(v0, v1, v2, v4){};
template<typename Tp, int N> 
inline Vector<Tp, N>::Vector(const Tp * vals)
	: Matrix<Tp, N, 1>(vals){};

template<typename Tp, int N> 
inline Vector<Tp, N>::Vector(const Vector & v) 
	: Matrix<Tp, N, 1>(v.val){};

template<typename Tp, int N> 
inline Vector<Tp, N>::Vector(const Matrix<Tp, N, 1> & m) 
	: Matrix<Tp, N, 1>(m){};

// Cross product
template<typename Tp, int N> 
inline Vector<Tp, N> Vector<Tp, N>::cross(const Vector<Tp, N>& a) const{
	cout << "Cross product is defined only in a three-dimensional vector" << endl;
	return Vector();
};
template<> 
inline Vector<double, 3> Vector<double, 3>::cross(const Vector<double, 3>& a) const{
	Vector<double, 3> vec;
	for(int i = 0; i < 3; ++ i){
		vec[i] = val[(i + 1) % 3] * a.val[(i + 2) % 3] - a.val[(i + 1) % 3] * val[(i + 2) % 3];
	}
	return vec;
};
template<> 
inline Vector<float, 3> Vector<float, 3>::cross(const Vector<float, 3>& a) const{
	Vector<float, 3> vec;
	for(int i = 0; i < 3; ++ i){
		vec[i] = val[(i + 1) % 3] * a.val[(i + 2) % 3] - a.val[(i + 1) % 3] * val[(i + 2) % 3];
	}
	return vec;
};
template<> 
inline Vector<int, 3> Vector<int, 3>::cross(const Vector<int, 3>& a) const{
	Vector<int, 3> vec;
	for(int i = 0; i < 3; ++ i){
		vec[i] = val[(i + 1) % 3] * a.val[(i + 2) % 3] - a.val[(i + 1) % 3] * val[(i + 2) % 3];
	}
	return vec;
};
// Inner product
template<typename Tp, int N> 
inline Tp Vector<Tp, N>::dot(const Vector<Tp, N>& a) const{
	Tp sum(0);
	for(int i = 0; i < N; ++ i)
		sum += this->val[i] * a.val[i];
	return sum;
};

// Accessor
template<typename Tp, int N> 
inline const Tp& Vector<Tp, N>::operator [](const int i) const{
	assert(i < N);
	return this->val[i];
};
template<typename Tp, int N> 
inline Tp& Vector<Tp, N>::operator[](const int i){
	assert(i < N);
	return this->val[i];
};

template<typename Tp, int N> 
inline const Tp& Vector<Tp, N>::operator ()(const int i) const{
	assert(i < N);
	return this->val[i];
};

template<typename Tp, int N> 
inline Tp& Vector<Tp, N>::operator ()(const int i){
	assert(i < N);
	return this->val[i];
};

template<typename Tp, int N> 
inline Vector<Tp, N>& Vector<Tp, N>::operator = (const Matrix<Tp, N, 1>& a){
	for(int i = 0; i < N; ++ i)
		this->val[i] = a(i);
	return (*this);
};
template<typename Tp, int N> 
inline Vector<Tp, N>& Vector<Tp, N>::operator = (const Matrix<Tp, 1, N>& a){
	for(int i = 0; i < N; ++ i)
		this->val[i] = a(i);
	return (*this);
};
template<typename Tp, int N> 
inline Vector<Tp, N>& Vector<Tp, N>::operator = (const Matrix<Tp, N, 1>&& a){
	for(int i = 0; i < N; ++ i)
		this->val[i] = a(i);
	return (*this);
};
template<typename Tp, int N> 
inline Vector<Tp, N>& Vector<Tp, N>::operator = (const Matrix<Tp, 1, N>&& a){
	for(int i = 0; i < N; ++ i)
		this->val[i] = a(i);
	return (*this);
};
//----------------------------------------------------------
// Global function
//----------------------------------------------------------
template<typename Tp, int N> 
inline Tp norm(const Vector<Tp, N>& a){
	return sqrt(a.dot(a));
};
// Angle between two vectors
template<typename Tp, int N>
inline Tp angle(const Vector<Tp, N>& a, const Vector<Tp, N>& b){
	return acos((a.dot(b))/(norm(a) * norm(b)));
};
#endif