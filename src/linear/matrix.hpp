//------------------------------------------------------------
// @author: Iwase Hajime
// @date : 2016.02.09
// @Description : Implementation of the class Matrix
//------------------------------------------------------------
#ifndef MATRIX_HPP
#define MATRIX_HPP
//----------------------------------------------------------
// Static methods
//----------------------------------------------------------
template<typename Tp, int M, int N>
inline Matrix<Tp, M, N> Matrix<Tp, M, N>::all(Tp a){
	Matrix<Tp, M, N> mat;
	for(int i = 0; i < M * N; ++ i){
		mat.val[i] = a;
	}
	return mat;
};
template<typename Tp, int M, int N>
inline Matrix<Tp, M, N> Matrix<Tp, M, N>::zeros(){
	return all(0);
};
template<typename Tp, int M, int N>
inline Matrix<Tp, M, N> Matrix<Tp, M, N>::ones(){ 
	return all(1);
};
template<typename Tp, int M, int N>
inline Matrix<Tp, M, N> Matrix<Tp, M, N>::eye(){
	Matrix<Tp, M, N> mat;
	for(int i = 0; i < min(M, N); ++ i){
		mat(i, i) = 1;
	}
	return mat;
};
template<typename Tp, int M, int N>
inline Matrix<Tp, M, N> Matrix<Tp, M, N>::diag(const diag_type& v0){
	Matrix<Tp, M, N> mat;
	for(int i = 0; i < min(M, N); ++ i){
		mat(i, i) = v0(i);
	}
	return mat;
};
//----------------------------------------------------------
// Constructor
//----------------------------------------------------------
template<typename Tp, int M, int N>
inline Matrix<Tp, M, N>::Matrix(){
	for(int i = 0; i < E_Channels; ++ i) val[i] = Tp(0);
};
template<typename Tp, int M, int N>
inline Matrix<Tp, M, N>::Matrix(Tp v0){
	val[0] = v0;
	for(int i = 1; i < E_Channels; ++ i) val[i] = Tp(0);
};
template<typename Tp, int M, int N>
inline Matrix<Tp, M, N>::Matrix(Tp v0, Tp v1){
	assert( E_Channels >= 2 );
	val[0] = v0;
	val[1] = v1;
	for(int i = 2; i < E_Channels; ++ i) val[i] = Tp(0);
}; 
template<typename Tp, int M, int N>
inline Matrix<Tp, M, N>::Matrix(Tp v0, Tp v1, Tp v2){
	assert( E_Channels >= 3 );
	val[0] = v0;
	val[1] = v1;
	val[2] = v2;
	for(int i = 3; i < E_Channels; ++ i) val[i] = Tp(0);
}; 
template<typename Tp, int M, int N>
inline Matrix<Tp, M, N>::Matrix(Tp v0, Tp v1, Tp v2, Tp v3){
	assert( E_Channels >= 4 );
	val[0] = v0;
	val[1] = v1;
	val[2] = v2;
	val[3] = v3;
	for(int i = 4; i < E_Channels; ++ i) val[i] = Tp(0);
}; 
template<typename Tp, int M, int N>
inline Matrix<Tp, M, N>::Matrix(Tp v0, Tp v1, Tp v2, Tp v3, Tp v4){
	assert( E_Channels >= 5 );
	val[0] = v0;
	val[1] = v1;
	val[2] = v2;
	val[3] = v3;
	val[4] = v4;
	for(int i = 5; i < E_Channels; ++ i) val[i] = Tp(0);
};
template<typename Tp, int M, int N>
inline Matrix<Tp, M, N>::Matrix(Tp v0, Tp v1, Tp v2, Tp v3, 
								Tp v4, Tp v5){
	assert( E_Channels >= 6 );
	val[0] = v0;
	val[1] = v1;
	val[2] = v2;
	val[3] = v3;
	val[4] = v4;
	val[5] = v5;
	for(int i = 6; i < E_Channels; ++ i) val[i] = Tp(0);
};
template<typename Tp, int M, int N>
inline Matrix<Tp, M, N>::Matrix(Tp v0, Tp v1, Tp v2, Tp v3, 
								Tp v4, Tp v5, Tp v6){
	assert( E_Channels >= 7 );
	val[0] = v0;
	val[1] = v1;
	val[2] = v2;
	val[3] = v3;
	val[4] = v4;
	val[5] = v5;
	val[6] = v6;
	for(int i = 7; i < E_Channels; ++ i) val[i] = Tp(0);

};
template<typename Tp, int M, int N>
inline Matrix<Tp, M, N>::Matrix(Tp v0, Tp v1, Tp v2, Tp v3, 
								Tp v4, Tp v5, Tp v6, Tp v7){
	assert( E_Channels >= 8 );
	val[0] = v0;
	val[1] = v1;
	val[2] = v2;
	val[3] = v3;
	val[4] = v4;
	val[5] = v5;
	val[6] = v6;
	val[7] = v7;
	for(int i = 8; i < E_Channels; ++ i) val[i] = Tp(0);

};
template<typename Tp, int M, int N>
inline Matrix<Tp, M, N>::Matrix(Tp v0, Tp v1, Tp v2, Tp v3, 
								Tp v4, Tp v5, Tp v6, Tp v7, 
								Tp v8){
	assert( E_Channels >= 9 );
	val[0] = v0;
	val[1] = v1;
	val[2] = v2;
	val[3] = v3;
	val[4] = v4;
	val[5] = v5;
	val[6] = v6;
	val[7] = v7;
	val[8] = v8;
	for(int i = 9; i < E_Channels; ++ i) val[i] = Tp(0);

};
template<typename Tp, int M, int N>
inline Matrix<Tp, M, N>::Matrix(Tp v0, Tp v1, Tp v2, Tp v3, 
								Tp v4, Tp v5, Tp v6, Tp v7, 
								Tp v8, Tp v9, Tp v10, Tp v11,
								Tp v12, Tp v13, Tp v14, Tp v15){
	assert( E_Channels >= 16 );
	val[0] = v0; val[1] = v1; val[2] = v2; val[3] = v3;
	val[4] = v4; val[5] = v5; val[6] = v6; val[7] = v7;
	val[8] = v8; val[9] = v9; val[10] = v10; val[11] = v11;
	val[12] = v12; val[13] = v13; val[14] = v14; val[15] = v15;
	for(int i = 16; i < E_Channels; ++ i) val[i] = Tp(0);
};
template<typename Tp, int M, int N> 
inline Matrix<Tp, M, N>::Matrix(const Tp* vals){
	for(int i = 0; i < M * N; ++ i)
		val[i] = vals[i];
};
template<typename Tp, int M, int N> 
inline Matrix<Tp, M, N>::Matrix(const Matrix<Tp, M, N> & a){
	for(int i = 0; i < M * N; ++ i)
		val[i] = a.val[i];
};
// Add operation Constructor
template<typename Tp, int M, int N>
inline Matrix<Tp, M, N>::Matrix(const Matrix<Tp, M, N>& a, const Matrix<Tp, M, N>& b, AddOperation){
	for(int i = 0; i < M * N; ++ i)
		val[i] = a.val[i] + b.val[i];
};
// Sub operation Constructor
template<typename Tp, int M, int N>
inline Matrix<Tp, M, N>::Matrix(const Matrix<Tp, M, N>& a, const Matrix<Tp, M, N>& b, SubOperation){
	for(int i = 0; i < M * N; ++ i)
		val[i] = a.val[i] - b.val[i];
};
// Scale operation Constructor
template<typename Tp, int M, int N> template<typename Tp2> 
inline Matrix<Tp, M, N>::Matrix(const Matrix<Tp, M, N>& a , Tp2 b, ScaleOperation){
	for(int i = 0; i < M * N; ++ i)
		val[i] = a.val[i] * b;
};
// Mul operation Constructor
template<typename Tp, int M, int N>
inline Matrix<Tp, M, N>::Matrix(const Matrix<Tp, M, N>& a, const Matrix<Tp, M, N>& b, MulOperation){
	for(int i = 0; i < M * N; ++ i)
		val[i] = a.val[i] * b.val[i];
};
// Matrix multiple operation Constructor
template<typename Tp, int M, int N> template<int L> 
inline Matrix<Tp, M, N>::Matrix(const Matrix<Tp, M, L>& a, const Matrix<Tp, L, N>& b, MatMulOperation){
	for(int i = 0; i < M; ++ i){
		for(int j = 0; j < N; ++ j){
			Tp sum = 0;
			for(int k = 0; k < L; ++ k){
				sum += (a(i, k) * b(k, j));
			}
			val[i * N + j] = sum;
		}
	}
};
// Transposition operation Constructor
template<typename Tp, int M, int N>
inline Matrix<Tp, M, N>::Matrix(const Matrix<Tp, N, M>& a, TransOperation){
	for(int i = 0; i < M; ++ i){
		for(int j = 0; j < N; ++ j){
			val[i * N + j] = a(j, i);
		}
	}
};

//----------------------------------------------------------
// Other methods
//----------------------------------------------------------
// Row vector accessor 
template<typename Tp, int M, int N>
inline Matrix<Tp, 1, N> Matrix<Tp, M, N>::row(const int i) const{
	assert(i < M);
	return Matrix<Tp, 1, N>(val[i * N]);
};
// Column vector accessor
template<typename Tp, int M, int N>
inline Matrix<Tp, M, 1> Matrix<Tp, M, N>::col(const int i) const{
	assert(i < N);
	Matrix<Tp, M, 1> mat;
	for(int j = 0; j < M; ++ j){
		mat(j) = val[i + j * N];
	}
	return mat;
};
// Diag vector accessor
template<typename Tp, int M, int N>
inline Matrix<Tp, (M < N ? M : N), 1> Matrix<Tp, M, N>::diag() const{
	Matrix<Tp, (M < N ? M : N), 1> mat;
	for(int i = 0; i < E_Diags; ++ i){
		mat(i) = val[i + i * N];
	}
	return mat;
};

// Extract part of the matrix
template<typename Tp, int M, int N> template<int K, int L> 
inline Matrix<Tp, K, L> Matrix<Tp, M, N>::get_minor(int y, int x) const{
	Matrix<Tp, K, L> mat;
	for(int i = y; (i-y < K) && (i < M); ++ i){
		for(int j = x; (j - x < L) && (j < N); ++ j){
			mat(i - y, j - x) = (*this)(i, j);
		}
	}	
	return mat;
};

// Transpose the matrix
template<typename Tp, int M, int N>
inline Matrix<Tp, N, M> Matrix<Tp, M, N>::t() const{
	return Matrix<Tp, M, N>((*this), TransOperation());
};

// Invert matrix the matrix
template<typename Tp, int M, int N>
inline Matrix<Tp, M, N> Matrix<Tp, M, N>::inv() const{
	return Matrix<Tp, M, N>();
};

// Multiply two matrices element-wise
template<typename Tp, int M, int N>
inline Matrix<Tp, M, N> Matrix<Tp, M, N>::mul(const Matrix<Tp, M, N>& a) const{
	return Matrix<Tp, M, N>((*this), a, MatMulOperation());
};

// Accessor
template<typename Tp, int M, int N>
inline const Tp& Matrix<Tp, M, N>::operator ()(int i, int j) const{
	assert((i >= 0 && j >= 0) && (i < M && j < N));
	return val[i * N + j];
};
template<typename Tp, int M, int N>
inline Tp& Matrix<Tp, M, N>::operator ()(int i, int j){
	assert((i >= 0 && j >= 0) && (i < M && j < N));
	return val[i * N + j];
};

// 1D accessor
template<typename Tp, int M, int N>
inline const Tp& Matrix<Tp, M, N>::operator ()(int i) const{
	assert((i >= 0) && i < M * N);
	return val[i];
};
template<typename Tp, int M, int N>
inline Tp& Matrix<Tp, M, N>::operator ()(int i){
	assert((i >= 0) && i < M * N);
	return val[i];
};
template<typename Tp, int M, int N>
inline Matrix<Tp, M, N> Matrix<Tp, M, N>::operator - (){
	return Matrix<Tp, M, N>(*this, -1, ScaleOperation());
};
template<typename Tp, int M, int N>
inline Matrix<Tp, M, N>& Matrix<Tp, M, N>::operator = (const Matrix<Tp, M, N>& a){
	for(int i = 0; i < M * N; ++ i)
		val[i] = a.val[i];
	return (*this);
};
template<typename Tp, int M, int N>
inline Matrix<Tp, M, N>& Matrix<Tp, M, N>::operator = (const Matrix<Tp, M, N>&& a){
	for(int i = 0; i < M * N; ++ i)
		val[i] = a.val[i];
	return (*this);
};

//----------------------------------------------------------
// Global function
//----------------------------------------------------------
// Determinant
template<typename Tp, int N> 
static inline double det(const Matrix<Tp, N, N>& a){
	return DetOperation<Tp, N>()(a);
};

template<typename Tp, int N>
struct DetOperation{
	double operator()(const Matrix<Tp, N, N>& a) const{
		cout << "4*4 Or more determinant is not defined" << endl;
	}
};
template<typename Tp>
struct DetOperation<Tp, 1>{
	double operator()(const Matrix<Tp, 1, 1>& a) const{
		return a(0, 0);
	}
};
template<typename Tp>
struct DetOperation<Tp, 2>{
	double operator()(const Matrix<Tp, 2, 2>& a) const{
		return a(0, 0) * a(1, 1) - a(0, 1) * a(1, 0);
	}
};
template<typename Tp>
struct DetOperation<Tp, 3>{
	double operator()(const Matrix<Tp, 3, 3>& a) const{
		double sum = 0;
		for(int i = 0; i < 3; ++ i){
			double r_oblique = 1;
			double l_oblique = 1;
			for(int j = 0; j < 3; ++ j){
				r_oblique *= a(j, (i + j) % 3);
				l_oblique *= a(j, (i + 3 - j) % 3);
			}
			sum += (r_oblique - l_oblique);
		}
		return sum;
	}
};

template<typename Tp, int L, int M, int N> 
static void svd(const Matrix<Tp, M, N>& mat, Matrix<Tp, M, L>& u, Matrix<Tp, L, L>& s, Matrix<Tp, L, N>& v){
	// Caluculate s
	Matrix<Tp, L, 1> eig = eigenvalue(mat.t()*mat);
	Matrix<Tp, L, 1> diag_s = eigenvalue(mat.t()*mat);
	for(int i = 0; i < L; ++ i)	diag_s(i) = sqrt(diag_s(i));

	// Caluculate u,v
	Matrix<Tp, M, 1> x[L];
	Matrix<Tp, N, 1> y[L];
	for(int i = 0; i < L; ++ i){
		x[i] = eigenvector(mat * mat.t(), eig[i]);
		y[i] = eigenvector(mat.t() * mat, eig[i]);
	}

	// Reflect the calculation result
	s = Matrix<Tp, L, L>::diag(diag_s);
	for(int i = 0; i < L; ++ i){
		for(int j = 0; j < M; ++ j){
		}
		for(int j = 0; j < N; ++ j){
			v(j, i) = (y[i])(j);
		}
	}
};

template<typename Tp, int N> 
static Matrix<Tp, N, 1> eigenvalue(const Matrix<Tp, N, N>& a0){
};
template<typename Tp, int N> 
static Matrix<Tp, N, 1> eigenvector(const Matrix<Tp, N, N>& a, const Tp& lambda){
	Matrix<Tp, N, 1> mat;
	for(int i = 0; i < N; ++ i){
		//at(i) = 
	}
	return mat;
};

template<typename Tp, int N> 
struct JacobiMethod{
	Matrix<Tp, N, 1> operator()(const Matrix<Tp, N, N>& a0){
		// Calculate eigen values using Jacobi-Method
		const Tp ABORT_THRESH(0.0001);
		int ind_i = 0;
		int ind_j = 0;
		Matrix<Tp, N, N> a = a0;
		Matrix<Tp, N, N> a_zero_diag = a - Matrix<Tp, N, N>::diag(a.diag());

		do{
			// Find max value in matrix
			ind_i = max_abs_index(a_zero_diag);
			ind_j = ind_i;
			ind_i %= N;
			ind_j /= N;
			// Make a diagonalize matrix
			Tp sign = 1;
			if(a(ind_j, ind_j) - a(ind_i, ind_i) < 0) sign = -1;
			Tp s = (a(ind_j, ind_j) - a(ind_i, ind_i)) / 2.0;

			Matrix<Tp, N, N> p = Matrix<Tp, N, N>::eye();
			Tp c_phi = sqrt(1.0 / 2.0 * (1 + sign * s / sqrt(s * s + a(ind_j, ind_i) * a(ind_j, ind_i))));
			Tp s_phi = sign * a(ind_j, ind_i) / (2.0 * sqrt(s * s + a(ind_j, ind_i) * a(ind_j, ind_i)) * c_phi);

			p(ind_j, ind_j) = c_phi;
			p(ind_i, ind_i) = c_phi;
			p(ind_j, ind_i) = -s_phi;
			p(ind_i, ind_j) = s_phi;

			// Update matrix a
			cout << a << endl;
			cout << "p :" <<  p << endl;
			a = p.t() * a * p;
			a_zero_diag = a - Matrix<Tp, N, N>::diag(a.diag());
		}while(max_abs_value(a_zero_diag) > ABORT_THRESH);
		
		Matrix<Tp, N, 1> eig = a.diag();
		// Sort in descending order
		Tp sorted_lam[N];
		for(int i = 0; i < N; ++ i)	sorted_lam[i] = eig(i);
		sort(sorted_lam, sorted_lam + N - 1, std::greater<Tp>()); 
		for(int i = 0; i < N; ++ i)	eig(i) = sorted_lam[i];
		return eig;
	}
};


template<typename Tp, int N>
struct LU_decomposition{
	void operator()(const Matrix<Tp, N, N>& ia, const Matrix<Tp, N, 1>& ib, Matrix<Tp, N, 1>& x){
		Matrix<Tp, N, N> a = ia;
		Matrix<Tp, N, 1> b = ib;
		Matrix<Tp, N, N> L;
		Matrix<Tp, N, N> U;
		clout_method(L, U, a, b);
		Matrix<Tp, N, 1> c;
		// Gaussian elimination
		for(int i = 0; i < N; ++ i){
			c(i) = b(i);
			for(int j = 0; j < i; ++ j){
				c(i) -= L(i, j) * c(j);
			}
			c(i) /= L(i, i);
		}
		// Gaussian elimination
		for(int i = N - 1; i >= 0; -- i){
			x(i) = c(i);
			for(int j = N - 1; j > i; -- j){
				x(i) -= U(i, j) * x(j);
			}
			x(i) /= U(i, i);
		}
	}
private:
	void clout_method(Matrix<Tp, N, N>& L, Matrix<Tp, N, N>& U, Matrix<Tp, N, N>& a, Matrix<Tp, N, 1>& b){
		U = Matrix<Tp, N, N>::eye();
		// First column
		for(int i = 0; i < N; ++ i){
			L(i, 0) = a(i, 0);
		}

		// First row
		for(int i = 1; i < N; ++ i){
			U(0, i) = a(0, i) / L(0, 0);
		}

		// Calculation loop
		for(int i = 0; i < N; ++ i){
			// Calculate Matrix L column i
			//pivot_selection(i, L, U, a, b);
			for(int j = i; j < N; ++ j){
				for(int k = 0; k <= i - 1; ++ k){
					L(j, i) = a(j, i);
					for(int k = 0; k < i - 1; ++ k){
						L(j, i) -= L(j, k) * U(k, i);  
					}
				}
			}
			// Calculate Matrix U row i
			for(int j = i; j < N; ++ j){
				U(i, j) = a(i, j);
				for(int k = 0; k <= i - 1; ++ k){
					U(i, j) -= L(i, k) * U(k, j);  
				}
				U(i, j) /= L(i, i);
			}
		}
	}
	/*inline void pivot_selection(int index, Matrix<Tp, N, N>& L, const Matrix<Tp, N, N>& U, Matrix<Tp, N, N>& a, Matrix<Tp, N, 1>& b){
		Matrix<Tp, N, N> std_L = L;
		Matrix<Tp, N, N> std_a = a;
		Matrix<Tp, N, 1> std_b = b;
		Matrix<Tp, N, 1> r;

		for(int i = 0; i < index - 1; ++ i){
			Tp m = max_abs_value(L.col(i));	
			for(int j = 0; j < N; ++ j){
				std_L(i, j) /= m;
				std_a(i, j) /= m;
			}
		}
		for(int i = 0; i < N; ++ i){
			r(i) = a(i, index);
			for(int k = 0; k < i - 1; ++ k){
				r(i) -= L(i, k) * U(k, index);
			}
		} 
		int pivot = max_abs_index(r);
		swap_row(L, pivot, index);
		swap_row(a, pivot, index);
		swap_row(b, pivot, index);
	}*/
};
template<typename Tp, int M, int N>
static inline void swap_row(Matrix<Tp, M, N>& a, int src, int dst){
	Matrix<Tp, 1, N> dst_m = a.row(dst);
	Matrix<Tp, 1, N> src_m = a.row(src);
	for(int i = 0; i < N; ++ i){
		a(dst, i) = src_m(i);
		a(src, i) = dst_m(i);
	}
};
template<typename Tp, int M, int N>
static inline int max_index(const Matrix<Tp, M, N>& a){
	Tp max = a(0, 0);
	Tp ind = 0;
	// Find max value in matrix
	for(int i = 0; i < M * N; ++ i){
		if(a(i) > max){
			max = a(i);
			ind = i;
		}
	}
	return ind;
};
template<typename Tp, int M, int N>
static inline Tp max_value(const Matrix<Tp, M, N>& a){
	Tp max = a(0, 0);
	// Find max value in matrix
	for(int i = 0; i < M * N; ++ i){
		if(a(i) > max ){
			max = a(i);
		}
	}
	return max;
};
template<typename Tp, int M, int N>
static inline int max_abs_index(const Matrix<Tp, M, N>& a){
	Tp max = 0;
	Tp ind = 0;
	// Find max value in matrix
	for(int i = 0; i < M * N; ++ i){
		if(abs(a(i)) > max){
			max = abs(a(i));
			ind = i;
		}
	}
	return ind;
};
template<typename Tp, int M, int N>
static inline Tp max_abs_value(const Matrix<Tp, M, N>& a){
	Tp max = 0;
	// Find max value in matrix
	for(int i = 0; i < M * N; ++ i){
		if(abs(a(i)) > max){
			max = abs(a(i));
		}
	}
	return max;
};

template<typename Tp, int M, int N>
static inline Matrix<Tp, M, N> operator + (const Matrix<Tp, M, N>& a, const Matrix<Tp, M, N>& b){
	return Matrix<Tp, M, N>(a, b, AddOperation());
};
template<typename Tp, int M, int N>
static inline Matrix<Tp, M, N> operator - (const Matrix<Tp, M, N>& a, const Matrix<Tp, M, N>& b){
	return Matrix<Tp, M, N>(a, b, SubOperation());
};
template<typename Tp, int L, int M, int N> 
static inline Matrix<Tp, M, L> operator * (const Matrix<Tp, M, N>& a, const Matrix<Tp, N, L>& b){
	return Matrix<Tp, M, L>(a, b, MatMulOperation());
};
template<typename Tp, int M, int N>
static inline Matrix<Tp, M, N> operator * (const int a, const Matrix<Tp, M, N>& b){
	return Matrix<Tp, M, N>(b, a, ScaleOperation());
};
template<typename Tp, int M, int N>
static inline Matrix<Tp, M, N> operator * (const Matrix<Tp, M, N>& a, const int b){
	return Matrix<Tp, M, N>(a, b, ScaleOperation());
};	
template<typename Tp, int M, int N>
static inline Matrix<Tp, M, N> operator * (const float a, const Matrix<Tp, M, N>& b){
	return Matrix<Tp, M, N>(b, a, ScaleOperation());
};
template<typename Tp, int M, int N>
static inline Matrix<Tp, M, N> operator * (const Matrix<Tp, M, N>& a, const float b){
	return Matrix<Tp, M, N>(a, b, ScaleOperation());
};
template<typename Tp, int M, int N>
static inline Matrix<Tp, M, N> operator * (const double a, const Matrix<Tp, M, N>& b){
	return Matrix<Tp, M, N>(b, a, ScaleOperation());
};
template<typename Tp, int M, int N>
static inline Matrix<Tp, M, N> operator * (const Matrix<Tp, M, N>& a, const double b){
	return Matrix<Tp, M, N>(a, b, ScaleOperation());
};

template<typename Tp, int M, int N>
static inline Matrix<Tp, M, N> operator / (const Matrix<Tp, M, N>& a, const int b){
	return Matrix<Tp, M, N>(a, 1.0 / b, ScaleOperation());
};	
template<typename Tp, int M, int N>
static inline Matrix<Tp, M, N> operator / (const Matrix<Tp, M, N>& a, const float b){
	return Matrix<Tp, M, N>(a, 1.0 / b, ScaleOperation());
};
template<typename Tp, int M, int N>
static inline Matrix<Tp, M, N> operator / (const Matrix<Tp, M, N>& a, const double b){
	return Matrix<Tp, M, N>(a, 1.0 / b, ScaleOperation());
};
template<typename Tp, int M, int N>
inline ostream& operator << (ostream & str, Matrix<Tp, M, N>& a){
	for(int i = 0; i < M; ++ i){
		cout << "[ " ;
		for(int j = 0; j < N; ++ j){
			cout << a(i, j) << " ";
		}
		cout << "]" << endl  ;
	}
	return str;
};
#endif