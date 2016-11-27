#include "linear.hpp"

using namespace la;

int main(){
	Mat44d R(1, -0.3, -0.1, 1, 3, 5, 2, 2, 2, 3, -0.4, 1,2, 3, -0.4, 5);
	Mat33d S(3, 0.01, 0.1, 0.01, 2, 0.01, 0.1, 0.01, 1);
	//Mat44d R = Mat44d::all(5);
	Vec4d b(1, 2, 3, 4);
	Vec4d x;
	cout << b << endl;
	cout << R << endl;
	LU_decomposition<double, 4>()(R, b, x);
	cout << R << endl;
	cout << x << endl;
	cout << b << endl;
	b = R * x;
	cout << b << endl;
	Vec3d c(1, 1, 1);
	c = c / norm(c);
	cout << " C: " << c << endl;
	Matrix<double, 3, 1> eig;
	eig = JacobiMethod<double, 3>()(S);
	cout << eig << endl;

	return 0;

}