#ifndef POLYGON_H
#define POLYGON_H
//----------------------------------------------------------
// include
//----------------------------------------------------------
#include <opencv2/core/core.hpp>
#include <iostream>
#include <vector>

//----------------------------------------------------------
// namespace
//----------------------------------------------------------
using namespace cv;
using namespace std;

// Container of 2 pointers
template<typename T>
class Line{
private:
	T begin;
	T end;
public:
	// Constructor
	Line();
	Line(const T&, const T&);

	T& operator[](const int);
	const T& operator[](const int) const;

	// Setter
	void set_begin(const T&);
	void set_end(const T&);
	// Getter
	T get_begin() const;
	T get_end() const;

	double length() const;
	T dir() const;
	T unit_dir() const;
};


//----------------------------------------------------------
// class Polygon
//----------------------------------------------------------
template<int N>
class Polygon{
private:
	vector<Vec3d> vertex;
	Vec3d normal;

public:
	Polygon();
	Polygon(const vector<Vec3d> &);
	Polygon(const Polygon<N> &);

	Polygon<N> operator=(const Polygon<N>&);

	// Getter
	Vec3d get_normal() const;
	vector<Vec3d>::iterator get_points();

	// Is x inside of polygon
	bool is_inner(const Vec3d &);

	// Get closest point from particle in Polygon 
	Vec3d get_closest_point(const Vec3d &);
};

template<typename T> struct BiotSavartLaw;

#include "polygon.cpp"

template class Polygon<3>;
template class Polygon<4>;
template class Polygon<5>;
template class Polygon<6>;
template class Polygon<7>;
template class Polygon<8>;
template class Polygon<9>;

typedef Polygon<3> Pol3;
typedef Polygon<4> Pol4;
typedef Polygon<5> Pol5;
typedef Polygon<6> Pol6;
typedef Polygon<7> Pol7;
typedef Polygon<8> Pol8;
typedef Polygon<9> Pol9;

#endif