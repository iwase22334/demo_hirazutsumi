#ifndef ATTRACTOR_H
#define ATTRACTOR_H
//----------------------------------------------------------
// include
//----------------------------------------------------------
#include <iostream>
#include <opencv2/core/core.hpp>
#include <vector>

//----------------------------------------------------------
// namespace
//----------------------------------------------------------
using namespace cv;
using namespace std;

//----------------------------------------------------------
// class Attractor
//----------------------------------------------------------
class Attractor{
public:
	enum E_Type { 
			E_Point = 0,
			E_Line = 1,
			E_Plane= 2
	};

private:
	E_Type type;
	double energy;
	// Attractor point
	Vec3d x;
	// Direction of Line or Plane Attractor 
	Vec3d dir;

public:
	Attractor();
	// Point type Attractor
	Attractor(const Vec3d &);
	// Line or Plane type Attractor
	Attractor(const E_Type, const Vec3d &, const Vec3d &);
	// Point type Attractor with energy
	Attractor(const double, const Vec3d &);
	// Line or Plane type Attractor with energy
	Attractor(const E_Type, const double, const Vec3d &, const Vec3d &);
	// Copy constructor
	Attractor(const Attractor &);

	// Setter
	void set_attraction(const Vec3d &);
	void set_attraction(const E_Type, const Vec3d &, const Vec3d &);
	void set_energy(const double);
	
	// Getter
	E_Type get_type() const;
	Vec3d get_x() const;
	Vec3d get_dir() const;
	double get_energy() const;

	// Get nearest point in attractor
	Vec3d get_near_point(Vec3d) const;
};

#endif