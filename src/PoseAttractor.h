#ifndef POSE_ATTRACTOR_H
#define POSE_ATTRACTOR_H
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
// class PoseAttractor
//----------------------------------------------------------
class PoseAttractor{
public:
	enum E_Type { 
			E_Point = 0,
			E_Line = 1,
			E_Plane= 2
	};

private:
	E_Type type;
	double energy;
	// PoseAttractor point
	// The point in the space which spanned by roll, pitch and yow
	Vec3d x;
	// Direction of Line or Plane PoseAttractor 
	Vec3d dir;

public:
	PoseAttractor();
	// Point type PoseAttractor
	PoseAttractor(const Vec3d &);
	// Line or Plane type PoseAttractor
	PoseAttractor(const E_Type, const Vec3d &, const Vec3d &);
	// Point type PoseAttractor with energy
	PoseAttractor(const double, const Vec3d &);
	// Line or Plane type PoseAttractor with energy
	PoseAttractor(const E_Type, const double, const Vec3d &, const Vec3d &);
	// Copy constructor
	PoseAttractor(const PoseAttractor &);

	// Setter
	void set_attraction(const Vec3d &);
	void set_attraction(const E_Type, const Vec3d &, const Vec3d &);
	void set_energy(const double);
	
	// Getter
	E_Type get_type() const;
	Vec3d get_x() const;
	Vec3d get_dir() const;
	double get_energy() const;

	// Get nearest point in Poseattractor
	Vec3d get_near_point(Vec3d) const;
};

#endif