//----------------------------------------------------------
// Include
//----------------------------------------------------------
#include "PoseAttractor.h"
//----------------------------------------------------------
// Constructor
//----------------------------------------------------------
PoseAttractor::PoseAttractor(){
	energy = 0;
	type = E_Point;
};
// Point type PoseAttractor
PoseAttractor::PoseAttractor(const Vec3d & i_x){
	energy = 1;
	type = E_Point;
	x = i_x;
};
// Line or Plane type PoseAttractor
PoseAttractor::PoseAttractor(const E_Type i_type, const Vec3d & i_x, const Vec3d & i_dir){
	energy = 1;
	type = i_type;
	x = i_x;
	dir = i_dir;
};
// Point type PoseAttractor with energy
PoseAttractor::PoseAttractor(const double e, const Vec3d & i_x){
	energy = e;
	type = E_Point;
	x = i_x;
};
// Line or Plane type PoseAttractor with energy
PoseAttractor::PoseAttractor(const E_Type i_type, const double e, const Vec3d & i_x, const Vec3d & i_dir){
	energy = e;
	type = i_type;
	x = i_x;
	dir = i_dir;
};
// Copy constructor
PoseAttractor::PoseAttractor(const PoseAttractor & att){
	energy = att.energy;
	type = att.type;
	x = att.x;
	dir = att.dir;
};

//----------------------------------------------------------
// Setter
//----------------------------------------------------------
void PoseAttractor::set_energy(const double e){ energy = e; };
void PoseAttractor::set_attraction(const Vec3d & i_x){
	type = E_Plane;
	x = i_x;
};
void PoseAttractor::set_attraction(const E_Type i_type, const Vec3d & i_x, const Vec3d & i_dir){
	type = i_type;
	x = i_x;
	dir = i_dir;
};

//----------------------------------------------------------
// Getter
//----------------------------------------------------------
PoseAttractor::E_Type PoseAttractor::get_type() const { return type; };
Vec3d PoseAttractor::get_x() const { return x; };
Vec3d PoseAttractor::get_dir() const { return dir; };
double PoseAttractor::get_energy() const { return energy; };

// Get nearest point in Poseattractor from i_x
Vec3d PoseAttractor::get_near_point(Vec3d i_x) const {
	Vec3d near_point;
	Vec3d unit_dir;
	Vec3d near_point_dir;
	switch(type){
		case E_Point:
			near_point = x; 
			break;

		case E_Line:
			unit_dir = dir / norm(dir);
			near_point_dir = unit_dir.dot(i_x - x) * unit_dir;
			near_point = x + near_point_dir;
			break;

		case E_Plane:
			unit_dir = dir / norm(dir);
			near_point_dir = unit_dir.dot( i_x - x ) * unit_dir;
			near_point = i_x - near_point_dir;
			break;

		default:
			cerr << "PoseAttractor : No such Poseattractor type!" << endl;
			exit(EXIT_FAILURE);
			break;
	}
	return near_point;
};



