//----------------------------------------------------------
// Include
//----------------------------------------------------------
#include "attractor.h"
//----------------------------------------------------------
// Constructor
//----------------------------------------------------------
Attractor::Attractor(){
	energy = 1;
	type = E_Point;
};
// Point type Attractor
Attractor::Attractor(const Vec3d & i_x){
	energy = 1;
	type = E_Point;
	x = i_x;
};
// Line or Plane type Attractor
Attractor::Attractor(const E_Type i_type, const Vec3d & i_x, const Vec3d & i_dir){
	energy = 1;
	type = i_type;
	x = i_x;
	dir = i_dir;
};
// Point type Attractor with energy
Attractor::Attractor(const double e, const Vec3d & i_x){
	energy = e;
	type = E_Point;
	x = i_x;
};
// Line or Plane type Attractor with energy
Attractor::Attractor(const E_Type i_type, const double e, const Vec3d & i_x, const Vec3d & i_dir){
	energy = e;
	type = i_type;
	x = i_x;
	dir = i_dir;
};
// Copy constructor
Attractor::Attractor(const Attractor & att){
	energy = att.energy;
	type = att.type;
	x = att.x;
	dir = att.dir;
};

//----------------------------------------------------------
// Setter
//----------------------------------------------------------
void Attractor::set_energy(const double e){ energy = e; };
void Attractor::set_attraction(const Vec3d & i_x){
	type = E_Plane;
	x = i_x;
};
void Attractor::set_attraction(const E_Type i_type, const Vec3d & i_x, const Vec3d & i_dir){
	type = i_type;
	x = i_x;
	dir = i_dir;
};

//----------------------------------------------------------
// Getter
//----------------------------------------------------------
Attractor::E_Type Attractor::get_type() const { return type; };
Vec3d Attractor::get_x() const { return x; };
Vec3d Attractor::get_dir() const { return dir; };
double Attractor::get_energy() const { return energy; };

// Get nearest point in attractor from i_x
Vec3d Attractor::get_near_point(Vec3d i_x) const {
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
			cerr << "Attractor : No such attractor type!" << endl;
			exit(EXIT_FAILURE);
			break;
	}
	return near_point;
};



