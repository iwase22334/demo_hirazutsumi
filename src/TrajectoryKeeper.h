#ifndef TRAJECTORY_KEEPER_H
#define TRAJECTORY_KEEPER_H
//----------------------------------------------------------
// include
//----------------------------------------------------------
#include <opencv2/core/core.hpp>
#include <iostream>
#include <list>
//----------------------------------------------------------
// namespace
//----------------------------------------------------------
using namespace cv;
using namespace std;
//----------------------------------------------------------
// class TrajectoryKeeper
//----------------------------------------------------------
class TrajectoryKeeper{
private:
	double length;
	double interval;
	list<Vec3d> points;

public:
	// Constructor
	TrajectoryKeeper(){};
	TrajectoryKeeper(const double);
	TrajectoryKeeper(const TrajectoryKeeper &);

	// Setter
	void add_point(const Vec3d &);

	// Getter
	double get_length() const;
	double get_interval() const;
	list<Vec3d>& get_points();

	// list handler
	void clear();
	list<Vec3d>::iterator begin();
	list<Vec3d>::iterator end();
	Vec3d& front();
	Vec3d& back();
	void push_back(const Vec3d &);\
	void pop_back();

};

#endif
