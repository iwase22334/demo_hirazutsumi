#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H
//----------------------------------------------------------
// include
//----------------------------------------------------------
#include <opencv2/core/core.hpp>
#include <vector>
#include <list>
#include "StringModel.h"
#include "particle.h"
#include "attractor.h"
#include "box.h"

//----------------------------------------------------------
// namespace
//----------------------------------------------------------
using namespace cv;
using namespace std;

//----------------------------------------------------------
// class Trajectory Generator
//----------------------------------------------------------

class TrajectoryGenerator{
private:
	Vec3d curr_x;
	Vec3d curr_pose;
	Vec3d next_x;
	Vec3d next_pose;
	list<Attractor> attractor_list;

public:
	TrajectoryGenerator();
	TrajectoryGenerator(list<Attractor> &);
	TrajectoryGenerator(const Vec3d &, const Vec3d &);
	TrajectoryGenerator(const Vec3d &, const Vec3d &, list<Attractor> &);
	TrajectoryGenerator(TrajectoryGenerator &);

	double get_attractor_num() const;
	Vec3d get_curr_x() const;
	Vec3d get_curr_pose() const;
	Vec3d get_next_x() const;
	Vec3d get_next_pose() const;

	void pop_back_attractor();
	void pop_front_attractor();
	void push_back_attractor(const Attractor &);
	void push_front_attractor(const Attractor &);

	void make_trajectory(const Particle &, const list<StringModel> &, const list<Box> &, const list< vector<Vec3d> > &);
	void make_trajectory(const Particle &, const Attractor &);
	void make_trajectory(const Vec3d &, const Attractor &);
};

#endif