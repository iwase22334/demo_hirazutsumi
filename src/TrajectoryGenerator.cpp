//----------------------------------------------------------
// include
//----------------------------------------------------------
#include "TrajectoryGenerator.h"

//----------------------------------------------------------
// Constructor
//----------------------------------------------------------
TrajectoryGenerator::TrajectoryGenerator(){ };
TrajectoryGenerator::TrajectoryGenerator(list<Attractor> & at_list) { 
	list<Attractor>::iterator it_attractor = at_list.begin();
	while(it_attractor != at_list.end()){
		attractor_list.push_back(*it_attractor);
	}
};
TrajectoryGenerator::TrajectoryGenerator(const Vec3d & i_x, const Vec3d & i_pose){
	curr_x = i_x;
	curr_pose = i_pose;
};
TrajectoryGenerator::TrajectoryGenerator(const Vec3d & i_x, const Vec3d & i_pose, list<Attractor> & at_list){
	curr_x = i_x;
	curr_pose = i_pose;
	list<Attractor>::iterator it_attractor = at_list.begin();
	while(it_attractor != at_list.end()){
		attractor_list.push_back(*it_attractor);
	}
};
TrajectoryGenerator::TrajectoryGenerator(TrajectoryGenerator & tg){
	curr_x = tg.curr_x;
	curr_pose = tg.curr_pose;
	next_x = tg.next_x;
	next_pose = tg.next_pose;
	list<Attractor>::iterator it_attractor = tg.attractor_list.begin();
	while(it_attractor != tg.attractor_list.end()){
		attractor_list.push_back(*it_attractor);
	}
};

//----------------------------------------------------------
// Getter
//----------------------------------------------------------

double TrajectoryGenerator::get_attractor_num() const { return attractor_list.size(); };
Vec3d TrajectoryGenerator::get_curr_x() const { return curr_x; };
Vec3d TrajectoryGenerator::get_curr_pose() const { return curr_pose; };
Vec3d TrajectoryGenerator::get_next_x() const { return next_x; };
Vec3d TrajectoryGenerator::get_next_pose() const { return next_pose; };

//----------------------------------------------------------
// List handler
//----------------------------------------------------------
void TrajectoryGenerator::pop_back_attractor() { attractor_list.pop_back(); };
void TrajectoryGenerator::pop_front_attractor() { attractor_list.pop_front(); };
void TrajectoryGenerator::push_back_attractor(const Attractor & att) { attractor_list.push_back( att ); };
void TrajectoryGenerator::push_front_attractor(const Attractor & att) { attractor_list.push_front( att ); };

//----------------------------------------------------------
// trajectory
//----------------------------------------------------------
void TrajectoryGenerator::make_trajectory(const Particle & par, const list<StringModel> & strModelList, const list<Box> & boxList, const list< vector<Vec3d> > & polygonList){
	const double NEAR_THRESH(0.001);
	curr_x = par.get_x();
	if(!attractor_list.empty()){
		next_x = attractor_list.front().get_near_point(curr_x);
	}
	else{
		next_x = curr_x;
	}
	if( norm(curr_x - next_x) < NEAR_THRESH){
		attractor_list.pop_front();
	}
};
void TrajectoryGenerator::make_trajectory(const Particle & par, const Attractor & att){
	const double NEAR_THRESH(0.001);
	curr_x = par.get_x();
	next_x = att.get_near_point(curr_x);
};
void TrajectoryGenerator::make_trajectory(const Vec3d & i_x, const Attractor & att){
	const double NEAR_THRESH(0.001);
	curr_x = i_x;
	next_x = att.get_near_point(curr_x);
};