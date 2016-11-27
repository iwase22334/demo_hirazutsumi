#include "command.h"
#include "linear/linear.hpp"
#include "interpreter.h"
//----------------------------------------------------------
// class Command
//----------------------------------------------------------

//----------------------------------------------------------
// Constructor
//----------------------------------------------------------
Command::Command(): 
	type(E_Wait),
	posture(E_Pos_Wait)
{};
Command::Command(const E_Type t):
	type(t),
	posture(E_Pos_Wait)
{};
Command::Command(const E_Posture ep):
	type(E_Stop),
	posture(ep)
{};

Command::Command(const E_Type t, const E_Posture ep):
	type(t),
	posture(ep)
{};

Command::Command(const double t):
	type(E_Stop),
	posture(E_Pos_Wait),
	time(t)
{};
Command::Command(const Attractor & a):
	type(E_Move),
	posture(E_Pos_Wait),
	time(0),
	attractor(a)
{};
Command::Command(const Attractor & a, const PoseAttractor & pa):
	type(E_Move),
	posture(E_Pos_Rot),
	time(0),
	attractor(a),
	pose_attractor(pa)
{};
Command::Command(const PoseAttractor & pa):
	type(E_Stop),
	posture(E_Pos_Rot),
	time(0),
	pose_attractor(pa)
{};
Command::Command(Particle* p):
	type(E_Grip),
	posture(E_Pos_Wait),
	time(0)
{
	lis_target.push_back(p);
};
Command::Command(const E_Type et, const double t):
	type(et),
	posture(E_Pos_Wait),
	time(t)
{}
Command::Command(const E_Type et, const Attractor& a):
	time(0),
	type(E_Move),
	posture(E_Pos_Wait),
	attractor(a)
{};
Command::Command(const E_Type et, const Attractor& a, const PoseAttractor & pa):
	time(0),
	type(E_Move),
	posture(E_Pos_Rot),
	attractor(a),
	pose_attractor(pa)
{};
Command::Command(const E_Type et, const PoseAttractor & pa):
	time(0),
	type(E_Stop),
	posture(E_Pos_Rot),
	pose_attractor(pa)
{};

Command::Command(const E_Type t, Particle* p):
	time(0),
	type(E_Grip),
	posture(E_Pos_Wait)
{
	lis_target.push_back(p);
};
Command::Command(const E_Type, list<Particle*>& tl):
	time(0),
	type(E_Grip),
	posture(E_Pos_Wait),
	lis_target(tl)
{};
Command::Command(const Command & com):
	time(com.time),
	type(com.type),
	posture(com.posture),
	attractor(com.attractor),
	pose_attractor(com.pose_attractor),
	lis_target(com.lis_target)
{};

//----------------------------------------------------------
// Setter
//----------------------------------------------------------
void Command::set_time(const double t) { time = t; };
void Command::set_type(const Command::E_Type t) { type = t; };
void Command::set_posture(const Command::E_Posture ep) { posture = ep; };
void Command::set_attractor(const Attractor & att){
	attractor.set_attraction(att.get_type(), att.get_x(), att.get_dir());
	attractor.set_energy(att.get_energy());
};
void Command::set_pose_attractor(const PoseAttractor & patt){
	pose_attractor.set_attraction(patt.get_type(), patt.get_x(), patt.get_dir());
	pose_attractor.set_energy(patt.get_energy());
};
void Command::set_target(Particle* par){
	lis_target.clear(); 
	lis_target.push_back(par); 
};
void Command::add_target(Particle* par){
	lis_target.push_back(par);
};
void Command::set_target_list(list<Particle*> tl){
	lis_target = tl;
};

//----------------------------------------------------------
// Getter
//----------------------------------------------------------
double Command::get_time() const { return time; };
Command::E_Type Command::get_type() const { return type; };
Command::E_Posture Command::get_posture() const { return posture; };
Attractor& Command::get_attractor(){ return attractor; };
PoseAttractor& Command::get_pose_attractor(){ return pose_attractor; };
list<Particle*>& Command::get_target_list(){ return lis_target; };

list<Particle*>::iterator Command::get_target_begin() { return lis_target.begin(); };
list<Particle*>::iterator Command::get_target_end() { return lis_target.end(); };
Particle* Command::get_target_front() { return lis_target.front(); };
Particle* Command::get_target_back() { return lis_target.back(); };

//----------------------------------------------------------
// Class Optimum Posture when Lifting state Class
//----------------------------------------------------------
OPLCommand::OPLCommand(const Hand* curr_lift, const Hand* curr_other, Hand* lift, Hand* other) : 
	Command(Command::E_Wait, Command::E_Pos_Special),
	curr_lift_hand(curr_lift),
	curr_other_hand(curr_other),
	lift_hand(lift),
	other_hand(other)
{};
void OPLCommand::special_move(){};

void OPLCommand::special_rot(){
    const Vec3d curr_rot = curr_lift_hand->get_rot();
    //Vec3d curr_rot = lift_hand->get_rot();
    
    const la::RMatd rmat0(curr_rot[0], curr_rot[1], curr_rot[2]);
    const la::Vec3d pointer(0, 0, 1);
    const la::Vec3d posture = rmat0 * pointer;
    const cv::Vec3d target_posture = curr_other_hand->get_x() - curr_lift_hand->get_x();
    la::Vec3d la_target_posture(target_posture[0], target_posture[1], target_posture[2]);
   	la_target_posture = la_target_posture / norm(la_target_posture);
    const la::Vec3d axis = posture.cross(la_target_posture);
    la::Mat33d next_mat;
    if(la::angle(posture, la_target_posture) > 0.001) {
    	const la::RMatd rmat(la::angle(posture, la_target_posture) * 0.4, axis / la::norm(axis));
    	//const la::RMatd rmat(la::angle(posture, la_target_posture), axis / la::norm(axis));
    	
    	next_mat = rmat * rmat0;
    }
    else{
    	next_mat = rmat0;
    }

    la::RMatd next_rmat(next_mat);
    la::Vec3d next_rot(la::alpha(next_rmat), la::beta(next_rmat), la::gamma(next_rmat));
    
    Vec3d rot(next_rot[0], next_rot[1], next_rot[2]);
    lift_hand->set_rot(rot);
};
bool OPLCommand::complete(){return true;};

//----------------------------------------------------------
// Instruction through hands in the hole
//----------------------------------------------------------

ThroughHoleCommand::ThroughHoleCommand(const Attractor& a, Hand * const h, ClothSimulator& s, const pair<Particle*, Particle*>& p):
	Command(Command::E_Move, Command::E_Pos_Special),
	hand(h),
	simulator(s),
	intersection(p)
{
	set_attractor(a);
};
void ThroughHoleCommand::special_move(){};
void ThroughHoleCommand::special_rot(){
	// Get hole
	vector<Vec3d> hole = simulator.get_hole(intersection.first, intersection.second);
	// Set target direction
	Vec3d dir = BiotSavartLaw<Vec3d>()(hand->get_x(), hole);
	dir /= norm(dir);

	// Get current rotation of the hand
	const Vec3d curr_rot = hand->get_rot();
	const la::RMatd rmat0(curr_rot[0], curr_rot[1], curr_rot[2]);
	const la::Vec3d pointer(0, 0, 1);
	const la::Vec3d posture = rmat0 * pointer;
	const la::Vec3d la_target_posture(dir[0], dir[1], dir[2]);
	const la::Vec3d axis = posture.cross(la_target_posture);
	la::Mat33d next_mat;

//	if(la::norm(axis) > 0.001) {
		la::RMatd rmat(la::angle(posture, la_target_posture), axis / la::norm(axis));
		next_mat = rmat * rmat0;
/*	}
	else{
		next_mat = rmat0;
	}
*/
	la::RMatd next_rmat(next_mat);
	la::Vec3d next_rot(la::alpha(next_rmat), la::beta(next_rmat), la::gamma(next_rmat));
	
	Vec3d rot(next_rot[0], next_rot[1], next_rot[2]);
	hand->set_rot(rot);
};
bool ThroughHoleCommand::complete(){
	TrajectoryGenerator tg;
	tg.make_trajectory(hand->get_x(), attractor);
	Vec3d dir = tg.get_next_x() - tg.get_curr_x();
	return norm(dir) < 0.001 ? true : false;
};