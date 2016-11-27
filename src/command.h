#ifndef COMMAND_H
#define COMMAND_H
//----------------------------------------------------------
// include
//----------------------------------------------------------
#include <opencv2/core/core.hpp>
#include <list>
#include <utility>
#include "TrajectoryGenerator.h"
#include "ClothSimulator.h"
#include "hand.h"
#include "particle.h"
#include "attractor.h"
#include "PoseAttractor.h"
#include "polygon.h"

//----------------------------------------------------------
// namespace
//----------------------------------------------------------
using namespace cv;
using namespace std;


//----------------------------------------------------------
// class Command
//----------------------------------------------------------
class Command{
public:
	enum E_Type{
		E_Wait = 0,
		E_Move = 1,
		E_Grip = 2,
		E_Release = 3,
		E_Stop = 4,
		E_Special = 5
	};
	enum E_Posture{
		E_Pos_Wait = 0,
		E_Pos_Rot = 1,
		E_Pos_Special = 2
	};

protected:
	E_Type type;
	E_Posture posture;
	double time;
	Attractor attractor;
	PoseAttractor pose_attractor;
	list<Particle*> lis_target;
	
public:
	Command();
	Command(const E_Type);
	Command(const E_Posture);
	Command(const E_Type, const E_Posture);
	Command(const double);
	Command(const Attractor &);
	Command(const Attractor &, const PoseAttractor &);
	Command(const PoseAttractor &);
	Command(Particle*);
	Command(const E_Type, const double);
	Command(const E_Type, const Attractor&);
	Command(const E_Type, const Attractor&, const PoseAttractor &);
	Command(const E_Type, const PoseAttractor &);
	Command(const E_Type, Particle*);
	Command(const E_Type, list<Particle*>&);
	Command(const Command &);

	// Setter
	void set_time(const double);
	void set_type(const E_Type);
	void set_posture(const E_Posture);
	void set_attractor(const Attractor &);
	void set_pose_attractor(const PoseAttractor &);
	void set_target(Particle*);
	void add_target(Particle*);
	void set_target_list(list<Particle*>);

	// Getter
	double get_time() const;
	E_Type get_type() const;
	E_Posture get_posture() const;
	Attractor& get_attractor();
	PoseAttractor& get_pose_attractor();
	Particle* get_target();
	list<Particle*>& get_target_list();
	list<Particle*>::iterator get_target_begin();
	list<Particle*>::iterator get_target_end();
	Particle* get_target_front();
	Particle* get_target_back();

	virtual void special_move(){};
	virtual void special_rot(){};
	virtual bool complete(){ return true; };
};

//----------------------------------------------------------
// Class Optimum Posture when Lifting state
//----------------------------------------------------------
// Optimum posture to hold the lifting state.
class OPLCommand : public Command{
private:
	const Hand* curr_lift_hand;
	const Hand* curr_other_hand;
	Hand* lift_hand;
	Hand* other_hand;

public:
	OPLCommand(const Hand*, const Hand*, Hand*, Hand*);
	virtual void special_move();
	virtual void special_rot();
	virtual bool complete();
};

//----------------------------------------------------------
// Instruction through hands in the hole
//----------------------------------------------------------
// Optimum posture to hold the lifting state.
class ThroughHoleCommand : public Command{
private:
	Hand* hand;
	ClothSimulator& simulator;
	pair<Particle*, Particle*> intersection;
public:
	ThroughHoleCommand(const Attractor&, Hand* const, ClothSimulator& , const pair<Particle*, Particle*>&);
	virtual void special_move();
	virtual void special_rot();
	virtual bool complete();
};



#endif