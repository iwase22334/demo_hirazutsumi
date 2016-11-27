//------------------------------------------------------------
// @author: Iwase Hajime
// @date : 2016.03.07
// @Description : This code is interpreter of Command.
//				interpreter is controller objects which made from command
//------------------------------------------------------------
#include "linear/linear.hpp"
#include "ROSFunction.h"
#include "ClothSimulator.h"
#include "command.h"
#include "TrajectoryGenerator.h"
#include "timer.h"
#include "hand.h"
//----------------------------------------------------------
// class
//----------------------------------------------------------

class Interpreter{
public:

	static const double VEL_THRESH;
	static const double PAST_TIME_THRESH;
	static const double NEAR_THRESH;
	static const double POW_THRESH;
	static const double GRIP_WAIT_TIME;
	static const double RELEASE_WAIT_TIME;

private:
	Vec3d hand_vel;
	Timer timer;
	Command* command;
	Hand* hand;
	Joints* joints;

public:
	Interpreter();
	Interpreter(Command*, Hand *, Joints *);
	Interpreter(const Interpreter &);

	Interpreter& operator=(const Interpreter&);

	void execute();
	bool complete();

private:
	void grip();
	void release();
	void move_hand();
	void rotate_hand();

	bool check_convergence();
};
