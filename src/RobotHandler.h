#ifndef ROBOT_HANDLER_H
#define ROBOT_HANDLER_H
//----------------------------------------------------------
// include
//----------------------------------------------------------
#include <baxter_core_msgs/JointCommand.h>
#include <opencv2/core/core.hpp>
#include <vector>
#include <array>
#include <list>

#include "ROSJointController.hpp"
#include "ROSFunction.h"
#include "particle.h"
#include "attractor.h"
#include "command.h"
#include "hand.h"
#include "interpreter.h"
#include "timer.h"

//----------------------------------------------------------
// namespace
//----------------------------------------------------------
using namespace cv;
using namespace std;

//----------------------------------------------------------
// class Robot Handler
//----------------------------------------------------------

class RobotHandler{
public: 
	static const double BOX_X;
	static const double BOX_Y;
	static const double BOX_Z;

	static const double LEFT_HOME_X;
	static const double LEFT_HOME_Y;
	static const double LEFT_HOME_Z;
	static const double LEFT_HOME_ROT_X;
	static const double LEFT_HOME_ROT_Y;
	static const double LEFT_HOME_ROT_Z;
	static const double LEFT_HOME_ROT_W;

	static const double RIGHT_HOME_X;
	static const double RIGHT_HOME_Y;
	static const double RIGHT_HOME_Z;
	static const double RIGHT_HOME_ROT_X;
	static const double RIGHT_HOME_ROT_Y;
	static const double RIGHT_HOME_ROT_Z;
	static const double RIGHT_HOME_ROT_W;

	static const Joints INIT_LEFT_ANGLES;
	static const Joints INIT_RIGHT_ANGLES;
private:
	bool moving;
	bool left_converged;
	bool right_converged;

	Command* left_curr_command;
	Command* right_curr_command;

	list<Command*> left_command_list;
	list<Command*> right_command_list;

	Interpreter left_interpreter;
	Interpreter right_interpreter;

	Hand left_hand;
	Hand right_hand;

	Hand curr_left_hand;
	Hand curr_right_hand;

	Joints left_joints;
	Joints right_joints;

public:
	RobotHandler();
	RobotHandler(const RobotHandler &);

	bool is_moving() const;
	void set_moving(const bool);

	bool is_left_converged() const;
	bool is_right_converged() const;

	void add_left_command(Command*);
	void add_right_command(Command*);
	
	void left_command(Command*);
	void right_command(Command*);

	int get_command_num();

	list<Command*>::iterator get_left_command_list();
	list<Command*>::iterator get_right_command_list();

	Command* get_left_command();
	Command* get_right_command();

	Hand& get_left_hand();
	Hand& get_right_hand();

	Hand& get_curr_left_hand();
	Hand& get_curr_right_hand();

	Joints& get_left_joints();
	Joints& get_right_joints();
	
	void move();

public:
	RobotHandler& operator=(const RobotHandler&);

private:
	bool is_converge(const Hand&, const Hand&) const;
};




#endif