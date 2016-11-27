//----------------------------------------------------------
// include
//----------------------------------------------------------
#include "RobotHandler.h"
#include "linear/linear.hpp"
//----------------------------------------------------------
// Constant Parameter
//----------------------------------------------------------
//const double RobotHandler::BOX_X(0.570);
const double RobotHandler::BOX_X(0.600);
const double RobotHandler::BOX_Y(0.001);
//const double RobotHandler::BOX_Z(0.043);
const double RobotHandler::BOX_Z(0.040);

const double RobotHandler::RIGHT_HOME_X(0.507 - BOX_X);
const double RobotHandler::RIGHT_HOME_Y(-0.576 - BOX_Y);
const double RobotHandler::RIGHT_HOME_Z(0.277 - BOX_Z);

const double RobotHandler::LEFT_HOME_ROT_X(3.077);
const double RobotHandler::LEFT_HOME_ROT_Y(-0.035);
const double RobotHandler::LEFT_HOME_ROT_Z(1.649);

const double RobotHandler::LEFT_HOME_X(0.583 - BOX_X);
const double RobotHandler::LEFT_HOME_Y(0.522 - BOX_Y);
const double RobotHandler::LEFT_HOME_Z(0.277 - BOX_Z);

const double RobotHandler::RIGHT_HOME_ROT_X(3.120);
const double RobotHandler::RIGHT_HOME_ROT_Y(0.006);
const double RobotHandler::RIGHT_HOME_ROT_Z(1.899);

// Initial angles in arm joints
const Joints RobotHandler::INIT_LEFT_ANGLES = { 0.362528, -1.24796, -0.894286, 1.52721, 0.211943, 1.35341, 1.73632 };
const Joints RobotHandler::INIT_RIGHT_ANGLES = { -0.783666, -0.840139, 1.54499, 1.59949, -0.71994, 1.50835, 1.24368 };
//----------------------------------------------------------
// class Robot Handler
//----------------------------------------------------------
RobotHandler::RobotHandler():
	moving(false),
	left_converged(true),
	right_converged(true),
	left_curr_command(NULL),
	right_curr_command(NULL),
	curr_left_hand(Vec3d(LEFT_HOME_X, LEFT_HOME_Y, LEFT_HOME_Z), Vec3d(LEFT_HOME_ROT_X, LEFT_HOME_ROT_Y, LEFT_HOME_ROT_Z)),
	curr_right_hand(Vec3d(RIGHT_HOME_X, RIGHT_HOME_Y, RIGHT_HOME_Z), Vec3d(RIGHT_HOME_ROT_X, RIGHT_HOME_ROT_Y, RIGHT_HOME_ROT_Z)),
	left_hand(Vec3d(LEFT_HOME_X, LEFT_HOME_Y, LEFT_HOME_Z), Vec3d(LEFT_HOME_ROT_X, LEFT_HOME_ROT_Y, LEFT_HOME_ROT_Z)),
	right_hand(Vec3d(RIGHT_HOME_X, RIGHT_HOME_Y, RIGHT_HOME_Z), Vec3d(RIGHT_HOME_ROT_X, RIGHT_HOME_ROT_Y, RIGHT_HOME_ROT_Z)),
	left_joints(INIT_LEFT_ANGLES),
	right_joints(INIT_RIGHT_ANGLES)
{
	left_hand.set_side(Hand::E_Left);
	curr_left_hand.set_side(Hand::E_Left);
	right_hand.set_side(Hand::E_Right);
	curr_right_hand.set_side(Hand::E_Right);
};
RobotHandler::RobotHandler(const RobotHandler & rh):
	moving(rh.moving),
	left_converged(rh.left_converged),
	right_converged(rh.right_converged),
	curr_left_hand(rh.curr_left_hand), 
	curr_right_hand(rh.curr_right_hand),
	left_hand(rh.left_hand), 
	right_hand(rh.right_hand),
	left_joints(rh.left_joints),
	right_joints(rh.right_joints)
{};
/*RobotHandler::RobotHandler(const RobotHandler & rh):
	moving(rh.moving),
	left_converged(rh.left_converged),
	right_converged(rh.right_converged), 
	left_curr_command(rh.left_curr_command),
	right_curr_command(rh.right_curr_command),
	curr_left_hand(rh.curr_left_hand), 
	curr_right_hand(rh.curr_right_hand),
	left_hand(rh.left_hand), 
	right_hand(rh.right_hand),
	left_interpreter(rh.left_interpreter),
	right_interpreter(rh.right_interpreter),
	left_command_list(rh.left_command_list),
	right_command_list(rh.right_command_list)
{};
*/
//----------------------------------------------------------
// Hand state handler
//----------------------------------------------------------
bool RobotHandler::is_moving() const { return moving; };
void RobotHandler::set_moving(const bool a){ moving = a; };
bool RobotHandler::is_left_converged() const { return is_converge(curr_left_hand, left_hand); };
bool RobotHandler::is_right_converged() const { return is_converge(curr_right_hand, right_hand); };

bool RobotHandler::is_converge(const Hand& curr_h, const Hand& target) const{
	const double X_THRESH(0.01);
	const double ROT_THRESH(0.003 * M_PI);
	const double norm_x = norm(target.get_x() - curr_h.get_x());
	const double norm_rot = norm(target.get_rot() - curr_h.get_rot());
	const la::RMatd t_rmat(target.get_rot()[0], target.get_rot()[1], target.get_rot()[2]);
	const la::RMatd c_rmat(curr_h.get_rot()[0], curr_h.get_rot()[1], curr_h.get_rot()[2]);
	const la::Vec3d z(0, 0, 1);
	const la::Vec3d t_z = t_rmat * z;
	const la::Vec3d c_z = c_rmat * z;
	const double ang = la::angle(t_z, c_z);
	//cout << ang << " " << ROT_THRESH << endl;
	return (norm_x < X_THRESH) && ( ang < ROT_THRESH ) ? true : false;
};	
//----------------------------------------------------------
// List handler
//----------------------------------------------------------
void RobotHandler::add_left_command(Command* com){ left_command_list.push_back(com); };
void RobotHandler::add_right_command(Command* com){ right_command_list.push_back(com); };

void RobotHandler::left_command(Command* com){ left_command_list.push_front(com); };
void RobotHandler::right_command(Command* com){ right_command_list.push_front(com); };

//----------------------------------------------------------
// Getter
//----------------------------------------------------------
int RobotHandler::get_command_num(){ return left_command_list.size(); };

list<Command*>::iterator RobotHandler::get_left_command_list() { return left_command_list.begin(); };
list<Command*>::iterator RobotHandler::get_right_command_list() { return right_command_list.begin(); };

Command* RobotHandler::get_left_command() { return left_command_list.front(); };
Command* RobotHandler::get_right_command() { return right_command_list.front(); };

Hand& RobotHandler::get_left_hand() { return left_hand; };
Hand& RobotHandler::get_right_hand() { return right_hand;};

Hand& RobotHandler::get_curr_left_hand() { return curr_left_hand; };
Hand& RobotHandler::get_curr_right_hand() { return curr_right_hand; };

Joints& RobotHandler::get_left_joints() { return left_joints; };
Joints& RobotHandler::get_right_joints() { return right_joints; };

//----------------------------------------------------------
// Move Robot
//----------------------------------------------------------
void RobotHandler::move(){
	// No command
	if(left_curr_command == NULL && right_curr_command == NULL){

		if(!left_command_list.empty() && !right_command_list.empty()){

			left_curr_command = (left_command_list.front());
			right_curr_command = (right_command_list.front());

			left_interpreter = Interpreter(left_curr_command, &left_hand, &left_joints);
			right_interpreter = Interpreter(right_curr_command, &right_hand, &right_joints);
		}

	}
	if(left_curr_command != NULL && right_curr_command != NULL){

		// Execute command
		left_interpreter.execute();
		right_interpreter.execute();

		// Whether or not Command is complete
		if(left_interpreter.complete() && right_interpreter.complete()){

			left_curr_command = NULL;
			right_curr_command = NULL;

			left_command_list.pop_front();
			right_command_list.pop_front();

		}
	}
};


//----------------------------------------------------------
// Operator 
//----------------------------------------------------------
RobotHandler& RobotHandler::operator=(const RobotHandler& rh){
    copy(rh.left_command_list.begin(), rh.left_command_list.end(), left_command_list.begin());
    copy(rh.right_command_list.begin(), rh.right_command_list.end(), right_command_list.begin());

	left_interpreter = rh.left_interpreter;
	right_interpreter = rh.right_interpreter;

	left_converged = rh.left_converged;
	right_converged = rh.right_converged;

	left_curr_command = rh.left_curr_command;
	right_curr_command = rh.right_curr_command;

	left_hand = rh.left_hand;
	right_hand = rh.right_hand;
	return *this;
};
