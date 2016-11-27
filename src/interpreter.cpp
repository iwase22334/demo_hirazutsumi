#include "interpreter.h"
//----------------------------------------------------------
// class interpreter
//----------------------------------------------------------
//----------------------------------------------------------
// Constant Parameter
//----------------------------------------------------------

const double Interpreter::VEL_THRESH(0.017);
const double Interpreter::PAST_TIME_THRESH(3.0);
const double Interpreter::NEAR_THRESH(0.003);
const double Interpreter::POW_THRESH(0.34);
const double Interpreter::GRIP_WAIT_TIME(5.0);
const double Interpreter::RELEASE_WAIT_TIME(5.0);

//----------------------------------------------------------
// Constructor
//----------------------------------------------------------
Interpreter::Interpreter():	
	hand_vel(0,0,0),
	timer(),
	command(NULL),
	hand()
{};
Interpreter::Interpreter(Command* c, Hand * h, Joints* j):
	hand_vel(0,0,0),
	timer(),
	command(c),
	hand(h),
	joints(j)
{

	timer.start();
};
Interpreter::Interpreter(const Interpreter & i):
	hand_vel(i.hand_vel),
	timer(i.timer),
	command(i.command),
	hand(i.hand),
	joints(i.joints)
{};

Interpreter& Interpreter::operator=(const Interpreter& i){
	hand_vel = i.hand_vel;
	timer = i.timer;
	command = i.command;
	hand = i.hand;
	joints = i.joints;
	return *this;
};
//----------------------------------------------------------
// Execute command
//----------------------------------------------------------
void Interpreter::execute(){
	assert(hand != NULL);
	assert(command != NULL);
	switch(command->get_type()){
		case Command::E_Wait:
			break;

		case Command::E_Move:
			move_hand();
			break;

		case Command::E_Grip:
			grip();
			timer.start();
			command->set_time(GRIP_WAIT_TIME);
			command->set_type(Command::E_Stop);
			break;

		case Command::E_Release:
			release();
			timer.start();
			command->set_time(RELEASE_WAIT_TIME);
			command->set_type(Command::E_Stop);
			break;

		case Command::E_Stop:
			break;

		case Command::E_Special:
			command->special_move();
			break;

		default:
			break;
	}

	switch(command->get_posture()){
		case Command::E_Pos_Wait:
			rf::call_ik(hand->get_side(), hand->get_x(), hand->get_rot(), *joints);
			break;
		case Command::E_Pos_Rot:
			rotate_hand();
			break;
		case Command::E_Pos_Special:
			command->special_rot();
			break;
		default:
			break;
	}

};

//----------------------------------------------------------
// Check the achievement
//----------------------------------------------------------
bool Interpreter::complete(){
	bool complete_flag(false);
	if(command == NULL){
		complete_flag = true;
	}
	else{
		switch(command->get_type()){
			case Command::E_Wait:
				complete_flag = true;
				break;

			case Command::E_Move:
				if(check_convergence()){
					complete_flag = true;
					command->set_type(Command::E_Wait);
				}
				break;

			case Command::E_Grip:
				break;

			case Command::E_Release:
				break;

			case Command::E_Stop:
				timer.stop();
				if(timer.get_interval() > command->get_time()){
					complete_flag = true;
				}
				break;
			case Command::E_Special:
				complete_flag = command->complete();
				break;

			default:
				complete_flag = true;
				break;
		}
	}
	return complete_flag;
};
//----------------------------------------------------------
// Grip particle
//----------------------------------------------------------
void Interpreter::grip(){
	assert(hand != NULL);
	assert(command != NULL);

	hand->set_target(command->get_target_list());
	hand->set_state(Hand::E_Handring);

	// Affix all particles 
	list<Particle*>::iterator it = command->get_target_list().begin();
	while(it != command->get_target_list().end()){
		(*it)->fix();
		++ it;
	}
};

//----------------------------------------------------------
// Release particle
//----------------------------------------------------------
void Interpreter::release(){
	assert(hand != NULL);
	assert(command != NULL);

	hand->release();
	hand->set_state(Hand::E_Opening);

	// Unfix all particles
	if(hand->get_target_list().size() > 1){
		list<Particle*>::iterator it = hand->get_target_list().begin();
		while(it != hand->get_target_list().end()){
			//(*it)->unfix();
			++ it;
		}
	}
};

//----------------------------------------------------------
// Move hand by attractor
//----------------------------------------------------------
void Interpreter::move_hand(){
	assert(hand != NULL);
	assert(command != NULL);

	TrajectoryGenerator tg;
	tg.make_trajectory(hand->get_x(), command->get_attractor());

	//double step_size = 0.00015;
	double step_size;
	if(hand->get_target_list().empty()){
		//step_size = 0.001;
		//step_size = 0.0003;
		step_size = 0.0003;
	}
	else{
		//step_size = 0.0003;
		step_size = 0.00015;
	}
	
	Vec3d dx;
	Vec3d dir = tg.get_next_x() - tg.get_curr_x();
	if(norm(dir) == 0){
		dx = 0;
	}
	else{
		Vec3d unit_dir = dir / norm(dir);
		if(!hand->get_target_list().empty()){
			Vec3d f;
			list<Particle*>::iterator it = hand->get_target_list().begin();
			while(it != hand->get_target_list().end()){
				f += (*it)->get_f();
				++ it;
			}
			Vec3d unit_f = f / norm(f);
			double thresh = POW_THRESH  *  hand->get_target_list().size() * hand->get_target_list().size();
			if((norm(f)) < thresh){
			   	dx = (unit_dir) * step_size;
			}
			else{
			   	//dx = (unit_dir + 1.0 * unit_f) * step_size;
			   	//dx = (unit_dir + norm(f) / thresh * unit_f) / norm((unit_dir + norm(f) / thresh * unit_f));
			   	//dx = (unit_dir + norm(f) / thresh * unit_f);

			   	dx = (unit_dir + norm(f) / thresh * unit_f);
			   	
			   	//dx = (unit_dir + 7 * (f - thresh * unit_f)) / norm(unit_dir + 7 * (f - thresh * unit_f));
			   	dx *= step_size;
			}

		}
		else{
			dx = (unit_dir) * step_size;
		}
	} 
	hand_vel = 0.95 * hand_vel + 0.05 * dx / ClothSimulator::DT;

	hand->set_x((hand->get_x() + dx));
};

//----------------------------------------------------------
// Rotate hand by pose attractor
//----------------------------------------------------------
void Interpreter::rotate_hand(){
	assert(hand != NULL);
	assert(command != NULL);
	//----------------------------------------------------------
	//  Temporary substitute
	//----------------------------------------------------------<<<
	//hand->set_rot( command->get_pose_attractor().get_near_point( hand->get_rot()) );
	//----------------------------------------------------------<<<
	PoseAttractor& pa = command->get_pose_attractor();
	Vec3d r_rot;
	switch(pa.get_type()){
		case PoseAttractor::E_Point:
			break;
		case PoseAttractor::E_Line:
			if(rf::search_angle(hand->get_side(), hand->get_x(), hand->get_rot(), pa.get_x(), r_rot, *joints))
				hand->set_rot(r_rot);
			break;
		case PoseAttractor::E_Plane:
			break;
		default:
			break;
	}
};

//----------------------------------------------------------
// Check whether or not hand reached the attraction point,
// or it can't move anymore.
//----------------------------------------------------------
bool Interpreter::check_convergence(){
	assert(hand != NULL);
	assert(command != NULL);

	Attractor & att = command->get_attractor() ;
	bool comp = false;
	
	// Is hand close enough to the attraction point.
	if( norm(att.get_near_point(hand->get_x()) - hand->get_x()) < NEAR_THRESH){
		comp = true;
	}

	// cloth is stretched to enough.
	if(!hand->get_target_list().empty()){
		double thresh = POW_THRESH  *  hand->get_target_list().size() * hand->get_target_list().size();
		
		Vec3d f;
		list<Particle*>::iterator it = hand->get_target_list().begin();
		while(it != hand->get_target_list().end()){
			f += (*it)->get_f();
			++ it;
		}

		timer.stop();
		cout << norm(hand_vel) << " " << timer.get_interval() << endl;
		if( norm(hand_vel) < VEL_THRESH && norm(f) > thresh && timer.get_interval() > PAST_TIME_THRESH){
			comp = true;
		}
	}
	return comp;

};

