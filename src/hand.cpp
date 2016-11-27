#include "hand.h"
//----------------------------------------------------------
// class Hand
//----------------------------------------------------------

Hand::Hand(const Vec3d & ix, const Vec3d & irot):
	x(ix),
	rot(irot),
	state(E_Opening)
{};

Hand::Hand(const Hand & hs): 
	side(hs.side),
	state(hs.state),
	x(hs.x),
	rot(hs.rot)
	//lis_target(hs.lis_target)
{};

//----------------------------------------------------------
// Setter
//----------------------------------------------------------
void Hand::set_side(const E_Side s){ side = s; };
void Hand::set_state(const E_State st){
	state = st;
};
void Hand::set_x(const Vec3d & i_x){
	// set hand position
	Vec3d px = x;
	x = i_x;

	// Move target 
	if(!lis_target.empty()){
		Vec3d sub = x - px;
		list<Particle*>::iterator it = lis_target.begin();
		while(it != lis_target.end()){
			Vec3d par_x = (*it)->get_x() + sub;
			//(*it)->set_x( par_x );
			(*it)->set_x( par_x );
			++ it;
		}
	}
};
void Hand::set_rot(const Vec3d & i_rot){
	rot = i_rot;
};
void Hand::set_target(Particle* par){
	lis_target.clear();
	lis_target.push_back(par);
};
void Hand::set_target(list<Particle*>& tl){
	lis_target.clear();
	lis_target = tl;
}
void Hand::release(){
	state = E_Opening;
	lis_target.clear();
};

//----------------------------------------------------------
// Getter
//----------------------------------------------------------
Hand::E_Side Hand::get_side() const { return side; };
Hand::E_State Hand::get_state() const { return state; };
Vec3d Hand::get_x() const { return x; };
Vec3d Hand::get_rot() const { return rot; };
list<Particle*>& Hand::get_target_list(){ return lis_target; };


//----------------------------------------------------------
// Getter
//----------------------------------------------------------
Hand& Hand::operator=(const Hand& h){
	side = side;
	state = h.state;
	x = h.x;
	rot = h.rot;
	//copy(h.lis_target.begin(), h.lis_target.end(), lis_target.begin());
	return *this;
};


//----------------------------------------------------------
// class Left Hand
//----------------------------------------------------------
LeftHand::LeftHand(const LeftHand & h) : Hand(h) {};
//----------------------------------------------------------
// class Right Hand
//----------------------------------------------------------
RightHand::RightHand(const RightHand & h) : Hand(h) {};