#ifndef HAND_H
#define HAND_H
//----------------------------------------------------------
// include
//----------------------------------------------------------
#include <opencv2/core/core.hpp>
#include <list>
#include <iostream>
#include "particle.h"

//----------------------------------------------------------
// namespace
//----------------------------------------------------------
using namespace cv;
using namespace std;

//----------------------------------------------------------
// class Hand
//----------------------------------------------------------
class Hand{
public:
	enum E_Side{
		E_Left = 0,
		E_Right = 1
	};
	enum E_State{
		E_Handring = 0,
		E_Opening = 1
	};
private:
	E_Side side;
	E_State state;
	Vec3d x;
	Vec3d rot;
	list<Particle*> lis_target;

public:
	Hand(){};
	Hand(const Vec3d &, const Vec3d &);
	Hand(const Hand &);
	// Settter
	void set_side(const E_Side);
	void set_state(const E_State);
	void set_x(const Vec3d &);
	void set_rot(const Vec3d &);
	void set_target(Particle*);
	void set_target(list<Particle*>&);
	void release();

	// Getter
	E_Side get_side() const;
	E_State get_state() const;
	Vec3d get_x() const;
	Vec3d get_rot() const;
	list<Particle*>& get_target_list();

public:
	Hand& operator=(const Hand&);
};
class LeftHand : public Hand{
	LeftHand(const LeftHand &);
};
class RightHand : public Hand{
	RightHand(const RightHand &);
};

#endif