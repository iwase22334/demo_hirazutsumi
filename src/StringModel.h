#ifndef STRING_MODEL_H
#define STRING_MODEL_H

//----------------------------------------------------------
// include
//----------------------------------------------------------
#include <list>
#include <iostream>
#include "particle.h"
#include "FCalculator.h"
//----------------------------------------------------------
// namespace
//----------------------------------------------------------
using namespace std;
//----------------------------------------------------------
// class
//----------------------------------------------------------
class StringModel{
public:
	// Container of 2 particle pointers
	class Line{
		private:
			Particle* par_begin;
			Particle* par_end;
		public:
			// Constructor
			Line();
			Line(Particle*, Particle*);
			// Setter
			void set_begin(Particle*);
			void set_end(Particle*);
			// Getter
			Particle* get_begin();
			Particle* get_end();
			Vec3d get_begin_point();
			Vec3d get_end_point();
	};

private:
	static const double INIT_DIST_PAR_TO_PAR;
	static const double INIT_K;
	static const double INIT_D;
	static const double INIT_THICKNESS;

private:
	// Motion parameter
	double stable_dist;
	double k;
	double d;
	double thickness;

	// Particles
	list<Particle> lis_particle;
	list<Particle*> lis_fixed_particle;
	list<Line> lis_line;
	Particle* handle;

public:
	// Constructor
	StringModel();
	StringModel(const cv::Vec3d &, const cv::Vec3d &);
	StringModel(const StringModel &);

	// Setter
	void set_dt(const double &);
	void set_stable_dist(const double &);
	void set_thickness(const double &);
	void set_handle(Particle*);

	// Getter
	int get_particle_num() const;
	double get_stable_dist() const;
	double get_thickness() const;
	Particle* get_handle() const;
	
	list<Particle>::iterator get_string_begin();
	list<Particle>::iterator get_string_back();
	list<Particle>::iterator get_string_end();
	
	list<Line>::iterator get_line_begin();
	list<Line>::iterator get_line_back();
	list<Line>::iterator get_line_end();

	// Get velocity of fastest particle
	Vec3d get_max_v();
	
	void calc_force();
	// Move all particles
	void move();

	StringModel& operator=(const StringModel& );

private:
	void update_fixity();
	// Check Courant-Friedrichs-Lewy Condition 
	void checkCLF();

};
#endif
