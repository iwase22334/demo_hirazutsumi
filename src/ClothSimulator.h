#ifndef CLOTH_SIMULATOR_H
#define CLOTH_SIMULATOR_H
//----------------------------------------------------------
// include
//----------------------------------------------------------
#include <opencv2/core/core.hpp>
#include <iostream>
#include <vector>
#include <list>

#include "polygon.h"
#include "timer.h"
#include "attractor.h"
#include "StringModel.h"
#include "particle.h"
#include "ClothSimulator.h"
#include "FCalculator.h"

//----------------------------------------------------------
// namespace
//----------------------------------------------------------
using namespace cv;
using namespace std;
//----------------------------------------------------------
// class cloth simulator
//----------------------------------------------------------
class ClothSimulator{
public:
	static const double DT;
	static const double BOX_WIDTH;
	static const double BOX_HEIGHT;
	static const double BOX_DEPTH;
	static const double STRING_LENGTH_W;
	static const double STRING_LENGTH_D;
	static const double PCM_STOP_THRESH;

private:
	// simulation time
	double time;
	Timer timer;

	list<Box> box_list;
	list<Pol4> polygon_list;
	list<StringModel> str_model_list;

public:
	//constructor
	ClothSimulator();
	ClothSimulator(const ClothSimulator &);
	
	double get_time() const;
	double get_calc_time() const;
	list<Box>& get_box_list();
	list< Pol4 >& get_polygon_list();
	list<StringModel>& get_str_model_list();
	vector<Vec3d> get_hole(const Particle*, const Particle*);	
	void get_closest_particle(Particle**, Particle**, StringModel &, StringModel &);

	void calc_force();
	void next_step();

	ClothSimulator& operator=(const ClothSimulator&);

private:
	void solve_predictor_corrector_method();
	void predict();
	void correct(ClothSimulator);
	double get_amount_of_change(ClothSimulator &, ClothSimulator &);

};

#endif