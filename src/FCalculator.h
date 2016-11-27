#ifndef FCALCULATOR_H
#define FCALCULATOR_H
//----------------------------------------------------------
// include
//----------------------------------------------------------
#include <opencv2/core/core.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include "box.h"
#include "particle.h"
#include "polygon.h"
//----------------------------------------------------------
// namespace
//----------------------------------------------------------
using namespace cv;
using namespace std;

//----------------------------------------------------------
// struct
//----------------------------------------------------------
struct FCalculator{
	static const double G;
	static const double AIR_D;
	static const double MU;
	static const double MINIMUM_DIST;
	static double gaussian_kernel(double, Vec3d);

	// Environment force
	static void add_g(Particle&);
	static void add_air_resistance(Particle&);

	// Connection force between two particles
	static void add_spring_force(const double, const double, Particle&, Particle&);
	static void add_damping_force(const double, Particle&, Particle&);

	// Reaction force from particle
	static void add_particle_force(const double, const double, Particle &, Particle &);

	// Reaction force from polygon
	static void add_box_force(const double, const double, Box &, Particle &);
	static void add_polygon_force(const double, const double, const double, const double, const Vec3d &, const Pol4 &, Particle &);
};

#endif