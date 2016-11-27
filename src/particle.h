#ifndef PARTICLE_H
#define PARTICLE_H
//----------------------------------------------------------
// include
//----------------------------------------------------------
#include <opencv2/core/core.hpp>

//----------------------------------------------------------
// namespace
//----------------------------------------------------------
using namespace cv;

//----------------------------------------------------------
// class
//----------------------------------------------------------
class Particle{
private:
	static const int DIM;
	static const double DEFAULT_RAD;
	static const double DEFAULT_DT;
	static const double DEFAULT_M;
	
private:
	bool fixity;
	bool standstill;
	double rad;
	double dt;
	double m;
	Vec3d prev_f;
	Vec3d f;
	Vec3d v;
	Vec3d x;

public:
	//constructor
	Particle();
	Particle(const Vec3d &);
	Particle(const Particle &);

	//setter
	void set_fixity(const bool);
	void set_standstill(const bool);
	void set_rad(const double);
	void set_dt(const double);
	void set_m(const double);
	void add_f(const Vec3d);
	void set_f(const Vec3d);
	void set_v(const Vec3d);
	void set_x(const Vec3d);
	void fix();
	void unfix();
	void immobilize();

	//getter
	bool get_fixity() const;
	bool get_standstill() const;
	double get_rad() const;
	double get_dt() const;
	double get_m() const;
	Vec3d get_prev_f() const;
	Vec3d get_f() const;
	Vec3d get_v() const;
	Vec3d get_x() const;

public:
	//predictor-corrector_method
	void correct(Vec3d);

	//euler method
	void move_euler();

	//2nd order runge kutta
	void move_rk2();

	//4th order runge kutta
	void move_rk4();

private:
	//Moving function
	Vec6d func(const double, const Vec6d &);

};
#endif