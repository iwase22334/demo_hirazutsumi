//----------------------------------------------------------
// include
//----------------------------------------------------------
#include "particle.h"
#include <iostream>

//----------------------------------------------------------
// Constant parameter
//----------------------------------------------------------
//dimension
const int Particle::DIM(3);

//const double Particle::DEFAULT_RAD(0.01);
//const double Particle::DEFAULT_DT(0.0005);
//const double Particle::DEFAULT_M(0.0001);

const double Particle::DEFAULT_RAD(0.005);
const double Particle::DEFAULT_DT(0.00015);
//const double Particle::DEFAULT_DT(0.001);
const double Particle::DEFAULT_M(0.0005);


//----------------------------------------------------------
// Constructor
//----------------------------------------------------------
Particle::Particle(){
	fixity = false;
	standstill = false;
	rad = DEFAULT_RAD;
	dt = DEFAULT_DT;
	m = DEFAULT_M;
};

Particle::Particle(const Vec3d & i_x){
	x = i_x;
	fixity = false;
	standstill = false;
	rad = DEFAULT_RAD;
	dt = DEFAULT_DT;
	m = DEFAULT_M;
};

Particle::Particle(const Particle & par){
	fixity = par.fixity;
	standstill = par.standstill;
	rad = par.rad;
	dt = par.dt;
	m = par.m;
	f = par.f;
	v = par.v;
	x = par.x;
};

//----------------------------------------------------------
// Setter
//----------------------------------------------------------
void Particle::set_fixity(const bool i_fixity) { fixity = i_fixity; };
void Particle::set_standstill(const bool i_standstill) { standstill = i_standstill; };
void Particle::set_rad(const double i_rad) { rad = i_rad; };
void Particle::set_dt(const double i_dt) { dt = i_dt; };
void Particle::set_m(const double i_m) { m = i_m; };
void Particle::add_f(const Vec3d i_f) { f = f + i_f; };
void Particle::set_f(const Vec3d i_f) { f = i_f; };
void Particle::set_v(const Vec3d i_v) { v = i_v; };
void Particle::set_x(const Vec3d i_x) { x = i_x; };

void Particle::fix() { fixity = true; };
void Particle::unfix() { fixity = false; };
void Particle::immobilize() { standstill = true; };

//----------------------------------------------------------
// Getter
//----------------------------------------------------------
bool Particle::get_fixity() const { return fixity; };
bool Particle::get_standstill() const { return standstill; };
double Particle::get_rad() const {return rad; };
double Particle::get_dt() const { return dt; };
double Particle::get_m() const { return m; };
Vec3d Particle::get_prev_f() const { return prev_f; };
Vec3d Particle::get_f() const { return f; };
Vec3d Particle::get_v() const { return v; };
Vec3d Particle::get_x() const { return x; };

//----------------------------------------------------------
//predictor-corrector_method
//----------------------------------------------------------
void Particle::correct(Vec3d i_v){
	//Particle state vector
	Vec6d n_x;
	Vec6d corr_x;

	//Convert x,v to 6d vector
	for(int i = 0; i < DIM; ++i){
		corr_x[i] = x[i];
		corr_x[i+DIM] = i_v[i];
	}

	//Convert x,v to 6d vector
	for(int i = 0; i < DIM; ++i){
		n_x[i] = x[i];
		n_x[i+DIM] = v[i];
	}

	//Calculate equation of motion by euler's method
	Vec6d ans = n_x + func(0, corr_x) * dt / 2.0;

	//Convert 6d vector to x,v
	for(int i = 0; i < DIM; ++i){
		x[i] = ans[i];
		v[i] = ans[i+DIM];
	}

	//init force
	prev_f = f;
	Vec3d zero(0, 0, 0);
	f = zero;
	standstill = false;
};

//----------------------------------------------------------
// Euler's method
//----------------------------------------------------------
void Particle::move_euler(){
	//Particle state vector
	Vec6d n_x;

	//Convert x,v to 6d vector
	for(int i = 0; i < DIM; ++i){
		n_x[i] = x[i];
		n_x[i+DIM] = v[i];
	}

	//Calculate equation of motion by euler's method
	Vec6d ans = n_x + func(0, n_x) * dt;

	//Convert 6d vector to x,v
	for(int i = 0; i < DIM; ++i){
		x[i] = ans[i];
		v[i] = ans[i+DIM];
	}

	//init force
	prev_f = f;
	Vec3d zero(0, 0, 0);
	f = zero;
	standstill = false;
};

//----------------------------------------------------------
// 2nd order runge kutta
//----------------------------------------------------------
void Particle::move_rk2(){
	//Particle state vector
	Vec6d n_x;

	//Convert x,v to 6d vector
	for(int i = 0; i < DIM; ++i){
		n_x[i] = x[i];
		n_x[i+DIM] = v[i];
	}

	//Calculate equation of motion by rk2
	Vec6d k1 = func(0, n_x) * dt;
	Vec6d k2 = func(0, n_x + k1) * dt;
	Vec6d ans = n_x + (k2 + k1) / 2.0;

	//Convert 6d vector to x,v
	for(int i = 0; i < DIM; ++i){
		x[i] = ans[i];
		v[i] = ans[i+DIM];
	}

	//init force
	prev_f = f;
	Vec3d zero(0, 0, 0);
	f = zero;
	standstill = false;
};

//----------------------------------------------------------
// 4th order runge kutta
//----------------------------------------------------------
void Particle::move_rk4(){
	//Particle state vector
	Vec6d n_x;

	//Convert x,v to 6d vector
	for(int i = 0; i < DIM; ++i){
		n_x[i] = x[i];
		n_x[i+DIM] = v[i];
	}

	//Calculate equation of motion by rk4
	Vec6d k1 = func(0, n_x);
	Vec6d k2 = func(0, n_x + (dt / 2.0) * k1);
	Vec6d k3 = func(0, n_x + (dt / 2.0) * k2);
	Vec6d k4 = func(0, dt * k3);
	Vec6d ans = n_x + (dt / 6.0) * (k1 + k2 / 2.0 + k3 / 2.0 + k4);

	//Convert 6d vector to x,v
	for(int i = 0; i < DIM; ++i){
		x[i] = ans[i];
		v[i] = ans[i+DIM];
	}

	//init force
	prev_f = f;
	Vec3d zero(0, 0, 0);
	f = zero;
	standstill = false;
};

//----------------------------------------------------------
// equation of motion
//----------------------------------------------------------
Vec6d Particle::func(const double i_t, const Vec6d & i_x){
	Vec6d y;

	//check particle weight
	if(m == 0){
		std::cout << "particle weight is zero" << std::endl;
		//return zero vector
		return y;
	}

	//calculate equation
	for(int i = 0; i < DIM; ++i){
		y[i] = i_x[i+DIM];
		if(!standstill){
			y[i+DIM] = f[i] / m;
		}
	}
	return y;
};