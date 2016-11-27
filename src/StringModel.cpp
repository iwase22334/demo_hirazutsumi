//----------------------------------------------------------
// include
//----------------------------------------------------------
#include "StringModel.h"
//----------------------------------------------------------
// Constant parameter
//----------------------------------------------------------
const double StringModel::INIT_DIST_PAR_TO_PAR(0.005);
//const double StringModel::INIT_K(100);
//const double StringModel::INIT_D(0.08);

//const double StringModel::INIT_K(500);
//const double StringModel::INIT_D(0.10);

const double StringModel::INIT_K(1500.0);
const double StringModel::INIT_D(0.12);

const double StringModel::INIT_THICKNESS(0.01);


//----------------------------------------------------------
// Class Line
// The containar of the 2 particle pointers
//----------------------------------------------------------
StringModel::Line::Line():
	par_begin(NULL),
	par_end(NULL)
{};
StringModel::Line::Line(Particle* begin, Particle* end):
	par_begin(begin),
	par_end(end)
{};
void StringModel::Line::set_begin(Particle* p){
	par_begin = p;
};
void StringModel::Line::set_end(Particle* p){
	par_end = p;
};
Particle* StringModel::Line::get_begin(){
	return par_begin;
};
Particle* StringModel::Line::get_end(){
	return par_end;
};
Vec3d StringModel::Line::get_begin_point(){
	return par_begin->get_x();
};
Vec3d StringModel::Line::get_end_point(){
	return par_end->get_x();
};


//----------------------------------------------------------
// class String Model
//----------------------------------------------------------
//----------------------------------------------------------
// Constructor
//----------------------------------------------------------
StringModel::StringModel():
	stable_dist(INIT_DIST_PAR_TO_PAR),
	k(INIT_K),
	d(INIT_D),
	thickness(INIT_THICKNESS)
{};

// StringModel(begin (fixed), back (fixed, handring) )
StringModel::StringModel(const cv::Vec3d & x1, const cv::Vec3d & x2):
	stable_dist(INIT_DIST_PAR_TO_PAR - 0.00001),
	k(INIT_K),
	d(INIT_D),
	thickness(INIT_THICKNESS)
{
	// Make Tip particle
	Particle par1(x1);
	Particle par2(x2);

	// Fix particle
	par1.fix();
	par2.fix();

	// Set particle
	lis_particle.push_back(par1);

	for(int i = 0; (i + 1) * INIT_DIST_PAR_TO_PAR < norm(x1 - x2); ++i ){
		cv::Vec3d n = (x2 - x1) * (1.0 / norm(x2 - x1));
		cv::Vec3d ix = (i + 1) * INIT_DIST_PAR_TO_PAR * n + x1;
		Particle par(ix);
		lis_particle.push_back(par);
	}

	// lis_particle.pop_back();
	lis_particle.push_back(par2);

	// Set list of fixed particle iterator 
	{
		list<Particle>::iterator it_fixedpar1;
		list<Particle>::iterator it_fixedpar2;
		Particle * p_par1;
		Particle * p_par2;

		it_fixedpar1 = lis_particle.begin();
		p_par1 = &(*it_fixedpar1);
		lis_fixed_particle.push_back(p_par1);

		p_par2 = &lis_particle.back();
		lis_fixed_particle.push_back(p_par2);
	}

	// Set list of line 
	{
		list<Particle>::iterator it_par1 = lis_particle.begin();
		list<Particle>::iterator it_par2 = it_par1;
		++ it_par2;
		while(it_par2 != lis_particle.end()){
			Particle * p_par1 = &(*it_par1);
			Particle * p_par2 = &(*it_par2);
			Line line(p_par1, p_par2);
			lis_line.push_back(line);
			++ it_par1;
			++ it_par2;
		}
	}

};

StringModel::StringModel(const StringModel & i_strModel):
	stable_dist(i_strModel.stable_dist),
	k(i_strModel.k),
	d(i_strModel.d),
	thickness(i_strModel.thickness),
	lis_particle(i_strModel.lis_particle)
{
	// Set list of line 
	{
		list<Particle>::iterator it_par1 = lis_particle.begin();
		list<Particle>::iterator it_par2 = it_par1;
		++ it_par2;
		while(it_par2 != lis_particle.end()){
			Particle * p_par1 = &(*it_par1);
			Particle * p_par2 = &(*it_par2);
			Line line(p_par1, p_par2);
			lis_line.push_back(line);
			++ it_par1;
			++ it_par2;
		}
	}

	update_fixity();
};


StringModel& StringModel::operator=(const StringModel& a){
	StringModel sm(a);
	stable_dist = sm.stable_dist;
	k = sm.k;
	d = sm.d;
	thickness = sm.thickness;
	{
		list<Particle>::iterator it_par1 = sm.lis_particle.begin();
		list<Particle>::iterator it_par2 = lis_particle.begin();
		while(it_par2 != lis_particle.end()){
			*it_par2 = *it_par1;
			++ it_par1;
			++ it_par2;
		}
	}	
	{
		list<Particle*>::iterator it_par1 = sm.lis_fixed_particle.begin();
		list<Particle*>::iterator it_par2 = lis_fixed_particle.begin();
		while(it_par2 != lis_fixed_particle.end()){
			*it_par2 = *it_par1;
			++ it_par1;
			++ it_par2;
		}
	}	
	{
		list<Line>::iterator it_par1 = sm.lis_line.begin();
		list<Line>::iterator it_par2 = lis_line.begin();
		while(it_par2 != lis_line.end()){
			*it_par2 = *it_par1;
			++ it_par1;
			++ it_par2;
		}
	}
	//handle = sm.handle;
};
//----------------------------------------------------------
// Setter
//----------------------------------------------------------

void StringModel::set_dt(const double & dt){
	list<Particle>::iterator it = lis_particle.begin();
	while(it != lis_particle.end()){
		(*it).set_dt(dt);
		++ it;
	}
};
// Set stable dist of spring 
void StringModel::set_stable_dist(const double & sDist) { stable_dist = sDist; };
// Set thickness of string model
void StringModel::set_thickness(const double & t) { thickness = t; };
// Set handring particle pointer
void StringModel::set_handle(Particle* i_handle) { handle = i_handle; };

//----------------------------------------------------------
// Getter
//----------------------------------------------------------

int StringModel::get_particle_num() const { return lis_particle.size(); };
// Retuen stable dist of spring
double StringModel::get_stable_dist() const { return stable_dist; };
// Return thickness of string model
double StringModel::get_thickness() const { return thickness; };
// Return handring particle pointer
Particle* StringModel::get_handle() const { return handle; };

// Return string iterator
list<Particle>::iterator StringModel::get_string_begin(){ 
	list<Particle>::iterator it = lis_particle.begin();
	return it; 
};

list<Particle>::iterator StringModel::get_string_back(){ 
	list<Particle>::iterator it = lis_particle.begin();
	for(int i = 0; i + 1 < lis_particle.size(); ++ i){
		++ it;
	}
	return it; 
};

list<Particle>::iterator StringModel::get_string_end(){ 
	list<Particle>::iterator it = lis_particle.end();
	return it; 
};

// Return line iterator
list<StringModel::Line>::iterator StringModel::get_line_begin(){ 
	list<StringModel::Line>::iterator it = lis_line.begin();
	return it; 
};

list<StringModel::Line>::iterator StringModel::get_line_back(){ 
	list<StringModel::Line>::iterator it = lis_line.begin();
	for(int i = 0; i + 1 < lis_line.size(); ++ i){
		++ it;
	}
	return it; 
};

list<StringModel::Line>::iterator StringModel::get_line_end(){ 
	list<StringModel::Line>::iterator it = lis_line.end();
	return it; 
};

// Get fastest particle velocity
Vec3d StringModel::get_max_v(){
	Vec3d maxVel;
	Vec3d vBuf;
	list<Particle>::iterator it = lis_particle.begin();
	maxVel = (*it).get_v();
	++ it;
	while(it != lis_particle.end()){
		vBuf = (*it).get_v();
		if(norm(maxVel) < norm(vBuf)){
			maxVel = vBuf;
		}
		++ it;
	}
	return maxVel;
};

//----------------------------------------------------------
// Move string
//----------------------------------------------------------
void StringModel::update_fixity(){
	lis_fixed_particle.clear();
	list<Particle>::iterator p1 = lis_particle.begin();
	Particle *par;
	while(p1 != lis_particle.end()){
		// Check fixity
		if(p1->get_fixity()){
			par = &(*p1);
			lis_fixed_particle.push_back(par);
		}
		++ p1;
	}
};

void StringModel::calc_force(){
	list<Particle>::iterator p1 = lis_particle.begin();
	list<Particle>::iterator p2 = lis_particle.begin();
	++ p2;
	// Add force 
	while(p2 != lis_particle.end()){
		FCalculator::add_spring_force(stable_dist, k, *p1, *p2);
		FCalculator::add_damping_force(d, *p1, *p2);
		++ p1;
		++ p2;
	}

	// Add gravity force and check fixity
	list<Particle>::iterator p3 = lis_particle.begin();
	Particle *par;
	while(p3 != lis_particle.end()){
		// Add gravity force
		FCalculator::add_g(*p3);
		++ p3;
	}
	
};
// Update string state
void StringModel::move(){
	// check fixity
	update_fixity();

	// Turn off the force of fixed particle 
	list<Particle*>::iterator fxp = lis_fixed_particle.begin();
	cv::Vec3d zero(0, 0, 0);
	while(fxp != lis_fixed_particle.end()){
		(*fxp)->set_f(zero);
		(*fxp)->set_v(zero);
		++ fxp;
	}

	// Move particle
	list<Particle>::iterator p1 = lis_particle.begin();
	while(p1 != lis_particle.end()){

		//p1->move_euler();
		//p1->move_rk2();
		p1->move_rk4();

		++ p1;
	}
};
