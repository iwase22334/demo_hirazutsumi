//----------------------------------------------------------
// include
//----------------------------------------------------------	
#include "ClothSimulator.h"
#include <cstdlib>
#include <cmath>
//----------------------------------------------------------
// Constant Parameter
//----------------------------------------------------------
const double ClothSimulator::DT(0.001);
const double ClothSimulator::BOX_WIDTH(0.16);
const double ClothSimulator::BOX_HEIGHT(0.06);
const double ClothSimulator::BOX_DEPTH(0.11);
const double ClothSimulator::STRING_LENGTH_W(0.18);
const double ClothSimulator::STRING_LENGTH_D(0.18);
const double ClothSimulator::PCM_STOP_THRESH(0.001);

//----------------------------------------------------------
// Constructor
//----------------------------------------------------------
//constructor
ClothSimulator::ClothSimulator():
	time(0)
{

	// Set box list
	Box box(BOX_WIDTH, BOX_HEIGHT, BOX_DEPTH);
	box_list.push_back(box);

	// Set polygon list
    vector<Vec3d> floor;
    Vec3d v1(1,1,-0.005);
    Vec3d v2(-1,1,-0.005);
    Vec3d v3(-1,-1,-0.005);
    Vec3d v4(1,-1,-0.005);
    floor.push_back( v1 );
    floor.push_back( v2 );
    floor.push_back( v3 );
    floor.push_back( v4 );
    Pol4 p_floor(floor);
    polygon_list.push_back(p_floor);

    // Set str model list
    Vec3d lPoint1(-BOX_DEPTH / 2.0 , 0 , 0);
    Vec3d rPoint1(-BOX_DEPTH / 2.0 - STRING_LENGTH_D, 0, 0);
    StringModel str1(lPoint1, rPoint1);
    str1.set_dt(DT);
    
    Vec3d lPoint2(BOX_DEPTH / 2.0 , 0, 0);
    Vec3d rPoint2(BOX_DEPTH / 2.0 + STRING_LENGTH_D, 0, 0);
    StringModel str2(lPoint2, rPoint2);
    str2.set_dt(DT);

	Vec3d rPoint3(0, -BOX_WIDTH / 2.0, 0);
	Vec3d lPoint3(0, -BOX_WIDTH / 2.0 - STRING_LENGTH_W, 0);
	StringModel str3(rPoint3, lPoint3);
	str3.set_dt(DT);

	Vec3d lPoint4(0, BOX_WIDTH / 2.0 , 0);
	Vec3d rPoint4(0, BOX_WIDTH / 2.0 + STRING_LENGTH_W, 0);
	StringModel str4(lPoint4, rPoint4);
	str4.set_dt(DT);

	str_model_list.push_back(str1);
	str_model_list.push_back(str2);
	str_model_list.push_back(str3);
	str_model_list.push_back(str4);

    list<StringModel>::iterator it = get_str_model_list().begin();
    while(it != get_str_model_list().end()){
    	(*it).set_handle(&(*(*it).get_string_back()));
    	++ it;
    }
	
/*
	// So many strings
    const int STRING_NUM(10);
	for(int i = 0; i < STRING_NUM; ++ i){
		Vec3d lPoint3( (double)rand() / RAND_MAX * 0.3 - 0.15, (double)rand() / RAND_MAX * 0.3 - 0.15, (double)rand() / RAND_MAX * 0.2 + 0.15);
		double th = (double)rand() / RAND_MAX * 2.0 * M_PI;
		double ph = (double)rand() / RAND_MAX * 2.0 * M_PI;
		Vec3d rPoint3;
		rPoint3[0] = lPoint3[0] + (STRING_LENGTH / 2.5) * cos(th) * sin(ph);
		rPoint3[1] = lPoint3[1] + (STRING_LENGTH / 2.5) * sin(th) * sin(ph);
		rPoint3[2] = lPoint3[2] + (STRING_LENGTH / 2.5) * cos(ph);
		StringModel str3(rPoint3, lPoint3);
		str3.get_string_begin()->unfix();
		str3.get_string_back()->unfix();
		str_model_list.push_back(str3);
	}
*/
};

ClothSimulator::ClothSimulator(const ClothSimulator & cs) :
	time(cs.time),
	box_list(cs.box_list),
	polygon_list(cs.polygon_list),
	str_model_list(cs.str_model_list)
{

    /*
	// Copy box list
	list<Box>::iterator it_box = cs.box_list.begin();
	while(it_box != cs.box_list.end()){
		box_list.push_back(*it_box);
		++ it_box;
	}
	*/
	/*
	// Copy polygon list
	list< Pol4 >::iterator it_polygon = cs.polygon_list.begin();
	while(it_polygon != c.polygon_list.end()){
		polygon_list.push_back(*it_polygon);
		++ it_polygon;
	}*/
	/*
	// Copy String model list
	list<StringModel>::iterator it_str_model = cs.str_model_list.begin();
	while(it_str_model != cs.str_model_list.end()){
		str_model_list.push_back(*it_str_model);
		++ it_str_model;
	}
	*/

};
ClothSimulator& ClothSimulator::operator=(const ClothSimulator& a){
	// Copy box list
	ClothSimulator cs(a);
	time = a.time;
	//copy(cs.box_list.begin(), cs.box_list.end(), box_list.begin());
	//copy(cs.polygon_list.begin(), cs.polygon_list.end(), polygon_list.begin());
	//copy(cs.str_model_list.begin(), cs.str_model_list.end(), str_model_list.begin());
	// Copy box list
	box_list.resize(a.box_list.size());
	list<Box>::iterator it_box1 = cs.box_list.begin();
	list<Box>::iterator it_box2 = box_list.begin();
	while(it_box1 != cs.box_list.end()){
		*it_box2 = *it_box1;
		++ it_box1;
		++ it_box2;
	}
	polygon_list.resize(a.polygon_list.size());
	// Copy polygon list
	list< Pol4 >::iterator it_polygon1 = cs.polygon_list.begin();
	list< Pol4 >::iterator it_polygon2 = polygon_list.begin();
	while(it_polygon1 != cs.polygon_list.end()){
		*it_polygon2 = *it_polygon1;
		++ it_polygon1;
		++ it_polygon2;
	}
	str_model_list.resize(a.str_model_list.size());
	// Copy String model list
	list<StringModel>::iterator it_str_model1 = cs.str_model_list.begin();
	list<StringModel>::iterator it_str_model2 = str_model_list.begin();
	while(it_str_model1 != cs.str_model_list.end()){
		*it_str_model2 = *it_str_model1;
		++ it_str_model1;
		++ it_str_model2;
	}
	return *this;
};


//----------------------------------------------------------
// Getter
//----------------------------------------------------------
double ClothSimulator::get_time() const { return time; };
double ClothSimulator::get_calc_time() const { return timer.get_interval(); };
list<Box>& ClothSimulator::get_box_list() { return box_list; };
list< Pol4 >& ClothSimulator::get_polygon_list() { return polygon_list; };
list<StringModel>& ClothSimulator::get_str_model_list() { return str_model_list; };

vector<Vec3d> ClothSimulator::get_hole(const Particle* p1, const Particle* p2){
		list<Vec3d> hole;
        StringModel & s1 = str_model_list.front();
        StringModel & s2 = str_model_list.back();
        {
			list<Particle>::iterator it = s1.get_string_begin();
			while((*it).get_x()[2] < BOX_HEIGHT) ++ it;
			while(&(*it) != p1){
				hole.push_back((*it).get_x());
				++ it;
			}
		}
		list<Vec3d> rev_hole;
		hole.push_back(p2->get_x());
		{
			list<Particle>::iterator it = s2.get_string_begin();
			while((*it).get_x()[2] < BOX_HEIGHT) ++ it;
			while(&(*it) != p2){
				rev_hole.push_back((*it).get_x());
				++ it;
			}
		}
		rev_hole.reverse();
		hole.insert(hole.end(), rev_hole.begin(), rev_hole.end());
		return vector<Vec3d>(hole.begin(),hole.end());
};	


void ClothSimulator::get_closest_particle(Particle** p1, Particle **p2, StringModel & sm1, StringModel & sm2){
	list<Particle>::iterator it_p1 = sm1.get_string_begin();
	list<Particle>::iterator it_p2 = sm2.get_string_begin();
	double min_dist = norm((*it_p1).get_x() - (*it_p2).get_x());
	*p1 = &(*it_p1);
	*p2 = &(*it_p2);

	while(it_p1 != sm1.get_string_end()){
		it_p2 = sm2.get_string_begin();
		while(it_p2 != sm2.get_string_end()){
			if( norm((*it_p1).get_x() - (*it_p2).get_x()) <= min_dist){
				min_dist =  norm((*it_p1).get_x() - (*it_p2).get_x());
				*p1 = &(*it_p1);
				*p2 = &(*it_p2);
			}
			++ it_p2;
		}
		++ it_p1;
	}
};
//----------------------------------------------------------
// Calculate force
//----------------------------------------------------------
void ClothSimulator::calc_force(){
	// Check self collision
	{	
		list<StringModel>::iterator it_str = str_model_list.begin();
		// Check all string models
		while(it_str != str_model_list.end()){
			list<Particle>::iterator it_target = (*it_str).get_string_begin();
			// Check all particle in the string
			while (it_target != (*it_str).get_string_end()){
			    list<Particle>::iterator it_check = (*it_str).get_string_begin();
			    list<Particle>::iterator prev_it_target = it_target;
			    list<Particle>::iterator next_it_target = it_target;
			    --prev_it_target;
			    ++next_it_target;
			    // Check self collision
			    while (it_check != (*it_str).get_string_end()){
			        if(it_check != it_target && it_check != prev_it_target && it_check != next_it_target){
			            FCalculator::add_particle_force(5.0, 0.04, *it_check, *it_target);
			        }
			        ++ it_check;
			    }
			    ++ it_target;
			}
			++ it_str;
		}   
	}


	// Check collision of the box
	{	
		list<StringModel>::iterator it_str = str_model_list.begin();
		// Check all string models
		while(it_str != str_model_list.end()){
			list<Particle>::iterator it_par = (*it_str).get_string_begin();
			// Check all particle in the string
			while (it_par != (*it_str).get_string_end()){
				list<Box>::iterator it_box = box_list.begin();
				// Check collision with all box
				while(it_box != box_list.end()){
			    	//FCalculator::add_box_force(40.0, 0.04, *it_box, *it_par);
			    	FCalculator::add_box_force(80.0, 0.04, *it_box, *it_par);
			    	++ it_box;
			    }
			    ++ it_par;
			}
			++ it_str;
		}
	}

	// Check collision of the polygon
	//----------------------------------------------------------
	// Since not performed calculation of the normal vectors, 
	// it can only be used on the floor
	//----------------------------------------------------------
	{
		// Normal vector of the floor
		Vec3d vn(0, 0, 1);
		list<StringModel>::iterator it_str = str_model_list.begin();
		// Check all string models
		while(it_str != str_model_list.end()){
		    list<Particle>::iterator it_par = (*it_str).get_string_begin();
		    // Check all particle in the string
		    while (it_par != (*it_str).get_string_end()){
		        FCalculator::add_polygon_force(40.0, 0.01, 0.9, 0.5, vn, (*polygon_list.begin()), *it_par);
		        ++ it_par;
		    }
	    	++ it_str;
		}
	}

	// Check collision of the other string model
	{
		list<StringModel>::iterator it_str_target = str_model_list.begin();
		// Check all string models
		while(it_str_target != str_model_list.end()){
	    	list<Particle>::iterator it_par_target = (*it_str_target).get_string_begin();
	    	// Check all particles in the string
	    	while (it_par_target != (*it_str_target).get_string_end()){
	    		list<StringModel>::iterator it_str_check = it_str_target;
				++ it_str_check;
				// Check all of the other string model
				while(it_str_check != str_model_list.end()){
					list<Particle>::iterator it_par_check = (*it_str_check).get_string_begin();
					// Check all particles in other string model
					while (it_par_check != (*it_str_check).get_string_end()){
					    FCalculator::add_particle_force(30.0, 0.04, *it_par_target, *it_par_check);
					    ++ it_par_check;
					}
					++ it_str_check;
				}
	    	    ++ it_par_target;
	    	}
	    	++ it_str_target;
		}
	}

	{
		list<StringModel>::iterator it_str = str_model_list.begin();
		// Check all string models
		while(it_str != str_model_list.end()){
			(*it_str).calc_force();
			++ it_str;
		}
	}
};


//----------------------------------------------------------
// Move to the next step
//----------------------------------------------------------
void ClothSimulator::next_step(){
	/*
	list<StringModel>::iterator it_str = str_model_list.begin();
	// Check all string models
	while(it_str != str_model_list.end()){
		(*it_str).move();
		++ it_str;
	}*/
	
	solve_predictor_corrector_method();
};

void ClothSimulator::solve_predictor_corrector_method(){
	ClothSimulator curr_cs(*this);
	predict();
	while(get_amount_of_change(*this, curr_cs) > PCM_STOP_THRESH){
		correct(curr_cs);
	}
};

void ClothSimulator::predict(){
	list<StringModel>::iterator it_str = str_model_list.begin();
	// Check all string models
	while(it_str != str_model_list.end()){
		(*it_str).move();
		++ it_str;
	}
};

void ClothSimulator::correct(ClothSimulator cs){
	calc_force();
	list<StringModel>::iterator it_str1 = str_model_list.begin();
	list<StringModel>::iterator it_str2 = cs.str_model_list.begin();
	// Check all string models
	while(it_str1 != str_model_list.end()){
		list<Particle>::iterator it1 = it_str1->get_string_begin();
		list<Particle>::iterator it2 = it_str2->get_string_begin();

		// Check all Particles in string
		while(it1 != it_str1->get_string_end()){
			it1->set_f((it1->get_f() + it2->get_f()));
			Vec3d v_sum = (it1->get_v() + it2->get_v());
			it1->set_v(it2->get_v());
			it1->set_x(it2->get_x());

			it1->correct(v_sum);

			++ it1;
			++ it2;
		}

		++ it_str1;
		++ it_str2;
	}

};
double ClothSimulator::get_amount_of_change(ClothSimulator & cs1, ClothSimulator &cs2){
	double v_sum(0);
	double x_sum(0);
	int p_num = 0;
	list<StringModel>::iterator it_str1 = cs1.str_model_list.begin();
	list<StringModel>::iterator it_str2 = cs2.str_model_list.begin();
	// Check all string models
	while(it_str1 != cs1.str_model_list.end()){
		list<Particle>::iterator it1 = it_str1->get_string_begin();
		list<Particle>::iterator it2 = it_str2->get_string_begin();

		// Check all Particles in string
		while(it1 != it_str1->get_string_end()){
			v_sum += norm(it1->get_v() - it2->get_v());
			x_sum += norm(it1->get_x() - it2->get_x());

			++ it1;
			++ it2;
			++ p_num;
		}

		++ it_str1;
		++ it_str2;
	}

	return (x_sum) / p_num;

};
	