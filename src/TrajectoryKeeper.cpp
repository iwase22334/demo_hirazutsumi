#include"TrajectoryKeeper.h"
//----------------------------------------------------------
// Constructor
//----------------------------------------------------------
TrajectoryKeeper::TrajectoryKeeper(const double i_interval) :
	interval(i_interval)
{};

TrajectoryKeeper::TrajectoryKeeper(const TrajectoryKeeper & tk) : 
	length(tk.length),
	interval(tk.interval),
	points(tk.points)
{};

//----------------------------------------------------------
// Setter
//----------------------------------------------------------
void TrajectoryKeeper::add_point(const Vec3d & x) { 
	Vec3d p(x);
	if(interval != 0 && points.size() > 2){
		std::list<Vec3d>::iterator it1 = points.end();
		-- it1;
		std::list<Vec3d>::iterator it2 = it1;
		-- it2;
		if(norm((*it1)-(*it2)) < interval){
			length -= norm((*it1)-(*it2));
			points.pop_back();
		}
	}

	length += norm(points.back() - p);
	points.push_back(p); 
};

//----------------------------------------------------------
// Getter
//----------------------------------------------------------
double TrajectoryKeeper::get_length() const { return length; };
double TrajectoryKeeper::get_interval() const { return interval; };
list<Vec3d>& TrajectoryKeeper::get_points() { return points; };


//----------------------------------------------------------
// list handler
//----------------------------------------------------------
void TrajectoryKeeper::clear() { 
	points.clear(); 
	length = 0;
};
list<Vec3d>::iterator TrajectoryKeeper::begin() { return points.begin(); };
list<Vec3d>::iterator TrajectoryKeeper::end() { return points.end(); };
Vec3d& TrajectoryKeeper::front(){ return points.front(); };
Vec3d& TrajectoryKeeper::back(){ return points.back(); };
void TrajectoryKeeper::push_back(const Vec3d & x) { 
	points.push_back(x); 
	length += norm(points.back() - x);
};
void TrajectoryKeeper::pop_back() { 
	std::list<Vec3d>::iterator it1 = points.end();
	-- it1;
	std::list<Vec3d>::iterator it2 = it1;
	-- it2;
	length -= norm((*it1)-(*it2));
	points.pop_back();
};

