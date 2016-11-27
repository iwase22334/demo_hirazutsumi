//----------------------------------------------------------
// include
//----------------------------------------------------------
#include "box.h"
//----------------------------------------------------------
// namespace
//----------------------------------------------------------
using namespace cv;
using namespace std;
//----------------------------------------------------------
// class Box
//----------------------------------------------------------

//----------------------------------------------------------
// Constant parameter
//----------------------------------------------------------
const int Box::SURFACE_NUM(6);
const double Box::DEFAULT_WIDTH(1);
const double Box::DEFAULT_HEIGHT(1);
const double Box::DEFAULT_DEPTH(1);

//----------------------------------------------------------
// Constructor
//----------------------------------------------------------
Box::Box():
	width(DEFAULT_WIDTH),
	height(DEFAULT_HEIGHT),
	depth(DEFAULT_DEPTH)
{
	set_vertex();
};
Box::Box(const Vec3d & i_trans):
	width(DEFAULT_WIDTH),
	height(DEFAULT_HEIGHT),
	depth(DEFAULT_DEPTH),
	trans(i_trans)
{
	set_vertex();
};
Box::Box(const Vec3d & i_trans, const Vec3d & i_rot):
	width(DEFAULT_WIDTH),
	height(DEFAULT_HEIGHT),
	depth(DEFAULT_DEPTH),
	trans(i_trans),
	rot(i_rot)
{
	set_vertex();
};
Box::Box(const double & w, const double & h, const double & d):
	width(w),
	height(h),
	depth(d)
{
	set_vertex();
};
Box::Box(const double & w, const double & h, const double & d, const Vec3d & i_trans, const Vec3d & i_rot):
	width(w),
	height(h),
	depth(d),
	trans(i_trans),
	rot(i_rot)
{
	set_vertex();
};
Box::Box(const Box & box):
	width(box.width),
	height(box.height),
	depth(box.depth),
	trans(box.trans),
	rot(box.rot),
	vertex(box.vertex)
{
	//vertex.reserve(box.vertex.size());
	//std::copy(box.vertex.begin(), box.vertex.end(), std::back_inserter(vertex));
};

//----------------------------------------------------------
// Setter
//----------------------------------------------------------
void Box::set_width(const double & w) { width = w; };
void Box::set_height(const double & h) { height = h; };
void Box::set_depth(const double & d) { depth = d; }; 
void Box::set_trans(const Vec3d & i_trans) { trans = i_trans; };
void Box::set_rot(const Vec3d & i_rot) { rot = i_rot; };

//----------------------------------------------------------
// Getter
//----------------------------------------------------------
double Box::get_width() const { return width; };
double Box::get_height() const { return height; };
double Box::get_depth() const {return depth; };
Vec3d Box::get_trans() const {return trans; };
Vec3d Box::get_rot() const {return rot; };

// Return Normal vector of the selected vertex
Vec3d Box::get_normal(E_Surface e_suf){
	Vec3d vec_n(0, 0, 0);
	switch(e_suf){
		case E_Top:
			vec_n[2] = 1;
			break;
		case E_Bottom:
			vec_n[2] = -1;
			break;
		case E_Left:
			vec_n[1] = -1;
			break;
		case E_Right:
			vec_n[1] = 1;
			break;
		case E_Front:
			vec_n[0] = 1;
			break;
		case E_Back:
			vec_n[0] = -1;
			break;
		default:
			break;
	}
	return vec_n;
};

// Return vertex of the selected surface
vector<Vec3d> Box::get_points(E_Surface e_suf){
	vector<Vec3d> vec;
	switch(e_suf){
		case E_Top:
			vec.push_back( vertex[0] );
			vec.push_back( vertex[1] );
			vec.push_back( vertex[2] );
			vec.push_back( vertex[3] );
			break;
		case E_Bottom:
			vec.push_back( vertex[4] );
			vec.push_back( vertex[5] );
			vec.push_back( vertex[6] );
			vec.push_back( vertex[7] );
			break;
		case E_Left:
			vec.push_back( vertex[3] );
			vec.push_back( vertex[2] );
			vec.push_back( vertex[6] );
			vec.push_back( vertex[7] );
			break;
		case E_Right:
			vec.push_back( vertex[0] );
			vec.push_back( vertex[4] );
			vec.push_back( vertex[5] );
			vec.push_back( vertex[1] );
			break;
		case E_Front:
			vec.push_back( vertex[0] );
			vec.push_back( vertex[3] );
			vec.push_back( vertex[7] );
			vec.push_back( vertex[4] );
			break;
		case E_Back:
			vec.push_back( vertex[1] );
			vec.push_back( vertex[5] );
			vec.push_back( vertex[6] );
			vec.push_back( vertex[2] );
			break;
		default:
			break;
	}
	return vec;
};


//----------------------------------------------------------
// Private method
// Calcurate corner position and set vertex
//----------------------------------------------------------
void Box::set_vertex(){
	double front = depth / 2.0;
	double back = -depth / 2.0;
	double left = -width / 2.0;
	double right = width / 2.0;
	double top = height; 
	double bottom = 0;

	//Calc corner position
	//box top
	Vec3d top_right_front(front, right, top);
	Vec3d top_right_back(back, right, top);
	Vec3d top_left_back(back, left, top);
	Vec3d top_left_front(front, left, top);
	//box bottom
	Vec3d bottom_right_front(front, right, bottom);
	Vec3d bottom_right_back(back, right, bottom);
	Vec3d bottom_left_back(back, left, bottom);
	Vec3d bottom_left_front(front, left, bottom);

	//Set points
	//box top
	vertex.push_back(top_right_front);
	vertex.push_back(top_right_back);
	vertex.push_back(top_left_back);
	vertex.push_back(top_left_front);
	//box bottom
	vertex.push_back(bottom_right_front);
	vertex.push_back(bottom_right_back);
	vertex.push_back(bottom_left_back);
	vertex.push_back(bottom_left_front);

};