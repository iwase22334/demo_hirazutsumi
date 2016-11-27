#ifndef BOX_H
#define BOX_H
//----------------------------------------------------------
// include
//----------------------------------------------------------
#include <opencv2/core/core.hpp>
#include <vector>
//----------------------------------------------------------
// namespace
//----------------------------------------------------------
using namespace cv;
using namespace std;
//----------------------------------------------------------
// class Box
//----------------------------------------------------------
class Box{
public:
	static const int SURFACE_NUM;
	enum E_Surface { 
			E_Top = 0,
			E_Bottom = 1,
			E_Left = 2,
			E_Right = 3,
			E_Front = 4,
			E_Back = 5
	};
		
private:
	static const double DEFAULT_WIDTH;
	static const double DEFAULT_HEIGHT;
	static const double DEFAULT_DEPTH;

private:
	// Box size 
	double width;
	double height;
	double depth;

	// [x, y, z]
	Vec3d trans;
	// [alpha, beta, gamma]
	Vec3d rot;
	// Vertex
	vector<Vec3d> vertex;

public:
	//constructor
	Box();
	// Box( trans )
	Box(const Vec3d &);
	// Box( trans , rot )
	Box(const Vec3d &, const Vec3d &);
	// Box( width , height , depth)
	Box(const double &, const double &, const double &);
	// Box( width , height , depth , trans , rot )
	Box(const double &, const double &, const double &, const Vec3d &, const Vec3d &);
	// Copy constructor
	Box(const Box &);
	
	//setter
	void set_width(const double &);
	void set_height(const double &);
	void set_depth(const double &);
	void set_trans(const Vec3d &);
	void set_rot(const Vec3d &);

	//getter
	double get_width() const;
	double get_height() const;
	double get_depth() const;
	Vec3d get_trans() const;
	Vec3d get_rot() const;

	// Get normal of selected surface
	Vec3d get_normal(E_Surface);
	// Get 4 points of selected surface
	vector<Vec3d> get_points(E_Surface);

private:
	// Calculate vertexes from (width, height, depth) and set vertexes
	void set_vertex();
};

#endif