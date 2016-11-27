#include "polygon.h"

//----------------------------------------------------------
// Line Constructor
//----------------------------------------------------------
template<typename T>
Line<T>::Line():
	begin(),
	end()
{};
template<typename T>
Line<T>::Line(const T& a, const T& b):
	begin(a),
	end(b)
{};

//----------------------------------------------------------
// Line Other functions
//----------------------------------------------------------

template<typename T> T& Line<T>::operator[](const int i){
	assert(i < 2);
	return i == 0 ? begin : end;
};
template<typename T> const T& Line<T>::operator[](const int i) const{
	assert(i < 2);	
	return i == 0 ? begin : end;
};

template<typename T> void Line<T>::set_begin(const T& a){ begin = a; };
template<typename T> void Line<T>::set_end(const T& a){ end = a; };
// Getter
template<typename T> T Line<T>::get_begin() const { return begin; };
template<typename T> T Line<T>::get_end() const { return end; };
template<typename T> double Line<T>::length() const { return norm(end - begin); };
template<typename T> T Line<T>::dir() const { return (end - begin); };
template<typename T> T Line<T>::unit_dir() const { return (end - begin)/norm(end - begin); };

//----------------------------------------------------------
// Constructor
//----------------------------------------------------------
template<int N>
Polygon<N>::Polygon() :
	vertex(N),
	normal(Vec3d(1,0,0))
{}; 

template<int N>
Polygon<N>::Polygon(const vector<Vec3d> & v) : 
	vertex(v)
{
	// Polygon must have over 2 vertexes.
	assert(N > 2);	
	Vec3d begin = vertex[N-1] - vertex[0];
	Vec3d end = vertex[1] - vertex[0];
	normal = begin.cross(end);
	normal /= norm(normal);
};

template<int N>
Polygon<N>::Polygon(const Polygon<N> & p) : 
	vertex(p.vertex), 
	normal(p.normal)
{};

template<int N>
Polygon<N> Polygon<N>::operator=(const Polygon<N>& a){
	vertex.resize(a.vertex.size());
	copy(a.vertex.begin(), a.vertex.end(), vertex.begin());
	normal = a.normal;
};
//----------------------------------------------------------
// Getter
//----------------------------------------------------------
template<int N>
Vec3d Polygon<N>::get_normal() const { return normal; };

template<int N>
vector<Vec3d>::iterator Polygon<N>::get_points() { return vertex.begin(); };

//----------------------------------------------------------
// Is x inside of polygon
//----------------------------------------------------------
template<int N>
bool Polygon<N>::is_inner(const Vec3d & x){
	// Map x on the plane of the polygon
	Vec3d shade_vec = (x - vertex.front()).dot(normal) * normal;
	Vec3d map_x = x - shade_vec;
	// Check mapped x
	Vec3d begin = map_x - vertex[0];
	Vec3d end = map_x - vertex[1];
	Vec3d normal_ref = begin.cross(end);
	for(int i = 1; i < N + 1; ++ i){
		begin = map_x - vertex[i % N];
		end = map_x - vertex[(i + 1) % N];
		if((begin.cross(end)).dot(normal_ref) < 0){
			return false;
		}
	}
	return true;
};

//----------------------------------------------------------
// Get closest point from vecticle in Polygon 
//----------------------------------------------------------
template<int N>
Vec3d Polygon<N>::get_closest_point(const Vec3d & x){
	Vec3d closest_point;
	Vec3d shade_vec = (x - vertex.front()).dot(normal) * normal;
	Vec3d map_x = x - shade_vec;

	// Is shade of vecticle inside of polygon
	if(is_inner(x)){
		closest_point = map_x;
	}

	// If shade is outside of polygon
	else{
		int closest_index1(0);
		int closest_index2(0);
		Vec3d closest_vec1(vertex[closest_index1]);
		Vec3d closest_vec2(vertex[closest_index2]);
		double min_dist1(norm(closest_vec1 - map_x));
		double min_dist2(norm(closest_vec2 - map_x));

		// find closest point
		for(int i = 1; i < N; ++ i){
			const Vec3d sub = vertex[i] - map_x;
			const double dist = norm(sub);
			if(min_dist1 > dist){
				min_dist1 = dist;
				closest_vec1 = vertex[i];
				closest_index1 = i;
			}
		}

		// find second closest point
		const int prev_index(((closest_index1 + N - 1) % N));
		const int next_index(((closest_index1 + N + 1) % N));
		const Vec3d prev = vertex[prev_index];
		const Vec3d next = vertex[next_index];
		if(norm(prev - map_x) < norm(next - map_x)){
			closest_index2 = prev_index;
			closest_vec2 = prev;
		}
		else{
			closest_index2 = next_index;
			closest_vec2 = next;
		}

		// find closest point between closest_vec2 and closest_vec1
		const Vec3d vec1 = (closest_vec2 - closest_vec1) ;
		const Vec3d vec1_uni = vec1 / norm(vec1);
		const Vec3d vec2 = map_x - closest_vec1;
		const double inner = vec1_uni.dot(vec2);
		const Vec3d dist = inner * vec1_uni;
		// Closest point is nearest vertex in polygon
		if(inner < 0){
			closest_point = closest_vec1;
		}
		// Closest point is between 2 vertexes
		else if(norm(dist) < norm(vec1)){
			closest_point = closest_vec1 + dist;
		}
		// Closest point is 2nd nearest vertex in polygon
		else{
			closest_point = closest_vec2;
		}
	}
	return closest_point;

};


//----------------------------------------------------------
// ClosedPath constructors
//----------------------------------------------------------
template<typename T> 
struct BiotSavartLaw{
	T operator()(const T& p, const vector<T>& closed_path){
		cout << "BiotSavartLaw is defined only Vec3d" << endl;
	};
	T operator()(const T& p, const Line<T>& l){
		cout << "BiotSavartLaw is defined only Vec3d" << endl;
	}
};
template<> 
struct BiotSavartLaw<Vec3d>{
	Vec3d operator()(const Vec3d& p, const vector<Vec3d>& closed_path){
		assert(closed_path.size() > 2);
		// Split long line into some shorter lines
		vector<Vec3d> hole;
		{
			const double min_length(0.001);
			auto it1 = closed_path.begin();
			auto it2 = ++ closed_path.begin();
			hole.push_back(*it1);
			while(it2 != closed_path.end()){
				if(norm(*it2 - *it1) > min_length){
					while(norm(*it2 - hole.back()) > min_length){
						hole.push_back(hole.back() + (*it2 - hole.back()) / norm((*it2 - hole.back())) * min_length);
					}
				}
				hole.push_back(*it2);
				++ it1;
				++ it2;
			}
			if(norm(hole.front() - hole.back()) > min_length){
				while(norm(hole.front() - hole.back()) > min_length){
					hole.push_back(hole.back() + (hole.front() - hole.back()) / norm((hole.front() - hole.back())) * min_length);
				}
			}
		}

		// Integrate
		Vec3d v;
		{
			auto it1 = hole.begin();
			auto it2 = ++ hole.begin();
			while(it2 != hole.end()){
				Line<Vec3d> l(*it1, *it2);
				v += BiotSavartLaw<Vec3d>::operator()(p, l);
				++ it1;
				++ it2;
			}
			Line<Vec3d> l(hole.back(), hole.front());
			v += BiotSavartLaw<Vec3d>::operator()(p, l);
			
		}
		return v;
	};
	Vec3d operator()(const Vec3d& p, const Line<Vec3d>& l){
		const Vec3d r = p - l.get_begin();
		const double r_d = norm(r); 
		const Vec3d dl = l.dir();
		const Vec3d dh = dl.cross(r) / (4 * M_PI * r_d * r_d * r_d);
		return dh;
	};
};