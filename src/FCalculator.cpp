#include "FCalculator.h"
//----------------------------------------------------------
// Constant parameter
//----------------------------------------------------------
const double FCalculator::G(9.8);
const double FCalculator::AIR_D(0.004);
const double FCalculator::MINIMUM_DIST(0.00001);

//----------------------------------------------------------
// gaussian kernel
//----------------------------------------------------------
double FCalculator::gaussian_kernel(double h, Vec3d i_v){
	double gau = exp(- (norm(i_v) * norm(i_v)) / h);
	return gau;
};

//----------------------------------------------------------
// Common factor
//----------------------------------------------------------
void FCalculator::add_g(Particle& par){
	Vec3d g_vec(0, 0, -1);
	par.add_f((G * par.get_m()) * g_vec);
};

void FCalculator::add_air_resistance(Particle& par){
	Vec3d res_vec = par.get_v();
	par.add_f(AIR_D*(-res_vec));
};

//----------------------------------------------------------
// Connection forces
//----------------------------------------------------------
void FCalculator::add_spring_force(const double len, const double k, Particle& par1, Particle& par2){
	double d = norm(par2.get_x() - par1.get_x());
	if(d == 0){
		std::cout << "Particles are too closed" << std::endl; 
		d = 0.0000001;
		// exit(EXIT_FAILURE);
	}
	if(d > len){
		const Vec3d n = (par2.get_x() - par1.get_x()) / d;
		Vec3d forc = (d - len) * k * n;
		par1.add_f(forc);
		par2.add_f(-forc);
	}
};

void FCalculator::add_damping_force(const double d, Particle& par1, Particle& par2){
	const double d_v = norm(par2.get_v() - par1.get_v());
	const Vec3d sub = (par2.get_v() - par1.get_v());
	Vec3d forc = sub * d;
	par1.add_f(forc);
	par2.add_f(-forc);
};

//----------------------------------------------------------
//  Particle reaction force
//----------------------------------------------------------
void FCalculator::add_particle_force(const double k, const double d, Particle & par1, Particle & par2){
	const Vec3d sub = (par1.get_x() - par2.get_x());
	const double dist = norm(sub);
	const double contact_rate(0.8);
	if(dist < (par1.get_rad() + par2.get_rad()) * contact_rate){
		const Vec3d k_forc = sub * k;
		const Vec3d d_forc = (-par1.get_v() + par2.get_v()) * d;
		par1.add_f(k_forc);
		par1.add_f(d_forc);
		par2.add_f(-k_forc);
		par2.add_f(-d_forc);
	}
};

//----------------------------------------------------------
//  Box reaction force
//----------------------------------------------------------
void FCalculator::add_box_force(const double k, const double d, Box & box, Particle & par){
	// Surface array
    Box::E_Surface surface[] = {
        Box::E_Top,
        Box::E_Bottom,
        Box::E_Left,
        Box::E_Right,
        Box::E_Front,
        Box::E_Back
    };

    // Add polygon force
    for (int i = 0; i < Box::SURFACE_NUM; ++ i) {
        vector<Vec3d> points = box.get_points(surface[i]);
        Vec3d normal = box.get_normal(surface[i]);
        FCalculator::add_polygon_force(k, d, 0.7, 0.6, normal, points, par);
    }
};


//----------------------------------------------------------
//  Poligon reaction force
//----------------------------------------------------------
void FCalculator::add_polygon_force(const double k, const double d, const double mu1 , const double mu2, const Vec3d & n, const Pol4 & lisPoints, Particle & par){
	Pol4 polygon(lisPoints);
	Vec3d cl_vec = polygon.get_closest_point(par.get_x());
	const Vec3d sub = (par.get_x() - cl_vec);
	const double dist = norm(sub);
	
	if(dist < par.get_rad()){
		
		// Friction force
		//----------------------------------------------------------
		//  Temporary substitute
		//----------------------------------------------------------
		if(par.get_f().dot(n) < 0){
			if(norm(par.get_v()) < 0.0001){
				const Vec3d fric_forc = ((-par.get_f()) - n * n.dot((-par.get_f())) / norm(n));
				// Stopping
				if( norm(fric_forc) < norm(n.dot(-par.get_f())) * mu1){
					Vec3d zero(0, 0, 0);
					par.set_v(zero);
					par.add_f(fric_forc);
				}
				// Start moving
				else{
					par.add_f((fric_forc / norm(fric_forc)) * mu1 * norm(n.dot(-par.get_f())));
				}
			}
			else{
				const double vel = norm(par.get_v());
				const Vec3d unit_vel = par.get_v() / vel;
				const Vec3d fric_forc = -unit_vel * mu2 * fabs(n.dot(-par.get_f()));
				Particle predic_par(par);
				predic_par.add_f(fric_forc);
				predic_par.move_euler();
				if( predic_par.get_v().dot(par.get_v()) < 0.0){
					Vec3d zero;
					par.add_f((-par.get_v() / par.get_dt()) * par.get_m());
				}
				else{
					par.add_f(fric_forc);
				}
			}
		}
		//----------------------------------------------------------
		
		// Reaction force
		//const Vec3d k_forc = sub * k;
		const Vec3d k_forc = dist * k * sub / norm(sub);
		//const Vec3d k_forc = (0.001 / (dist)) / norm(sub) * sub;
		const Vec3d d_forc = (-par.get_v().dot(n) * n) * d;
		par.add_f(k_forc);
		par.add_f(d_forc);

	}


};