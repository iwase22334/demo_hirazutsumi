#ifndef GL_FUNCTIONS_H
#define GL_FUNCTIONS_H

#include <opencv2/opencv.hpp> 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "linear/linear.hpp"
#include "attractor.h"
#include "ClothSimulator.h"
#include "StringModel.h"
#include "particle.h"
#include "box.h"
using namespace cv;

namespace gf{
	static inline void draw_particle(const Vec3d & pos, const double r){
		glPushMatrix();
		glTranslated(pos[0], pos[1], pos[2]);
		glutSolidSphere(r, 20, 20);
		glPopMatrix();
	};
	static inline void draw_quads(const vector<Vec3d>& p){
		assert(p.size() == 4);
		for (int j = 0; j < 4; ++ j) {
		    GLdouble vertex [] = {(p[j])[0], (p[j])[1], (p[j])[2]};
		    glVertex3dv(vertex);
		}
	};
	static inline void draw_box(Box& b){
		Box::E_Surface surface[] = {
		        Box::E_Top,
		        Box::E_Bottom,
		        Box::E_Left,
		        Box::E_Right,
		        Box::E_Front,
		        Box::E_Back
		};
		glPushMatrix();
		glBegin(GL_QUADS);
		glColor3f(0.3, 0.3, 0.3);
		for (int i = 0; i < Box::SURFACE_NUM; ++ i) {
		    draw_quads(b.get_points(surface[i]));
		}
		glEnd();
		glPopMatrix();
		
	};
	static inline void draw_line(const double w, const cv::Vec3d& begin, const cv::Vec3d& end){
		glLineWidth(w);
        glBegin( GL_LINES );
        glVertex3d( begin[0], begin[1], begin[2]);
        glVertex3d( end[0], end[1], end[2]);
        glEnd();
	};
	static inline void draw_line(const double w, const la::Vec3d& begin, const la::Vec3d& end){
		glLineWidth(w);
        glBegin( GL_LINES );
        glVertex3d( begin[0], begin[1], begin[2]);
        glVertex3d( end[0], end[1], end[2]);
        glEnd();
	};
	static inline void draw_lines(const double w, const list<Vec3d>::iterator& it_begin, const list<Vec3d>::iterator& it_end){
		auto it1 = it_begin;
		auto it2 = it1;
		++ it2;
		while(it2 != it_end){
			draw_line(w, *it1, *it2);
			++ it1;
			++ it2;
		}
	};	
	static inline void draw_lines(const double w, const vector<Vec3d>::iterator& it_begin, const vector<Vec3d>::iterator& it_end){
		auto it1 = it_begin;
		auto it2 = it1;
		++ it2;
		while(it2 != it_end){
			draw_line(w, *it1, *it2);
			++ it1;
			++ it2;
		}
	};
	static inline void draw_arrow(const Vec3d& begin, const Vec3d& end){
		GLUquadricObj *arrows[2];
		double len, ang;
		Vec3d dir = end - begin;
		len = norm(dir);
		if(len != 0.0){
			ang = acos(dir[2]*len/(norm(dir)*len))/M_PI*180.0;

			glPushMatrix();
				glTranslated( begin[0], begin[1], begin[2]);
				glRotated( ang, -dir[1]*len, dir[0]*len, 0.0);
				arrows[0] = gluNewQuadric();
				gluQuadricDrawStyle(arrows[0], GLU_FILL);
				gluCylinder(arrows[0], len/40, len/40, len*0.9, 20, 20);
				glPushMatrix();
					glTranslated( 0.0, 0.0, len*0.9);
					arrows[1] = gluNewQuadric();
					gluQuadricDrawStyle(arrows[1], GLU_FILL);
					gluCylinder(arrows[1], len/30, 0.0f, len/10, 20, 20);
				glPopMatrix();
			glPopMatrix();
		}
	};
	static inline void draw_bio(vector<Vec3d>& circle){
        const double step_size = 0.01;
        const double step_num = 15;
        //Vec3d pos(-step_size*step_num/2, -step_size*step_num/2, -step_size*step_num/2);
        Vec3d pos(-step_size*step_num/2, 0, 0);
        
        for(int i = 0; i < step_num; ++ i){
            //for(int j = 0; j < step_num; ++ j){
                for(int k = 0; k < step_num; ++ k){
                    Vec3d dir = BiotSavartLaw<Vec3d>()(pos, circle);
                    dir = dir/ norm(dir) / 30;
                    gf::draw_arrow(pos, pos + dir);
                    pos += Vec3d(step_size, 0, 0);   
                }
                //pos[0] = -step_size*step_num/2;
              //  pos += Vec3d(0, step_size, 0);
            //}
            //pos[1] = -step_size*step_num/2;
            pos[0] = -step_size*step_num/2;
            pos += Vec3d(0, 0, step_size);
        }
	};		
	static inline void draw_coordination(const tf::Vector3& tf_pos, const tf::Matrix3x3 rot){

        tf::Vector3 xv(0.15, 0, 0);
        tf::Vector3 yv(0, 0.15, 0);
        tf::Vector3 zv(0, 0, 0.2);
        
        xv = rot * xv + tf_pos;
        yv = rot * yv + tf_pos;
        zv = rot * zv + tf_pos;

        Vec3d cv_x(xv[0], xv[1], xv[2]);
        Vec3d cv_y(yv[0], yv[1], yv[2]);
        Vec3d cv_z(zv[0], zv[1], zv[2]);

        Vec3d pos(tf_pos[0], tf_pos[1], tf_pos[2]);

        draw_arrow(pos, cv_x);
        draw_arrow(pos, cv_y);
        draw_arrow(pos, cv_z);

	};
	static inline void draw_coordination(const Vec3d& pos, const Vec3d& rot){

        la::Vec3d xa(pos[0], pos[1], pos[2]);
        la::Vec3d ya(pos[0], pos[1], pos[2]);
        la::Vec3d za(pos[0], pos[1], pos[2]);

        la::Vec3d la_pos(pos[0], pos[1], pos[2]);

        la::Vec3d xv(0.07, 0, 0);
        la::Vec3d yv(0, 0.07, 0);
        la::Vec3d zv(0, 0, 0.07);
        
        la::RMatd rmat(rot[0], rot[1], rot[2]);

        xa = rmat * xv + xa;
        ya = rmat * yv + ya;
        za = rmat * zv + za;

        Vec3d cv_x(xa[0], xa[1], xa[2]);
        Vec3d cv_y(ya[0], ya[1], ya[2]);
        Vec3d cv_z(za[0], za[1], za[2]);
        
        draw_arrow(pos, cv_x);
        draw_arrow(pos, cv_y);
        draw_arrow(pos, cv_z);

	};
	static inline void draw_hand(const Vec3d& pos, const Vec3d& rot){

        la::Vec3d xa(pos[0], pos[1], pos[2]);
        la::Vec3d ya(pos[0], pos[1], pos[2]);
        la::Vec3d za(pos[0], pos[1], pos[2]);

        la::Vec3d la_pos(pos[0], pos[1], pos[2]);

        la::Vec3d xv(0.04, 0, 0);
        la::Vec3d yv(0, 0.04, 0);
        la::Vec3d zv(0, 0, 0.08);
        
        la::RMatd rmat(rot[0], rot[1], rot[2]);

        xa = rmat * xv + xa;
        ya = rmat * yv + ya;
        za = rmat * zv + za;

        draw_line(1, xa, la_pos);
        draw_line(1, ya, la_pos);
        Vec3d cv_z(za[0], za[1], za[2]);
        draw_arrow(pos, cv_z);

	};
	static inline void draw_closest_set(ClothSimulator& s){
        Particle * p1 = NULL;
        Particle * p2 = NULL;

        StringModel & s1 = (s.get_str_model_list()).front();
        StringModel & s2 = (s.get_str_model_list()).back();

        s.get_closest_particle(&p1, &p2, s1, s2);

        Vec3d pos1 = p1->get_x();
        Vec3d pos2 = p2->get_x();

        const double SIZE(0.023);
        gf::draw_particle(pos1, SIZE);
        gf::draw_particle(pos2, SIZE);
	};
	static inline void draw_string_model(StringModel & s){
		list<Particle>::iterator it_par = s.get_string_begin();
		while(it_par != s.get_string_end()){
		    gf::draw_particle((*it_par).get_x(), (*it_par).get_rad());
		    ++ it_par;
		}
		/*
		glLineWidth(20);
   		list<StringModel::Line>::iterator it_line = (s).get_line_begin();
   		while(it_line != (*it_str).get_line_end()){
   		    glBegin( GL_LINES );
   		    Vec3d begin = (*it_line).get_begin_point();
   		    Vec3d end =  (*it_line).get_end_point();
   		    glVertex3d( begin[0], begin[1], begin[2]);
   		    glVertex3d( end[0], end[1], end[2]);
   		    glEnd();
   		    ++ it_line;
   		}    
       
        */
	};

	// Draw Attractor
	/*{
	    if(robo_handler.get_left_command().get_type() == Command::E_Move){

	        Vec3d pos = robo_handler.get_left_command().get_attractor().get_x();
	        Vec3d dir = robo_handler.get_left_command().get_attractor().get_dir();
	        switch(robo_handler.get_left_command().get_attractor().get_type()){
	            case Attractor::E_Point:
	                glPushMatrix();
	                glTranslated(pos[0], pos[1], pos[2]);
	                glutSolidSphere( 0.02, 8, 8);
	                glPopMatrix();
	                break;

	            case Attractor::E_Line:
	                glPushMatrix();
	                glBegin( GL_LINES );
	                glVertex3d( pos[0], pos[1], pos[2]);
	                glVertex3d( pos[0] + dir, pos[1] + dir, pos[2] + dir);
	                glEnd();
	                glPopMatrix();
	                break;

	            case Attractor::E_Plane:
	                glPushMatrix();
	                glBegin(GL_QUADS);
	                glColor3f(0.0, 0.0, 1.0);
	                for (int j = 0; j < 4; ++ j) {
	                    
	                    GLdouble vertex [] = {(points[j])[0], (points[j])[1], (points[j])[2]};
	                    glVertex3dv(vertex);
	                }
	                glEnd();
	                glPopMatrix();
	                break;
	            default:
	                break;
	        }
	    }
	}*/
};

 #endif