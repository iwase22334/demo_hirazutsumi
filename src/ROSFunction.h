#ifndef ROS_FUNCTIONS_H
#define ROS_FUNCTIONS_H
#include <iostream>
# include <random>
#include <array>

#include "linear/linear.hpp"
#include "ROSJointController.hpp"
#include "ROSIKClient.hpp"
#include "hand.h"

#include <opencv2/opencv.hpp> 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <baxter_core_msgs/SolvePositionIK.h>
#include <lis_msgs/Joint_angles.h>
#include <lis_msgs/End_poses.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>

extern const double BOX_X;
extern const double BOX_Y;
extern const double BOX_Z;

extern ROSIKClient ik_client;

namespace rf{
	template<typename I, typename O>
	static inline void assign_n(const I& in, const unsigned int n, O& out){
		for(int i = 0; i < n; ++ i){
			out[i] = in[i];
		}
	};
	template<typename I, typename O>
	static inline O assign_n(const I& in, const unsigned int n){
		O out;
		for(int i = 0; i < n; ++ i){
			out[i] = in[i];
		}
		return out;
	};
	struct Transform{
		inline cv::Vec3d operator()(const cv::Vec3d& pos, const cv::Vec3d& rot)
		{
			// Effector 
			const la::Vec3d effector_pos(0, 0, -0.12);
			// Current rotation matrix
			const la::RMatd rmat0(rot[0], rot[1], rot[2]);
			const la::Vec3d shift = rmat0 * effector_pos;
			cv::Vec3d res;
			res[0] = pos[0] + BOX_X + shift[0];
			res[1] = pos[1] + BOX_Y + shift[1];
			res[2] = pos[2] + BOX_Z + shift[2];
			return res;
		}
	};
 	template<typename T>
 	static inline 
 	T to_quaternion(const cv::Vec3d rot){
 		la::RMatd rmat(rot[0], rot[1], rot[2]);

 		tf::Matrix3x3 tf_rmat(rmat(0), rmat(1), rmat(2), rmat(3), rmat(4), rmat(5), rmat(6), rmat(7), rmat(8));

 		tf::Quaternion q;
 		tf_rmat.getRotation(q);
 		
 		T res;
 		res[0] = q.x();
 		res[1] = q.y();
 		res[2] = q.z();
 		res[3] = q.w();
 		return res;

 	}
 	static inline
 	cv::Vec3d to_zxy(const tf::Quaternion q){
 		tf::Vector3 v = q.getAxis();
 		double a = q.getAngle();
 		la::RMatd rmat(a, la::Vec3d(v[0], v[1], v[2]));
 		return cv::Vec3d(alpha(rmat), beta(rmat), gamma(rmat));
 	}
 	static inline
 	Joints random_seed(const Joints j){
 		std::random_device rd;
 		std::mt19937 mt(rd());
 		std::uniform_real_distribution<double> urd(0.0,1.0);
 		la::Vector<double, 7> gap;
 		for(int i = 0; i < 7; ++i) gap[i] = urd(mt);
 		gap = gap / norm(gap);

 		la::Vector<double, 7> curr_j = assign_n<Joints, la::Vector<double, 7> >(j, 7);
 		la::Vector<double, 7> res_j = curr_j + gap;
 		return assign_n<la::Vector<double, 7>, Joints>(res_j, 7);
 	};

	static inline
	bool search_angle(const Hand::E_Side side, const cv::Vec3d& x, const cv::Vec3d& curr_rot, const cv::Vec3d& target, cv::Vec3d& r_rot, Joints& joints){
		const int SEED_AUTO(0);
		const int SEED_USER(1);
		const int SEED_CURRENT(2);
		const int SEED_NS_MAP(3);
		Joints curr_joints(joints);

		bool solved = false;

		// Current rotation matrix
		const la::RMatd rmat0(curr_rot[0], curr_rot[1], curr_rot[2]);

		// pointing direction of Referenced hand
		const la::Vec3d pointer(0, 0, 1);
		const la::Vec3d posture = rmat0 * pointer;
		
		// Target pointer direction
		const cv::Vec3d target_posture = target;
		la::Vec3d la_target_posture(target_posture[0], target_posture[1], target_posture[2]);
		la_target_posture = la_target_posture / norm(la_target_posture);
		
		// Rotation axis
		const la::Vec3d axis = posture.cross(la_target_posture);
		const la::Vec3d unit_axis = axis / la::norm(axis);
		
		// Angle in Rotation
		double angle = la::angle(posture, la_target_posture);
		double latest_angle = angle;

		// Division of the angle for BinarySearch 
		double div_angle = angle;
		
		cv::Vec3d latest_rot = cv::Vec3d(curr_rot[0], curr_rot[1], curr_rot[2]);
		
		Joints latest_joints;
		copy_n(joints.begin(), ROSJointController::JOINTS_NUM, latest_joints.begin());

		// Solve IK 
		if(side == Hand::E_Right){
			ROSIKClient::RightRequest req(SEED_USER, curr_joints, assign_n<cv::Vec3d, array<double, 3> >( Transform()(x, curr_rot), 3 ), to_quaternion<array<double, 4> >(curr_rot) );
			if(ik_client.call( req )){
				solved = true;
				copy_n(ik_client.get_right_joints().begin() , ROSJointController::JOINTS_NUM, latest_joints.begin());
			}
		}
		else{
			ROSIKClient::LeftRequest req(SEED_USER, curr_joints, assign_n<cv::Vec3d, array<double, 3> >( Transform()(x, curr_rot), 3 ), to_quaternion<array<double, 4> >(curr_rot) );
			if(ik_client.call( req )){
				solved = true;
				copy_n(ik_client.get_left_joints().begin() , ROSJointController::JOINTS_NUM, latest_joints.begin());
			}
		}

		const double ROUNDING_THRESH(0.03);
		const int SPLIT_NUM(100);
		// Binary search for solvable posture
		for(int i = 0; i < SPLIT_NUM ; ++ i){
			if(angle < ROUNDING_THRESH){
				// Rotation is not needed
				std::cout << "rf::search_angle - rotation is not needed" << std::endl;
				// Return current rotation
				break;	
			}			
			div_angle /= 2.0;
			if(div_angle < ROUNDING_THRESH){
				std::cout << "rf::search_angle - stop division is toosmall : " << div_angle << std::endl;
				break;	
			}
			
			la::RMatd next_rmat(angle, unit_axis);
			la::Mat33d mat = next_rmat * rmat0;
			la::RMatd rmat(mat);
			cv::Vec3d rot(alpha(rmat), beta(rmat), gamma(rmat));

			// Solve IK
			bool vaild(false);
			if(side == Hand::E_Left){
				ROSIKClient::LeftRequest req(SEED_USER, curr_joints, assign_n<cv::Vec3d, array<double, 3> >( Transform()(x, rot), 3 ), to_quaternion<array<double, 4> >(rot) );
				if(ik_client.call( req )){
					vaild = true;
					copy_n(ik_client.get_left_joints().begin() , ROSJointController::JOINTS_NUM, latest_joints.begin());
				}
			}
			else{
				ROSIKClient::RightRequest req(SEED_USER, curr_joints, assign_n<cv::Vec3d, array<double, 3> >( Transform()(x, rot), 3 ), to_quaternion<array<double, 4> >(rot) );
				if(ik_client.call( req )){
					vaild = true;
					copy_n(ik_client.get_right_joints().begin() , ROSJointController::JOINTS_NUM, latest_joints.begin());
				}
			}

			// If ik return valid value
			if(vaild){
				solved = true;
				latest_rot = rot;
				latest_angle = angle;
				angle += div_angle;
				if(i == 0){
					// Ik solvable. Search is not needed.
					std::cout << "rf::search_angle - Ik solvable Search is not needed. : " << side << std::endl;
					break;	
				}
				
			}
			// If ik return invalid value
			else{
				if(i == 0)
					std::cout << "rf::search_angle - Search start. : " << std::endl;
				std::cout << "rf::search_angle - Can't solve *** "  << angle << std::endl;
				angle -= div_angle;	
			}

		}	
		// If No answer, search ik solution from random seed
		/*while(!solved){
			// Solve IK 
			if(side == Hand::E_Right){
				//ROSIKClient::RightRequest req(SEED_USER, random_seed(curr_joints), assign_n<cv::Vec3d, array<double, 3> >( Transform()(x, curr_rot), 3 ), to_quaternion<array<double, 4> >(curr_rot) );
				ROSIKClient::RightRequest req(SEED_AUTO, assign_n<cv::Vec3d, array<double, 3> >( Transform()(x, curr_rot), 3 ), to_quaternion<array<double, 4> >(curr_rot) );
				if(ik_client.call( req )){
					solved = true;
					copy_n(ik_client.get_right_joints().begin() , ROSJointController::JOINTS_NUM, latest_joints.begin());
				}
			}
			else{
				//ROSIKClient::LeftRequest req(SEED_USER, random_seed(curr_joints), assign_n<cv::Vec3d, array<double, 3> >( Transform()(x, curr_rot), 3 ), to_quaternion<array<double, 4> >(curr_rot) );
				ROSIKClient::LeftRequest req(SEED_AUTO, assign_n<cv::Vec3d, array<double, 3> >( Transform()(x, curr_rot), 3 ), to_quaternion<array<double, 4> >(curr_rot) );
				if(ik_client.call( req )){
					solved = true;
					copy_n(ik_client.get_left_joints().begin() , ROSJointController::JOINTS_NUM, latest_joints.begin());
				}
			}
 			std::cout << "rf::search_angle - random seed mode " << std::endl;
		}*/
		copy_n(latest_joints.begin(), ROSJointController::JOINTS_NUM, joints.begin());
		r_rot = latest_rot;

		return solved;
	}
	static inline
	bool call_ik(const Hand::E_Side side, const cv::Vec3d& x, const cv::Vec3d& rot, Joints& joints){
		const int SEED_AUTO(0);
		const int SEED_USER(1);
		const int SEED_CURRENT(2);
		const int SEED_NS_MAP(3);
		const Joints curr_joints(joints);

		bool vaild(false);
		if(side == Hand::E_Left){
			ROSIKClient::LeftRequest req(SEED_USER, curr_joints, assign_n<cv::Vec3d, array<double, 3> >( Transform()(x, rot), 3 ), to_quaternion<array<double, 4> >(rot) );
			if(ik_client.call( req )){
				vaild = true;
				copy_n(ik_client.get_left_joints().begin() , ROSJointController::JOINTS_NUM, joints.begin());
			}
		}
		else{
			ROSIKClient::RightRequest req(SEED_USER, curr_joints, assign_n<cv::Vec3d, array<double, 3> >( Transform()(x, rot), 3 ), to_quaternion<array<double, 4> >(rot) );
			if(ik_client.call( req )){
				vaild = true;
				copy_n(ik_client.get_right_joints().begin() , ROSJointController::JOINTS_NUM, joints.begin());
			}
		}
		
		return vaild;
	}

};

 #endif