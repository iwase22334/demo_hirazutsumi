//----------------------------------------------------------
// definition
//----------------------------------------------------------
//#define NO_ROS
#define CALIB_MODE
//----------------------------------------------------------
// include
//----------------------------------------------------------
#include <iostream>
#include <cstdio>
#include <cmath>
#include <ctime>
#include <utility>
#include <list>
#include <vector>
#include <sstream>
#include <pthread.h>

#include <opencv2/opencv.hpp> 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <GL/glut.h>
#include <GL/freeglut.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <lis_msgs/End_poses.h>
#include <lis_msgs/End_PosesArray.h>
#include <std_msgs/String.h>
#include <baxter_core_msgs/JointCommand.h>

#include "ROSHookController.hpp"
#include "ROSJointController.hpp"
#include "ROSIKClient.hpp"
#include "ROSFunction.h"
#include "GLFunction.h"
#include "linear/linear.hpp"
#include "RobotHandler.h"
#include "TrajectoryKeeper.h"
#include "TrajectoryGenerator.h"
#include "attractor.h"
#include "ClothSimulator.h"
#include "timer.h"
#include "StringModel.h"
#include "particle.h"
#include "box.h"
#include "FCalculator.h"
#include "CommandPool.h"
//----------------------------------------------------------
// namespace
//----------------------------------------------------------
using namespace std;
using namespace cv;

//----------------------------------------------------------
// Constant parameter
//----------------------------------------------------------
//constant paramator
const int LOOP_NUM(100000);
const int WINDOWSIZE_X(800);
const int WINDOWSIZE_Y(600);
enum E_ProgramState{
    E_Stopping,
    E_Moving,
    E_Ending
};
//----------------------------------------------------------
// Declaration of structures
//----------------------------------------------------------
struct Camera{
    int width;
    int height;
    Vec6d state;
};

enum E_Arm{
    E_Right = 0,
    E_Left = 1,
    E_Both = 2
};

enum E_Process{
    E_Proc_1_0 = 0,
    E_Proc_1_1 = 1,
    E_Proc_1_2 = 2,
    E_Proc_1_3 = 3, 
    E_Proc_1_4 = 4,
    E_Proc_0 = 5
};

//----------------------------------------------------------
// Declaration of function
//----------------------------------------------------------
void Display(void);
void Idle(void);
void InitOpenGL(void);
void Resize(int, int);
void PolorView(void);
void NormalKeyIn(unsigned char, int, int);
void SpecialKeyIn(int, int, int);
void Mouse(int, int, int, int);

void init(void);

void add_command(const E_Process);

//----------------------------------------------------------
// Declaration of Global variables
//----------------------------------------------------------
pair<Particle*, Particle*> intersectoin;

//----------------------------------------------------------
// simulation state
// *** Mutex is required ***
// since This Object is referenced from multiple threads. 
E_ProgramState simulation_state(E_Stopping);
pthread_mutex_t mutex_simulation_state;
//----------------------------------------------------------

//----------------------------------------------------------
// Robot state planner
// *** Mutex is required ***
// since This Object is referenced from multiple threads. 
RobotHandler robo_handler;
ClothSimulator simulator;
pthread_mutex_t mutex_simulator;
//----------------------------------------------------------


//----------------------------------------------------------
// current process number
// *** Mutex is required ***
// since This Object is referenced from multiple threads.
E_Process curr_proc = E_Proc_1_1;
pthread_mutex_t mutex_curr_proc;
//----------------------------------------------------------


//----------------------------------------------------------
// state initialized flag
// *** Mutex is required ***
// since This Object is referenced from multiple threads.
bool recv_flag = false;
pthread_mutex_t mutex_recv_flag;
//----------------------------------------------------------

//----------------------------------------------------------
// calculate next step forcibly
// *** Mutex is required ***
// since This Object is referenced from multiple threads.
bool calc_forcibly_flag = false;
pthread_mutex_t mutex_calc_forcibly_flag;
//----------------------------------------------------------

ROSHookController hook_controller;
ROSJointController joint_controller;
ROSIKClient ik_client;

Camera cam;
Vec2d mouse;

// 60Hz timer
Timer timer( 60 ); 

CommandPool l_command_pool;
CommandPool r_command_pool;
TrajectoryGenerator l_trajectory, r_trajectory;

TrajectoryKeeper tk_l(0.001);
TrajectoryKeeper tk_r(0.001);


//const double BOX_X(0.570);
const double BOX_X(0.600);
const double BOX_Y(0.001);
//const double BOX_Z(0.043);
const double BOX_Z(0.040);
const Vec3d L_HOME(0.507 - BOX_X, -0.576 - BOX_Y, 0.277 - BOX_Z);
const Vec3d R_HOME(0.583 - BOX_X, 0.522 - BOX_Y, 0.277 - BOX_Z);

Particle* l_handle;
Particle* r_handle;

pthread_t t_send_id, t_receive_id;
bool quit_flag(false);

//----------------------------------------------------------
// Functions fot Multithreading.
//----------------------------------------------------------

void* receive_msg_thread(void* msg){

    Hand curr_left_hand;
    Hand curr_right_hand;

    Timer timer_recv(120);//10hz timer
#ifndef NO_ROS
    cout << "receive_msg_thread : ros recv mode" << endl;
    tf::TransformListener listener;
    tf::StampedTransform tf_pose_l, tf_pose_r;
#else
    cout << "receive_msg_thread : No ros mode"  << endl; 
#endif

    while(!(*(bool*)msg)){
        timer_recv.start();

#ifndef NO_ROS
        try{

            //ros::Time now = ros::Time::now();
            ros::Time now = ros::Time(0);
                        
            listener.waitForTransform("base", "left_endgripper2", now, ros::Duration(1.0));
            listener.lookupTransform("base", "left_endgripper2", now, tf_pose_l);       //baseから見た位置姿勢をTFから取得する　ros::Time(0)は最新のタイムスタンプ

            listener.waitForTransform("base", "right_endgripper2", now, ros::Duration(1.0));
            listener.lookupTransform("base", "right_endgripper2", now, tf_pose_r);       //baseから見た位置姿勢をTFから取得する　ros::Time(0)は最新のタイムスタンプ
        
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }

        tf::Quaternion ql(tf_pose_l.getRotation().x(), tf_pose_l.getRotation().y(), tf_pose_l.getRotation().z(), tf_pose_l.getRotation().w());
        tf::Quaternion qr(tf_pose_r.getRotation().x(), tf_pose_r.getRotation().y(), tf_pose_r.getRotation().z(), tf_pose_r.getRotation().w());
        cv::Vec3d pl(tf_pose_l.getOrigin().x(), tf_pose_l.getOrigin().y(), tf_pose_l.getOrigin().z());
        cv::Vec3d pr(tf_pose_r.getOrigin().x(), tf_pose_r.getOrigin().y(), tf_pose_r.getOrigin().z());

        Vec3d box_pos(BOX_X,  BOX_Y, BOX_Z);

        Vec3d rot_l = rf::to_zxy(ql);
        Vec3d rot_r = rf::to_zxy(qr);

        curr_right_hand.set_x(pr - box_pos);
        curr_right_hand.set_rot(rot_r);
        curr_left_hand.set_x(pl - box_pos);
        curr_left_hand.set_rot(rot_l);

        pthread_mutex_lock(&mutex_simulator);
        robo_handler.get_curr_left_hand().set_x(curr_left_hand.get_x());
        robo_handler.get_curr_left_hand().set_rot(curr_left_hand.get_rot());
        robo_handler.get_curr_right_hand().set_x(curr_right_hand.get_x());
        robo_handler.get_curr_right_hand().set_rot(curr_right_hand.get_rot());
        pthread_mutex_unlock(&mutex_simulator);
#else

        pthread_mutex_lock(&mutex_simulator);
        robo_handler.get_curr_left_hand().set_x(robo_handler.get_left_hand().get_x());
        robo_handler.get_curr_left_hand().set_rot(robo_handler.get_left_hand().get_rot());
        robo_handler.get_curr_right_hand().set_x(robo_handler.get_right_hand().get_x());
        robo_handler.get_curr_right_hand().set_rot(robo_handler.get_right_hand().get_rot());
        pthread_mutex_unlock(&mutex_simulator);

#endif
        recv_flag = true;
        timer_recv.stop();
        timer_recv.sleep();

    }
    cout << "receive_msg_thread : recv stop" << endl;
    return NULL;
}

void* send_msg_thread(void* msg){


    // 2hz timer
    Timer send_msg_timer(50);

     while(!(*(bool*)msg)){
        send_msg_timer.start();
        //move objects
        pthread_mutex_lock(&mutex_simulation_state);
        E_ProgramState sm(simulation_state);
        pthread_mutex_unlock(&mutex_simulation_state);

        pthread_mutex_lock(&mutex_simulator);
        bool is_move = robo_handler.is_moving() ? (robo_handler.is_left_converged() && robo_handler.is_right_converged()) : true;
        pthread_mutex_unlock(&mutex_simulator); 
        
        is_move = true;
        //cout << robo_handler.is_left_converged() << robo_handler.is_right_converged() << endl;

        pthread_mutex_lock(&mutex_recv_flag);
        is_move = is_move & recv_flag;
        pthread_mutex_unlock(&mutex_recv_flag);

        pthread_mutex_lock(&mutex_calc_forcibly_flag);
        is_move = is_move | calc_forcibly_flag;
        pthread_mutex_unlock(&mutex_calc_forcibly_flag);

        if(sm == E_Moving && is_move){
            // Make copy
            pthread_mutex_lock(&mutex_simulator);
            // force calculation
            simulator.calc_force();
            pthread_mutex_unlock(&mutex_simulator);

            pthread_mutex_lock(&mutex_simulator);
            // Move hand
            robo_handler.move();
            pthread_mutex_unlock(&mutex_simulator); 

            pthread_mutex_lock(&mutex_simulator);
            // Advance the simulation
            simulator.next_step();
            pthread_mutex_unlock(&mutex_simulator); 

            const int command_num = robo_handler.get_command_num();
            if(command_num == 0){

                pthread_mutex_lock(&mutex_curr_proc);
                E_Process cp_curr_proc(curr_proc);
                pthread_mutex_unlock(&mutex_curr_proc);

                switch(cp_curr_proc){
                    case E_Proc_1_0:
                        cp_curr_proc = E_Proc_1_1;
                        cout << "proc 1-0 complete" << endl;
                        break;
                    case E_Proc_1_1:
                        cp_curr_proc = E_Proc_1_2;
                        cout << "proc 1-1 complete" << endl;
                        break;
                    case E_Proc_1_2:
                        cp_curr_proc = E_Proc_1_3;
                        cout << "proc 1-2 complete" << endl;
                        break;
                    case E_Proc_1_3:
                        cp_curr_proc = E_Proc_1_4;
                        cout << "proc 1-3 complete" << endl;
                        break;
                    case E_Proc_1_4:
                        cp_curr_proc = E_Proc_0;
                        cout << "proc 1-4 complete" << endl;
                        break;
                    default:
                        break;
                }

                pthread_mutex_lock(&mutex_curr_proc);
                curr_proc = cp_curr_proc;
                pthread_mutex_unlock(&mutex_curr_proc);

                pthread_mutex_lock(&mutex_simulator);
                add_command(cp_curr_proc);
                pthread_mutex_unlock(&mutex_simulator);
            }


#ifndef NO_ROS

            robo_handler.set_moving(true);

            pthread_mutex_lock(&mutex_simulator);
            joint_controller.set_command(ROSJointController::LeftJCommand(robo_handler.get_left_joints()));
            joint_controller.set_command(ROSJointController::RightJCommand(robo_handler.get_right_joints()));
            pthread_mutex_unlock(&mutex_simulator);
#endif

            pthread_mutex_lock(&mutex_calc_forcibly_flag);
            calc_forcibly_flag = false;
            pthread_mutex_unlock(&mutex_calc_forcibly_flag);

            send_msg_timer.stop();
            send_msg_timer.sleep();

        }

        #ifndef NO_ROS

        pthread_mutex_lock(&mutex_simulator);

        const ROSHookController::E_Left_State grip_l(robo_handler.get_left_hand().get_state() != Hand::E_Handring ? 
            ROSHookController::E_Left_State::E_Open :
            ROSHookController::E_Left_State::E_Close);

        const ROSHookController::E_Right_State grip_r(robo_handler.get_right_hand().get_state() != Hand::E_Handring ?  
            ROSHookController::E_Right_State::E_Open :
            ROSHookController::E_Right_State::E_Close);

        pthread_mutex_unlock(&mutex_simulator);

        hook_controller.set_command(grip_l, grip_r);

        #endif

    }
    cout << "send_msg_thread : send stop" << endl;
    return NULL;
}


//----------------------------------------------------------
// Command Table
//----------------------------------------------------------
void add_command(const E_Process proc){
Vec3d vec_up(0, 0, 1);

    Vec3d up_margin(0, 0 ,0.03);
    Vec3d sink(0, 0, -0.01);

    Attractor att_l_home(L_HOME);
    Attractor att_r_home(R_HOME);

    Attractor att_sink(Attractor::E_Plane, sink, vec_up);
    Attractor att_box_abov(Attractor::E_Plane, Vec3d(0, 0, ClothSimulator::BOX_HEIGHT * 3), vec_up);
    Attractor att_box_height(Attractor::E_Plane, Vec3d(0, 0, ClothSimulator::BOX_HEIGHT + 0.03), vec_up);
   
    Attractor att_N_target(Attractor::E_Line, Vec3d(1, 0, 0), vec_up);
    Attractor att_S_target(Attractor::E_Line, Vec3d(-1, 0, 0), vec_up);
    Attractor att_W_target(Attractor::E_Line, Vec3d(0, 1, 0), vec_up);
    Attractor att_E_target(Attractor::E_Line, Vec3d(0, -1, 0), vec_up);

    Attractor att_N_target2(Attractor::E_Line, Vec3d(0.3, 0, 0), vec_up);
    Attractor att_S_target2(Attractor::E_Line, Vec3d(-0.3, 0, 0), vec_up);
    Attractor att_W_target2(Attractor::E_Line, Vec3d(0, 0.3, 0), vec_up);
    Attractor att_E_target2(Attractor::E_Line, Vec3d(0, -0.3, 0), vec_up);

    l_command_pool.clear();
    r_command_pool.clear();
    if(proc == E_Proc_1_0){     
        list<StringModel>::iterator it = simulator.get_str_model_list().begin();
        Attractor att_N_handle((*it).get_handle()->get_x() + up_margin);
        ++ it;
        Attractor att_S_handle((*it).get_handle()->get_x() + up_margin);
        ++ it;
        Attractor att_W_handle((*it).get_handle()->get_x() + up_margin);
        ++ it;
        Attractor att_E_handle((*it).get_handle()->get_x() + up_margin);

        Attractor att_N_box(Vec3d(-ClothSimulator::BOX_DEPTH / 2, 0, 0.01));
        Attractor att_S_box(Vec3d(ClothSimulator::BOX_DEPTH / 2, 0, 0.01));    
        Attractor att_W_box(Vec3d(0, -ClothSimulator::BOX_WIDTH / 2, 0.01));    
        Attractor att_E_box(Vec3d(0, ClothSimulator::BOX_WIDTH / 2, 0.01));    
        // Move North end to the south 
        {
            Command rcom(Command::E_Move, att_N_handle);
            Command lcom(Command::E_Move, att_S_handle);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }

        {
            Command rcom(Command::E_Stop, 7.0);
            Command lcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());;
        }  
        {
            Command rcom(Command::E_Move, att_W_handle);
            Command lcom(Command::E_Move, att_E_handle);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }   
        {
            Command rcom(Command::E_Stop, 7.0);
            Command lcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }    
        {
            Command rcom(att_l_home);
            Command lcom(att_r_home);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }
    }
    else if(proc == E_Proc_1_1){     
        list<StringModel>::iterator it = simulator.get_str_model_list().begin();
        Attractor att_N_handle((*it).get_handle()->get_x() + up_margin);
        Particle* N_handle((*it).get_handle());

        // Move North end to the south 
        {
            Command rcom(Command::E_Move, att_N_handle);
            Command lcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }

        {
            Command rcom(Command::E_Stop, 3.0);
            Command lcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());;
        }          
        {
            Command rcom(Command::E_Move, att_sink);
            Command lcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }

        {
            Command rcom(Command::E_Stop, 3.0);
            Command lcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());;
        }  

        {
            Command rcom(Command::E_Grip, N_handle);
            Command lcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }   
        {
            Command rcom(Command::E_Move, att_box_abov);
            Command lcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }    
        {
            Command rcom(Command::E_Move, att_N_target);
            Command lcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }
        {
            Command rcom(Command::E_Release);
            Command lcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }
        {
            Command rcom(Command::E_Move, att_box_height);
            Command lcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }
        {
            Command rcom(Command::E_Move, att_N_target2);
            Command lcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }
        {
            Command rcom(Command::E_Move, att_box_abov);
            Command lcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }
        {
            Command rcom(Command::E_Move, att_l_home);
            Command lcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }
        {
            Command rcom(Command::E_Wait);
            Command lcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }
    }
    else if(proc == E_Proc_1_2){     
        simulator.get_str_model_list().pop_front();
        list<StringModel>::iterator it = simulator.get_str_model_list().begin();
        Attractor att_S_handle((*it).get_handle()->get_x() + up_margin);
        Particle* S_handle((*it).get_handle());

        // Move South end to the North side 
        {
            Command lcom(Command::E_Move, att_S_handle);
            Command rcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }

        {
            Command lcom(Command::E_Stop, 3.0);
            Command rcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());;
        }
        {
            Command lcom(Command::E_Move, att_sink);
            Command rcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }

        {
            Command lcom(Command::E_Stop, 3.0);
            Command rcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());;
        }    
        {
            Command lcom(Command::E_Grip, S_handle);
            Command rcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }   
        {
            Command lcom(Command::E_Move, att_box_abov);
            Command rcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }    
        {
            Command lcom(Command::E_Move, att_S_target);
            Command rcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }
        {
            Command lcom(Command::E_Release);
            Command rcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }
        {
            Command lcom(Command::E_Move, att_box_height);
            Command rcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }
        {
            Command lcom(Command::E_Move, att_S_target2);
            Command rcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }
        {
            Command lcom(Command::E_Move, att_box_abov);
            Command rcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }
        {
            Command lcom(Command::E_Move, att_r_home);
            Command rcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }
        {
            Command lcom(Command::E_Wait);
            Command rcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }
    }

    else if(proc == E_Proc_1_3){       
        simulator.get_str_model_list().pop_front();
        list<StringModel>::iterator it = simulator.get_str_model_list().begin();
        Attractor att_W_handle((*it).get_handle()->get_x() + up_margin);
        Particle* W_handle((*it).get_handle());
        // Move left end to the right side 
        {
            Command rcom(Command::E_Move, att_W_handle);
            Command lcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }

        {
            Command rcom(Command::E_Stop, 3.0);
            Command lcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());;
        }  
        {
            Command rcom(Command::E_Move, att_sink);
            Command lcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }

        {
            Command rcom(Command::E_Stop, 3.0);
            Command lcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());;
        }  
        {
            Command rcom(Command::E_Grip, W_handle);
            Command lcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }   
        {
            Command rcom(Command::E_Move, att_box_abov);
            Command lcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }    
        {
            Command rcom(Command::E_Move, att_W_target);
            Command lcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }
        {
            Command rcom(Command::E_Release);
            Command lcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }
        {
            Command rcom(Command::E_Move, att_box_height);
            Command lcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }
        {
            Command rcom(Command::E_Move, att_W_target2);
            Command lcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }
        {
            Command rcom(Command::E_Move, att_box_abov);
            Command lcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }
        {
            Command rcom(Command::E_Move, att_l_home);
            Command lcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }
        {
            Command rcom(Command::E_Wait);
            Command lcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }
    }

    else if(proc == E_Proc_1_4){     
        simulator.get_str_model_list().pop_front();
        list<StringModel>::iterator it = simulator.get_str_model_list().begin();
        Attractor att_E_handle((*it).get_handle()->get_x() + up_margin);
        Particle* E_handle((*it).get_handle());
        // Move right end to the left side 
        {
            Command lcom(Command::E_Move, att_E_handle);
            Command rcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }
        {
            Command lcom(Command::E_Stop, 3.0);
            Command rcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());;
        }  
        {
            Command lcom(Command::E_Move, att_sink);
            Command rcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }

        {
            Command lcom(Command::E_Stop, 3.0);
            Command rcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());;
        }  

        {
            Command lcom(Command::E_Grip, E_handle);
            Command rcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }   
        {
            Command lcom(Command::E_Move, att_box_abov);
            Command rcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }    
        {
            Command lcom(Command::E_Move, att_E_target);
            Command rcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }
        
        {
            Command lcom(Command::E_Release);
            Command rcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }
        {
            Command lcom(Command::E_Move, att_box_height);
            Command rcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }
        {
            Command lcom(Command::E_Stop, 3.0);
            Command rcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());;
        }  
        {
            Command lcom(Command::E_Move, att_E_target2);
            Command rcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }

        {
            Command lcom(Command::E_Move, att_box_abov);
            Command rcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }
        {
            Command lcom(Command::E_Move, att_r_home);
            Command rcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }
        {
            Command lcom(Command::E_Wait);
            Command rcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }
    }


    else{
        if(simulator.get_str_model_list().size() != 0)  simulator.get_str_model_list().pop_front();
        /*{
            Command rcom(Command::E_Move, att_l_home);
            Command lcom(Command::E_Move, att_r_home);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }*/
        {
            Command lcom(Command::E_Wait);
            Command rcom(Command::E_Wait);

            r_command_pool.basic.push_front(rcom);
            l_command_pool.basic.push_front(lcom);

            robo_handler.add_right_command(&r_command_pool.basic.front());
            robo_handler.add_left_command(&l_command_pool.basic.front());
        }
    }

    
}

void init(){
#ifndef NO_ROS
    hook_controller.init();
    joint_controller.init();
    ik_client.init();    
#endif

    Vec3d upward(0, 0, 0.20);
    Vec3d downward(0, 0, ClothSimulator::BOX_HEIGHT);
    Vec3d lproc1( ClothSimulator::BOX_DEPTH / 3.0 * 1.0, -ClothSimulator::BOX_WIDTH / 3 * 2, 0);
    Vec3d rproc1( -ClothSimulator::BOX_DEPTH / 3.0 * 1.0, ClothSimulator::BOX_WIDTH / 3 * 2, 0);
    
    Vec3d rproc2( 1, 0, 0);
    Vec3d lproc2( -1, 0, 0);

    Attractor attdown(Attractor::E_Plane, downward, upward);
    Attractor attup(Attractor::E_Plane, upward, upward);
    //Attractor attup(upward);

    Attractor attl1(Attractor::E_Line, rproc1, upward);
    Attractor attl2(Attractor::E_Line, rproc2, upward);
    Attractor attr1(Attractor::E_Line, lproc1, upward);
    Attractor attr2(Attractor::E_Line, lproc2, upward);
    Attractor attlhome(L_HOME);
    Attractor attrhome(R_HOME);
    l_handle = simulator.get_str_model_list().front().get_handle();
    r_handle = simulator.get_str_model_list().back().get_handle();

#ifdef CALIB_MODE
    {
        curr_proc = E_Proc_1_0;
        add_command(curr_proc);
    }
#else
    {
        curr_proc = E_Proc_1_1;
        add_command(curr_proc);
    }
#endif
    /*
    {

        Attractor attlhome(L_HOME);
        Attractor attrhome(R_HOME);

        l_command_pool.push_front(attrhome);
        r_command_pool.push_front(attlhome);

        robo_handler.add_left_command(l_command_pool.front());
        robo_handler.add_right_command(r_command_pool.front());

        cout << "---------------free MODE-------------------" << endl;
        //OPLCommand rcom(&robo_handler.get_curr_right_hand(), &robo_handler.get_curr_left_hand(), &robo_handler.get_right_hand(), &robo_handler.get_left_hand(), &robo_handler.get_right_joints());
        OPLCommand lcom(&robo_handler.get_curr_left_hand(), &robo_handler.get_curr_right_hand(), &robo_handler.get_left_hand(), &robo_handler.get_right_hand(), &robo_handler.get_left_joints());
        
        Command rcom(Command::E_Stop, 300.0);

        r_command_pool.push_front(rcom);
        l_command_pool.push_front(lcom);

        robo_handler.add_right_command(r_command_pool.front());
        robo_handler.add_left_command(l_command_pool.front());
    }
    */
    
}

//----------------------------------------------------------
// Call back method - Display function
//----------------------------------------------------------
void Display(){
    static E_Process prev_proc(E_Proc_0);
    cam.state[5] -= 0.2;

    timer.start();

    pthread_mutex_lock(&mutex_curr_proc);
    E_Process cp_curr_proc(curr_proc);
    pthread_mutex_unlock(&mutex_curr_proc);

    if(prev_proc != cp_curr_proc){
        tk_l.clear();
        tk_r.clear();
    }
    prev_proc = curr_proc;

    // Make copy
    pthread_mutex_lock(&mutex_simulator);
    ClothSimulator cp_simulator(simulator);
    pthread_mutex_unlock(&mutex_simulator);

    pthread_mutex_lock(&mutex_simulator);
    RobotHandler cp_robo_handler(robo_handler);
    pthread_mutex_unlock(&mutex_simulator);


    glRasterPos3d(0, 0, 0);
    glutBitmapString(GLUT_BITMAP_HELVETICA_18,(const unsigned char*)("Hello Free glut Font"));
    //clear buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    //set matrix mode 
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    //initial camera pos
    gluLookAt(1.2, 1.2, 0.7, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    //move camera
    PolorView(); 
/*    {  
        if(cp_curr_proc == E_Proc_2_1){
            vector<Vec3d> hole;

            // Make copy
            pthread_mutex_lock(&mutex_simulator);
            hole = simulator.get_hole(intersectoin.first, intersectoin.second);
            pthread_mutex_unlock(&mutex_simulator);

            gf::draw_bio(hole);
            gf::draw_lines(3, hole.begin(), hole.end());
            gf::draw_line(3, hole.back(), hole.front());
        }

    }*/
    // Draw hands
    {
        gf::draw_coordination((cp_robo_handler.get_left_hand()).get_x(), (cp_robo_handler.get_left_hand()).get_rot());
        gf::draw_coordination((cp_robo_handler.get_right_hand()).get_x(), (cp_robo_handler.get_right_hand()).get_rot());
    } 
    // Draw hands
    /*
    {
        gf::draw_coordination((cp_robo_handler.get_curr_left_hand()).get_x(), (cp_robo_handler.get_curr_left_hand()).get_rot());
        gf::draw_coordination((cp_robo_handler.get_curr_right_hand()).get_x(), (cp_robo_handler.get_curr_right_hand()).get_rot());
    }     
    */
    // Closest set
    {
        //gf::draw_closest_set(cp_simulator);
    }
    // Draw String Models
    {
        list<StringModel>::iterator it_str = (cp_simulator.get_str_model_list()).begin();
        while(it_str != (cp_simulator.get_str_model_list()).end()){
            gf::draw_string_model(*it_str);
            ++ it_str;
        }
    }
    // Draw trajectory
    {
        const double LINE_WIDTH(3);
        gf::draw_lines(LINE_WIDTH, tk_l.begin(), tk_l.end());
        gf::draw_lines(LINE_WIDTH, tk_r.begin(), tk_r.end());
    }
    // Draw Boxes
    {
        list<Box>::iterator it_box = (cp_simulator.get_box_list()).begin();
        while(it_box != (cp_simulator.get_box_list()).end()){
            gf::draw_box(*it_box);
            ++ it_box;
        }
    }
    // Box coordinate
    {   
        gf::draw_coordination(Vec3d(0, 0, 0) , Vec3d(0, 0, 0));
    }
    // Hold the hand trajectory 
    tk_l.add_point((cp_robo_handler.get_left_hand().get_x()));
    tk_r.add_point((cp_robo_handler.get_right_hand().get_x()));
    //cout << cp_robo_handler.get_left_state() << " " << cp_robo_handler.get_right_state() << endl;

    //swap display buffer
    glutSwapBuffers(); 

    timer.stop();
    // cout << "time: " << timer.get_interval() << "[s]" << endl;
}

//----------------------------------------------------------
// Init openGL
//----------------------------------------------------------
//init opengl
void InitOpenGL(void) {
    //(x,y,z,alpha,beta,ganma)
    cam.width = WINDOWSIZE_X;
    cam.height = WINDOWSIZE_Y;
    glutInitDisplayMode(GLUT_DEPTH | GLUT_RGBA | GLUT_DOUBLE);
    glutInitWindowPosition(0,0);
    glutInitWindowSize(cam.width, cam.height);
    glutCreateWindow("StringApp");
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    GLfloat light0pos[] = { 0, -0.5, 0, 1 };
    glLightfv(GL_LIGHT0, GL_POSITION, light0pos);
   // glShadeModel(GL_SMOOTH);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
}

//----------------------------------------------------------
// Change view point
//----------------------------------------------------------
void PolorView(void){
    glTranslated(-cam.state[0], -cam.state[1], -cam.state[2]);
    glRotated(-cam.state[3], 1.0, 0.0, 0.0);
    glRotated(-cam.state[4], 0.0, 1.0, 0.0);
    glRotated(-cam.state[5], 0.0, 0.0, 1.0);
    //cam.state.null();
}
//----------------------------------------------------------
// Call back method - Idle function
//----------------------------------------------------------
void Idle(void){
    timer.stop();
    if(timer.get_start() > timer.get_stop()){
        if(timer.get_interval() + 1.0 > timer.get_period()){
            glutPostRedisplay();
        }
    }
    else{
        if(timer.get_interval() > timer.get_period()){
            glutPostRedisplay();
        }
    }
}

//----------------------------------------------------------
// Call back method - Resize function
//----------------------------------------------------------
//reshape function
void Resize(int w, int h) { 
    double aspect = (double)w/(double)h;
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(30.0, aspect, 1.0, 5000.0);
} 

//----------------------------------------------------------
// Call back method - key function
//----------------------------------------------------------
//keyfunc
void NormalKeyIn(unsigned char key, int x, int y){ 
    switch(key){ 
    	case 27:
            quit_flag = true;
            cout << "join_waiting" << endl;
            pthread_join( t_send_id , NULL);
            pthread_join( t_receive_id, NULL );
            cout << "see you" << endl;
        	exit(0); 
        	break; 
    	case 'e': 
			cam.state[1] += 0.1;
        	break; 
    	case 'q':    
			cam.state[1] -= 0.1;
        	break; 
    	case 'w':  
        	cam.state[2] -= 0.1;
        	break; 
    	case 's':  	
        	cam.state[2] += 0.1;
        	break; 
    	case 'd':  
			cam.state[0] += 0.1;
        	break; 
    	case 'a':  
			cam.state[0] -= 0.1;
        case 'f':   
            cam.state[4] += 0.1;
            break; 
        case 'r':   
            cam.state[4] -= 0.1;
        	break;
        case 'n':   
            pthread_mutex_lock(&mutex_calc_forcibly_flag);
            calc_forcibly_flag = true;
            pthread_mutex_unlock(&mutex_calc_forcibly_flag);
            break;  
        case '/':
            pthread_mutex_lock(&mutex_simulator);
            if(robo_handler.get_right_hand().get_state() == Hand::E_Opening)
                robo_handler.get_right_hand().set_state(Hand::E_Handring);
            else
                robo_handler.get_right_hand().set_state(Hand::E_Opening);
            pthread_mutex_unlock(&mutex_simulator);
            cout << " / : pushed" << endl;
            break;
        case '\\':
            pthread_mutex_lock(&mutex_simulator);
            if(robo_handler.get_left_hand().get_state() == Hand::E_Opening)
                robo_handler.get_left_hand().set_state(Hand::E_Handring);
            else
                robo_handler.get_left_hand().set_state(Hand::E_Opening);
            pthread_mutex_unlock(&mutex_simulator);
            cout << " \\ : pushed" << endl;
            break;
		case 'm':
            if(simulation_state == E_Moving){
                simulation_state = E_Stopping;
            }
            else if(simulation_state == E_Stopping){
                simulation_state = E_Moving;
            }
			break;
    	default: 
        	break; 
    }
    //glutPostRedisplay();
    timer.stop();
    if(timer.get_start() > timer.get_stop()){
        if(timer.get_interval() + 1.0 > timer.get_period()){
            glutPostRedisplay();
        }
    }
    else{
        if(timer.get_interval() > timer.get_period()){
            glutPostRedisplay();
        }
    }
}

void SpecialKeyIn(int key, int x, int y){
	switch(key){
		case GLUT_KEY_LEFT:      
			cam.state[5] -= 1.0;
			break;
		case GLUT_KEY_RIGHT:     
			cam.state[5] += 1.0;
			break;
		case GLUT_KEY_UP:
			cam.state[3] -= 1.0;
			break;
		case GLUT_KEY_DOWN:
			cam.state[3] += 1.0;
			break;
		default:
			break;
	} 
    timer.stop();
    if(timer.get_start() > timer.get_stop()){
        if(timer.get_interval() + 1.0 > timer.get_period()){
            glutPostRedisplay();
        }
    }
    else{
        if(timer.get_interval() > timer.get_period()){
            glutPostRedisplay();
        }
    }

}

//----------------------------------------------------------
// Call back method - Mouse function
//----------------------------------------------------------
void Mouse(int button, int state, int x, int y){
    mouse[0] = x;
    mouse[1] = y;
    cout << mouse[0] << " " << mouse[1] << endl;
}


//main entry point of this program
int main(int argc, char *argv[]){
    pthread_mutex_init(&mutex_simulation_state, NULL);
    pthread_mutex_init(&mutex_simulator, NULL);
    pthread_mutex_init(&mutex_curr_proc, NULL);
    pthread_mutex_init(&mutex_recv_flag, NULL);
    pthread_mutex_init(&mutex_calc_forcibly_flag, NULL);

    ros::init(argc, argv, "cloth_simulator");
    ros::NodeHandle n;
    init();
    cout << "start" << endl;
  	glutInit(&argc, argv);
    //init OpenGL
  	InitOpenGL();
  	//set Display function
  	glutDisplayFunc(Display);
  	//set reshape function
	glutReshapeFunc(Resize);
	//set keybord function
	glutKeyboardFunc(NormalKeyIn);
	//set special key function
	glutSpecialFunc(SpecialKeyIn);
    //set mouce function
    glutMouseFunc(Mouse);
    //set Idle function
    glutIdleFunc(Idle);
    // Start message loop
    // thread for transmiting Message 
    {
        int t_check = pthread_create(&t_send_id, NULL, send_msg_thread, (void *)&quit_flag);
        if(t_check != 0){
          cout << "err! : thread cant open" << endl;
          return -1;
        }
    }
    // thread for receive Message     
    {
        int t_check = pthread_create(&t_receive_id, NULL, receive_msg_thread, (void *)&quit_flag);
        if(t_check != 0){
            cout << "err! : thread cant open" << endl;
            return -1;
        }
    }
	//start main loop
	glutMainLoop();

	return 0;
}
