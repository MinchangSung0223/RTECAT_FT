#ifndef RTECAT_MOTOR_CLIENT_H_
#define RTECAT_MOTOR_CLIENT_H_

#include <stdio.h>
#include "iostream"
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>

#include <sched.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <rtdm/ipc.h> 

#undef debug
//QT
// #include <thread>
// #include <pthread.h>


#include "Ecat_Master.h"
#include "ServoAxis_Motor.h"
#include <PropertyDefinition.h>

#define XDDP_PORT 0	/* [0..CONFIG-XENO_OPT_PIPE_NRDEV - 1] */

#define NSEC_PER_SEC 			1000000000
unsigned int cycle_ns = 1000000; // 2 ms
double period=((double) cycle_ns)/((double) NSEC_PER_SEC);	//period in second unit

// For RT thread management
static int run = 1;

unsigned long periodCycle = 0, worstCycle = 0;
unsigned long periodLoop = 0, worstLoop = 0;
unsigned int overruns = 0;


// Interface to physical axes
NRMKHelper::ServoAxis Axis[JOINTNUM];
const int 	 zeroPos[JOINTNUM] = {0};
const INT32 	 gearRatio[JOINTNUM] = {1};

// EtherCAT System interface object
Master ecat_master;
Ecat_iServo ecat_iservo[JOINTNUM];
Ecat_FT ecat_FT[JOINTNUM];

// When all slaves or drives reach OP mode,
// system_ready becomes 1.
int system_ready = 0;

// Global time (beginning from zero)
double gt=0;

// Trajectory parameers
double traj_time=0;
int motion=-1;

typedef struct STATE{
	JVec q;
	JVec q_dot;
	JVec q_ddot;
	JVec tau;
	JVec tau_ext;
	JVec e;
	JVec edot;
	JVec eint;
	JVec G;

	Vector6d x;                           //Task space
	Vector6d x_dot;
	Vector6d x_ddot;
	Vector6d F;
	Vector6d F_CB;
    Vector6d F_ext;
    
    double s_time;
}state;

typedef struct MOTOR_INFO{
    double torque_const[JOINTNUM];
    double gear_ratio[JOINTNUM];
    double rate_current[JOINTNUM];
}Motor_Info;

typedef struct ROBOT_INFO{
	int Position;
	int q_inc[JOINTNUM];
    int dq_inc[JOINTNUM];
	int tau_per[JOINTNUM];
	int statusword[JOINTNUM];
    int modeofop[JOINTNUM];

	JVec q_target;
	JVec qdot_target;
	JVec qddot_target;
	JVec traj_time;
	unsigned int idx;

	STATE act;
	STATE des;
	STATE nom;

    Motor_Info motor;

}ROBOT_INFO;

#endif  // /* RTECAT_MOTOR_CLIENT_H */