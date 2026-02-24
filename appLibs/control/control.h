#pragma once
#include"stdlib.h"
#include"alloca.h"

typedef struct  __attribute__((packed)){
	float pos_1;
	float pos_2;
	float vel_1;
	float vel_2;
	float ref;
}controlArgs;

typedef float (*controlStr)(void* ctx, controlArgs* args);

typedef struct {
	controlStr controller;
	void* ctx;
}controller;


typedef enum{
	pend_pos,
	pend_vel,
	cart_pos,
	cart_vel

}controlled_vars;

typedef struct {
	float kp;

	float ki;
	float integral;

	float kd;
	float prev_e;

	float lpf_coeff;
	float prev_e_dot;
	float sampling_time;

	controlled_vars controlled_var;

}PID_context;




typedef struct {
	float kp;
	float kv;
	controlled_vars controlled_var;
}PV_context;


typedef enum { GO_POS = 1, GO_NEG = 2 } linsw_state_t;
typedef struct {
	controlStr controller;
	void* controllerContext;
	float switchVal;
	float refVal;
	linsw_state_t status;

}LinSwingUp_context;

typedef struct {
	float k1;
	float k2;
	float k3;
	float k4;
}FSF_context;

typedef struct {
	controlStr swingUpControl;
	void* swingUpContext;
	controlStr stabilityControl;
	void* stabilityContext;
	float switchVal;

}Hybrid_context;

typedef struct{
	float switchVals[6];
	size_t switchValsSize;
	size_t switchIndx;
	int status;
}SquareWave_context;


typedef struct{
	controlStr outterControl;
	void* outterControlContext;
	controlStr innerControl;
	void* innerControlContext;

}cascade_context;

typedef struct{
	controlStr controller;
	void* controllerContext;
	float integralVal;
	float sampling_time;

}integrator_context;


void control_PID_init(controller* pid_cont,controlled_vars controlled_var, float kp, float ki, float kd, float sampling_time, float lpf_fc);
void control_PV_init(controller* pv_cont, controlled_vars controlled_var, float kp, float kv);
void control_FSF_init(controller* fsf_cont,float k1, float k2, float k3, float k4);
void control_Hybrid_init(controller* hybrid_cont, controller* swingUp_cont, controller* stab_cont, float switchVal);
void control_linSwingUp_init(controller* swingUp_cont, controller* linear_cont, float switchVal, float refVal);
void control_squareWave_init(controller* squareWave_cont, float* switchVals, size_t switchValsSize);
void control_cascade_init(controller* cascade_cont, controller* outter_cont, controller* inner_cont);
void control_integrator_init(controller* int_cont, controller* cont, float sampling_time);

float control_PID_control(void* ctx, controlArgs* args);
float control_PV_control(void* ctx, controlArgs* args);
float control_FSF_control(void* ctx, controlArgs* args);
float control_Hybrid_control(void* ctx, controlArgs* args);
float control_linSwingUp_control(void* ctx, controlArgs* args);
float control_squareWave_control(void* ctx, controlArgs* args);
float control_cascade_control(void* ctx, controlArgs* args);
float control_integral_control(void* ctx, controlArgs* args);
float control_Homing_control(void* ctx, controlArgs* args);

float control_run(controller* cont, controlArgs* args);


#define controller_AllocOnStack(name, type)                          \
    controller* name = (controller*)alloca(sizeof(controller));   \
    type* name##_ctx = (type*)alloca(sizeof(type)); \
    name->ctx = name##_ctx;                                       \


#define controller_AllocStatic(name, type)           \
    static controller name##_obj;                    \
    static type name##_ctx;                          \
    name = &name##_obj;                  \
    name->ctx = &name##_ctx;

