#include "control.h"
#include "utility.h"
#include <string.h>
#include <math.h>


static float _control_lpf(float x, float y_prev, float a){
	float y_filtered = (1-a)*x + a*y_prev;

	return y_filtered;
}

void control_PID_init(controller* pid_cont,controlled_vars controlled_var, float kp, float ki, float kd, float sampling_time, float lpf_fc){
	pid_cont->controller = control_PID_control;
	PID_context* pidCtx = (PID_context*)(pid_cont->ctx);
	pidCtx->controlled_var = controlled_var;
	pidCtx->kp = kp;
	pidCtx->ki = ki;
	pidCtx->kd = kd;
	pidCtx->integral = 0;
	pidCtx->prev_e = 0;
	pidCtx->prev_e_dot = 0;
	pidCtx->sampling_time = sampling_time;

	float fs = 1.0f/sampling_time;
	pidCtx->lpf_coeff = expf(-TWO_PI*lpf_fc/fs);

}

void control_PV_init(controller* pv_cont,controlled_vars controlled_var, float kp, float kv){
	pv_cont->controller = control_PV_control;
	PV_context* pvCtx = (PV_context*)(pv_cont->ctx);
	pvCtx->controlled_var = controlled_var;
	pvCtx->kp = kp;
	pvCtx->kv = kv;

}

void control_FSF_init(controller* fsf_cont,float k1, float k2, float k3, float k4){
	fsf_cont->controller = control_FSF_control;
	FSF_context* fsfCtx = (FSF_context*)(fsf_cont->ctx);
	fsfCtx->k1 = k1;
	fsfCtx->k2 = k2;
	fsfCtx->k3 = k3;
	fsfCtx->k4 = k4;
}

void control_linSwingUp_init(controller* swingUp_cont, controller* linear_cont, float switchVal, float refVal){
	 swingUp_cont->controller = control_linSwingUp_control;
	 LinSwingUp_context* swingUpCtx = (LinSwingUp_context*)(swingUp_cont->ctx);
	 swingUpCtx->controller = linear_cont->controller;
	 swingUpCtx->controllerContext = linear_cont->ctx;
	 swingUpCtx->switchVal = switchVal;
	 swingUpCtx->refVal = refVal;
	 swingUpCtx->status = GO_POS;
}

void control_Hybrid_init(controller* hybrid_cont, controller* swingUp_cont, controller* stab_cont, float switchVal){
	hybrid_cont->controller = control_Hybrid_control;
	Hybrid_context* hybridCtx = (Hybrid_context*)(hybrid_cont->ctx);
	hybridCtx->swingUpControl = swingUp_cont->controller;
	hybridCtx->swingUpContext = swingUp_cont->ctx;
	hybridCtx->stabilityControl = stab_cont->controller;
	hybridCtx->stabilityContext = stab_cont->ctx;
	hybridCtx->switchVal = switchVal;

}

void control_squareWave_init(controller* squareWave_cont, float* switchVals, size_t switchValsByteSize){
	squareWave_cont->controller = control_squareWave_control;
	SquareWave_context* squareCtx = (SquareWave_context*)(squareWave_cont->ctx);
	if(switchValsByteSize > 6*sizeof(float)){
		squareWave_cont = NULL;
		return;
	}
	squareCtx->switchValsSize = switchValsByteSize/sizeof(float);
	squareCtx->switchIndx = 0;
	squareCtx->status = 1;
	memcpy(squareCtx->switchVals, switchVals, switchValsByteSize);
}


void control_cascade_init(controller* cascade_cont, controller* outter_cont, controller* inner_cont){
	cascade_cont->controller = control_cascade_control;
	cascade_context* cascadeCtx = (cascade_context*)(cascade_cont->ctx);
	cascadeCtx->outterControl = outter_cont->controller;
	cascadeCtx->outterControlContext = outter_cont->ctx;
	cascadeCtx->innerControl = inner_cont->controller;
	cascadeCtx->innerControlContext = inner_cont->ctx;


}

void control_integrator_init(controller* int_cont, controller* cont, float sampling_time){
	int_cont->controller = control_integral_control;
	integrator_context* integralCtx = (integrator_context*)(int_cont->ctx);
	integralCtx->controller = cont->controller;
	integralCtx->controllerContext = cont->ctx;
	integralCtx->integralVal = 0;
	integralCtx->sampling_time = sampling_time;

}

static float _control_PID(float ref, float x, PID_context* pid_ctx){
	float kp = pid_ctx->kp;
	float ki = pid_ctx->ki;
	float kd = pid_ctx->kd;

	float e = ref - x;
	float e_dot = (e - pid_ctx->prev_e)/pid_ctx->sampling_time;
	float e_dot_filtered = _control_lpf(e_dot, pid_ctx->prev_e_dot, pid_ctx->lpf_coeff);


	pid_ctx->prev_e = e;
	pid_ctx->integral+= e * pid_ctx->sampling_time;
	pid_ctx->prev_e_dot = e_dot_filtered;



	float u = kp* e + ki * pid_ctx->integral + kd * e_dot_filtered;
	return u;

}

float control_PID_control(void* ctx, controlArgs* args){
	PID_context* pid_ctx = (PID_context*)ctx;
	controlled_vars controlled_var = pid_ctx->controlled_var;


	float u;

	switch(controlled_var){
		case(cart_pos):
			u = _control_PID(args->ref, args->pos_1,pid_ctx);
			break;

		case(cart_vel):
			u = _control_PID(args->ref, args->vel_1,pid_ctx);
			break;

		case(pend_pos):
			u = _control_PID(args->ref, args->pos_2,pid_ctx);
			break;

		case(pend_vel):
		    u = _control_PID(args->ref, args->vel_2,pid_ctx);
			break;

		default:
			u = 0;
	}
	return u;
}


static float _control_PV(float ref, float x, float x_dot, PV_context* pv_ctx){
	float e = ref - x;
	float u = pv_ctx->kp * e - pv_ctx->kv * x_dot;
	return u;

}

float control_PV_control(void* ctx, controlArgs* args){
	PV_context* pv_ctx = (PV_context*)ctx;
	controlled_vars controlled_var = pv_ctx->controlled_var;
	float u;

	switch(controlled_var){
		case(cart_pos):
			u = _control_PV(args->ref, args->pos_1, args->vel_1, pv_ctx);
			break;

		case(pend_pos):
			u = _control_PV(args->ref, args->pos_2, args->vel_2, pv_ctx);
			break;

		default:
			u = 0;
	}
	return u;
}




float control_FSF_control(void* ctx, controlArgs* args){
	FSF_context* fsf_ctx = (FSF_context*)ctx;
	float u = -(fsf_ctx->k1*args->pos_1 + fsf_ctx->k2*args->pos_2 + fsf_ctx->k3*args->vel_1 + fsf_ctx->k4*args->vel_2);
	return u;
}

float control_linSwingUp_control(void* ctx, controlArgs* args){
	LinSwingUp_context* swingUp_ctx = (LinSwingUp_context*)ctx;
	switch(swingUp_ctx->status){
		case 1:
			args->ref = swingUp_ctx->refVal;
			if(args->pos_1 > swingUp_ctx->switchVal)
				swingUp_ctx->status = GO_NEG;
			break;
		case 2:
			args->ref  = -swingUp_ctx->refVal;
			if(args->pos_1 < -swingUp_ctx->switchVal)
				swingUp_ctx->status = GO_POS;
			break;
		default:
			break;
	}

	return swingUp_ctx->controller(swingUp_ctx->controllerContext,args);
	//return 0;
}

float control_Hybrid_control(void* ctx, controlArgs* args){
	Hybrid_context* hybrid_ctx = (Hybrid_context*)ctx;
	if (wrap_to_pi(args->pos_2) > hybrid_ctx->switchVal || wrap_to_pi(args->pos_2) < -hybrid_ctx->switchVal) {

		return hybrid_ctx->swingUpControl(hybrid_ctx->swingUpContext, args);

	} else {
		return hybrid_ctx->stabilityControl(hybrid_ctx->stabilityContext, args);

	}
}

float control_squareWave_control(void* ctx, controlArgs* args){
	SquareWave_context * squareCtx = ctx;
	float u;
	switch(squareCtx->status){
		case 1:
			u = squareCtx->switchVals[squareCtx->switchIndx];
			if(args->pos_1 > 0.35)
				squareCtx->status = 2;
			break;
		case 2:
			u = -squareCtx->switchVals[squareCtx->switchIndx];
			if(args->pos_1 < -0.35)
				squareCtx->status = 3;
			break;
		case 3:
			u = -squareCtx->switchVals[squareCtx->switchIndx];
			squareCtx->switchIndx++;
			squareCtx->status = 1;
			break;
		default:
			break;
	}
	if(squareCtx->switchIndx == squareCtx->switchValsSize){
		squareCtx->switchIndx = 0;
	}
	return u;
}

float control_cascade_control(void* ctx, controlArgs* args){
	cascade_context * cascadeCtx = ctx;
	float outer_ref = cascadeCtx->outterControl(cascadeCtx->outterControlContext, args);
	args->ref = outer_ref;
	float u = cascadeCtx->innerControl(cascadeCtx->innerControlContext, args);
	return u;
}

float control_integral_control(void* ctx, controlArgs* args){
	integrator_context* integratorCtx = ctx;
	float u = integratorCtx->controller(integratorCtx->controllerContext,args);

	return integratorCtx->integralVal += u*integratorCtx->sampling_time;

}

float control_Homing_control(void* ctx, controlArgs* args){
	float e = 0 - args->pos_1;
	float u = 26.2810 * e - 0.5872 * args->vel_1;
	return u;
}

float control_run(controller* cont, controlArgs* args){
	if(args->pos_1 > 0.40 || args->pos_1 < -0.40)
		return 0;
	else
		return cont->controller(cont->ctx,args);
}



