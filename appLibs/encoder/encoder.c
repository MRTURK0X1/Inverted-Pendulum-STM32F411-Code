#include "encoder.h"
#include <math.h>
#define PI 3.14159265359
#define LPF_SCALE 1
#define LPF_ALPHA_DEFAULT 0.7

void encoder_init(encoder *self, TIM_HandleTypeDef *timh, float CPR,float radius, float ts, uint32_t initial_count, float lpf_fc) {
	self->timh = timh;
	self->CPR = CPR;
	self->count = initial_count;
	self->radius = radius;
	self->pos = 0.0f;
	self->vel = 0.0f;
	self->ts = ts;

	float fs = 1.0f/ts;
	self->lpf_alpha = expf(-2*PI*lpf_fc/fs);

	__HAL_TIM_SET_COUNTER(timh,initial_count);
}

void encoder_start(encoder* self){
	HAL_TIM_Encoder_Start(self->timh, TIM_CHANNEL_ALL);
}

void encoder_sample(encoder *self){
	self->count =__HAL_TIM_GET_COUNTER(self->timh);
}

void encoder_sampleProcess(encoder *self, encoderData *out) {
	float pos = (((float) self->count * PI * 2 * (self->radius)) / (self->CPR));

	float raw_vel = (pos - self->pos) / (self->ts);
	float vel = (1 - self->lpf_alpha)*raw_vel + self->lpf_alpha *self->vel;

	out->pos = pos;
	out->vel = vel;

	self->pos = pos;
	self->vel = vel;

}

void encoder_stop(encoder* self){
	HAL_TIM_Encoder_Stop(self->timh, TIM_CHANNEL_ALL);

}

void encoder_configLPF(encoder *self, float lpf_alpha) {
	self->lpf_alpha = lpf_alpha;
}


