#pragma once
#include "stm32f4xx_hal.h"
#include <alloca.h>

typedef struct {
	TIM_HandleTypeDef *timh;
	uint32_t CPR;
	int16_t count;
	float radius;
	float pos;
	float vel;
	float ts;
	float lpf_alpha;
}encoder;

typedef struct __attribute__((packed)){
    float pos;
    float vel;
} encoderData;

void encoder_init(encoder* self, TIM_HandleTypeDef* timh, float CPR, float radius, float ts, uint32_t initial_count, float lpf_fc);
void encoder_start(encoder* self);
void encoder_sample(encoder* self);
void encoder_sampleProcess(encoder* self, encoderData* out);
void encoder_stop(encoder* self);
void encoder_configLPF(encoder* self, float lpf_alpha);

