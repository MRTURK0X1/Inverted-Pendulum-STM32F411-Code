#pragma once

#include "stm32f4xx_hal.h"
#include <alloca.h>

typedef struct {
	TIM_HandleTypeDef* bts_timer;
	uint32_t bts_ch1;
	uint32_t bts_ch2;
	GPIO_TypeDef* en_port;
	uint32_t en_pin;
	float deadzone;
	uint32_t arr_fullscale;
	float deadzoneCoeff;
}btsMotorDriver;

void btsMotorDriver_init(btsMotorDriver *self, TIM_HandleTypeDef *htim,
		uint32_t ch1, uint32_t ch2, GPIO_TypeDef* EN_PORT, uint32_t EN_PIN,
		uint32_t deadzone, uint32_t arr_fullscale);

void btsMotorDriver_start(btsMotorDriver *self);

void btsMotorDriver_motorTurn(btsMotorDriver* self, int16_t arr_val);


