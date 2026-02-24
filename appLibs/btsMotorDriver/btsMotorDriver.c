#include "btsMotorDriver.h"

static void btsMotorDriver_motor_left(btsMotorDriver* self, uint16_t arr_val)//positive
{
	HAL_GPIO_WritePin(self->en_port, self->en_pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(self->bts_timer, self->bts_ch2, 0);
    if(arr_val>self->arr_fullscale){
        __HAL_TIM_SET_COMPARE(self->bts_timer, self->bts_ch1, self->arr_fullscale);
    }
    else{
        __HAL_TIM_SET_COMPARE(self->bts_timer, self->bts_ch1, arr_val);
    }

}

static void btsMotorDriver_motor_right(btsMotorDriver* self, uint16_t arr_val)//negative
{
	HAL_GPIO_WritePin(self->en_port, self->en_pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(self->bts_timer, self->bts_ch1, 0);
    if(arr_val>self->arr_fullscale){
        __HAL_TIM_SET_COMPARE(self->bts_timer, self->bts_ch2, self->arr_fullscale);
    }
    else{
        __HAL_TIM_SET_COMPARE(self->bts_timer, self->bts_ch2, arr_val);
    }

}

static void btsMotorDriver_motor_stop(btsMotorDriver* self)
{
	HAL_GPIO_WritePin(self->en_port, self->en_pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(self->bts_timer, self->bts_ch1, 0);
    __HAL_TIM_SET_COMPARE(self->bts_timer, self->bts_ch2, 0);
}

void btsMotorDriver_init(btsMotorDriver *self, TIM_HandleTypeDef *htim,
		uint32_t ch1, uint32_t ch2, GPIO_TypeDef* EN_PORT, uint32_t EN_PIN,
		uint32_t deadzone, uint32_t arr_fullscale) {
	__HAL_TIM_SET_COMPARE(htim, ch1, 0);
	__HAL_TIM_SET_COMPARE(htim, ch2, 0);
	self->bts_timer = htim;
	self->bts_ch1 = ch1;
	self->bts_ch2 = ch2;
	self->en_port = EN_PORT;
	self->en_pin = EN_PIN;
	self->deadzone = deadzone;
	self->arr_fullscale = arr_fullscale;
	self->deadzoneCoeff = ((float)(self->arr_fullscale - self->deadzone))/((float)self->arr_fullscale);
}

void btsMotorDriver_start(btsMotorDriver *self){
	HAL_TIM_PWM_Start(self->bts_timer, self->bts_ch1);
	HAL_TIM_PWM_Start(self->bts_timer, self->bts_ch2);
	HAL_TIM_Base_Start(self->bts_timer);
}


void btsMotorDriver_motorTurn(btsMotorDriver* self, int16_t arr_val) {

	if(arr_val>0){
		arr_val = self->deadzone + arr_val*self->deadzoneCoeff;
		btsMotorDriver_motor_left(self, (uint16_t)arr_val);
	}
	else if(arr_val<0){
		arr_val = -self->deadzone + arr_val*self->deadzoneCoeff;
		btsMotorDriver_motor_right(self, (uint16_t)(-arr_val));
	}
	else{
		btsMotorDriver_motor_stop(self);
	}

}

