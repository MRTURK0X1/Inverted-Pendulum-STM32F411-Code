#include "main.h"
#include "encoder.h"
#include "btsMotorDriver.h"
#include "control.h"
#include "coms_tx.h"


#define SWITCH_TRESHOLD     0.5235

#if DEBUG_MODE
static float debug_pos_p;
static float debug_pos_c;
static float debug_vel_p;
static float debug_vel_c;
static float debug_ref;
#endif

static controller* cont;
static controller* Pv_swingUp;
static controller* stabilityFSF;
static controller* invPendHybrid;
static controller* swingUpPv;
static controller* integrator;
static controller* vel_cascade;
static controller* squareWave;
static controller* PI_vel;

void control_task_handler(void *argument) {
	controller_AllocStatic(Pv_swingUp, PV_context);
	controller_AllocStatic(swingUpPv, LinSwingUp_context);

	controller_AllocStatic(integrator, integrator_context);
	controller_AllocStatic(vel_cascade, cascade_context);
	controller_AllocStatic(PI_vel, PID_context);

	controller_AllocStatic(stabilityFSF, FSF_context);
	controller_AllocStatic(invPendHybrid, Hybrid_context);
	controller_AllocStatic(squareWave, SquareWave_context);

	controlArgs controlArgs = {0};
	state_msg_t state_msg = STATE_MSG_INIT();
	//debug_msg_t debug_msg = DEBUG_MSG_INIT(0x11);
	encoder cartEncoder;
	encoder pendEncoder;
	btsMotorDriver motorDriver;
	encoderData cartData = {0};
	encoderData pendData = {0};


	/*Swing-up controller init   */
	control_PV_init(Pv_swingUp,cart_pos,26.2810,0.5872);
	control_linSwingUp_init(swingUpPv, Pv_swingUp, 0.30, 0.32);

	/*Stabilization controller init   */
	control_FSF_init(stabilityFSF, -0.5495, 20.4943, -3.8074, 4.2358); // u_1[acceleration]
	control_integrator_init(integrator, stabilityFSF, 0.01); // u_1*1/s = u_2[velocity]
	control_PID_init(PI_vel,cart_vel, 8.0503, 129.6381, 0, 0.01, 20); // PI velocity controller for u_2 reference
	control_cascade_init(vel_cascade, integrator, PI_vel); // cascade structure implementation

	control_Hybrid_init(invPendHybrid, swingUpPv, vel_cascade, SWITCH_TRESHOLD);
	cont = invPendHybrid;

	float squareWave_switchVals[] = {1.5,2.0,5.5,4.0,3.0};
	control_squareWave_init(squareWave, squareWave_switchVals, sizeof(squareWave_switchVals));


	encoder_init(&cartEncoder, &htim4, 360*4, 0.0063, 0.01, 0, 20);
	encoder_init(&pendEncoder, &htim2, 400*4, 1, 0.01, 800, 20);
	btsMotorDriver_init(&motorDriver, &htim3, TIM_CHANNEL_2, TIM_CHANNEL_1, GPIOB, GPIO_PIN_0, 400, 6000);

	encoder_start(&cartEncoder);
	encoder_start(&pendEncoder);
	btsMotorDriver_start(&motorDriver);

	//xMessageBufferSend(debug_buffer,(uint8_t*)&debug_msg,sizeof(debug_msg_t),0);
	SEGGER_SYSVIEW_Start();
	start_sampler();
	for (;;) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		encoder_sample(&cartEncoder);
		encoder_sample(&pendEncoder);

		encoder_sampleProcess(&cartEncoder, &cartData);
		encoder_sampleProcess(&pendEncoder, &pendData);

		controlArgs.pos_1 = cartData.pos;
		controlArgs.pos_2 = pendData.pos;
		controlArgs.vel_1 = cartData.vel;
		controlArgs.vel_2 = pendData.vel;

		float u_val = control_run(cont,&controlArgs);
		u_val = v_12_saturate(u_val);
		int16_t arr_val = v_12_to_arr(u_val);
		btsMotorDriver_motorTurn(&motorDriver, arr_val);

		state_msg.pos_1 = cartData.pos;
		state_msg.pos_2 = pendData.pos;
		state_msg.vel_1 = cartData.vel;
		state_msg.vel_2 = pendData.vel;
		state_msg.u = u_val;
		xMessageBufferSend(state_buffer,(uint8_t*)&state_msg,sizeof(state_msg_t),0);
		xTaskNotify(uartTx_task_handle, BIT_STATE_BUFFER,eSetBits);
#if DEBUG_MODE
		debug_pos_p = pendData.pos;
		debug_vel_p = pendData.vel;
		debug_pos_c = cartData.pos;
		debug_vel_c = cartData.vel;
		debug_ref = controlArgs.ref;
#endif

	}

}


void command_task_handler(void *argument) {

	for(;;){
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	}

}

void uartTx_task_handler(void *argument){
	MessageBufferHandle_t msgBuffs[] = {state_buffer, debug_buffer};
	size_t msgBuffLen = sizeof(msgBuffs)/sizeof(msgBuffs[0]);

	static uint8_t txBuffer[MAX_MSG_SIZE];
	static size_t txBuffLen;
	uint32_t events = 0;
	uint8_t uart_busy = 0;
	for(;;){

		xTaskNotifyWait(pdFALSE, 0xFFFFFFFF, &events, portMAX_DELAY);

		if(events & BIT_TX_DONE)
			uart_busy = 0;

		if(!uart_busy){
			if(coms_tx_getMsgFromBuffers(msgBuffs,msgBuffLen,txBuffer,&txBuffLen)){

				HAL_UART_Transmit_DMA(&huart1, txBuffer, txBuffLen);
				uart_busy = 1;
			}
			else
				continue;
			}
		}
	}





void uartRx_task_handler(void *argument){

	for(;;){

	}
}



void vApplicationIdleHook(){

	for (;;) {

#if SIMULATION_MODE
		static uint32_t tim2_test_buff[] = { 13200, 26800, 35400, 42100, 50900, 61200, 72000, 81500 };
		static uint16_t tim4_test_buff[] = { 1100, 1650, 2500, 3300, 4150, 4950, 5200, 5900 };
		static uint8_t idx = 0;
		static uint8_t buffLen = 8;
		TIM2->CNT = tim2_test_buff[idx];
		TIM4->CNT = tim4_test_buff[idx];
		idx++;
		if (idx >= buffLen)
			idx = 0;
#endif

		__WFI();
	}
}

void vApplicationStackOverflowHook( TaskHandle_t xTask, char * pcTaskName){

}
