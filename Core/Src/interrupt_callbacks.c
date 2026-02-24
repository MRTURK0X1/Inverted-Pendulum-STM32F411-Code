#include "main.h"
#include "coms_tx.h"


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM5){
	    HAL_IncTick();
	}

	if(htim->Instance == TIM1){
		BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(control_task_handle,  &pxHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart1){
		BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
		xTaskNotifyFromISR(uartTx_task_handle, BIT_TX_DONE,eSetBits,&pxHigherPriorityTaskWoken );
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
	}
}


