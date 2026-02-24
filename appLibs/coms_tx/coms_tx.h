#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"

#define BIT_TX_DONE  (1<<0)
#define BIT_STATE_BUFFER (1<<1)
#define BIT_DEBUG_BUFFER (1<<2)

#define MAX_MSG_SIZE 	25


typedef struct __attribute__((packed)){
	uint8_t sync0;
	uint8_t sync1;
	uint8_t code;
	float pos_1;
	float pos_2;
	float vel_1;
	float vel_2;
	float u;
}state_msg_t;

typedef struct __attribute__((packed)){
	uint8_t sync0;
	uint8_t sync1;
	uint8_t code;
}debug_msg_t;



void coms_tx_Push(MessageBufferHandle_t* msgBuffh, uint8_t* data, size_t len);
uint8_t coms_tx_getMsgFromBuffers(MessageBufferHandle_t* msgBufferHandles,
                                  size_t msgBufferHandles_len,
                                  uint8_t* buffAddr,
                                  size_t* buff_len);


#define STATE_MSG_INIT() {0xAA, 0x55, 0x01, 0, 0, 0, 0, 0}
#define DEBUG_MSG_INIT(debug_code) {0xAA,0x55, debug_code}



