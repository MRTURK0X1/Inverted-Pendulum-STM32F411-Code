#include"coms_tx.h"


void coms_tx_Push(MessageBufferHandle_t* msgBuffh, uint8_t* data, size_t len){
    xMessageBufferSend(*msgBuffh, data, len, 0);
}


uint8_t coms_tx_getMsgFromBuffers(MessageBufferHandle_t* msgBufferHandles,
                                  size_t msgBufferHandles_len,
                                  uint8_t* buffAddr,
                                  size_t* buff_len){

    for (size_t i = 0; i < msgBufferHandles_len; i++) {

        if (xMessageBufferIsEmpty(msgBufferHandles[i]) == pdTRUE)
            continue;

        *buff_len = xMessageBufferReceive(msgBufferHandles[i], buffAddr, MAX_MSG_SIZE,0);

        if (*buff_len > 0) {
            return 1; // Found a message
        }else{
        	continue;
        }
    }
    return 0; // No messages found
}
