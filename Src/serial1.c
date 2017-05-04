/*
 * serial.c
 *
 *  Created on: Nov 28, 2016
 *      Author: jamesliu
 */

#include "serial1.h"

extern UART_HandleTypeDef huart4;

uint8_t Serial1_writeStr_Buf[SERIAL1_BUFFER_SIZE_TX]; 
 
uint8_t Serial1_buffer[SERIAL1_BUFFER_SIZE_RX]; 

static uint8_t *Serial1_tail = Serial1_buffer;
static uint8_t *Serial1_max = Serial1_buffer + SERIAL1_BUFFER_SIZE_RX; //points just outside the bounds
uint8_t Serial1_Ovf = 0;

void Serial1_begin(){
	HAL_UART_Receive_DMA(&huart4, Serial1_buffer, SERIAL1_BUFFER_SIZE_RX);
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//	if(Serial1_Ovf < 3) Serial1_Ovf++;
//}

static uint8_t *Serial1_getHead(){ //Volatile! Avoid use as much as possible!
//	return Serial1_buffer + SERIAL1_BUFFER_SIZE - LL_DMA_GetDataLength(DMA?,LL_DMA_CHANNEL_?);
	return Serial1_buffer + SERIAL1_BUFFER_SIZE_RX - (huart4.hdmarx->Instance->NDTR & 0xffff);
}

int Serial1_available(){
	uint8_t *head = Serial1_getHead();
	if(Serial1_Ovf==0){
		return head - Serial1_tail;
	}else if((Serial1_Ovf==1) && (head <= Serial1_tail)){
		return SERIAL1_BUFFER_SIZE_RX - (Serial1_tail - head);
	}else{
		Serial1_tail = head;
		Serial1_Ovf = 1;
		return SERIAL1_BUFFER_SIZE_RX;
	}
}

uint8_t Serial1_peek(){
	if(Serial1_available()){
		return *Serial1_tail;
	}else{
		return 0; //null is appropriate return value for nothing buffered
	}
}

uint8_t Serial1_read(){
	if(Serial1_available()){
		uint8_t data = *Serial1_tail;
		Serial1_tail++; //1 byte version of dequeue()
		if(Serial1_tail >= Serial1_max){
			Serial1_tail = Serial1_buffer;
			Serial1_Ovf--;
		}
		return data;
	}else{
		return 0; //null is appropriate return value for nothing buffered
	}
}

int Serial1_readBytes(uint8_t *buffer, int length){
//	BUFFER OVERFLOW WARNING!!! DO NOT REQUEST MORE THAN YOU CAN TAKE!
	if(Serial1_available()>=length){
		if(Serial1_tail+length >= Serial1_max){
			int half = Serial1_max - Serial1_tail;
			memcpy(buffer, Serial1_tail, half);
			memcpy(buffer+half, Serial1_buffer, length-half);
			Serial1_tail = Serial1_buffer + length - half;
			Serial1_Ovf--;
		}else{
			memcpy(buffer, Serial1_tail, length);
			Serial1_tail += length;
		}
		return 0;
	}
	return -1;
}

int Serial1_find(uint8_t data){
	//different from Arduino: this returns index of char of interest!
	for(int i=0; i<Serial1_available(); i++){
		if(*((Serial1_tail+i >= Serial1_max)?
				Serial1_tail+i-SERIAL1_BUFFER_SIZE_RX:
				Serial1_tail+i) ==data) return i;
	}
	return -1;
}

int Serial1_findAny(uint8_t *match, int length){
	for(int i=0; i<Serial1_available(); i++){
		uint8_t input = *((Serial1_tail+i >= Serial1_max)?
				Serial1_tail+i-SERIAL1_BUFFER_SIZE_RX:
				Serial1_tail+i);
		for(int j=0; j<length; j++){
			if(input == *(match+j)) return i;
		}
	}
	return -1;
}

int Serial1_readUntil(uint8_t *buffer, uint8_t data){
	if(Serial1_available()){
		int found = Serial1_find(data);
		if(found > -1){
			Serial1_readBytes(buffer, found);
			return 0;
		}
	}
	return -1;
}

int Serial1_readCommand(uint8_t *buffer){ //returns length of command
	while(Serial1_peek() == 0x0A || Serial1_peek() == 0x0D){ //NL(LF) or CR, respectively
		Serial1_read(); //clear leading line breaks
	}
	uint8_t delimiters[2] = {0x0A, 0x0D};
	int nextDelim = Serial1_findAny(delimiters, 2);
	if(nextDelim==-1){
		return -1;
	}else{
		Serial1_readBytes(buffer, nextDelim);
		return nextDelim;
	}
}

int Serial1_availableForWrite(){
	HAL_UART_StateTypeDef state = HAL_UART_GetState(&huart4);
	if(state == HAL_UART_STATE_BUSY_TX || state == HAL_UART_STATE_BUSY_TX_RX){
		return 0;
	}else{
		return 1;
	}
}

/*
 * Below this point be all the write functionality
 */

static uint8_t Serial1_charToWrite;
static uint8_t Serial1_Buffer_tx[SERIAL1_BUFFER_SIZE_TX];
static uint8_t *Serial1_tail_tx = Serial1_Buffer_tx;
static uint8_t *Serial1_head_tx = Serial1_Buffer_tx;
static uint8_t *Serial1_max_tx = Serial1_Buffer_tx + SERIAL1_BUFFER_SIZE_TX;
static uint8_t Serial1_ovf_tx = 0;
static uint16_t currentWrite = 0; //length of ongoing dma transaction
uint8_t Serial1_txWillTrigger = 0;

static int Serial1_available_tx(){
	if(Serial1_ovf_tx==0){
		return Serial1_head_tx - Serial1_tail_tx;
	}else if((Serial1_ovf_tx==1) && (Serial1_head_tx <= Serial1_tail_tx)){
		return SERIAL1_BUFFER_SIZE_TX - (Serial1_tail_tx - Serial1_head_tx);
	}else{
		Serial1_tail_tx = Serial1_head_tx;
		Serial1_ovf_tx = 1;
		return SERIAL1_BUFFER_SIZE_TX;
	}
}

static void Serial1_dequeue_tx(int length){
	Serial1_tail_tx+=length;
	if(Serial1_tail_tx >= Serial1_max_tx){
		Serial1_tail_tx -= SERIAL1_BUFFER_SIZE_TX;
		Serial1_ovf_tx--;
	}
}

void Serial1_doTx(uint8_t fromISR){
	static int txavail;
	txavail = Serial1_available_tx();
	if(txavail){
		if(Serial1_tail_tx + txavail > Serial1_max_tx){
			currentWrite = Serial1_max_tx - Serial1_tail_tx;
		}else{
			currentWrite = txavail;
		}
		HAL_UART_Transmit_DMA(&huart4, Serial1_tail_tx, currentWrite);
		Serial1_dequeue_tx(currentWrite);
		Serial1_txWillTrigger = 0;
	}
}

//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
//	if(Serial1_txWillTrigger == 0) Serial1_Serial1_doTx(1);
//}

//IF YOU ARE USING RTOS, PLEASE USE MUTEXES, OR WRAP THE BELOW IN CRITICAL SECTIONS.
void Serial1_writeBytes(uint8_t *data, uint16_t length){
	Serial1_txWillTrigger = 1;
	if(Serial1_head_tx + length >= Serial1_max_tx){
		uint16_t half = Serial1_max_tx-Serial1_head_tx;
		memcpy(Serial1_head_tx, data, half);
		memcpy(Serial1_Buffer_tx, data+half, length-half);
		Serial1_head_tx = Serial1_head_tx + length - SERIAL1_BUFFER_SIZE_TX;
		Serial1_ovf_tx++;
	}else{
		memcpy(Serial1_head_tx, data, length);
		Serial1_head_tx += length;
	}
	if(Serial1_availableForWrite()){
		Serial1_doTx(0);
	}else{
		Serial1_txWillTrigger = 0;
	}
}

void Serial1_write(uint8_t data){
	Serial1_charToWrite = data;
	Serial1_writeBytes(&Serial1_charToWrite, 1);
}
