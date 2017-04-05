/*
 * serial1.h
 *
 *  Created on: Nov 28, 2016
 *      Author: jamesliu
 */

#ifndef SERIAL1_H_
#define SERIAL1_H_

#include "main.h"
#include <String.h>

#ifndef SERIAL1_BUFFER_SIZE
#define SERIAL1_BUFFER_SIZE 64
#endif

#ifndef SERIAL1_BUFFER_SIZE_TX
#define SERIAL1_BUFFER_SIZE_TX 128
#endif

//this is for writing an existing, assigned buffer:
#define Serial1_writeBuf(str) Serial1_writeBytes((str), sizeof((str))-1)

//this is for writing a string literal:
//#define Serial1_writeStr(str) Serial1_writeStr_Buf=(str); Serial1_writeBytes(Serial1_writeStr_Buf, sizeof((str))-1)

uint8_t Serial1_writeStr_Buf[SERIAL1_BUFFER_SIZE_TX];

uint8_t Serial1_buffer[SERIAL1_BUFFER_SIZE];

void Serial1_begin();
int Serial1_available();
uint8_t Serial1_peek();
uint8_t Serial1_read();
int Serial1_readBytes(uint8_t *buffer, int length);
int Serial1_find(uint8_t data);
int Serial1_findAny(uint8_t *match, int length);
int Serial1_readUntil(uint8_t *buffer, uint8_t data);
int Serial1_readCommand(uint8_t *buffer);
int Serial1_availableForWrite();
void Serial1_write(uint8_t data);
void Serial1_writeBytes(uint8_t *data, uint16_t length);

#endif
