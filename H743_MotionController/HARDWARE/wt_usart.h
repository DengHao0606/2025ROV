#ifndef __WT_USART_H
#define __WT_USART_H

#include "main.h" 
#include "usart.h" 
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct
{
	float yaw;
	float pitch;
	float roll;
	float ax;
	float ay;
	float az;
}Attitude;

uint16_t crc16(const void *data, size_t length);
void attitude_solve(uint8_t* buf, uint8_t rx_st);
void WIT_UART_Transmit();
#endif 
