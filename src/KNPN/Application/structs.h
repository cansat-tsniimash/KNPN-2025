/*
 * structs.h
 *
 *  Created on: Oct 19, 2024
 *      Author: Install
 */

#ifndef STRUCTS_H_
#define STRUCTS_H_

typedef struct{
	uint8_t flag;
	uint16_t num;
	uint32_t time_ms;
	int16_t accl[3]; //6
	int16_t gyro[3]; // 6
	int16_t mag[3];// 6
	uint16_t crc;
}pack1_t;

#endif /* STRUCTS_H_ */
