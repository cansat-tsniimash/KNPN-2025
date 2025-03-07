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
	int16_t bme_temp; // 2
	uint32_t bme_press; //4
	int16_t bme_humidity; // 2
	float bme_height; // 2
	float lux_board;
	float lux_sp;
	uint8_t state;
	int16_t lidar;
	uint16_t crc;
}pack1_t;

typedef struct{
	uint8_t flag;
	uint16_t num;
	uint32_t time_ms;
	int16_t fix;
	float lat;
	float lon;
	float alt;
	uint32_t gps_time_s;
	float  current;
	uint16_t bus_voltage;
	uint16_t MICS_5524;
	uint16_t MICS_CO;
	uint16_t MICS_NO2;
	uint16_t MICS_NH3;
	uint16_t CCS_CO2;
	uint16_t CCS_TVOC;
	int16_t bme_temp_g; // 2
	uint32_t bme_press_g; //4
	int16_t bme_humidity_g; // 2
	uint16_t crc;
}pack2_t;






#endif /* STRUCTS_H_ */
