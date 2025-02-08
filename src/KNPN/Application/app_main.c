/*
 * app_main.c
 *
 *  Created on: Oct 12, 2024
 *      Author: Install
 */


#include "main.h"
#include "Shift_Register/shift_reg.h"
#include <LSM6DS3/DLSM.h>
#include <LIS3MDL/DLIS3.h>
#include <string.h>
#include <stdio.h>
#include <float.h>
#include <ATGM336H/nmea_gps.h>
#include <nRF24L01_PL/nrf24_upper_api.h>
#include <nRF24L01_PL/nrf24_lower_api_stm32.h>
#include <nRF24L01_PL/nrf24_defs.h>
#include <BME280_I2C/its_bme280.h>
#include <BME280_I2C/bme280_defs.h>
#include <BME280_I2C/bme280.h>
#include "ina219/inc/ina219_helper.h"
#include <Photorezistor/photorezistor.h>
#include "DWT_Delay/dwt_delay.h"
#include "structs.h"
#include <fatfs.h>


extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi4;
extern I2C_HandleTypeDef hi2c1;



uint16_t Crc16(uint8_t *buf, uint16_t len) {
	uint16_t crc = 0xFFFF;
	while (len--) {
		crc ^= *buf++ << 8;
		for (uint8_t i = 0; i < 8; i++)
			crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
	}
	return crc;
}

FRESULT mount_again(FIL *File1, FIL *File2, FIL *Fileb, FATFS *fileSystem, const char *path1, const char *path2, const char *pathb){
	FRESULT is_mount;
	f_mount(0, "0", 1);
	extern Disk_drvTypeDef disk;
	disk.is_initialized[0] = 0;
	is_mount = f_mount(fileSystem, "", 1);
	f_open(File1, (char*)path1, FA_WRITE | FA_OPEN_APPEND);
	f_open(File2, (char*)path2, FA_WRITE | FA_OPEN_APPEND);
	f_open(Fileb, (char*)pathv, FA_WRITE | FA_OPEN_APPEND);
	return is_mount;
}

FATFS fileSystem; // переменная типа FATFS
FIL File1; // хендлер файла
FIL File2;
FIL Fileb;
const char path1[] = "packet1.csv";
const char path2[] = "packet2.csv";
const char pathv[] = "packetv.csv";

FRESULT is_mount = 0;
int needs_mount = 0;

FRESULT res1 = 255;
FRESULT res2 = 255;
FRESULT resb = 255;

int app_main(){

//файлы

	memset(&fileSystem, 0x00, sizeof(fileSystem));

	extern Disk_drvTypeDef disk;
	disk.is_initialized[0] = 0;
	is_mount = f_mount(&fileSystem, "", 1);

	if(is_mount == FR_OK) { // монтируете файловую систему по пути SDPath, проверяете, что она смонтировалась, только при этом условии начинаете с ней работать
		res1 = f_open(&File1, (char*)path1, FA_WRITE | FA_OPEN_APPEND); // открытие файла, обязательно для работы с ним
		needs_mount = needs_mount || res1 != FR_OK;
		f_puts("num; time_ms; accl1; accl2; accl3; gyro1; gyro2; gyro3; mag1; mag2; mag3; bme_temp; bme_press; bme_humidity; bme_height; lux_board; lux_sp; state; lidar;\n", &File1);
		res1 = f_sync(&File1);
		needs_mount = needs_mount || res1 != FR_OK;
	}
	if(is_mount == FR_OK) { // монтируете файловую систему по пути SDPath, проверяете, что она смонтировалась, только при этом условии начинаете с ней работать
		res2 = f_open(&File2, (char*)path2, FA_WRITE | FA_OPEN_APPEND); // открытие файла, обязательно для работы с ним
		needs_mount = needs_mount || res2 != FR_OK;
		f_puts("num; time_ms; fix; lat; lon; alt; gps_time_s; gps_time_s; current; bus_voltage; MICS_5524; MICS_CO; MICS_NO2; MICS_NH3; CCS_CO2; CCS_TVOC; bme_temp_g; bme_press_g; bme_humidity_g\n", &File2);
		res2 = f_sync(&File2);
		needs_mount = needs_mount || res2 != FR_OK;
	}
	if(is_mount == FR_OK){
		resb = f_open(&Fileb, pathb, FA_WRITE | FA_OPEN_APPEND);
		needs_mount = needs_mount || resb != FR_OK;
	}





	double bmp_temp;
	double bmp_press;
	double bmp_humidity;
	float acc_g[3] = {0};
	float gyro_dps[3] = {0};
	float mag[3] = {0};
	float temperature_celsius_mag = 0.0;
	float temperature_celsius_gyro = 0.0;
	uint16_t num1 = 0 ;
	uint16_t crc = 0;

//сдвиговый регистр
	shift_reg_t shift_reg_r;
	shift_reg_r.bus = &hspi1;
	shift_reg_r.latch_port = GPIOB;
	shift_reg_r.latch_pin = GPIO_PIN_14;
	shift_reg_r.oe_port = GPIOA;
	shift_reg_r.oe_pin = GPIO_PIN_8;
	shift_reg_r.value = 0;
	shift_reg_init(&shift_reg_r);
	shift_reg_write_8(&shift_reg_r, 0x00);
	shift_reg_write_bit_8(&shift_reg_r, 7, 0);

//стх и структура лcмa
	stmdev_ctx_t ctx_lsm;
	struct lsm_spi_intf_sr lsm_sr;
	lsm_sr.sr_pin = 6;
	lsm_sr.spi = &hspi1;
	lsm_sr.sr = &shift_reg_r;
	lsmset_sr(&ctx_lsm, &lsm_sr);

//стх и структура лиса
	stmdev_ctx_t ctx_lis;
	struct lis_spi_intf_sr lis_sr;
	lis_sr.sr_pin = 2;
	lis_sr.spi = &hspi1;
	lis_sr.sr = &shift_reg_r;
	lisset_sr(&ctx_lis, &lis_sr);



	//bme280
	bme_important_shit_t bme_shit;
	its_bme280_init(UNKNOWN_BME);

	//настройка радио
	nrf24_spi_pins_t nrf_pins;
	nrf_pins.ce_port = GPIOC;
	nrf_pins.cs_port = GPIOC;
	nrf_pins.ce_pin = GPIO_PIN_13;
	nrf_pins.cs_pin = GPIO_PIN_14;
	nrf24_lower_api_config_t nrf24;
	nrf24_spi_init(&nrf24, &hspi4, &nrf_pins);

	//printf("before setup\n");
	//nrf_dump_regs(&nrf24);

	nrf24_mode_power_down(&nrf24);
	nrf24_rf_config_t nrf_config;
	nrf_config.data_rate = NRF24_DATARATE_250_KBIT;
	nrf_config.tx_power = NRF24_TXPOWER_MINUS_0_DBM;
	nrf_config.rf_channel = 10;		//101;
	nrf24_setup_rf(&nrf24, &nrf_config);
	nrf24_protocol_config_t nrf_protocol_config;
	nrf_protocol_config.crc_size = NRF24_CRCSIZE_1BYTE;
	nrf_protocol_config.address_width = NRF24_ADDRES_WIDTH_5_BYTES;
	nrf_protocol_config.en_dyn_payload_size = true;
	nrf_protocol_config.en_ack_payload = false;
	nrf_protocol_config.en_dyn_ack = false;
	nrf_protocol_config.auto_retransmit_count = 0;
	nrf_protocol_config.auto_retransmit_delay = 0;
	nrf24_setup_protocol(&nrf24, &nrf_protocol_config);
	nrf24_pipe_set_tx_addr(&nrf24, 0xacacacacac);

	nrf24_pipe_config_t pipe_config;
	for (int i = 1; i < 6; i++)
	{
		pipe_config.address = 0xacacacacac;
		pipe_config.address = (pipe_config.address & ~((uint64_t)0xff << 32)) | ((uint64_t)(i + 7) << 32);
		pipe_config.enable_auto_ack = false;
		pipe_config.payload_size = -1;
		nrf24_pipe_rx_start(&nrf24, i, &pipe_config);
	}

	pipe_config.address = 0xafafafaf01;
	pipe_config.enable_auto_ack = false;
	pipe_config.payload_size = -1;
	nrf24_pipe_rx_start(&nrf24, 0, &pipe_config);

	nrf24_mode_standby(&nrf24);
	printf("\n\n");
	printf("after setup\n");
	//nrf_dump_regs(&nrf24);
	printf("\n\n");
	nrf24_mode_tx(&nrf24);

	int16_t magg[3];
	int16_t gyro[3];
	int16_t acc_raw[3];
	pack1_t pack1 = {0};
	pack1.flag = 0xAA;

int a;
	while(1){

		its_bme280_read(UNKNOWN_BME, &bme_shit);
		bmp_temp = bme_shit.temperature;
		bmp_press = bme_shit.pressure;
		bmp_humidity = bme_shit.humidity;

		lsmread(&ctx_lsm, &temperature_celsius_gyro, &acc_g, &gyro_dps);
		lisread(&ctx_lis, &temperature_celsius_mag, &mag);

		lsm6ds3_acceleration_raw_get(&ctx_lsm, acc_raw);
		lsm6ds3_angular_rate_raw_get(&ctx_lsm, gyro);
		lis3mdl_magnetic_raw_get(&ctx_lis, magg);


		for (int i = 0; i < 3; i++){
			pack1.accl[i] = acc_raw[i];
			pack1.gyro[i] = gyro[i];
			pack1.mag[i] = magg[i];
		}

			num1 += 1;
			pack1.num = num1;
			pack1.time_ms = HAL_GetTick();
			pack1.crc = 0;

			nrf24_fifo_write(&nrf24, (uint8_t *)&pack1, sizeof(pack1), false);
			nrf24_irq_get(&nrf24,&a);
			//nrf24_fifo_flush_tx(&nrf24);


	}

	return 0;

}
