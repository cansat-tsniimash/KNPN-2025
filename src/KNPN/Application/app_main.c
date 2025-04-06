/*
 * app_main.c
 *
 *  Created on: Oct 12, 2024
 *      Author: Install
 */

#include <stdlib.h>
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
#include "AD5593/ad5593.h"
#include "CCS811.h"

extern SPI_HandleTypeDef hspi1;
//extern SPI_HandleTypeDef hspi4;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;


typedef enum
{
	SD_PACK_1,
	SD_PACK_2,
	SD_WAIT,
}sd_state_t;

uint16_t Crc16(uint8_t *buf, uint16_t len) {
	uint16_t crc = 0xFFFF;
	while (len--) {
		crc ^= *buf++ << 8;
		for (uint8_t i = 0; i < 8; i++)
			crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
	}
	return crc;
}

FRESULT mount_again(FIL *File1, FIL *File2, FIL *Fileb, FATFS *fileSystem, const char *pack1, const char *pack2, const char *pathb){
	FRESULT is_mount;
	f_mount(0, "0", 1);
	extern Disk_drvTypeDef disk;
	disk.is_initialized[0] = 0;
	is_mount = f_mount(fileSystem, "", 1);
	f_open(File1, (char*)pack1, FA_WRITE | FA_OPEN_APPEND);
	f_open(File2, (char*)pack2, FA_WRITE | FA_OPEN_APPEND);
	f_open(Fileb, (char*)pathb, FA_WRITE | FA_OPEN_APPEND);
	return is_mount;
}

FATFS fileSystem; // переменная типа FATFS
FIL File_1csv; // хендлер файла
FIL File_2csv; // хендлер файла
FIL File_b; // хендлер файла
const char path1[] = "packet1.csv\0";
const char path2[] = "packet2.csv\0";
const char pathb[] = "packetb.bin\0";

FRESULT is_mount = 0;
int needs_mount = 0;


FRESULT res1csv; // результат выполнения функции
FRESULT res2csv; // результат выполнения функции
FRESULT resb; // результат выполнения функции
char str_buf[300];

uint16_t sd_parse_to_bytes_pac1(char *buffer, pack1_t *pack1) {
    memset(buffer, 0, 300);
    uint16_t num_written = snprintf(
            buffer, 300,
			"%d;%d;%d;%d;%d;%ld;%d;%d;%d;%d;%d;%d;%d;%ld;%d;%f;%f;%f;%d;%d;%d\n",
			pack1->flag,pack1->num,pack1->accl[0],pack1->accl[1],pack1->accl[2],pack1->time_ms,
			pack1->gyro[0],pack1->gyro[1],pack1->gyro[2],pack1->mag[0],
			pack1->mag[1],pack1->mag[2],pack1->bme_temp,pack1->bme_press,pack1->bme_humidity,pack1->bme_height,
			pack1->lux_board,pack1->lux_sp,pack1->state,pack1->lidar,pack1->crc);
    return num_written;
}
uint16_t sd_parse_to_bytes_pac2(char *buffer, pack2_t *pack2) {
    memset(buffer, 0, 300);
    uint16_t num_written = snprintf(
            buffer, 300,
            "%d;%d;%ld;%d;%f;%f;%f;%ld;4%f;%d;%d;%d;%d;%d;%d;%d;%d;%ld;%d;%d\n",
			pack2->flag,pack2->num,pack2->time_ms,pack2->fix,pack2->lat,
			pack2->lon,pack2->alt,pack2->gps_time_s,pack2->current,pack2->bus_voltage,
			pack2->MICS_5524,pack2->MICS_CO,pack2->MICS_NO2,pack2->MICS_NH3,pack2->CCS_CO2,
			pack2->CCS_TVOC,pack2->bme_temp_g,pack2->bme_press_g,pack2->bme_humidity_g,pack2->crc);
    return num_written;
}

static ad5593_t adc;

static int setup_adc()
{
	ad5593_t * dev = &adc;
	int rc = ad5593_ctor(dev, AD5593_ADDR_A0_HIGH, &hi2c1);
	if (0 != rc)
	{
		perror("unable to ctor");
		return EXIT_FAILURE;
	}

	HAL_Delay(1);

	// Включаем питание на чем хотим
	printf("power config\n");
	rc = ad5593_power_config(dev, AD5593_POWER_DAC_REF, 0x00);
	if (0 != rc)
	{
		perror("unable to power config");
		return EXIT_FAILURE;
	}

	// Аналоговая конфигурация
	printf("analog config\n");
	ad5593_an_config_t an_config;
	an_config.adc_buffer_enable = false;
	an_config.adc_buffer_precharge = false;
	an_config.adc_range = AD5593_RANGE_1REF;
	rc = ad5593_an_config(dev, &an_config);
	if (0 != rc)
	{
		perror("unable to an config");
		return EXIT_FAILURE;
	}

	// Настройка пинов
	printf("pin config\n");
	rc = ad5593_pin_config(dev, 0xFF, AD5593_PINMODE_ADC);
	if (0 != rc)
	{
		perror("unable to setup pins");
		return EXIT_FAILURE;
	}

	return 0;
}



	float foto_sp;
	float foto_state;
	uint16_t R_MICS_5524;
	uint16_t R_MICS_CO;
	uint16_t R_MICS_NO2;
	uint16_t R_MICS_NH3;


static void test_adc()
{
	ad5593_t * dev = &adc;

	ad5593_channel_id_t channels[] = {
			AD5593_ADC_0, AD5593_ADC_1, AD5593_ADC_2, AD5593_ADC_3,
			AD5593_ADC_4, AD5593_ADC_5,

	};
		ad5593_channel_id_t channel_0 = channels[0];
		ad5593_channel_id_t channel_1 = channels[1];
		ad5593_channel_id_t channel_2 = channels[2];
		ad5593_channel_id_t channel_3 = channels[3];
		ad5593_channel_id_t channel_4 = channels[4];
		ad5593_channel_id_t channel_5 = channels[5];

		ad5593_adc_read(dev, channel_0, &R_MICS_5524);
		ad5593_adc_read(dev, channel_1, &R_MICS_CO);
		ad5593_adc_read(dev, channel_2, &R_MICS_NO2);
		ad5593_adc_read(dev, channel_3, &R_MICS_NH3);
		ad5593_adc_read(dev, channel_4, (uint16_t*)&foto_sp);
		ad5593_adc_read(dev, channel_5, (uint16_t*)&foto_state);

		float volts_sp = foto_sp * 3.3 / 4095;  //Volts
		float ohms_sp = volts_sp*(3300)/(3.3-volts_sp);    //Ohms
		float lux_sp = exp((3.823-log(ohms_sp/1000))/0.816)*10.764;

		float volts_state = foto_state * 3.3 / 4095;  //Volts
		float ohms_state = volts_state*(3300)/(3.3-volts_state);    //Ohms
		float lux_state = exp((3.823-log(ohms_state/1000))/0.816)*10.764;

		foto_sp = lux_sp;
		foto_state = lux_state;

}



int app_main(){

//файлы
	UINT Bytes = 0;

	memset(&fileSystem, 0x00, sizeof(fileSystem));

	extern Disk_drvTypeDef disk;
	disk.is_initialized[0] = 0;
	is_mount = f_mount(&fileSystem, "", 1);

	if(is_mount == FR_OK) { // монтируете файловую систему по пути SDPath, проверяете, что она смонтировалась, только при этом условии начинаете с ней работать
		res1csv = f_open(&File_1csv, (char*)path1, FA_WRITE | FA_OPEN_APPEND); // открытие файла, обязательно для работы с ним
		needs_mount = needs_mount || res1csv != FR_OK;

		int res1csv2 = f_puts("num; time_ms; accl1; accl2; accl3; gyro1; gyro2; gyro3; mag1; mag2; mag3; bme_temp; bme_press; bme_humidity; bme_height; lux_board; lux_sp; state; lidar\n", &File_1csv);
		res1csv = f_sync(&File_1csv);
		needs_mount = needs_mount || res1csv != FR_OK;
	}
	if(is_mount == FR_OK) { // монтируете файловую систему по пути SDPath, проверяете, что она смонтировалась, только при этом условии начинаете с ней работать
		res2csv = f_open(&File_2csv, (char*)path2, FA_WRITE | FA_OPEN_APPEND); // открытие файла, обязательно для работы с ним
		needs_mount = needs_mount || res2csv != FR_OK;
		int res2csv2 = f_puts("num; time_ms; fix; lat; lon; alt; gps_time_s; gps_time_s; current; bus_voltage; MICS_5524; MICS_CO; MICS_NO2; MICS_NH3; CCS_CO2; CCS_TVOC; bme_temp_g; bme_press_g; bme_humidity_g\n", &File_2csv);
		res2csv = f_sync(&File_2csv);
		needs_mount = needs_mount || res2csv != FR_OK;
	}
	if(is_mount == FR_OK){
		resb = f_open(&File_b, pathb, FA_WRITE | FA_OPEN_APPEND);
		needs_mount = needs_mount || resb != FR_OK;
	}
/*
	f_close(&File_b);
	f_close(&File_1csv);
	f_close(&File_2csv);
*/




	sd_state_t sd_state = SD_PACK_1;
	double bmp_temp;
	double bmp_press;
	double bmp_humidity;
	float height;
	float acc_g[3] = {0};
	float gyro_dps[3] = {0};
	float mag[3] = {0};
	float temperature_celsius_mag = 0.0;
	float temperature_celsius_gyro = 0.0;
	float lat;
	float lon;
	float alt;
	float lats;
	float lons;
	float alts;
	int64_t cookie;
	int fix;
	uint64_t gps_time_s;
	uint32_t gps_time_us;
	uint16_t num1 = 0 ;
	uint16_t num2 = 0 ;
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



	// Инициализация ad5593
	setup_adc();

	int16_t magg[3];
	int16_t gyro[3];
	int16_t acc_raw[3];

	pack1_t pack1 = {0};
	pack2_t pack2 = {0};
	gps_init();
	gps_work();

	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);

	int a = 0;

	uint16_t num_written;

/*
	//uint Bytes;
	//char message[] = ".|.";
	//uint8_t settings[] = {0xC0, 0x04, 0x01, 0x17};
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, SET);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, SET);
	//HAL_UART_Transmit(&huart1, settings, sizeof(settings), HAL_MAX_DELAY);
	//HAL_UART_Receive(&huart1, settings, 3, HAL_MAX_DELAY);

	//uint8_t settings1[] = {0xC1, 0x00, 0x08};
	//uint8_t result[11] = {0};
	//HAL_UART_Transmit(&huart1, settings1, sizeof(settings1), HAL_MAX_DELAY);
	//HAL_UART_Receive(&huart1, result,  sizeof(result), HAL_MAX_DELAY);
*/



	//test_adc();

	uint16_t co2, tvoc;

	CCS811_Init();

	while(1){

		if(is_mount != FR_OK) {

			f_mount(0, "0", 1);
			extern Disk_drvTypeDef disk;
			disk.is_initialized[0] = 0;
			is_mount = f_mount(&fileSystem, "", 1);

			test_adc();

			f_open(&File_1csv, (char*)path1, FA_WRITE | FA_OPEN_APPEND); // открытие файла
			f_puts("num; time_ms; accl1; accl2; accl3; gyro1; gyro2; gyro3; mag1; mag2; mag3; bme_temp; bme_press; bme_humidity; bme_height; lux_board; lux_sp; state; lidar\n", &File_1csv);
			f_open(&File_2csv, (char*)path2, FA_WRITE | FA_OPEN_APPEND); // открытие файла
			f_puts("num; time_ms; fix; lat; lon; alt; gps_time_s; gps_time_s; current; bus_voltage; MICS_5524; MICS_CO; MICS_NO2; MICS_NH3; CCS_CO2; CCS_TVOC; bme_temp_g; bme_press_g; bme_humidity_g\n", &File_2csv);
			f_open(&File_b, pathb, FA_WRITE | FA_OPEN_APPEND); // открытие файла
		}

		its_bme280_read(UNKNOWN_BME, &bme_shit);
		bmp_temp = bme_shit.temperature;
		bmp_press = bme_shit.pressure;
		bmp_humidity = bme_shit.humidity;
		double ground_pressure = bme_shit.pressure;
		height = 44330 * (1 - pow(bmp_press / ground_pressure, 1.0 / 5.255));

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

		//gps
		gps_work();
		gps_get_coords(&cookie, &lat, &lon, &alt, &fix);
		gps_get_time(&cookie, &gps_time_s, &gps_time_us);


		CCS811_SetEnvironmentalData(45.0f, 24.0f); // 45% RH, 24°C

		if (CCS811_DataAvailable())
	    {
		    CCS811_ReadAlgorithmResults(&co2, &tvoc);
	    }
			num1 += 1;

			pack1.num = num1;
			pack1.time_ms = HAL_GetTick();
			pack1.flag = 0xAA;
			pack1.bme_height = height;
			pack1.bme_humidity = bmp_humidity;
			pack1.bme_press = bmp_press;
			pack1.bme_temp = bmp_temp;
			pack1.lidar = 666;
			pack1.lux_board = foto_state;
			pack1.lux_sp = foto_sp;
			pack1.state = 0;
			pack1.crc = Crc16((uint8_t *)&pack1, sizeof(pack1) - 2);

			num2 += 1;

			pack2.num = num2;
			pack2.time_ms = HAL_GetTick();
			pack2.flag = 0xBB;
			pack2.MICS_CO = R_MICS_CO;
			pack2.MICS_NH3 = R_MICS_NH3;
			pack2.MICS_NO2 = R_MICS_NO2;
			pack2.MICS_5524 = R_MICS_5524;
			pack2.CCS_TVOC = 666;
			pack2.CCS_CO2 = 666;
			pack2.lat = 666;
			pack2.lon = 666;
			pack2.alt = 666;
			pack2.gps_time_s = 666;
			pack2.fix = 666;
			pack2.bme_humidity_g = 666;
			pack2.bme_press_g = 666;
			pack2.bme_temp_g = 666;
			pack2.bus_voltage = 666;
			pack2.current = 666;
			pack2.crc = Crc16((uint8_t *)&pack2, sizeof(pack2) - 2);

			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, RESET);

			int size_pack1 = sizeof(pack1);
			int size_pack2 = sizeof(pack2);

			HAL_UART_Transmit(&huart1, (uint8_t*)&pack1, sizeof(pack1), HAL_MAX_DELAY);
			HAL_Delay(100);
			HAL_UART_Transmit(&huart1, (uint8_t*)&pack2, sizeof(pack2), HAL_MAX_DELAY);
			HAL_Delay(100);

			uint8_t lidar[18];
			HAL_UART_Receive(&huart6, lidar, 18, 100);

			for(int i = 0; i < 9; i++){
				if ((lidar[0] != 0x59) || (lidar[1] != 0x59)){
					for(int j = 0; j < 17; j++){
						lidar[j] = lidar[j + 1];
					}
				}
			}

			uint16_t lidar_1 = (lidar[3] << 8) | lidar[2];

			//uint16_t crc_lidar = lidar[0] + lidar[0] + lidar[1] + lidar[0] + lidar[1] + lidar[2] + lidar[0] + lidar[1] + lidar[2] + lidar[3] + lidar[0] + lidar[1] + lidar[2] + lidar[3] + lidar[4] + lidar[0] + lidar[1] + lidar[2] + lidar[3] + lidar[4] + lidar[5] + lidar[0] + lidar[1] + lidar[2] + lidar[3] + lidar[4] + lidar[5] + lidar[6] + lidar[0] + lidar[1] + lidar[2] + lidar[3] + lidar[4] + lidar[5] + lidar[6] + lidar[7];
			//uint16_t crc_lidar_8 = crc_lidar & 0x00FF;



			switch(sd_state) {
				case SD_PACK_1:


					if (res1csv == FR_OK){
						num_written = sd_parse_to_bytes_pac1(str_buf, &pack1);

						res1csv = f_write(&File_1csv,str_buf,num_written, &Bytes); // отправка на запись в файл
						res1csv = f_sync(&File_1csv); // запись в файл (на sd контроллер пишет не сразу, а по закрытии файла. Также можно использовать эту команду)
					}
					if (resb == FR_OK){
						resb = f_write(&File_b,(uint8_t *)&pack1,sizeof(pack1), &Bytes); // отправка на запись в файл
						resb = f_sync(&File_b); // запись в файл (на sd контроллер пишет не сразу, а по закрытии файла. Также можно использовать эту команду)
					}
					a++;
					sd_state = SD_WAIT;
				break;

				case SD_PACK_2:


					if (res2csv == FR_OK){
						num_written = sd_parse_to_bytes_pac2(str_buf, &pack2);

						res2csv = f_write(&File_2csv,str_buf,num_written, &Bytes); // отправка на запись в файл
						res2csv = f_sync(&File_2csv); // запись в файл (на sd контроллер пишет не сразу, а по закрытии файла. Также можно использовать эту команду)
					}
					if (resb == FR_OK){
						resb = f_write(&File_b,(uint8_t *)&pack2,sizeof(pack2), &Bytes); // отправка на запись в файл
						resb = f_sync(&File_b); // запись в файл (на sd контроллер пишет не сразу, а по закрытии файла. Также можно использовать эту команду)
					}

					sd_state = SD_WAIT;
					a=0;
				break;

				case SD_WAIT:
					if(a == 4)
						sd_state = SD_PACK_2;
					else
						sd_state = SD_PACK_1;

				break;







	}

			//nrf24_fifo_write(&nrf24, (uint8_t *)&pack1, sizeof(pack1), false);
			//nrf24_irq_get(&nrf24,&a);
			//nrf24_fifo_flush_tx(&nrf24);
	}
	return 0;
}

