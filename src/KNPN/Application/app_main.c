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
#include "cyclebuffer.h"
#include <math.h>

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

typedef enum
{
	STATE_READY,
	STATE_IN_ROCKET ,
	PARACHUTE_DESCENT,
	FALL,
	LANDING,
	STATE_OPEN_SP,
	SUN_SEARCH,
	ENERGY,
} state_t;

state_t state_now;


typedef struct INA219_DATA{
	uint16_t power;
	uint16_t current;
	uint16_t voltage;
	uint16_t shunt_voltage;
}INA219_DATA;

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

cbuffer_t lidar_buff;

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
			"%d;%d;%ld;%d;%d;%ld;%d;%d;%d;%d;%d;%d;%d;%ld;%d;%f;%f;%f;%d;%d;%d\n",
			pack1->flag,pack1->num,pack1->time_ms,pack1->accl[0],pack1->accl[1],pack1->accl[2],
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
	int rc = ad5593_ctor(dev, AD5593_ADDR_A0_LOW, &hi2c1);
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
	an_config.adc_range = AD5593_RANGE_2REF;
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

		ad5593_adc_read(dev, channel_5, &R_MICS_5524);
		R_MICS_5524 = (((float)R_MICS_5524 * 5)/0xFFF) *1000;
		ad5593_adc_read(dev, channel_3, &R_MICS_CO);
		R_MICS_CO = (((float)R_MICS_CO * 5)/0xFFF) *1000;
		ad5593_adc_read(dev, channel_4, &R_MICS_NO2);
		R_MICS_NO2 = (((float)R_MICS_NO2 * 5)/0xFFF) *1000;
		ad5593_adc_read(dev, channel_2, &R_MICS_NH3);
		R_MICS_NH3 = (((float)R_MICS_NH3 * 5)/0xFFF) *1000;

		uint16_t foto_sp_raw;
		uint16_t foto_state_raw;
		ad5593_adc_read(dev, channel_1, &foto_sp_raw);
		foto_sp_raw = (((float)foto_sp_raw * 5)/0xFFF) *1000;

		ad5593_adc_read(dev, channel_0, &foto_state_raw);
		foto_state_raw = (((float)foto_state_raw * 5)/0xFFF) *1000;

		foto_sp = foto_sp_raw;
		foto_state = foto_state_raw;

		float volts_sp = foto_sp * 3.3 / 4095;  //Volts
		float ohms_sp = volts_sp*(3300)/(3.3-volts_sp);    //Ohms
		float lux_sp = exp((3.823-log(ohms_sp/1000))/0.816)*10.764;

		float volts_state = foto_state * 3.3 / 4095;  //Volts
		float ohms_state = volts_state*(3300)/(3.3-volts_state);    //Ohms
		float lux_state = exp((3.823-log(ohms_state/1000))/0.816)*10.764;

		foto_sp = lux_sp;
		foto_state = lux_state;

}



// Состояния алгоритма наведения солнечной панели
typedef enum {
    IDLE,               // Ожидание старта
    HORIZ_90_START,     // Начало поворота по горизонтали на 90°
    HORIZ_90_WAIT,      // Ожидание завершения поворота на 90°
    VERT_SCAN_START,    // Начало вертикального сканирования
    VERT_SCAN_WAIT,     // Ожидание завершения вертикального сканирования и поиск макс. освещённости
    VERT_SCAN_BACK,     // Возврат в точку макс. освещённости по вертикали
    HORIZ_SCAN_START,   // Начало горизонтального сканирования
    HORIZ_SCAN_WAIT,    // Ожидание завершения горизонтального сканирования и поиск макс. освещённости
    HORIZ_SCAN_BACK,    // Возврат в точку макс. освещённости по горизонтали
    COMPLETE            // Процесс завершён
} SunTrackState;

// Текущее состояние FSM
SunTrackState state = IDLE;

// Время начала текущего состояния
uint32_t state_start_time = 0;
// Время (в пределах шага), когда был зафиксирован максимум света
uint32_t max_light_time = 0;
// Значение максимальной освещённости, обнаруженной фоторезистором
float max_light_value = 0;

int flag_paneli = 0;

shift_reg_t shift;

// Главная функция FSM для запуска процесса наведения
void SunTrack_Run(shift_reg_t *shift) {

    uint32_t now = HAL_GetTick(); // Получаем текущее время

    switch (state) {

        case IDLE:
            // Начинаем процесс наведения
            state = HORIZ_90_START;
            break;

        case HORIZ_90_START:
            shift_reg_write_bit_16(shift, 6, 1); // Вращаем по горизонтали вперёд
            state_start_time = now;
            state = HORIZ_90_WAIT;
            break;

        case HORIZ_90_WAIT:
            if (now - state_start_time >= 750) { // Ожидаем 0.75 секунды (90°)
                shift_reg_write_bit_16(shift, 6, 0); // Останавливаем мотор
                state = VERT_SCAN_START;
            }
            break;

        case VERT_SCAN_START:
            shift_reg_write_bit_16(shift, 4, 1); // Вращаем по вертикали вперёд
            state_start_time = now;
            max_light_value = 0;
            max_light_time = 0;
            state = VERT_SCAN_WAIT;
            break;

        case VERT_SCAN_WAIT:
            // Обновляем максимум, если текущее значение выше
            if (foto_sp > max_light_value) {
                max_light_value = foto_sp;
                max_light_time = now - state_start_time;
            }
            // Если прошло 11,7 секунд — полный оборот
            if (now - state_start_time >= 10450) {
                shift_reg_write_bit_16(shift, 4, 0);
                state_start_time = now;
                shift_reg_write_bit_16(shift, 5, 1); // Вращаем обратно
                state = VERT_SCAN_BACK;
            }
            break;

        case VERT_SCAN_BACK:
            // Ждём столько, сколько потребовалось, чтобы найти максимум
            if (now - state_start_time >= 10450 - max_light_time) {
                shift_reg_write_bit_16(shift, 5, 0);
                state = HORIZ_SCAN_START;
            }
            break;

        case HORIZ_SCAN_START:
        	if(flag_paneli == 0){
        		shift_reg_write_bit_16(shift, 7, 1); // Вращаем обратно
        		state_start_time = now;
                flag_paneli = 2;
        	}
            if (now - state_start_time >= 750) { // Ожидаем 0.75 секунды (90°)
            	shift_reg_write_bit_16(shift, 7, 0); // Останавливаем мотор
                flag_paneli = 1;
            }
            if(flag_paneli == 1){
            	shift_reg_write_bit_16(shift, 6, 1); // Вращаем по горизонтали вперёд
            	state_start_time = now;
            	max_light_value = 0;
            	max_light_time = 0;
            	state = HORIZ_SCAN_WAIT;
            }

            break;

        case HORIZ_SCAN_WAIT:
            if (foto_sp > max_light_value) {
                max_light_value = foto_sp;
                max_light_time = now - state_start_time;
            }
            if (now - state_start_time >= 3000) { // Полный оборот по горизонтали
                shift_reg_write_bit_16(shift, 6, 0);
                state_start_time = now;
                shift_reg_write_bit_16(shift, 7, 1); // Вращаем обратно
                state = HORIZ_SCAN_BACK;
            }
            break;

        case HORIZ_SCAN_BACK:
            if (now - state_start_time >= 3000 - max_light_time) {
                shift_reg_write_bit_16(shift, 7, 0);
                state = COMPLETE;
            }
            break;

        case COMPLETE:
        	state_now = ENERGY;
            break;
    }
}

typedef enum {
	EMPTY,
	CALCULATING_ANGLE,
	SKY_MOTORS_ON,
	SKY_MOTORS_OFF,
	GROUND_MOTOR_ON,
	GROUND_MOTOR_OFF,
	COMPLETE_BLOOM,
}OPENING_PETALS;

OPENING_PETALS state_bloom = EMPTY;

const int id_motor[] = {3+8, 2+8, 1+8, 7+8, 6+8, 5+8, 4+8, 0+8};

int target_petals[4] = {0, 1, 2, 3};
int not_target_petals[4] = {0, 1, 2, 3};

uint32_t bloom_start_time = 0;

int min = 0;

int count_angle(int x, int y){
	int v_cos = x / (sqrt((x*x)+(y*y)));
	int v_acos = acos(v_cos);

	if(y < 0){
		v_acos = 2*M_PI - v_acos;
	}
	return v_acos / M_PI * 180;
}

int bloom(int x, int y,shift_reg_t *shift){

	int min = 0;

	uint32_t now = HAL_GetTick(); // Получаем текущее время

	switch(state_bloom){

		case EMPTY:

			state_bloom = CALCULATING_ANGLE;
			break;

		case CALCULATING_ANGLE:

			for(int i = 1; i < 8; i++){
					int dif = abs(count_angle(x,y) - (35 + 45 * i));
					int dif_min = abs(count_angle(x,y) - (35 + 45 * min));

					if(dif < dif_min){
						min = i;
					}
				}

			target_petals[0] = min + 2;
			target_petals[1] = min + 1;
			target_petals[2] = min;
			target_petals[0] = min - 1;

			not_target_petals[0] = min + 4;
			not_target_petals[1] = min + 3;
			not_target_petals[2] = min - 2;
			not_target_petals[3] = min - 3;

			for(int i = 0; i < 4; i++){
				if(target_petals[i] < 0)
					target_petals[i] += 8;
				if(target_petals[i] > 7)
					target_petals[i] -= 8;
				if(not_target_petals[i] < 0)
					not_target_petals[i] += 8;
				if(not_target_petals[i] > 7)
					not_target_petals[i] -= 8;
			}

			state_bloom = SKY_MOTORS_ON;
			break;

		case SKY_MOTORS_ON:

			for(int i = 0; i < 4; i++){
				int motor = target_petals[i];
				shift_reg_write_bit_16(shift, id_motor[motor], 1);
			}

			bloom_start_time = now;

			state_bloom = SKY_MOTORS_OFF;
			break;

		case SKY_MOTORS_OFF:

			if (now - bloom_start_time >= 3000){
				for(int i = 0; i < 4; i++){
					int motor = target_petals[i];
					shift_reg_write_bit_16(shift, id_motor[motor], 0);
				}
				state_bloom = GROUND_MOTOR_ON;
			}
			break;

		case GROUND_MOTOR_ON:

			for(int i = 0; i < 4; i++){
				int motor = not_target_petals[i];
				shift_reg_write_bit_16(shift, id_motor[motor], 1);
			}

			bloom_start_time = now;

			state_bloom = GROUND_MOTOR_OFF;
			break;

		case GROUND_MOTOR_OFF:

			if (now - bloom_start_time >= 3000){
				for(int i = 0; i < 4; i++){
					int motor = not_target_petals[i];
					shift_reg_write_bit_16(shift, id_motor[motor], 0);
				}
				state_bloom = COMPLETE_BLOOM;
			}

			break;

		case COMPLETE_BLOOM:
			state_now = STATE_OPEN_SP;
			break;
	}

}


void accept_lidar_byte(uint8_t byte){
	sbuffer_push(&lidar_buff, byte);
}



int app_main(){

	uint32_t start;
	uint32_t stop;
//файлы
	UINT Bytes = 0;

	memset(&fileSystem, 0x00, sizeof(fileSystem));

	sbuffer_init(&lidar_buff);

	extern Disk_drvTypeDef disk;
	disk.is_initialized[0] = 0;
	is_mount = f_mount(&fileSystem, "", 1);

	if(is_mount == FR_OK) { // монтируете файловую систему по пути SDPath, проверяете, что она смонтировалась, только при этом условии начинаете с ней работать
		res1csv = f_open(&File_1csv, (char*)path1, FA_WRITE | FA_OPEN_APPEND); // открытие файла, обязательно для работы с ним
		needs_mount = needs_mount || res1csv != FR_OK;

		int res1csv2 = f_puts("flag; num; time_ms; accl1; accl2; accl3; gyro1; gyro2; gyro3; mag1; mag2; mag3; bme_temp; bme_press; bme_humidity; bme_height; lux_board; lux_sp; state; lidar; crc\n", &File_1csv);
		res1csv = f_sync(&File_1csv);
		needs_mount = needs_mount || res1csv != FR_OK;
	}
	if(is_mount == FR_OK) { // монтируете файловую систему по пути SDPath, проверяете, что она смонтировалась, только при этом условии начинаете с ней работать
		res2csv = f_open(&File_2csv, (char*)path2, FA_WRITE | FA_OPEN_APPEND); // открытие файла, обязательно для работы с ним
		needs_mount = needs_mount || res2csv != FR_OK;
		int res2csv2 = f_puts("flag; num; time_ms; fix; lat; lon; alt; gps_time_s; current; bus_voltage; MICS_5524; MICS_CO; MICS_NO2; MICS_NH3; CCS_CO2; CCS_TVOC; bme_temp_g; bme_press_g; bme_humidity_g; crc\n", &File_2csv);
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
	int64_t cookie;
	int fix;
	uint64_t gps_time_s;
	uint32_t gps_time_us;
	float current;
	float bus_voltage;
	struct ina219_t ina219;




//сдвиговый регистр
	shift_reg_t shift_reg_r;
	shift_reg_r.bus = &hspi1;
	shift_reg_r.latch_port = GPIOB;
	shift_reg_r.latch_pin = GPIO_PIN_14;
	shift_reg_r.oe_port = GPIOA;
	shift_reg_r.oe_pin = GPIO_PIN_8;
	shift_reg_r.value = 0;
	shift_reg_init(&shift_reg_r);
	shift_reg_write_16(&shift_reg_r, 0x0000);
	shift_reg_write_bit_16(&shift_reg_r, 1, 0);
	shift_reg_oe(&shift_reg_r, 0);

//стх и структура лcмa
	stmdev_ctx_t ctx_lsm;
	struct lsm_spi_intf lsm;
	lsm.spi = &hspi1;
	lsm.GPIO_Port = GPIOA;
	lsm.GPIO_Pin = GPIO_PIN_9;
	lsmset(&ctx_lsm, &lsm);


//стх и структура лиса
	stmdev_ctx_t ctx_lis;
	struct lis_spi_intf lis;
	lis.spi = &hspi1;
	lis.GPIO_Port = GPIOA;
	lis.GPIO_Pin = GPIO_PIN_10;
	lisset(&ctx_lis, &lis);



	//bme280
	bme_important_shit_t bme_shit;
	its_bme280_init(UNKNOWN_BME1);

	bme_important_shit_t bme2_shit;
	its_bme280_init(UNKNOWN_BME2);

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

	__HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_ERR);

	int a = 0;

	uint16_t num_written;



	//uint8_t settings_uartrate[] = {0xC0, 0x02, 0x01, 0x62};
	//uint8_t settings_speed[] = {0xC0, 0x02, 0x01, 0x65};
	//uint8_t settings_channel[] = {0xC0, 0x04, 0x01, 0x17};
	uint8_t settings_channel[] = {0xC0, 0x04, 0x01, 0x28};
	uint8_t settings_speed_uartrate[] = {0xC0, 0x02, 0x01, 0xA2};//A5

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, SET);

	uint8_t channel_answer[2] = {0};
	HAL_UART_Transmit(&huart1, settings_channel, sizeof(settings_channel), 100);
	//HAL_UART_Receive(&huart1, channel_answer,  sizeof(channel_answer), 100);
	HAL_Delay(1000);
	HAL_UART_Transmit(&huart1, settings_speed_uartrate, sizeof(settings_speed_uartrate), 100);
	HAL_Delay(1000);



	uint8_t settings1[] = {0xC1, 0x00, 0x08};
	uint8_t result[11] = {0};
	HAL_UART_Transmit(&huart1, settings1, sizeof(settings1), 100);
	HAL_UART_Receive(&huart1, result,  sizeof(result), 100);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, RESET);

	huart1.Init.BaudRate = 38400;
	HAL_UART_Init(&huart1);

	test_adc();

	ina219_primary_data_t primary_data;
	ina219_secondary_data_t secondary_data;
	ina219_init_default(&ina219,&hi2c1,INA219_I2CADDR_A1_GND_A0_GND, HAL_MAX_DELAY);


	uint16_t co2, tvoc;
	CCS811_Init();




	state_now = STATE_READY;
/*
    shift_reg_write_bit_16(&shift_reg_r, 6, 1); // Вращаем по горизонтали вперёд
    HAL_Delay(3000);
    shift_reg_write_bit_16(&shift_reg_r, 6, 0); // Вращаем по горизонтали вперёд
    shift_reg_write_bit_16(&shift_reg_r, 4, 1); // Вращаем по вертикали вперёд
    HAL_Delay(11700);
    shift_reg_write_bit_16(&shift_reg_r, 4, 0); // Вращаем по вертикали вперёд
*/


	// переменные гермообъёма
	static uint32_t last_activation_time = 0;
	static uint8_t valve_open = 0;

	// переменные для состояний
	float foto_state_out, foto_state_in;
	uint32_t start_time_luxes = 0;
	uint32_t incinerator_parashute = 0;
	uint32_t incinerator_sp = 0;
	uint32_t time_motor = 0;
	uint32_t time_motor_lepestok = 0;
	float height_old = 200.0;
	bool flag_foto = 1;
	bool flag_incinerator_sp = 1;
	bool flag_lepestok = 0;
	int count_height = 0;









	while(1){


		//работа гермообъёма срабатывание каждые 15 секунд на 5 секунд
		uint32_t current_time = HAL_GetTick();

		if (!valve_open && (current_time - last_activation_time >= 15000))
		{
		    // Открыть клапан
		    shift_reg_write_bit_16(&shift_reg_r, 0, 1);
		    valve_open = 1;
		    last_activation_time = current_time; // время открытия
		}
		else if (valve_open && (current_time - last_activation_time >= 5000))
		{
		    // Закрыть клапан
		    shift_reg_write_bit_16(&shift_reg_r, 0, 0);
		    valve_open = 0;
		    last_activation_time = current_time; // сброс на следующий цикл
		}


		int knopka  = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
/*
		if(knopka  == 1){
			//HAL_Delay(5000);

			shift_reg_write_bit_16(&shift_reg_r, 15, 1);
			shift_reg_write_bit_16(&shift_reg_r, 9, 1);
			shift_reg_write_bit_16(&shift_reg_r, 10, 1);
			shift_reg_write_bit_16(&shift_reg_r, 11, 1);
			shift_reg_write_bit_16(&shift_reg_r, 8, 1);
			shift_reg_write_bit_16(&shift_reg_r, 12, 1);
			shift_reg_write_bit_16(&shift_reg_r, 13, 1);
			shift_reg_write_bit_16(&shift_reg_r, 14, 1);

			shift_reg_write_bit_16(&shift_reg_r, 0, 1);

		}
		else{

			shift_reg_write_bit_16(&shift_reg_r, 15, 0);
			shift_reg_write_bit_16(&shift_reg_r, 9, 0);
			shift_reg_write_bit_16(&shift_reg_r, 10, 0);
			shift_reg_write_bit_16(&shift_reg_r, 11, 0);
			shift_reg_write_bit_16(&shift_reg_r, 8, 0);
			shift_reg_write_bit_16(&shift_reg_r, 12, 0);
			shift_reg_write_bit_16(&shift_reg_r, 13, 0);
			shift_reg_write_bit_16(&shift_reg_r, 14, 0);

			shift_reg_write_bit_16(&shift_reg_r, 0, 0);
		}
*/



		if(is_mount != FR_OK) {

			f_mount(0, "0", 1);
			extern Disk_drvTypeDef disk;
			disk.is_initialized[0] = 0;
			is_mount = f_mount(&fileSystem, "", 1);

			test_adc();

			f_open(&File_1csv, (char*)path1, FA_WRITE | FA_OPEN_APPEND); // открытие файла
			f_puts("flag; num; time_ms; accl1; accl2; accl3; gyro1; gyro2; gyro3; mag1; mag2; mag3; bme_temp; bme_press; bme_humidity; bme_height; lux_board; lux_sp; state; lidar; crc\n", &File_1csv);
			f_open(&File_2csv, (char*)path2, FA_WRITE | FA_OPEN_APPEND); // открытие файла
			f_puts("flag; num; time_ms; fix; lat; lon; alt; gps_time_s; gps_time_s; current; bus_voltage; MICS_5524; MICS_CO; MICS_NO2; MICS_NH3; CCS_CO2; CCS_TVOC; bme_temp_g; bme_press_g; bme_humidity_g; crc\n", &File_2csv);
			f_open(&File_b, pathb, FA_WRITE | FA_OPEN_APPEND); // открытие файла
		}

		its_bme280_read(UNKNOWN_BME1, &bme_shit);
		its_bme280_read(UNKNOWN_BME2, &bme2_shit);
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


		CCS811_SetEnvironmentalData(bme2_shit.humidity, bme2_shit.temperature); // 45% RH, 24°C

		if (CCS811_DataAvailable())
		{
		    CCS811_ReadAlgorithmResults(&co2, &tvoc);
	    }

		HAL_StatusTypeDef tr;

		uint8_t lidar[18];
		size_t lidar_data_size = 0;
		//HAL_UART_Receive(&huart6, lidar, 18, 100);
		for (int i = 0; i < 18; i++)
		{
			int value = sbuffer_pop(&lidar_buff);
			if (value < 0)
				break;

			lidar[i] = value;
			lidar_data_size++;
		}

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

		int res_ina219 = ina219_read_primary(&ina219,&primary_data);
		if (res_ina219 == 2)
		{
			I2C_ClearBusyFlagErratum(&hi2c1, 20);
		}
		res_ina219 = ina219_read_secondary(&ina219,&secondary_data);
		if (res_ina219 == 2)
		{
			I2C_ClearBusyFlagErratum(&hi2c1, 20);
		}



		current = ina219_current_convert(&ina219, secondary_data.current);
		const float power = ina219_power_convert(&ina219, secondary_data.power);
		const float busv = power / current;

		bus_voltage = ina219_bus_voltage_convert(&ina219, primary_data.busv);
		start = HAL_GetTick();

		test_adc();
		//SunTrack_Run(&shift_reg_r);


		pack1.flag = 0xAA;
		pack1.bme_height = height;
		pack1.bme_humidity = bmp_humidity;
		pack1.bme_press = bmp_press;
		pack1.bme_temp = bmp_temp;
		pack1.lidar = lidar_1;
		pack1.lux_board = foto_state;
		pack1.lux_sp = foto_sp;
		pack1.state = state_now;
		pack1.crc = Crc16((uint8_t *)&pack1, sizeof(pack1) - 2);


		pack2.flag = 0xBB;
		pack2.MICS_CO = R_MICS_CO;
		pack2.MICS_NH3 = R_MICS_NH3;
		pack2.MICS_NO2 = R_MICS_NO2;
		pack2.MICS_5524 = R_MICS_5524;
		pack2.CCS_TVOC = tvoc;
		pack2.CCS_CO2 = co2;
		pack2.lat = lat;
		pack2.lon = lon;
		pack2.alt = alt;
		pack2.gps_time_s = gps_time_s;
		pack2.fix = fix;
		pack2.bme_humidity_g = bme2_shit.humidity;
		pack2.bme_press_g = bme2_shit.pressure;
		pack2.bme_temp_g = bme2_shit.temperature;
		pack2.bus_voltage = bus_voltage;
		pack2.current = current;


		int size_pack1 = sizeof(pack1);
		int size_pack2 = sizeof(pack2);






		switch(sd_state) {
			case SD_PACK_1:

				pack1.num++;
				pack1.time_ms = HAL_GetTick();
				pack1.crc = Crc16((uint8_t *)&pack1, sizeof(pack1) - 2);

				if (res1csv == FR_OK){
					num_written = sd_parse_to_bytes_pac1(str_buf, &pack1);

					res1csv = f_write(&File_1csv,str_buf,num_written, &Bytes); // отправка на запись в файл
					if (pack1.num % 50 == 0)
						res1csv = f_sync(&File_1csv); // запись в файл (на sd контроллер пишет не сразу, а по закрытии файла. Также можно использовать эту команду)
				}
				if (resb == FR_OK){
					resb = f_write(&File_b,(uint8_t *)&pack1,sizeof(pack1), &Bytes); // отправка на запись в файл
					if (pack1.num % 50 == 0)
						resb = f_sync(&File_b); // запись в файл (на sd контроллер пишет не сразу, а по закрытии файла. Также можно использовать эту команду)
				}

				if(	HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)){
					a++;
					HAL_UART_Transmit(&huart1, (uint8_t*)&pack1, sizeof(pack1), 100);
				}

				if(a >= 4)
					sd_state = SD_PACK_2;
				else
					sd_state = SD_PACK_1;
			break;

			case SD_PACK_2:

				pack2.num++;
				pack2.time_ms = HAL_GetTick();
				pack2.crc = Crc16((uint8_t *)&pack2, sizeof(pack2) - 2);

				if (res2csv == FR_OK){
					num_written = sd_parse_to_bytes_pac2(str_buf, &pack2);

					res2csv = f_write(&File_2csv,str_buf,num_written, &Bytes); // отправка на запись в файл
					if (pack2.num % 50 == 0)
						res2csv = f_sync(&File_2csv); // запись в файл (на sd контроллер пишет не сразу, а по закрытии файла. Также можно использовать эту команду)
				}
				if (resb == FR_OK){
					resb = f_write(&File_b,(uint8_t *)&pack2,sizeof(pack2), &Bytes); // отправка на запись в файл
					if (pack2.num % 50 == 0)
						resb = f_sync(&File_b); // запись в файл (на sd контроллер пишет не сразу, а по закрытии файла. Также можно использовать эту команду)
				}

				if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)) {
					a = 0;
					sd_state = SD_PACK_1;
					HAL_UART_Transmit(&huart1, (uint8_t*)&pack2, sizeof(pack2), 100);
				}
				else{
					volatile int x = 0;
				}

			break;
		}


		switch(state_now){
			case STATE_READY:
				if(knopka == 1){
					bmp_press = bme_shit.pressure;
					test_adc();
					foto_state_out = foto_state;
					state_now = STATE_IN_ROCKET;
					start_time_luxes = HAL_GetTick();
				}
			break;

			case STATE_IN_ROCKET:

				if (HAL_GetTick()-start_time_luxes >= 8000){
					shift_reg_write_bit_16(&shift_reg_r, 1, 1);
				}
				if (HAL_GetTick()-start_time_luxes >= 10000){
					shift_reg_write_bit_16(&shift_reg_r, 1, 0);
					if(flag_foto == 1){
						test_adc();
						foto_state_in = foto_state;
						flag_foto = 0;
					}
					else{
						test_adc();
						if(foto_state >= ((foto_state_out - foto_state_in) * 0.5 + foto_state_in)){
							state_now = PARACHUTE_DESCENT;
						}

					}

				}
			break;

			case PARACHUTE_DESCENT:
				if(height <= 15.0){
					if(lidar_1 <= 400){
						state_now = FALL;
						shift_reg_write_16(&shift_reg_r, 0xFF00);
						shift_reg_write_bit_16(&shift_reg_r, 3, 1);
						incinerator_parashute = HAL_GetTick();
						time_motor = HAL_GetTick();
					}
				}
			break;

			case FALL:
				if(HAL_GetTick() - incinerator_parashute >= 1000){
					shift_reg_write_bit_16(&shift_reg_r, 3, 0);
				}
				if(HAL_GetTick() - time_motor >= 200){
					shift_reg_write_16(&shift_reg_r, 0x0000);
				}
				if(height_old - height <= 1.5){
					count_height ++;
					height_old = height;
				}
				else{
					count_height = 0;
					height_old = height;
				}
				if (count_height == 10)
					state_now = LANDING;

				break;

			case LANDING:
				//bloom(acc_g[0], acc_g[1], &shift_reg_r);
				shift_reg_write_16(&shift_reg_r, 0xFF00);
				time_motor_lepestok = HAL_GetTick();
				state_now = STATE_OPEN_SP;


				break;

			case STATE_OPEN_SP:
				if(HAL_GetTick() - time_motor_lepestok >= 3000){
					shift_reg_write_16(&shift_reg_r, 0x0000);
					flag_lepestok = 1;
				}
				if(flag_lepestok == 1){
					shift_reg_write_bit_16(&shift_reg_r, 2, 1);
					incinerator_sp = HAL_GetTick();

					state_now = SUN_SEARCH;
				}
				break;

			case SUN_SEARCH:
				if((HAL_GetTick() - incinerator_sp >= 1000) && flag_incinerator_sp == 1){
					shift_reg_write_bit_16(&shift_reg_r, 2, 0);
					flag_incinerator_sp = 0;
				}

				SunTrack_Run(&shift_reg_r);
				break;

			case ENERGY:
				shift_reg_write_bit_16(&shift_reg_r, 1, 1);
				state_now = ENERGY;
				break;
		}


		stop = HAL_GetTick();
		uint32_t delta = stop - start;
		printf("delta = %ld\n", delta);
	}

	return 0;
}



