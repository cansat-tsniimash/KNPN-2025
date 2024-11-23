#include "its_bme280.h"
#include "../../Application/freakheader.h"

#include <stdint.h>
#include <assert.h>

#include <stm32f4xx_hal.h>

#include "i2C_crutch/i2c-crutch.h"

#include <stdio.h>
#include <string.h>
#include "bme280.h"
#include <math.h>

#include <math.h>

extern I2C_HandleTypeDef hi2c1;


#define ITS_BME_HAL_TIMEOUT (300)


struct its_bme280_t
{
	//! Шина I2C на которой датчик работает
	I2C_HandleTypeDef * bus;
	//! Адрес датчика на шине
	uint8_t addr;
	//! Структура родного драйвера
	struct bme280_dev driver;
};


static its_bme280_t * _dev_by_id(its_bme280_id_t id);
static int8_t _i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
static int8_t _i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
static void _delay_ms(uint32_t ms);


static its_bme280_t _devices[1] = {
	{
		.addr = BME280_I2C_ADDR_PRIM,
		.bus = &hi2c1,
		.driver =
		{
			.dev_id = UNKNOWN_BME,
			.intf = BME280_I2C_INTF,
			.read = _i2c_read,
			.write = _i2c_write,
			.delay_ms = _delay_ms,
			.settings =
			{
					.filter = BME280_FILTER_COEFF_OFF,
					.osr_h = BME280_OVERSAMPLING_16X,
					.osr_p = BME280_OVERSAMPLING_16X,
					.osr_t = BME280_OVERSAMPLING_16X,
					.standby_time = BME280_STANDBY_TIME_500_MS
			}
		}
	}
};


static its_bme280_t * _dev_by_id(its_bme280_id_t id)
{
	assert(id >= 0 && id < 1);
	its_bme280_t * const dev = &_devices[id];

	return dev;
}


static int8_t _i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	its_bme280_t * dev = _dev_by_id(id);

	HAL_StatusTypeDef hrc = HAL_I2C_Mem_Read(
			dev->bus,
			dev->addr << 1,
			reg_addr,
			1,
			data,
			len,
			ITS_BME_HAL_TIMEOUT
	);

	if (hrc != HAL_OK)
	{
		I2C_ClearBusyFlagErratum(&hi2c1, 120);
		return (int8_t)hrc;


	}

	return (int8_t)hrc;
}


static int8_t _i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	its_bme280_t * dev = _dev_by_id(id);

	HAL_StatusTypeDef hrc = HAL_I2C_Mem_Write(
			dev->bus,
			dev->addr << 1,
			reg_addr,
			1,
			data,
			len,
			ITS_BME_HAL_TIMEOUT
	);
	if (hrc == HAL_BUSY)
	{
		I2C_ClearBusyFlagErratum(&hi2c1, 120);
		return (int8_t)hrc;

	}
	return (int8_t)hrc;
}



static void _delay_ms(uint32_t ms)
{
	HAL_Delay(ms);
}


int its_bme280_init(its_bme280_id_t id)
{
	its_bme280_t * const dev = _dev_by_id(id);

	int rc = bme280_soft_reset(&dev->driver);
	if (0 != rc)
		return rc;

	rc = bme280_init(&dev->driver);
	if (0 != rc)
		return rc;

	rc = bme280_set_sensor_settings(BME280_ALL_SETTINGS_SEL, &dev->driver);
	if (0 != rc)
		return rc;

	rc = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev->driver);
	if (0 != rc)
		return rc;

	HAL_Delay(10); // Иначе первые данные получаются плохие
	return 0;
}


int its_bme280_punish(its_bme280_id_t id)
{
	its_bme280_t * const dev = _dev_by_id(id);

	// Будем действовать радикально и будем сразу стрелять шину
	HAL_I2C_DeInit(dev->bus);
	HAL_Delay(10);
	HAL_I2C_Init(dev->bus);

	// Пробуем перезапустить датчик
	int rc;
	rc = its_bme280_init(id);
	if (0 != rc)
		return rc;

	return 0;
}

double bme_mid_meaning(its_bme280_id_t id, bme_important_shit_t * data)
{
	its_bme280_t * const dev = _dev_by_id(id);
	double summ_epta;
	struct bme280_data bme280_data;
	for (int i = 0; i<=10; i++)
	{
		bme280_get_sensor_data(BME280_PRESS, &bme280_data, &dev->driver);
		summ_epta += bme280_data.pressure;
	}
	return summ_epta/10;
}


//Заменить в функци структура мавлинка на собственную
int its_bme280_read(its_bme280_id_t id, bme_important_shit_t * data)
{
	its_bme280_t * const dev = _dev_by_id(id);

	struct bme280_data bme280_data;

	int rc = bme280_get_sensor_data(BME280_ALL, &bme280_data, &dev->driver);
	if (0 != rc)
		return rc;

	data->time_steady = HAL_GetTick();

	data->pressure = bme280_data.pressure;
	data->temperature = bme280_data.temperature;
	data->humidity = bme280_data.humidity;
	data->altitude =  44330.0*(1.0 - pow((float)bme280_data.pressure/data->bme_pres_grow, 1.0/5.255)); // Написать функцию пересчета в высоту
	return 0;
}
