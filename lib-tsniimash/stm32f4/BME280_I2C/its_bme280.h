#ifndef INC_SENSORS_ITS_BME280_H_
#define INC_SENSORS_ITS_BME280_H_


#include <stdbool.h>
#include "main.h"
#include "../../Application/freakheader.h"



typedef enum its_bme280_id_t
{
	UNKNOWN_BME = 0,
} its_bme280_id_t;


struct its_bme280_t;
typedef struct its_bme280_t its_bme280_t;

//! Инициализация (или переинициализация) датчика
/*! Инициализация начинается с программной перезагрузки */
int its_bme280_init(its_bme280_id_t id);

//! API для комиссара - пробует перезапустить датчик
int its_bme280_punish(its_bme280_id_t id);

//! Получение mavlink пакета с данными BME
int its_bme280_read(its_bme280_id_t id, bme_important_shit_t * data);


#endif /* INC_SENSORS_ITS_BME280_H_ */
