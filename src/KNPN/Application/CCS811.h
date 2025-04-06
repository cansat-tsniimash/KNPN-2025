#ifndef INC_CCS811_H_
#define INC_CCS811_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define CCS811_ADDRESS 0x5A // Адрес I2C по умолчанию

// Регистр
#define CCS811_STATUS_REG           0x00
#define CCS811_MEAS_MODE_REG        0x01
#define CCS811_ALG_RESULT_DATA      0x02
#define CCS811_ENV_DATA             0x05
#define CCS811_APP_START            0xF4
#define CCS811_HW_ID_REG            0x20

extern I2C_HandleTypeDef hi2c1;

bool CCS811_Init(void);
bool CCS811_DataAvailable(void);
bool CCS811_ReadAlgorithmResults(uint16_t *eCO2, uint16_t *TVOC);
void CCS811_SetEnvironmentalData(float humidity, float temperature);

#endif /* INC_CCS811_H_ */
