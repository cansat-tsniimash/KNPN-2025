#include "CCS811.h"
#include "stm32f4xx_hal.h"
// Объявление внешнего I2C хэндла, определённого в main.c или i2c.c
extern I2C_HandleTypeDef hi2c1;

// Инициализация датчика CCS811
// Проверяет ID, запускает приложение, устанавливает режим измерений
bool CCS811_Init(void)
{
    uint8_t id = 0;
    // Чтение регистра HW_ID для проверки ID устройства
    HAL_I2C_Mem_Read(&hi2c1, CCS811_ADDRESS << 1, CCS811_HW_ID_REG, 1, &id, 1, 100);
    if (id != 0x81) return false; // Ожидаем ID 0x81

    uint8_t cmd = 0x00;
    // Команда запуска приложения (APP_START)
    HAL_I2C_Mem_Write(&hi2c1, CCS811_ADDRESS << 1, CCS811_APP_START, 1, &cmd, 0, 100);
    HAL_Delay(100);

    uint8_t mode = 0x10; // Drive mode 1 — 1 измерение/сек
    // Установка режима измерений
    HAL_I2C_Mem_Write(&hi2c1, CCS811_ADDRESS << 1, CCS811_MEAS_MODE_REG, 1, &mode, 1, 100);

    return true;
}

// Проверка наличия новых данных от CCS811
// Возвращает true, если установлен бит DATA_READY
bool CCS811_DataAvailable(void)
{
    uint8_t status;
    HAL_I2C_Mem_Read(&hi2c1, CCS811_ADDRESS << 1, CCS811_STATUS_REG, 1, &status, 1, 100);
    return (status & 0x08); // bit 3 — DATA_READY
}

// Чтение результатов алгоритма: eCO2 и TVOC
bool CCS811_ReadAlgorithmResults(uint16_t *eCO2, uint16_t *TVOC)
{
    uint8_t buffer[4];
    // Чтение 4 байт данных из регистра ALG_RESULT_DATA
    HAL_I2C_Mem_Read(&hi2c1, CCS811_ADDRESS << 1, CCS811_ALG_RESULT_DATA, 1, buffer, 4, 100);

    // Парсинг значений CO2 и TVOC
    *eCO2 = ((uint16_t)buffer[0] << 8) | buffer[1];
    *TVOC = ((uint16_t)buffer[2] << 8) | buffer[3];

    return true;
}

// Установка текущей температуры и влажности в CCS811
// Это позволяет датчику компенсировать показания с учётом окружающей среды
void CCS811_SetEnvironmentalData(float humidity, float temperature)
{
    uint8_t envData[4];

    // Преобразование влажности (0–100%) в формат по документации (fixed point * 512)
    uint16_t hum = (uint16_t)(humidity * 512.0f + 0.5f);
    // Преобразование температуры (°C + 25) в fixed point * 512
    uint16_t temp = (uint16_t)((temperature + 25.0f) * 512.0f + 0.5f);

    // Заполнение массива данных для передачи
    envData[0] = (hum >> 8) & 0xFF;
    envData[1] = hum & 0xFF;
    envData[2] = (temp >> 8) & 0xFF;
    envData[3] = temp & 0xFF;

    // Отправка данных во внутренний регистр ENV_DATA
    HAL_I2C_Mem_Write(&hi2c1, CCS811_ADDRESS << 1, CCS811_ENV_DATA, 1, envData, 4, 100);
}
