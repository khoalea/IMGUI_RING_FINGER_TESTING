#include "User_code/calibration.h"
#include <string.h>

/* Use the handle defined in main.c */
extern CRC_HandleTypeDef hcrc;

#define CALIB_FLASH_ADDR 0x0800FC00

/* Linker section attribute */
__attribute__((section(".calib_data")))
const CalibrationData flash_calib_data;

uint8_t Calib_Read(CalibrationData* dest) {
memcpy(dest, &flash_calib_data, sizeof(CalibrationData));

/* Calculate CRC of payload (everything except the last 4 bytes) */
uint32_t data_words = (sizeof(CalibrationData) - 4) / 4;
uint32_t calculated_crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)dest, data_words);

if (calculated_crc == dest->crc) {
    return 1;
} else {
    /* Default safety values if CRC fails */
    memset(dest, 0, sizeof(CalibrationData));
    dest->motor.torque_limit = 0.5f;
    dest->joints[0].scale = 1.0f;
    dest->joints[1].scale = 1.0f;
    return 0;
}


}

uint8_t Calib_Write(CalibrationData* src) {
uint32_t data_words = (sizeof(CalibrationData) - 4) / 4;
src->crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)src, data_words);

HAL_FLASH_Unlock();

FLASH_EraseInitTypeDef eraseInit;
eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
eraseInit.PageAddress = CALIB_FLASH_ADDR;
eraseInit.NbPages = 1;

uint32_t pageError = 0;
if (HAL_FLASHEx_Erase(&eraseInit, &pageError) != HAL_OK) {
    HAL_FLASH_Lock();
    return 0;
}

uint32_t *dataPtr = (uint32_t *)src;
for (uint32_t i = 0; i < (sizeof(CalibrationData) / 4); i++) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, CALIB_FLASH_ADDR + (i * 4), dataPtr[i]) != HAL_OK) {
        HAL_FLASH_Lock();
        return 0;
    }
}

HAL_FLASH_Lock();
return 1;


}
