#ifndef DXL_STM32_HAL_H_
#define DXL_STM32_HAL_H_

#include "dxl_transport.h"
#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    UART_HandleTypeDef *huart;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    GPIO_PinState tx_state;
    GPIO_PinState rx_state;
} DXL2_STM32HAL_Context;

DXL2_Status dxl2_stm32hal_transport_init(DXL2_Transport *transport,
                                          DXL2_STM32HAL_Context *context,
                                          UART_HandleTypeDef *huart);
DXL2_Status dxl2_stm32hal_transport_init_half_duplex(DXL2_Transport *transport,
                                                     DXL2_STM32HAL_Context *context,
                                                     UART_HandleTypeDef *huart,
                                                     GPIO_TypeDef *dir_port,
                                                     uint16_t dir_pin,
                                                     GPIO_PinState tx_state);

#ifdef __cplusplus
}
#endif

#endif /* DXL_STM32_HAL_H_ */
