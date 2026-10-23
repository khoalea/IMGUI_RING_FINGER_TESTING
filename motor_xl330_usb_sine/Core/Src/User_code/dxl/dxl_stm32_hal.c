#include "User_code/dxl/dxl_stm32_hal.h"

static DXL2_Status hal_to_dxl_status(HAL_StatusTypeDef status)
{
    switch (status) {
    case HAL_OK:
        return DXL2_OK;
    case HAL_TIMEOUT:
        return DXL2_ERR_TIMEOUT;
    default:
        return DXL2_ERR_TRANSPORT;
    }
}

static DXL2_Status stm32hal_write(void *user, const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    DXL2_STM32HAL_Context *ctx = (DXL2_STM32HAL_Context *)user;
    if (ctx == 0 || ctx->huart == 0 || data == 0) {
        return DXL2_ERR_BAD_PARAM;
    }
    return hal_to_dxl_status(HAL_UART_Transmit(ctx->huart, (uint8_t *)data, len, timeout_ms));
}

static DXL2_Status stm32hal_read_byte(void *user, uint8_t *byte, uint32_t timeout_ms)
{
    DXL2_STM32HAL_Context *ctx = (DXL2_STM32HAL_Context *)user;
    if (ctx == 0 || ctx->huart == 0 || byte == 0) {
        return DXL2_ERR_BAD_PARAM;
    }
    return hal_to_dxl_status(HAL_UART_Receive(ctx->huart, byte, 1u, timeout_ms));
}

static void stm32hal_set_tx_mode(void *user)
{
    DXL2_STM32HAL_Context *ctx = (DXL2_STM32HAL_Context *)user;
    if (ctx != 0 && ctx->dir_port != 0) {
        HAL_GPIO_WritePin(ctx->dir_port, ctx->dir_pin, ctx->tx_state);
    }
}

static void stm32hal_set_rx_mode(void *user)
{
    DXL2_STM32HAL_Context *ctx = (DXL2_STM32HAL_Context *)user;
    if (ctx != 0 && ctx->dir_port != 0) {
        HAL_GPIO_WritePin(ctx->dir_port, ctx->dir_pin, ctx->rx_state);
    }
}

static uint32_t stm32hal_millis(void *user)
{
    (void)user;
    return HAL_GetTick();
}

DXL2_Status dxl2_stm32hal_transport_init(DXL2_Transport *transport,
                                          DXL2_STM32HAL_Context *context,
                                          UART_HandleTypeDef *huart)
{
    return dxl2_stm32hal_transport_init_half_duplex(transport, context, huart, 0, 0u, GPIO_PIN_SET);
}

DXL2_Status dxl2_stm32hal_transport_init_half_duplex(DXL2_Transport *transport,
                                                     DXL2_STM32HAL_Context *context,
                                                     UART_HandleTypeDef *huart,
                                                     GPIO_TypeDef *dir_port,
                                                     uint16_t dir_pin,
                                                     GPIO_PinState tx_state)
{
    if (transport == 0 || context == 0 || huart == 0) {
        return DXL2_ERR_BAD_PARAM;
    }

    context->huart = huart;
    context->dir_port = dir_port;
    context->dir_pin = dir_pin;
    context->tx_state = tx_state;
    context->rx_state = (tx_state == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;

    transport->user = context;
    transport->write = stm32hal_write;
    transport->read_byte = stm32hal_read_byte;
    transport->set_tx_mode = stm32hal_set_tx_mode;
    transport->set_rx_mode = stm32hal_set_rx_mode;
    transport->millis = stm32hal_millis;

    return DXL2_OK;
}
