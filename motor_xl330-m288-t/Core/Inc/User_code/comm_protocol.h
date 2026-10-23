#ifndef INC_COMM_PROTOCOL_H_
#define INC_COMM_PROTOCOL_H_

#include "main.h"
#include <stdint.h>

// ---------------- CONFIG ----------------
#define COMM_RX_BUF_SIZE  256
#define HEADER_B1         0xAA
#define HEADER_B2         0x55
#define FOOTER_B          0x03

// ---------------- DATA STRUCTURES ----------------
typedef struct __attribute__((packed)) {
float position1;
float velocity1;
float position2;
float velocity2;
} Comm_TxData;

typedef struct __attribute__((packed)) {
float torque;
} Comm_RxData;

// ---------------- API ----------------
void Comm_Init(UART_HandleTypeDef *huart);
void Comm_Task(void);

// Setters (Thread-safe)
void Comm_SetTxData(float p1, float v1, float p2, float v2);

// Request triggers
void Comm_TxRequestFromISR(void);

// Getters
float Comm_GetRxTorque(void);

// Commands
void Comm_SendOK(void);

// User Overrides (Implemented in main.c)
extern void Handle_Calib_Command(uint8_t index);
extern void Update_And_Verify_Param(float new_value);

#endif /* INC_COMM_PROTOCOL_H_ */
