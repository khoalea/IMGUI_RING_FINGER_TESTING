#include "User_code/comm_protocol.h"
#include <string.h>

static UART_HandleTypeDef *comm_uart;

// Internal buffers
static uint8_t tx_buffer[64];
static Comm_TxData current_tx;
static Comm_RxData current_rx;

static volatile uint8_t tx_request = 0;

static uint8_t rx_dma_buffer[COMM_RX_BUF_SIZE];
static uint16_t rx_read_pos = 0;

// Protocol FSM
typedef enum { RX_IDLE, RX_HEADER2, RX_DATA, RX_CS, RX_ETX } RxState_t;
static RxState_t rx_state = RX_IDLE;
static uint8_t rx_pkt_buf[sizeof(Comm_RxData)];
static uint8_t rx_pkt_idx = 0;
static uint8_t rx_cs_calc = 0;

// Calibration Command Detector
static const char calib_cmd[] = "CALIB";
static uint8_t calib_detect_idx = 0;
extern uint8_t is_calibrating;

void Comm_Init(UART_HandleTypeDef *huart) {
comm_uart = huart;
HAL_UART_Receive_DMA(comm_uart, rx_dma_buffer, COMM_RX_BUF_SIZE);
}

void Comm_SetTxData(float p1, float v1, float p2, float v2) {
	uint32_t irq = __get_PRIMASK();
	__disable_irq();
	current_tx.position1 = p1;
	current_tx.velocity1 = v1;
	current_tx.position2 = p2;
	current_tx.velocity2 = v2;
	__set_PRIMASK(irq);
}

void Comm_TxRequestFromISR(void) { tx_request = 1; }
float Comm_GetRxTorque(void) { return current_rx.torque; }

static void HandleTx(void) {
if (comm_uart->gState != HAL_UART_STATE_READY || is_calibrating) return;
if (!tx_request) return;

tx_request = 0;
uint8_t type = 'N'; // Normal Data
uint16_t len = sizeof(Comm_TxData);

tx_buffer[0] = HEADER_B1;
tx_buffer[1] = HEADER_B2;
tx_buffer[2] = type;

memcpy(&tx_buffer[3], &current_tx, len);

uint8_t cs = type;
for(int i=0; i<len; i++) cs += tx_buffer[3+i];
tx_buffer[3+len] = cs;
tx_buffer[4+len] = FOOTER_B;

HAL_UART_Transmit_DMA(comm_uart, tx_buffer, 5 + len);


}

static void ProcessByte(uint8_t b) {
	// 1. Detection of "CALIB#"
	if (!is_calibrating) {
	if (b == calib_cmd[calib_detect_idx]) {
		calib_detect_idx++;
		if (calib_detect_idx >= 5) {
		// Next byte should be the index
			rx_state = RX_IDLE; // Reset binary FSM to avoid conflicts
		}
	} else if (calib_detect_idx == 5) {
		// This byte is the '#' index
		uint8_t index = b - '0';
		Handle_Calib_Command(index);
		calib_detect_idx = 0;
	} else {
		calib_detect_idx = 0;
	}
	} else {
		// We are in Calib mode: Wait for 4 bytes of float
		static uint8_t float_buf[4];
		static uint8_t float_idx = 0;
		float_buf[float_idx++] = b;
		if (float_idx >= 4) {
		float val;
		memcpy(&val, float_buf, 4);
		Update_And_Verify_Param(val);
		float_idx = 0;
	}
	return; // Don't process binary protocol while calibrating
	}

	// 2. Binary FSM for Normal Torque Control
	switch (rx_state) {
		case RX_IDLE:
			if (b == HEADER_B1) rx_state = RX_HEADER2;
			break;
		case RX_HEADER2:
			if (b == HEADER_B2) {
				rx_state = RX_DATA;
				rx_pkt_idx = 0;
				rx_cs_calc = 0;
			} else rx_state = RX_IDLE;
			break;
		case RX_DATA:
			rx_pkt_buf[rx_pkt_idx++] = b;
			rx_cs_calc += b;
			if (rx_pkt_idx >= sizeof(Comm_RxData)) rx_state = RX_CS;
			break;
		case RX_CS:
			rx_state = (b == rx_cs_calc) ? RX_ETX : RX_IDLE;
			break;
		case RX_ETX:
			if (b == FOOTER_B) memcpy(&current_rx, rx_pkt_buf, sizeof(Comm_RxData));
			rx_state = RX_IDLE;
			break;
	}


}

void Comm_Task(void) {
	uint16_t head = COMM_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(comm_uart->hdmarx);
	while (rx_read_pos != head) {
		ProcessByte(rx_dma_buffer[rx_read_pos]);
		if (++rx_read_pos >= COMM_RX_BUF_SIZE) rx_read_pos = 0;
	}
	HandleTx();
}

void Comm_SendOK(void) {
	static const uint8_t msg[] = "OK\n";
	HAL_UART_Transmit(comm_uart, (uint8_t*)msg, 3, 100);
}
