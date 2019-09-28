// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  Copyright (C) 2016-2019  Nikolay Sukharev
 *  Copyright (C) 2019	     Roman Penyaev
 */

#ifndef __USART_H
#define __USART_H

enum USART_Status {
	USART_OK        = 0,
	USART_BUSY      = 1,
	USART_CRC_ERROR = 2,
	USART_ERROR     = 3,
	USART_TIMEOUT   = 4,
};

enum {
	REPEAT_TX_REQS_x5 = 5,
};

void USART_Init(UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim,
				unsigned channel);
void USART_Process(void);
enum USART_Status USART_GetStatus(void);
int USART_SendAsync(const void *tx_buf, uint8_t tx_len);
int USART_SendRecvAsync(const void *tx_buf, uint8_t tx_len,
						void *rx_buf, uint8_t rx_len_max,
						uint8_t tx_repeat);
int USART_SendRecvSync(const void *tx_buf, uint8_t tx_len,
					   void *rx_buf, uint8_t rx_len_max,
					   uint8_t tx_repeat);
int USART_RecvSendSync(void *ctx, void *rx_buf, uint8_t rx_len_max,
					   int (*handle_rx)(void *ctx, uint8_t rx_len,
										void **tx_buf_out,
										uint8_t *tx_len_out));
uint8_t USART_CountRXBytes(void);

#endif /* __USART_H */
