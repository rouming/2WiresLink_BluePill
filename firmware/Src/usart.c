// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  Copyright (C) 2016-2019  Nikolay Sukharev
 *  Copyright (C) 2019	     Roman Penyaev
 */

#include "stm32f1xx_hal.h"
#include <string.h>
#include "usart.h"

#undef offsetof
#ifdef __compiler_offsetof
#define offsetof(TYPE, MEMBER)	__compiler_offsetof(TYPE, MEMBER)
#else
#define offsetof(TYPE, MEMBER)	((size_t)&((TYPE *)0)->MEMBER)
#endif

enum {
	USART_MAGIC      = 0x55,
	USART_RX_TIMEOUT = 20, /* ms for each byte */
};

enum USART_State {
	USART_IDLE_STATE = 0,
	USART_RX_STATE,
	USART_RX_NEED_CRC_STATE,
	USART_TX_STATE,
};

struct usart_hdr {
	uint8_t magic;
	uint8_t len;
	uint8_t crc;
};

static struct {
	struct {
		struct usart_hdr hdr;
		uint8_t *buf;
	} rx_pkt;
	struct {
		struct usart_hdr hdr;
		uint8_t buf[256];
	} tx_pkt;

	UART_HandleTypeDef *handle;
	TIM_HandleTypeDef  *tim_handle;
	unsigned            tim_channel;
	unsigned tx_repeat;
	unsigned rx_len_total;
	unsigned rx_len_max;
	unsigned rx_timestamp;
	uint8_t  rx_byte;
	enum USART_Status status;
	enum USART_State  state;
} usart;

static unsigned char const crc8x_table[] = {
	0x00, 0x31, 0x62, 0x53, 0xc4, 0xf5, 0xa6, 0x97, 0xb9, 0x88, 0xdb, 0xea, 0x7d,
	0x4c, 0x1f, 0x2e, 0x43, 0x72, 0x21, 0x10, 0x87, 0xb6, 0xe5, 0xd4, 0xfa, 0xcb,
	0x98, 0xa9, 0x3e, 0x0f, 0x5c, 0x6d, 0x86, 0xb7, 0xe4, 0xd5, 0x42, 0x73, 0x20,
	0x11, 0x3f, 0x0e, 0x5d, 0x6c, 0xfb, 0xca, 0x99, 0xa8, 0xc5, 0xf4, 0xa7, 0x96,
	0x01, 0x30, 0x63, 0x52, 0x7c, 0x4d, 0x1e, 0x2f, 0xb8, 0x89, 0xda, 0xeb, 0x3d,
	0x0c, 0x5f, 0x6e, 0xf9, 0xc8, 0x9b, 0xaa, 0x84, 0xb5, 0xe6, 0xd7, 0x40, 0x71,
	0x22, 0x13, 0x7e, 0x4f, 0x1c, 0x2d, 0xba, 0x8b, 0xd8, 0xe9, 0xc7, 0xf6, 0xa5,
	0x94, 0x03, 0x32, 0x61, 0x50, 0xbb, 0x8a, 0xd9, 0xe8, 0x7f, 0x4e, 0x1d, 0x2c,
	0x02, 0x33, 0x60, 0x51, 0xc6, 0xf7, 0xa4, 0x95, 0xf8, 0xc9, 0x9a, 0xab, 0x3c,
	0x0d, 0x5e, 0x6f, 0x41, 0x70, 0x23, 0x12, 0x85, 0xb4, 0xe7, 0xd6, 0x7a, 0x4b,
	0x18, 0x29, 0xbe, 0x8f, 0xdc, 0xed, 0xc3, 0xf2, 0xa1, 0x90, 0x07, 0x36, 0x65,
	0x54, 0x39, 0x08, 0x5b, 0x6a, 0xfd, 0xcc, 0x9f, 0xae, 0x80, 0xb1, 0xe2, 0xd3,
	0x44, 0x75, 0x26, 0x17, 0xfc, 0xcd, 0x9e, 0xaf, 0x38, 0x09, 0x5a, 0x6b, 0x45,
	0x74, 0x27, 0x16, 0x81, 0xb0, 0xe3, 0xd2, 0xbf, 0x8e, 0xdd, 0xec, 0x7b, 0x4a,
	0x19, 0x28, 0x06, 0x37, 0x64, 0x55, 0xc2, 0xf3, 0xa0, 0x91, 0x47, 0x76, 0x25,
	0x14, 0x83, 0xb2, 0xe1, 0xd0, 0xfe, 0xcf, 0x9c, 0xad, 0x3a, 0x0b, 0x58, 0x69,
	0x04, 0x35, 0x66, 0x57, 0xc0, 0xf1, 0xa2, 0x93, 0xbd, 0x8c, 0xdf, 0xee, 0x79,
	0x48, 0x1b, 0x2a, 0xc1, 0xf0, 0xa3, 0x92, 0x05, 0x34, 0x67, 0x56, 0x78, 0x49,
	0x1a, 0x2b, 0xbc, 0x8d, 0xde, 0xef, 0x82, 0xb3, 0xe0, 0xd1, 0x46, 0x77, 0x24,
	0x15, 0x3b, 0x0a, 0x59, 0x68, 0xff, 0xce, 0x9d, 0xac};

static inline uint8_t crc8(const void *mem, size_t len)
{
	const uint8_t *data = mem;
	uint8_t crc = 0xff;

	while (len--)
		crc = crc8x_table[crc ^ *data++];

	return crc;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart != usart.handle)
		return;

	/* Stop carry */
	HAL_TIM_PWM_Stop(usart.tim_handle, usart.tim_channel);

	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC)) {
		/* Transmission succeeded */

		if (!usart.rx_pkt.buf) {
			/* Done, report success if no response is awaited */
			usart.status = USART_OK;
			usart.state = USART_IDLE_STATE;
		} else {
			/* Switch to RX state and wait for response */
			usart.rx_len_total = 0;
			usart.rx_timestamp = HAL_GetTick() + USART_RX_TIMEOUT;
			usart.state = USART_RX_STATE;
		}
	}
}


static void USART_HandleRX(uint8_t byte)
{
	if (usart.state != USART_RX_STATE)
		return;

	if (usart.rx_len_total == offsetof(struct usart_hdr, magic)) {
		if (byte != USART_MAGIC) {
			/* Incorrect magic */

			return;
		}
	} else if (usart.rx_len_total == offsetof(struct usart_hdr, len)) {
		if (byte > usart.rx_len_max) {
			/* Incorrect length */
			usart.status = USART_ERROR;
			usart.state = USART_IDLE_STATE;

			return;
		}
		usart.rx_pkt.hdr.len = byte;
	} else if (usart.rx_len_total == offsetof(struct usart_hdr, crc)) {
		usart.rx_pkt.hdr.crc = byte;
	} else if (usart.rx_len_total - sizeof(struct usart_hdr) <
		   usart.rx_pkt.hdr.len) {
		usart.rx_pkt.buf[usart.rx_len_total -
				 sizeof(struct usart_hdr)] = byte;
	}
	usart.rx_timestamp = HAL_GetTick() + USART_RX_TIMEOUT;
	usart.rx_len_total++;

	if (usart.rx_len_total >= sizeof(struct usart_hdr) &&
	    usart.rx_len_total - sizeof(struct usart_hdr) >=
	    usart.rx_pkt.hdr.len) {
		/*
		 * Done with receive, but don't do crc from the interrupt,
		 * offload to someone else.
		 */
		usart.state = USART_RX_NEED_CRC_STATE;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart != usart.handle)
		return;

        /* In case of overrun error */
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) ||
	    __HAL_UART_GET_FLAG(huart, UART_FLAG_FE)) {
		usart.status = USART_ERROR;
		usart.state = USART_IDLE_STATE;
        } else {
		USART_HandleRX(usart.rx_byte);
		HAL_UART_Receive_IT(huart, &usart.rx_byte, 1);
        }
}

void USART_Process(void)
{
	uint8_t crc;

	switch (usart.state) {
	case USART_RX_STATE:
		if (HAL_GetTick() >= usart.rx_timestamp) {
			if (usart.tx_repeat) {
				/* Repeat transmission */
				usart.tx_repeat--;
				usart.state = USART_TX_STATE;
				HAL_UART_Transmit_IT(usart.handle,
					(uint8_t *)&usart.tx_pkt,
					usart.tx_pkt.hdr.len +
					sizeof(struct usart_hdr));
			} else {
				usart.status = USART_TIMEOUT;
				usart.state = USART_IDLE_STATE;
			}
		}
		break;
	case USART_RX_NEED_CRC_STATE:
		crc = crc8(usart.rx_pkt.buf, usart.rx_pkt.hdr.len);
		usart.status = crc != usart.rx_pkt.hdr.crc ?
			USART_CRC_ERROR : USART_OK;
		usart.state = USART_IDLE_STATE;
		break;
	default:
		break;
	}
}

static void setup_pwm(TIM_HandleTypeDef *htim, uint32_t ch, uint16_t duration)
{
    uint16_t pulse;

    pulse = ((htim->Init.Period + 1) * duration) / 100;
    __HAL_TIM_SET_COMPARE(htim, ch, pulse);
}

void USART_Init(UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim,
				unsigned channel)
{
	usart.handle = huart;
	usart.tim_handle = htim;
	usart.tim_channel = channel;
	usart.state = USART_IDLE_STATE;
	/* Start receiving bytes immediately */
	HAL_UART_Receive_IT(huart, &usart.rx_byte, 1);
	/* Setup pwm with 50% duration and start carry */
	setup_pwm(htim, channel, 50);
}

enum USART_Status USART_GetStatus(void)
{
	return (usart.state != USART_IDLE_STATE ? USART_BUSY : usart.status);
}

uint8_t USART_CountRXBytes(void)
{
	return usart.rx_pkt.hdr.len;
}

static inline int __USART_SendAsync(const void *tx_buf, uint8_t tx_len,
				    void *rx_buf, uint8_t rx_len_max,
				    uint8_t tx_repeat)
{
	if (usart.state != USART_IDLE_STATE)
		return -1;

	if (tx_repeat && !rx_buf)
		/* Invalid args */
		return -1;

	/* Setup RX */
	usart.rx_pkt.buf = rx_buf;
	usart.rx_len_max = rx_len_max;
	/* Setup TX */
	usart.tx_repeat = tx_repeat;
	usart.tx_pkt.hdr.magic = USART_MAGIC;
	usart.tx_pkt.hdr.len = tx_len;
	usart.tx_pkt.hdr.crc = crc8(tx_buf, tx_len);
	memcpy(usart.tx_pkt.buf, tx_buf, tx_len);
	usart.state = USART_TX_STATE;

	/* Start carry */
	HAL_TIM_PWM_Start(usart.tim_handle, usart.tim_channel);
	/* Eventually send data out */
	HAL_UART_Transmit_IT(usart.handle,
			     (uint8_t *)&usart.tx_pkt,
			     tx_len + sizeof(struct usart_hdr));

	return 0;
}

int USART_SendAsync(const void *tx_void, uint8_t tx_len)
{
	return __USART_SendAsync(tx_void, tx_len, NULL, 0, 0);
}

int USART_SendRecvAsync(const void *tx_buf, uint8_t tx_len,
			void *rx_buf, uint8_t rx_len_max,
			uint8_t tx_repeat)
{
	return __USART_SendAsync(tx_buf, tx_len, rx_buf, rx_len_max,
				 tx_repeat);
}

int USART_RecvAsync(void *rx_buf, uint8_t rx_len_max)
{
	if (usart.state != USART_IDLE_STATE)
		return -1;

	usart.rx_len_total = 0;
	usart.rx_pkt.buf = rx_buf;
	usart.rx_len_max = rx_len_max;
	usart.rx_timestamp = HAL_GetTick() + USART_RX_TIMEOUT;
	/* No retransmissions, plain receive */
	usart.tx_repeat = 0;
	usart.state = USART_RX_STATE;

	return 0;
}

int USART_SendRecvSync(const void *tx_buf, uint8_t tx_len,
		       void *rx_buf, uint8_t rx_len_max,
		       uint8_t tx_repeat)
{
	enum USART_Status status;
	int ret;

	/* Send request and receive response */
	ret = USART_SendRecvAsync(tx_buf, tx_len, rx_buf, rx_len_max, tx_repeat);
	if (ret)
		return ret;

	do {
		USART_Process();
	} while ((status = USART_GetStatus()) == USART_BUSY);

	ret = (status == USART_OK ? 0 : -1);
	if (!ret)
		ret = USART_CountRXBytes();

	return ret;
}

int USART_RecvSendSync(void *ctx, void *rx_buf, uint8_t rx_len_max,
		       int (*handle_rx)(void *ctx, uint8_t rx_len,
					void **tx_buf_out, uint8_t *tx_len_out))
{
	enum USART_Status status;
	uint8_t *tx_buf;
	uint8_t tx_len;
	int ret;

	/* Recv request */
	ret = USART_RecvAsync(rx_buf, rx_len_max);
	if (ret)
		return ret;

	do {
		USART_Process();
	} while ((status = USART_GetStatus()) == USART_BUSY);

	if (status != USART_OK)
		return -1;

	ret = handle_rx(ctx, USART_CountRXBytes(),
			(void **)&tx_buf, &tx_len);
	if (ret)
		return ret;

	/* Send response */
	ret = USART_SendAsync(tx_buf, tx_len);
	if (ret)
		return ret;

	do {
		USART_Process();
	} while ((status = USART_GetStatus()) == USART_BUSY);

	ret = (status == USART_OK ? 0 : -1);

	return ret;
}
