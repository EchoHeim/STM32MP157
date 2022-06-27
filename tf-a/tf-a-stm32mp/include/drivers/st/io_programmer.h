/*
 * Copyright (c) 2015-2019, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __IO_PROGRAMMER_H__
#define __IO_PROGRAMMER_H__

/* Phase definition */
#define PHASE_FLASHLAYOUT	0
#define PHASE_FSBL1		1
#define PHASE_FSBL2		2
#define PHASE_SSBL		3

/* Command definition */
#define GET_CMD_COMMAND		0x00
#define GET_VER_COMMAND		0x01
#define GET_ID_COMMAND		0x02
#define PHASE_COMMAND		0x03
#define START_COMMAND		0x21
#define DOWNLOAD_COMMAND	0x31

/* Answer defines  */
#define INIT_BYTE		0x7F
#define ACK_BYTE		0x79
#define NACK_BYTE		0x1F
#define ABORT			0x5F

#define PROGRAMMER_TIMEOUT	0xFFFFFFFE

#define DEVICE_ID_BYTE1		0x05
#define DEVICE_ID_BYTE2		0x00

/* phase structure */
struct phase_struct {
	uint32_t keep_header;
	uint32_t current_packet;
	size_t max_size;
	uint8_t phase_id;
};

/* current phase struct variable */
static struct phase_struct current_phase = {
	.phase_id = PHASE_FSBL1,
	.max_size = 0,
	.keep_header = 0,
	.current_packet = 0,
};

#endif /* __IO_PROGRAMMER_H__ */
