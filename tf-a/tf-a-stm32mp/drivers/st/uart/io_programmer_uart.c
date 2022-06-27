/*
 * Copyright (c) 2015-2020, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <errno.h>
#include <string.h>

#include <platform_def.h>

#include <arch_helpers.h>
#include <common/debug.h>
#include <drivers/io/io_driver.h>
#include <drivers/io/io_storage.h>
#include <drivers/st/io_programmer.h>
#include <drivers/st/io_stm32image.h>
#include <drivers/st/io_uart.h>
#include <drivers/st/stm32_iwdg.h>

/* USART bootloader protocol version V4.0*/
#define USART_BL_VERSION	0x40

#define UART_ISR_ERRORS		(USART_ISR_ORE | USART_ISR_NE | \
				 USART_ISR_FE | USART_ISR_PE)

/* array of supported command */
static const uint8_t command_tab[] = {
	GET_CMD_COMMAND,
	GET_VER_COMMAND,
	GET_ID_COMMAND,
	PHASE_COMMAND,
	START_COMMAND,
	DOWNLOAD_COMMAND
};

static uint8_t header_buffer[512] __aligned(4);
static int32_t header_length_read;

/* UART device functions */
static int uart_dev_open(const uintptr_t init_params, io_dev_info_t **dev_info);
static int uart_block_open(io_dev_info_t *dev_info, const uintptr_t spec,
			   io_entity_t *entity);
static int uart_dev_init(io_dev_info_t *dev_info, const uintptr_t init_params);
static int uart_block_read(io_entity_t *entity, uintptr_t buffer,
			   size_t length, size_t *length_read);
static int uart_block_close(io_entity_t *entity);
static int uart_dev_close(io_dev_info_t *dev_info);
static int uart_block_len(io_entity_t *entity, size_t *length);
static io_type_t device_type_uart(void);

/* variables */
static const io_dev_connector_t uart_dev_connector = {
	.dev_open = uart_dev_open
};

static const io_dev_funcs_t uart_dev_funcs = {
	.type = device_type_uart,
	.open = uart_block_open,
	.seek = NULL,
	.size = uart_block_len,
	.read = uart_block_read,
	.write = NULL,
	.close = uart_block_close,
	.dev_init = uart_dev_init,
	.dev_close = uart_dev_close,
};

static const io_dev_info_t uart_dev_info = {
	.funcs = &uart_dev_funcs,
	.info = (uintptr_t)0,
};

static UART_HandleTypeDef *uart_handle_programmer;

/* Identify the device type as memmap */
static io_type_t device_type_uart(void)
{
	return IO_TYPE_UART;
}

static int uart_write_byte(uint8_t byte)
{
	if (HAL_UART_GetState(uart_handle_programmer) == HAL_UART_STATE_RESET)
		uart_handle_programmer->gState = HAL_UART_STATE_READY;
	return HAL_UART_Transmit(uart_handle_programmer, (uint8_t *)&byte, 1,
				 PROGRAMMER_TIMEOUT);
}

static int uart_write_uint32(uint32_t value)
{
	if (HAL_UART_GetState(uart_handle_programmer) == HAL_UART_STATE_RESET)
		uart_handle_programmer->gState = HAL_UART_STATE_READY;
	return HAL_UART_Transmit(uart_handle_programmer, (uint8_t *)&value, 4,
				 PROGRAMMER_TIMEOUT);
}

static int uart_read_byte(uint8_t *byte)
{
	HAL_StatusTypeDef ret;

	if (HAL_UART_GetState(uart_handle_programmer) == HAL_UART_STATE_RESET)
		uart_handle_programmer->RxState = HAL_UART_STATE_READY;

	ret = HAL_UART_Receive(uart_handle_programmer, byte, 1,
			       PROGRAMMER_TIMEOUT);

	if (ret || (uart_handle_programmer->Instance->ISReg & UART_ISR_ERRORS))
		return -EIO;

	return 0;
}

static void uart_flush_rx_fifo(uint32_t timeout)
{
	uint8_t byte = 0;

	/* Clear all errors */
	__HAL_UART_CLEAR_FLAG(uart_handle_programmer, UART_ISR_ERRORS);

	while ((__HAL_UART_GET_FLAG(uart_handle_programmer, UART_FLAG_RXNE) !=
		RESET)  && timeout) {
		timeout--;
		uart_read_byte(&byte);
	}
}

/* Open a connection to the uart device */
static int uart_dev_open(const uintptr_t init_params, io_dev_info_t **dev_info)
{
	int result = 0;

	assert(dev_info);
	*dev_info = (io_dev_info_t *)&uart_dev_info;

	uart_handle_programmer = (UART_HandleTypeDef *)init_params;

	/* Init UART to enable FIFO mode */
	if (HAL_UART_Init(uart_handle_programmer) != HAL_OK) {
		return -EIO;
	}

	uart_flush_rx_fifo(PROGRAMMER_TIMEOUT);

	uart_write_byte(NACK_BYTE);

	return result;
}

static int uart_dev_init(io_dev_info_t *dev_info, const uintptr_t init_params)
{
	return 0;
}

/* Close a connection to the uart device */
static int uart_dev_close(io_dev_info_t *dev_info)
{
	return 0;
}

/* Return the size of a file on the uart device */
static int uart_block_len(io_entity_t *entity, size_t *length)
{
	boot_api_image_header_t *header =
			(boot_api_image_header_t *)&header_buffer[0];

	assert(entity);
	assert(length);

	header_length_read = 0;
	header->magic = 0;

	uart_block_read(entity, (uintptr_t)&header_buffer[0],
			sizeof(boot_api_image_header_t),
			(size_t *)&header_length_read);

	if (header->magic != BOOT_API_IMAGE_HEADER_MAGIC_NB)
		return -EIO;

	if (header->image_length > current_phase.max_size)
		return -EIO;

	*length = header->image_length;

	INFO("binary size 0x%x\n", header->image_length);

	return 0;
}

/* Open a file on the uart device */
static int uart_block_open(io_dev_info_t *dev_info, const  uintptr_t spec,
			   io_entity_t *entity)
{
	int result = -EIO;
	const struct stm32image_part_info *partition_spec =
		(struct stm32image_part_info *)spec;
	uint32_t length = 0;
	uint32_t layout_length = 0;

	/* Use PHASE_FSBL1 like init value*/
	if (current_phase.phase_id == PHASE_FSBL1) {
		assert(partition_spec);
		assert(entity);

		current_phase.current_packet = 0;

		if (!strcmp(partition_spec->name, BL33_IMAGE_NAME)) {
			/* read flashlayout first for U-boot */
			current_phase.phase_id = PHASE_FLASHLAYOUT;
			current_phase.max_size = FLASHLAYOUT_LIMIT;
			current_phase.keep_header = 1;
			uart_block_len(entity, &layout_length);
			uart_block_read(entity, FLASHLAYOUT_BASE,
					layout_length,
					&length);

			flush_dcache_range((unsigned long)FLASHLAYOUT_BASE,
					   layout_length +
					   sizeof(boot_api_image_header_t));

			current_phase.current_packet = 0;
			current_phase.phase_id = PHASE_SSBL;
			current_phase.max_size = dt_get_ddr_size();
			current_phase.keep_header = 0;
		}
		entity->info = (uintptr_t)&current_phase;
		result = 0;
	} else {
		WARN("A UART device is already active. Close first.\n");
		result = -EIO;
	}
	return result;
}

static int uart_receive_command(uint8_t *command)
{
	uint8_t byte = 0;
	uint8_t xor = 0;
	uint8_t counter = 0, found = 0;
	int ret;

	/* check command */
	ret = uart_read_byte(&byte);
	if (ret)
		return ret;

	for (counter = 0; counter < sizeof(command_tab); counter++) {
		if (command_tab[counter] == byte) {
			found = 1;
			break;
		}
	}

	if (found) {
		ret = uart_read_byte(&xor);
		if (ret)
			return ret;
		if ((byte ^ xor) != 0xFF) {
			WARN("UART: Command XOR check fail (byte=0x%x, xor=0x%x)\n",
			     byte, xor);
			return -EPROTO;
		}
	} else {
		if (byte != INIT_BYTE) {
			WARN("UART: Command unknown (byte=0x%x)\n", byte);
			return -EPROTO;
		}
	}

	*command = byte;

	return 0;
}

static int get_cmd_command(void)
{
	uint8_t counter = 0x0;

	uart_write_byte(sizeof(command_tab));
	uart_write_byte(USART_BL_VERSION);

	for (counter = 0; counter < sizeof(command_tab); counter++)
		uart_write_byte(command_tab[counter]);

	return 0;
}

static int get_version_command(void)
{
	uart_write_byte(STM32_TF_VERSION);

	return 0;
}

static int get_id_command(void)
{
	/* Send Device IDCode */
	uart_write_byte(0x1);
	uart_write_byte(DEVICE_ID_BYTE1);
	uart_write_byte(DEVICE_ID_BYTE2);

	return 0;
}

static int uart_send_phase(uint32_t address)
{
	uart_write_byte(0x05);			/* length of data - 1 */
	/* Send the ID of next partition */
	uart_write_byte(current_phase.phase_id);	/* partition ID */

	uart_write_uint32(address);		/* destination address */
	uart_write_byte(0x00);			/* length of extra data */

	return 0;
}

static int uart_download_part(uint8_t *buffer, uint32_t *length_read)
{
	uint8_t byte = 0;
	uint8_t xor = 0;
	uint8_t operation = 0;
	uint32_t packet_number = 0;
	uint8_t packet_size = 0;
	int i = 0;
	volatile uint8_t *ptr = (uint8_t *)buffer;

	/* get operation number */
	if (uart_read_byte(&operation))
		return -EIO;
	xor = operation;

	/* get packet Number */
	for (i = 3, byte = 0; i > 0; i--) {
		if (uart_read_byte(&byte))
			return -EIO;
		xor ^= byte;
		packet_number = (packet_number << 8) | byte;
	}

	if (packet_number != current_phase.current_packet) {
		WARN("UART: Bad packet number receive: %i\n", packet_number);
		return -EPROTO;
	}

	/* checksum */
	if (uart_read_byte(&byte))
		return -EIO;

	if (xor != byte) {
		WARN("UART: Download Command header checksum fail: calculated xor: %i, received xor:%i\n",
		     xor, byte);
		return -EPROTO;
	}

	uart_write_byte(ACK_BYTE);

	if (uart_read_byte(&packet_size))
		return -EIO;

	xor = packet_size;

	for (i = packet_size; i >= 0; i--) {
		/* Reload watchdog, once every 8 loops */
		if (i % 8)
			stm32_iwdg_refresh();

		if (uart_read_byte(&byte))
			return -EIO;
		*(volatile uint8_t *)ptr = byte;
		xor ^= byte;
		ptr++;
	}

	/* checksum */
	if (uart_read_byte(&byte))
		return -EIO;

	if (xor != byte) {
		WARN("UART: Download Command data checksum fail: calculated xor: 0x%x, received xor:0x%x\n",
		     xor, byte);
		return -EPROTO;
	}

	current_phase.current_packet++;
	*length_read = (uint32_t)packet_size + 1;

	return 0;
}

static int uart_start_cmd(boot_api_image_header_t *header, uintptr_t buffer)
{
	uint8_t byte = 0;
	uint8_t xor = 0;
	int i = 0;
	uint32_t start_address = 0;
	int result = 0;

	/* get address */
	for (i = 4; i > 0; i--) {
		if (uart_read_byte(&byte))
			return -EIO;
		xor ^= byte;
		start_address = (start_address << 8) | byte;
	}

	/* checksum */
	if (uart_read_byte(&byte))
		return -EIO;

	if (xor != byte) {
		WARN("UART: Start command checksum fail: calculated xor: %i, received xor:%i\n",
		     xor, byte);
		return -EPROTO;
	}

	switch (current_phase.phase_id) {
	case PHASE_FLASHLAYOUT:
		result = stm32mp_check_header(header, buffer +
				 sizeof(boot_api_image_header_t));
		if (result)
			return result;
		break;

	case PHASE_SSBL:
		if (start_address != BL33_BASE) {
			VERBOSE("BL33 address provided: 0x%x, using: 0x%x\n",
				start_address, BL33_BASE);
		}

		return stm32mp_check_header(header, buffer);

	default:
		ERROR("Invalid phase ID : %i\n", current_phase.phase_id);
		return -EINVAL;
	}

	return 0;
}

/* Read data from the uart device */
static int uart_block_read(io_entity_t *entity, uintptr_t buffer,
			   size_t length, size_t *length_read)
{
	uint32_t read_length = 0;
	uint32_t total_length = 0;
	uint32_t ptr_offset = 0;
	uint8_t command = 0;
	uint8_t all_commands_done = 0;
	boot_api_image_header_t *header =
			(boot_api_image_header_t *)&header_buffer[0];

	if (header_length_read && current_phase.keep_header) {
		memcpy((uint8_t *)buffer, (uint8_t *)&header_buffer[0],
		       header_length_read);
		ptr_offset += header_length_read;
	} else if (header_length_read &&
		  ((header_length_read -
		    sizeof(boot_api_image_header_t)) > 0)) {
#if TRUSTED_BOARD_BOOT
		stm32mp_save_loaded_header(header_buffer);
#endif
		memcpy((uint8_t *)buffer,
		       (uint8_t *)
		       &header_buffer[sizeof(boot_api_image_header_t)],
		       header_length_read -
		       sizeof(boot_api_image_header_t));
		ptr_offset += header_length_read -
			      sizeof(boot_api_image_header_t);
	}

	while (!all_commands_done) {
		int result;

		/* Reload watchdog */
		stm32_iwdg_refresh();

		result = uart_receive_command(&command);
		if (result) {
			if (result == -EIO) {
				WARN("UART: device error on command receive\n");
			}

			mdelay(2);

			uart_flush_rx_fifo(PROGRAMMER_TIMEOUT);
			uart_write_byte(NACK_BYTE);
			continue;
		}

		/* Send ack */
		uart_write_byte(ACK_BYTE);

		switch (command) {
		case INIT_BYTE:
			/* Nothing to do */
			continue;
		case GET_CMD_COMMAND:
			result = get_cmd_command();
			break;

		case GET_VER_COMMAND:
			result = get_version_command();
			break;

		case GET_ID_COMMAND:
			result = get_id_command();
			break;

		case PHASE_COMMAND:
			result = uart_send_phase((uint32_t)buffer);
			break;

		case DOWNLOAD_COMMAND:
			result = uart_download_part((uint8_t *)(buffer +
								ptr_offset),
						    &read_length);
			if (!result) {
				ptr_offset += read_length;
				total_length += read_length;
				if ((total_length >= length) &&
				    !header_length_read) {
					/* read header only go out*/
					all_commands_done = 1;
				}
			}

			break;

		case START_COMMAND:
			result = uart_start_cmd(header, buffer);
			if (!result)
				all_commands_done = 1;

			break;

		default:
			/* Not supported command  */
			WARN("UART: Unknown command\n");
			uart_flush_rx_fifo(PROGRAMMER_TIMEOUT);
			uart_write_byte(NACK_BYTE);
			continue;
		}
		if (!result) {
			/* Send ack */
			uart_write_byte(ACK_BYTE);
		} else if (result == -EPROTO) {
			uart_flush_rx_fifo(0xFFFF);
			uart_write_byte(NACK_BYTE);
		} else {
			uart_flush_rx_fifo(PROGRAMMER_TIMEOUT);
			uart_write_byte(NACK_BYTE);
			continue;
		}
	}

	*length_read = total_length;

	INFO("Read block in buffer 0x%lx size 0x%x phase ID %i\n",
	     buffer, length, current_phase.phase_id);

	return 0;
}

/* Close a file on the uart device */
static int uart_block_close(io_entity_t *entity)
{
	current_phase.phase_id = PHASE_FSBL1;

	return 0;
}

/* Exported functions */

/* Register the uart driver with the IO abstraction */
int register_io_dev_uart(const io_dev_connector_t **dev_con)
{
	int result;

	assert(dev_con);

	result = io_register_device(&uart_dev_info);
	if (!result)
		*dev_con = &uart_dev_connector;

	return result;
}
