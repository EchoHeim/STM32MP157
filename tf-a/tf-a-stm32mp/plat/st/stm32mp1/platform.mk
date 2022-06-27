#
# Copyright (c) 2015-2020, ARM Limited and Contributors. All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
#

ARM_CORTEX_A7		:=	yes
ARM_WITH_NEON		:=	yes
BL2_AT_EL3		:=	1
USE_COHERENT_MEM	:=	0

# Add specific ST version
ST_VERSION 		:=	r1.0
VERSION_STRING		:=	v${VERSION_MAJOR}.${VERSION_MINOR}-${ST_VERSION}(${BUILD_TYPE}):${BUILD_STRING}

TRUSTED_BOARD_BOOT	:=	1

# Please don't increment this value without good understanding of
# the monotonic counter
STM32_TF_VERSION	?=	0
$(eval $(call add_define_val,STM32_TF_VERSION,${STM32_TF_VERSION}))

# Enable dynamic memory mapping
PLAT_XLAT_TABLES_DYNAMIC :=	1
$(eval $(call assert_boolean,PLAT_XLAT_TABLES_DYNAMIC))
$(eval $(call add_define,PLAT_XLAT_TABLES_DYNAMIC))

# STM32 image header version v1.0
STM32_HEADER_VERSION_MAJOR:=	1
STM32_HEADER_VERSION_MINOR:=	0

# Not needed for Cortex-A7
WORKAROUND_CVE_2017_5715:=	0

AARCH32_EXCEPTION_DEBUG	:=	1

# Number of TF-A copies in the device
STM32_TF_A_COPIES		:=	2
$(eval $(call add_define,STM32_TF_A_COPIES))
ifeq ($(AARCH32_SP),optee)
PLAT_PARTITION_MAX_ENTRIES	:=	$(shell echo $$(($(STM32_TF_A_COPIES) + 4)))
else
PLAT_PARTITION_MAX_ENTRIES	:=	$(shell echo $$(($(STM32_TF_A_COPIES) + 1)))
endif
$(eval $(call add_define,PLAT_PARTITION_MAX_ENTRIES))

# Boot devices
STM32MP_EMMC		?=	0
STM32MP_SDMMC		?=	0
STM32MP_RAW_NAND	?=	0
STM32MP_SPI_NAND	?=	0
STM32MP_SPI_NOR		?=	0

# Serial boot devices
STM32MP_UART_PROGRAMMER	?=	0
STM32MP_USB_PROGRAMMER	?=	0

ifeq ($(filter 1,${STM32MP_EMMC} ${STM32MP_SDMMC} ${STM32MP_RAW_NAND} \
	${STM32MP_SPI_NAND} ${STM32MP_SPI_NOR} ${STM32MP_UART_PROGRAMMER} \
	${STM32MP_USB_PROGRAMMER}),)
$(error "No boot device driver is enabled")
endif

$(eval $(call assert_boolean,STM32MP_EMMC))
$(eval $(call assert_boolean,STM32MP_SDMMC))
$(eval $(call assert_boolean,STM32MP_RAW_NAND))
$(eval $(call assert_boolean,STM32MP_SPI_NAND))
$(eval $(call assert_boolean,STM32MP_SPI_NOR))
$(eval $(call add_define,STM32MP_EMMC))
$(eval $(call add_define,STM32MP_SDMMC))
$(eval $(call add_define,STM32MP_RAW_NAND))
$(eval $(call add_define,STM32MP_SPI_NAND))
$(eval $(call add_define,STM32MP_SPI_NOR))

$(eval $(call assert_boolean,STM32MP_UART_PROGRAMMER))
$(eval $(call assert_boolean,STM32MP_USB_PROGRAMMER))
$(eval $(call add_define,STM32MP_UART_PROGRAMMER))
$(eval $(call add_define,STM32MP_USB_PROGRAMMER))

PLAT_INCLUDES		:=	-Iplat/st/common/include/
PLAT_INCLUDES		+=	-Iplat/st/stm32mp1/include/

# Device tree
DTB_FILE_NAME		?=	stm32mp157c-ev1.dtb
FDT_SOURCES		:=	$(addprefix fdts/, $(patsubst %.dtb,%.dts,$(DTB_FILE_NAME)))
DTC_FLAGS		+=	-Wno-unit_address_vs_reg

include lib/libfdt/libfdt.mk

PLAT_BL_COMMON_SOURCES	:=	plat/st/common/stm32mp_common.c				\
				plat/st/stm32mp1/stm32mp1_private.c

PLAT_BL_COMMON_SOURCES	+=	drivers/st/uart/aarch32/stm32_console.S

ifneq (${ENABLE_STACK_PROTECTOR},0)
PLAT_BL_COMMON_SOURCES	+=	plat/st/stm32mp1/stm32mp1_stack_protector.c
endif

include lib/xlat_tables_v2/xlat_tables.mk
PLAT_BL_COMMON_SOURCES	+=	${XLAT_TABLES_LIB_SRCS}

PLAT_BL_COMMON_SOURCES	+=	lib/cpus/aarch32/cortex_a7.S

PLAT_BL_COMMON_SOURCES	+=	drivers/arm/tzc/tzc400.c				\
				drivers/delay_timer/delay_timer.c			\
				drivers/delay_timer/generic_delay_timer.c		\
				drivers/st/bsec/bsec2.c					\
				drivers/st/clk/stm32mp_clkfunc.c			\
				drivers/st/clk/stm32mp1_clk.c				\
				drivers/st/ddr/stm32mp1_ddr_helpers.c			\
				drivers/st/gpio/stm32_gpio.c				\
				drivers/st/i2c/stm32_i2c.c				\
				drivers/st/iwdg/stm32_iwdg.c				\
				drivers/st/pmic/stm32mp_pmic.c				\
				drivers/st/pmic/stpmic1.c				\
				drivers/st/regulator/stm32mp_dummy_regulator.c		\
				drivers/st/regulator/stm32mp_regulator.c		\
				drivers/st/reset/stm32mp1_reset.c			\
				plat/st/common/stm32mp_dt.c				\
				plat/st/common/stm32mp_shres_helpers.c			\
				plat/st/stm32mp1/stm32mp1_context.c			\
				plat/st/stm32mp1/stm32mp1_dbgmcu.c			\
				plat/st/stm32mp1/stm32mp1_helper.S			\
				plat/st/stm32mp1/stm32mp1_security.c			\
				plat/st/stm32mp1/stm32mp1_syscfg.c

BL2_SOURCES		+=	drivers/io/io_block.c					\
				drivers/io/io_dummy.c					\
				drivers/io/io_mtd.c					\
				drivers/io/io_storage.c					\
				drivers/st/crypto/stm32_hash.c				\
				drivers/st/io/io_stm32image.c				\
				plat/st/common/bl2_io_storage.c				\
				plat/st/stm32mp1/bl2_plat_setup.c

ifeq (${TRUSTED_BOARD_BOOT},1)
AUTH_SOURCES		:=	drivers/auth/auth_mod.c					\
				drivers/auth/crypto_mod.c				\
				drivers/auth/img_parser_mod.c

BL2_SOURCES		+= 	$(AUTH_SOURCES)						\
				plat/st/common/stm32mp_cot.c				\
				plat/st/common/stm32mp_crypto_lib.c			\
				plat/st/common/stm32mp_img_parser_lib.c			\
				plat/st/common/stm32mp_trusted_boot.c
endif

ifneq ($(filter 1,${STM32MP_EMMC} ${STM32MP_SDMMC}),)
BL2_SOURCES		+=	drivers/mmc/mmc.c					\
				drivers/partition/gpt.c					\
				drivers/partition/partition.c				\
				drivers/st/io/io_mmc.c					\
				drivers/st/mmc/stm32_sdmmc2.c
endif

ifeq (${STM32MP_RAW_NAND},1)
$(eval $(call add_define_val,NAND_ONFI_DETECT,1))
BL2_SOURCES		+=	drivers/mtd/nand/raw_nand.c				\
				drivers/st/fmc/stm32_fmc2_nand.c
endif

ifeq (${STM32MP_SPI_NAND},1)
BL2_SOURCES		+=	drivers/mtd/nand/spi_nand.c
endif

ifeq (${STM32MP_SPI_NOR},1)
BL2_SOURCES		+=	drivers/mtd/nor/spi_nor.c
endif

ifneq ($(filter 1,${STM32MP_SPI_NAND} ${STM32MP_SPI_NOR}),)
BL2_SOURCES		+=	drivers/mtd/spi-mem/spi_mem.c				\
				drivers/st/spi/stm32_qspi.c
endif

ifneq ($(filter 1,${STM32MP_RAW_NAND} ${STM32MP_SPI_NAND}),)
BL2_SOURCES		+=	drivers/mtd/nand/core.c
endif

ifneq ($(filter 1,${STM32MP_RAW_NAND} ${STM32MP_SPI_NAND} ${STM32MP_SPI_NOR}),)
BL2_SOURCES		+=	plat/st/stm32mp1/stm32mp1_boot_device.c
endif

ifeq (${STM32MP_UART_PROGRAMMER},1)
BL2_SOURCES		+=	drivers/st/uart/io_programmer_uart.c			\
				drivers/st/uart/stm32mp1xx_hal_uart.c
endif

BL2_SOURCES		+=	drivers/st/ddr/stm32mp1_ddr.c				\
				drivers/st/ddr/stm32mp1_ram.c

BL2_SOURCES		+=	common/desc_image_load.c				\
				plat/st/stm32mp1/plat_bl2_mem_params_desc.c		\
				plat/st/stm32mp1/plat_image_load.c

ifeq (${STM32MP_USB_PROGRAMMER},1)
BL2_SOURCES		+=	drivers/st/io/io_programmer_st_usb.c			\
				drivers/st/usb_dwc2/usb_dwc2.c				\
				lib/usb/usb_core.c					\
				lib/usb/usb_st_dfu.c					\
				plat/st/stm32mp1/stm32mp1_usb_desc.c
endif

ifeq ($(AARCH32_SP),optee)
BL2_SOURCES		+=	lib/optee/optee_utils.c
endif


# Do not use neon in TF-A code, it leads to issues in low-power functions
TF_CFLAGS		+=	-mfloat-abi=soft

# Macros and rules to build TF binary
STM32_TF_ELF_LDFLAGS	:=	--hash-style=gnu --as-needed
STM32_DT_BASENAME	:=	$(DTB_FILE_NAME:.dtb=)
STM32_TF_STM32		:=	${BUILD_PLAT}/tf-a-${STM32_DT_BASENAME}.stm32
STM32_TF_BINARY		:=	$(STM32_TF_STM32:.stm32=.bin)
STM32_TF_MAPFILE	:=	$(STM32_TF_STM32:.stm32=.map)
STM32_TF_LINKERFILE	:=	$(STM32_TF_STM32:.stm32=.ld)
STM32_TF_ELF		:=	$(STM32_TF_STM32:.stm32=.elf)
STM32_TF_DTBFILE	:=      ${BUILD_PLAT}/fdts/${DTB_FILE_NAME}
STM32_TF_OBJS		:=	${BUILD_PLAT}/stm32mp1.o

# Variables for use with stm32image
STM32IMAGEPATH		?= tools/stm32image
STM32IMAGE		?= ${STM32IMAGEPATH}/stm32image${BIN_EXT}
STM32IMAGE_SRC		:= ${STM32IMAGEPATH}/stm32image.c

.PHONY: check_dtc_version stm32image clean_stm32image
.SUFFIXES:

all: check_dtc_version stm32image ${STM32_TF_STM32}

ifeq ($(AARCH32_SP),sp_min)
# BL32 is built only if using SP_MIN
BL32_DEP		:= bl32
BL32_PATH		:= -DBL32_BIN_PATH=\"${BUILD_PLAT}/bl32.bin\"
endif

distclean realclean clean: clean_stm32image

stm32image: ${STM32IMAGE}

${STM32IMAGE}: ${STM32IMAGE_SRC}
	${Q}${MAKE} CPPFLAGS="" --no-print-directory -C ${STM32IMAGEPATH}

clean_stm32image:
	${Q}${MAKE} --no-print-directory -C ${STM32IMAGEPATH} clean

check_dtc_version:
	$(eval DTC_V = $(shell $(DTC) -v | awk '{print $$NF}'))
	$(eval DTC_VERSION = $(shell printf "%d" $(shell echo ${DTC_V} | cut -d- -f1 | sed "s/\./0/g")))
	@if [ ${DTC_VERSION} -lt 10404 ]; then \
		echo "dtc version too old (${DTC_V}), you need at least version 1.4.4"; \
		false; \
	fi


${STM32_TF_OBJS}:	plat/st/stm32mp1/stm32mp1.S bl2 ${BL32_DEP} ${STM32_TF_DTBFILE}
			@echo "  AS      $<"
			${Q}${AS} ${ASFLAGS} ${TF_CFLAGS} \
				${BL32_PATH} \
				-DBL2_BIN_PATH=\"${BUILD_PLAT}/bl2.bin\" \
				-DDTB_BIN_PATH=\"${STM32_TF_DTBFILE}\" \
				-c plat/st/stm32mp1/stm32mp1.S -o $@

${STM32_TF_LINKERFILE}:	plat/st/stm32mp1/stm32mp1.ld.S ${BUILD_PLAT}
			@echo "  LDS     $<"
			${Q}${AS} ${ASFLAGS} ${TF_CFLAGS} -P -E $< -o $@

${STM32_TF_ELF}:	${STM32_TF_OBJS} ${STM32_TF_LINKERFILE}
			@echo "  LDS     $<"
			${Q}${LD} -o $@ ${STM32_TF_ELF_LDFLAGS} -Map=${STM32_TF_MAPFILE} --script ${STM32_TF_LINKERFILE} ${STM32_TF_OBJS}

${STM32_TF_BINARY}:	${STM32_TF_ELF}
			${Q}${OC} -O binary ${STM32_TF_ELF} $@
			@echo
			@echo "Built $@ successfully"
			@echo

${STM32_TF_STM32}:	${STM32IMAGE} ${STM32_TF_BINARY}
			@echo
			@echo "Generate $@"
			$(eval LOADADDR =  $(shell cat ${STM32_TF_MAPFILE} | grep RAM | awk '{print $$2}'))
			$(eval ENTRY =  $(shell cat ${STM32_TF_MAPFILE} | grep "__BL2_IMAGE_START" | awk '{print $$1}'))
			@${STM32IMAGE} -s ${STM32_TF_BINARY} -d $@ \
				-l $(LOADADDR) -e ${ENTRY} \
				-v ${STM32_TF_VERSION} \
				-m ${STM32_HEADER_VERSION_MAJOR} \
				-n ${STM32_HEADER_VERSION_MINOR}
			@echo
