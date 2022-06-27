/*
 * Copyright (c) 2015-2020, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <platform_def.h>

#include <arch.h>
#include <asm_macros.S>
#include <common/bl_common.h>
#include <drivers/st/stm32_gpio.h>
#include <smccc_helpers.h>

#define GPIO_TX_SHIFT		(DEBUG_UART_TX_GPIO_PORT << 1)

	.globl	platform_mem_init
	.globl	plat_report_exception
#if AARCH32_EXCEPTION_DEBUG
	.globl	plat_report_undef_inst
	.globl	plat_report_prefetch_abort
	.globl	plat_report_data_abort
#endif
	.globl	plat_get_my_entrypoint
	.globl	plat_secondary_cold_boot_setup
	.globl	plat_reset_handler
	.globl	plat_is_my_cpu_primary
	.globl	plat_my_core_pos
	.globl	plat_crash_console_init
	.globl	plat_crash_console_flush
	.globl	plat_crash_console_putc
	.globl	plat_panic_handler
	.globl	wfi_svc_int_enable

func platform_mem_init
	/* Nothing to do, don't need to init SYSRAM */
	bx	lr
endfunc platform_mem_init

func plat_report_exception
#if DEBUG
	mov	r8, lr

	/* Test if an abort occurred */
	cmp	r0, #MODE32_abt
	bne	undef_inst_lbl
	ldr	r4, =abort_str
	bl	asm_print_str
	b	print_excpetion_info

undef_inst_lbl:
	/* Test for an undefined instruction */
	cmp	r0, #MODE32_und
	bne	other_excpetion_lbl
	ldr	r4, =undefined_str
	bl	asm_print_str
	b	print_excpetion_info

other_excpetion_lbl:
	/* Other exceptions */
	mov	r9, r0
	ldr	r4, =exception_start_str
	bl	asm_print_str
	mov	r4, r9
	bl	asm_print_hex
	ldr	r4, =exception_end_str
	bl	asm_print_str

print_excpetion_info:
	mrs	r4, lr_svc
	sub	r4, r4, #4
	bl	asm_print_hex

	ldr	r4, =end_error_str
	bl	asm_print_str

	bx	r8
#else
	bx	lr
#endif
endfunc plat_report_exception

#if AARCH32_EXCEPTION_DEBUG
func plat_report_undef_inst
#if DEBUG
	mov	r8, lr

	mov	r9, r0

	ldr	r4, =undefined_str
	bl	asm_print_str

	mov	r4, r9
	sub	r4, r4, #4
	bl	asm_print_hex

	ldr	r4, =end_error_str
	bl	asm_print_str

	bx	r8
#else
	bx	lr
#endif
endfunc plat_report_undef_inst

func plat_report_prefetch_abort
#if DEBUG
	mov	r8, lr
	mov	r9, r0

	ldr	r4, =prefetch_abort_str
	bl	asm_print_str

	mov	r4, r9
	sub	r4, r4, #4
	bl	asm_print_hex

	ldr	r4, =ifsr_str
	bl	asm_print_str

	ldcopr	r4, IFSR
	bl	asm_print_hex

	ldr	r4, =ifar_str
	bl	asm_print_str

	ldcopr	r4, IFAR
	bl	asm_print_hex

	ldr	r4, =end_error_str
	bl	asm_print_str

	bx	r8
#else
	bx	lr
#endif
endfunc plat_report_prefetch_abort

func plat_report_data_abort
#if DEBUG
	mov	r8, lr
	mov	r9, r0

	ldr	r4, =data_abort_str
	bl	asm_print_str

	mov	r4, r9
	sub	r4, r4, #8
	bl	asm_print_hex

	ldr	r4, =dfsr_str
	bl	asm_print_str

	ldcopr	r4, DFSR
	bl	asm_print_hex

	ldr	r4, =dfar_str
	bl	asm_print_str

	ldcopr	r4, DFAR
	bl	asm_print_hex

	ldr	r4, =end_error_str
	bl	asm_print_str

	bx	r8
#else
	bx	lr
#endif
endfunc plat_report_data_abort
#endif

func plat_reset_handler
	bx	lr
endfunc plat_reset_handler

	/* ------------------------------------------------------------------
	 * unsigned long plat_get_my_entrypoint (void);
	 *
	 * Main job of this routine is to distinguish between a cold and warm
	 * boot.
	 *
	 * Currently supports only cold boot
	 * ------------------------------------------------------------------
	 */
func plat_get_my_entrypoint
	mov	r0, #0
	bx	lr
endfunc plat_get_my_entrypoint

	/* ---------------------------------------------
	 * void plat_secondary_cold_boot_setup (void);
	 *
	 * Cold-booting secondary CPUs is not supported.
	 * ---------------------------------------------
	 */
func plat_secondary_cold_boot_setup
	b	.
endfunc plat_secondary_cold_boot_setup

	/* -----------------------------------------------------
	 * unsigned int plat_is_my_cpu_primary (void);
	 *
	 * Find out whether the current cpu is the primary cpu.
	 * -----------------------------------------------------
	 */
func plat_is_my_cpu_primary
	ldcopr	r0, MPIDR
	ldr	r1, =(MPIDR_CLUSTER_MASK | MPIDR_CPU_MASK)
	and	r0, r1
	cmp	r0, #STM32MP_PRIMARY_CPU
	moveq	r0, #1
	movne	r0, #0
	bx	lr
endfunc plat_is_my_cpu_primary

	/* -------------------------------------------
	 *  int plat_stm32mp1_get_core_pos(int mpidr);
	 *
	 *  Return CorePos = (ClusterId * 4) + CoreId
	 * -------------------------------------------
	 */
func plat_stm32mp1_get_core_pos
	and	r1, r0, #MPIDR_CPU_MASK
	and	r0, r0, #MPIDR_CLUSTER_MASK
	add	r0, r1, r0, LSR #6
	bx	lr
endfunc plat_stm32mp1_get_core_pos

	/* ------------------------------------
	 *  unsigned int plat_my_core_pos(void)
	 * ------------------------------------
	 */
func plat_my_core_pos
	ldcopr	r0, MPIDR
	b	plat_stm32mp1_get_core_pos
endfunc plat_my_core_pos

	/* ---------------------------------------------
	 * int plat_crash_console_init(void)
	 *
	 * Initialize the crash console without a C Runtime stack.
	 * ---------------------------------------------
	 */
func plat_crash_console_init
	/* Enable GPIOs for UART TX */
	ldr	r1, =(RCC_BASE + DEBUG_UART_TX_GPIO_BANK_CLK_REG)
	ldr	r2, [r1]
	/* Configure GPIO */
	orr	r2, r2, #DEBUG_UART_TX_GPIO_BANK_CLK_EN
	str	r2, [r1]
	ldr	r1, =DEBUG_UART_TX_GPIO_BANK_ADDRESS
	/* Set GPIO mode alternate */
	ldr	r2, [r1, #GPIO_MODE_OFFSET]
	bic	r2, r2, #(GPIO_MODE_MASK << GPIO_TX_SHIFT)
	orr	r2, r2, #(GPIO_MODE_ALTERNATE << GPIO_TX_SHIFT)
	str	r2, [r1, #GPIO_MODE_OFFSET]
	/* Set GPIO speed low */
	ldr	r2, [r1, #GPIO_SPEED_OFFSET]
	bic	r2, r2, #(GPIO_SPEED_MASK << GPIO_TX_SHIFT)
	str	r2, [r1, #GPIO_SPEED_OFFSET]
	/* Set no-pull */
	ldr	r2, [r1, #GPIO_PUPD_OFFSET]
	bic	r2, r2, #(GPIO_PULL_MASK << GPIO_TX_SHIFT)
	str	r2, [r1, #GPIO_PUPD_OFFSET]
	/* Set alternate */
	ldr	r2, =DEBUG_UART_TX_GPIO_PORT
	cmp	r2, #GPIO_ALT_LOWER_LIMIT
	ldrge	r2, [r1, #GPIO_AFRH_OFFSET]
	bicge	r2, r2, #(GPIO_ALTERNATE_MASK << ((DEBUG_UART_TX_GPIO_PORT - GPIO_ALT_LOWER_LIMIT) << 2))
	orrge	r2, r2, #(DEBUG_UART_TX_GPIO_ALTERNATE << ((DEBUG_UART_TX_GPIO_PORT - GPIO_ALT_LOWER_LIMIT) << 2))
	strge	r2, [r1, #GPIO_AFRH_OFFSET]
	ldrlt	r2, [r1, #GPIO_AFRL_OFFSET]
	biclt	r2, r2, #(GPIO_ALTERNATE_MASK << (DEBUG_UART_TX_GPIO_PORT << 2))
	orrlt	r2, r2, #(DEBUG_UART_TX_GPIO_ALTERNATE << (DEBUG_UART_TX_GPIO_PORT << 2))
	strlt	r2, [r1, #GPIO_AFRL_OFFSET]
	/* Enable UART clock, with its source */
	ldr	r1, =(RCC_BASE + DEBUG_UART_TX_CLKSRC_REG)
	mov	r2, #DEBUG_UART_TX_CLKSRC
	str	r2, [r1]
	ldr	r1, =(RCC_BASE + DEBUG_UART_TX_EN_REG)
	ldr	r2, [r1]
	orr	r2, r2, #DEBUG_UART_TX_EN
	str	r2, [r1]

	ldr	r0, =STM32MP_DEBUG_USART_BASE
	ldr	r1, =STM32MP_DEBUG_USART_CLK_FRQ
	ldr	r2, =STM32MP_UART_BAUDRATE
	b	console_stm32_core_init
endfunc plat_crash_console_init

	/* ---------------------------------------------
	 * int plat_crash_console_flush(void)
	 *
	 * Flush the crash console without a C Runtime stack.
	 * ---------------------------------------------
	 */
func plat_crash_console_flush
	ldr	r1, =STM32MP_DEBUG_USART_BASE
	b	console_stm32_core_flush
endfunc plat_crash_console_flush

	/* ---------------------------------------------
	 * int plat_crash_console_putc(int c)
	 *
	 * Print a character on the crash console without a C Runtime stack.
	 * Clobber list : r1 - r3
	 *
	 * In case of bootloading through uart, we keep console crash as this.
	 * Characters could be sent to the programmer, but will be ignored.
	 * No specific code in that case.
	 * ---------------------------------------------
	 */
func plat_crash_console_putc
	ldr	r1, =STM32MP_DEBUG_USART_BASE
	b	console_stm32_core_putc
endfunc plat_crash_console_putc

	/* ----------------------------------------------------------
	 * void plat_panic_handler(void) __dead2;
	 * Report exception + endless loop.
	 *
	 * r6 holds the address where the fault occurred.
	 * Filling lr with this value allows debuggers to reconstruct
	 * the backtrace.
	 * ----------------------------------------------------------
	 */
func plat_panic_handler
	mrs	r0, cpsr
	and	r0, #MODE32_MASK
	bl	plat_report_exception
	mov	lr, r6
	b	.
endfunc plat_panic_handler

#if DEBUG
.section .rodata.rev_err_str, "aS"
abort_str:
	.asciz "\nAbort at: 0x"
#if AARCH32_EXCEPTION_DEBUG
prefetch_abort_str:
	.asciz "\nPrefetch Abort at: 0x"
data_abort_str:
	.asciz "\nData Abort at: 0x"
#endif
undefined_str:
	.asciz "\nUndefined instruction at: 0x"
exception_start_str:
	.asciz "\nException mode=0x"
exception_end_str:
	.asciz " at: 0x"
#if AARCH32_EXCEPTION_DEBUG
dfsr_str:
	.asciz " DFSR = 0x"
dfar_str:
	.asciz " DFAR = 0x"
ifsr_str:
	.asciz " IFSR = 0x"
ifar_str:
	.asciz " IFAR = 0x"
#endif
end_error_str:
	.asciz "\n\r"
#endif

func wfi_svc_int_enable
	push	{r4,r8,lr}
	ldcopr	r4, SCR
	mov	r8, sp
	mov	sp, r0
	add	r0, r0, #STM32MP_INT_STACK_SIZE
	str	r0, [sp, #SMC_CTX_SP_MON]
	str	r4, [sp, #SMC_CTX_SCR]
	cps	#MODE32_svc
	cpsie	af
	dsb
	isb
	wfi
	cpsid	af
	cps	#MODE32_mon
	mov	sp, r8
	pop	{r4,r8,lr}
	bx	lr
endfunc wfi_svc_int_enable
