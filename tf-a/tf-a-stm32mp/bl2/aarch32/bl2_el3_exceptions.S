/*
 * Copyright (c) 2017, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <arch.h>
#include <asm_macros.S>
#include <common/bl_common.h>

	.globl	bl2_vector_table

vector_base bl2_vector_table
	b	bl2_entrypoint
#if AARCH32_EXCEPTION_DEBUG
	b	report_undef_inst	/* Undef */
#else
	b	report_exception	/* Undef */
#endif
	b	report_exception	/* SVC call */
#if AARCH32_EXCEPTION_DEBUG
	b	report_prefetch_abort	/* Prefetch abort */
	b	report_data_abort	/* Data abort */
#else
	b	report_exception	/* Prefetch abort */
	b	report_exception	/* Data abort */
#endif
	b	report_exception	/* Reserved */
	b	report_exception	/* IRQ */
	b	report_exception	/* FIQ */
