/*
 * Copyright (c) 2016-2020, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>

#include <libfdt.h>

#include <platform_def.h>

#include <common/bl_common.h>
#include <common/debug.h>
#include <drivers/arm/gicv2.h>
#include <dt-bindings/interrupt-controller/arm-gic.h>
#include <lib/utils.h>
#include <plat/common/platform.h>

struct stm32_gic_instance {
	uint32_t cells;
	uint32_t phandle_node;
};

/******************************************************************************
 * On a GICv2 system, the Group 1 secure interrupts are treated as Group 0
 * interrupts.
 *****************************************************************************/
static const interrupt_prop_t stm32_interrupt_props[] = {
	PLATFORM_G1S_PROPS(GICV2_INTR_GROUP0),
	PLATFORM_G0_PROPS(GICV2_INTR_GROUP0)
};

/* Fix target_mask_array as secondary core is not able to initialize it */
static unsigned int target_mask_array[PLATFORM_CORE_COUNT] = {1, 2};

static gicv2_driver_data_t platform_gic_data = {
	.interrupt_props = stm32_interrupt_props,
	.interrupt_props_num = ARRAY_SIZE(stm32_interrupt_props),
	.target_masks = target_mask_array,
	.target_masks_num = ARRAY_SIZE(target_mask_array),
};

static struct stm32_gic_instance stm32_gic;

static uint32_t enable_gic_interrupt(const fdt32_t *array)
{
	unsigned int id, cfg;

	switch (fdt32_to_cpu(*array)) {
	case GIC_SPI:
		id = MIN_SPI_ID;
		break;

	case GIC_PPI:
		id = MIN_PPI_ID;
		break;

	default:
		id = MIN_SGI_ID;
		break;
	}

	id += fdt32_to_cpu(*(array + 1));
	cfg = (fdt32_to_cpu(*(array + 2)) < IRQ_TYPE_LEVEL_HIGH) ?
		GIC_INTR_CFG_EDGE : GIC_INTR_CFG_LEVEL;

	if ((id >= MIN_SPI_ID) && (id <= MAX_SPI_ID)) {
		VERBOSE("Enable IT %i\n", id);
		gicv2_set_interrupt_type(id, GICV2_INTR_GROUP0);
		gicv2_set_interrupt_priority(id, STM32MP_IRQ_SEC_SPI_PRIO);
		gicv2_set_spi_routing(id, STM32MP_PRIMARY_CPU);
		gicv2_interrupt_set_cfg(id, cfg);
		gicv2_enable_interrupt(id);
	}

	return id;
}

static void find_next_interrupt(const fdt32_t **array)
{
	int node;
	const fdt32_t *cuint;
	void *fdt;

	assert(fdt32_to_cpu(**array) != stm32_gic.phandle_node);

	if (fdt_get_address(&fdt) == 0) {
		panic();
	}

	node = fdt_node_offset_by_phandle(fdt, fdt32_to_cpu(**array));
	if (node < 0) {
		panic();
	}

	cuint = fdt_getprop(fdt, node, "#interrupt-cells", NULL);
	if (cuint == NULL) {
		panic();
	}

	*array += fdt32_to_cpu(*cuint) + 1;
}

void stm32_gic_init(void)
{
	int node;
	void *fdt;
	const fdt32_t *cuint;
	struct dt_node_info dt_gic;

	if (fdt_get_address(&fdt) == 0) {
		panic();
	}

	node = dt_get_node(&dt_gic, -1, "arm,cortex-a7-gic");
	if (node < 0) {
		panic();
	}

	platform_gic_data.gicd_base = dt_gic.base;

	cuint = fdt_getprop(fdt, node, "reg", NULL);
	if (cuint == NULL) {
		panic();
	}

	platform_gic_data.gicc_base = fdt32_to_cpu(*(cuint + 2));

	cuint = fdt_getprop(fdt, node, "#interrupt-cells", NULL);
	if (cuint == NULL) {
		panic();
	}

	stm32_gic.cells = fdt32_to_cpu(*cuint);

	stm32_gic.phandle_node = fdt_get_phandle(fdt, node);
	if (stm32_gic.phandle_node == 0U) {
		panic();
	}

	gicv2_driver_init(&platform_gic_data);
	gicv2_distif_init();

	stm32_gic_pcpu_init();
}

void stm32_gic_pcpu_init(void)
{
	gicv2_pcpu_distif_init();
	gicv2_set_pe_target_mask(plat_my_core_pos());
	gicv2_cpuif_enable();
}

int stm32_gic_enable_spi(int node, const char *name)
{
	const fdt32_t *cuint;
	void *fdt;
	int res, len;
	int index = -1;
	int i = 0;
	int id = -1;
	bool extended;
	const fdt32_t *t_array, *max;

	if (fdt_get_address(&fdt) == 0) {
		panic();
	}

	cuint = fdt_getprop(fdt, node, "interrupt-parent", NULL);
	if (cuint != NULL) {
		if (stm32_gic.phandle_node != fdt32_to_cpu(*cuint)) {
			return -FDT_ERR_NOTFOUND;
		}
	}

	if (name != NULL) {
		switch (fdt_get_status(node)) {
		case DT_SECURE:
			index = fdt_stringlist_search(fdt, node,
						      "interrupt-names", name);
			break;
		default:
			index = fdt_stringlist_search(fdt, node,
						      "secure-interrupt-names",
						      name);
			break;
		}

		if (index < 0) {
			return index;
		}
	}

	res = fdt_get_interrupt(node, &t_array, &len, &extended);
	if (res < 0) {
		return res;
	}

	max = t_array + (len / sizeof(uint32_t));

	while ((t_array < max) && ((i <= index) || (index == -1))) {
		if (!extended) {
			if ((index == -1) || (i == index)) {
				id = enable_gic_interrupt(t_array);
			}
			t_array += stm32_gic.cells;
		} else {
			if (fdt32_to_cpu(*t_array) == stm32_gic.phandle_node) {
				t_array++;
				if ((index == -1) || (i == index)) {
					id = enable_gic_interrupt(t_array);
				}
				t_array += stm32_gic.cells;
			} else {
				find_next_interrupt(&t_array);
			}
		}
		i++;
	}

	return id;
}
