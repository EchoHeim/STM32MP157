/*
 * Copyright (c) 2017-2020, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <errno.h>

#include <libfdt.h>

#include <platform_def.h>

#include <common/debug.h>
#include <drivers/st/stm32_gpio.h>

#include <stm32mp_dt.h>

static int fdt_checked;

static void *fdt = (void *)(uintptr_t)STM32MP_DTB_BASE;

/*******************************************************************************
 * This function checks device tree file with its header.
 * Returns 0 on success and a negative FDT error code on failure.
 ******************************************************************************/
int dt_open_and_check(void)
{
	int ret = fdt_check_header(fdt);

	if (ret == 0) {
		fdt_checked = 1;
	}

	return ret;
}

/*******************************************************************************
 * This function gets the address of the DT.
 * If DT is OK, fdt_addr is filled with DT address.
 * Returns 1 if success, 0 otherwise.
 ******************************************************************************/
int fdt_get_address(void **fdt_addr)
{
	if (fdt_checked == 1) {
		*fdt_addr = fdt;
	}

	return fdt_checked;
}

/*******************************************************************************
 * This function check the presence of a node (generic use of fdt library).
 * Returns true if present, else return false.
 ******************************************************************************/
bool fdt_check_node(int node)
{
	int len;
	const char *cchar;

	cchar = fdt_get_name(fdt, node, &len);

	return (cchar != NULL) && (len >= 0);
}

/*******************************************************************************
 * This function return global node status (generic use of fdt library).
 ******************************************************************************/
uint8_t fdt_get_status(int node)
{
	uint8_t status = DT_DISABLED;
	int len;
	const char *cchar;

	cchar = fdt_getprop(fdt, node, "status", &len);
	if ((cchar == NULL) ||
	    (strncmp(cchar, "okay", (size_t)len) == 0)) {
		status |= DT_NON_SECURE;
	}

	cchar = fdt_getprop(fdt, node, "secure-status", &len);
	if (cchar == NULL) {
		if (status == DT_NON_SECURE) {
			status |= DT_SECURE;
		}
	} else if (strncmp(cchar, "okay", (size_t)len) == 0) {
		status |= DT_SECURE;
	}

	return status;
}

#if ENABLE_ASSERTIONS
/*******************************************************************************
 * This function returns the address cells from the node parent.
 * Returns:
 * - #address-cells value if success.
 * - invalid value if error.
 * - a default value if undefined #address-cells property as per libfdt
 *   implementation.
 ******************************************************************************/
static int fdt_get_node_parent_address_cells(int node)
{
	int parent;

	parent = fdt_parent_offset(fdt, node);
	if (parent < 0) {
		return -FDT_ERR_NOTFOUND;
	}

	return fdt_address_cells(fdt, parent);
}

/*******************************************************************************
 * This function returns the size cells from the node parent.
 * Returns:
 * - #size-cells value if success.
 * - invalid value if error.
 * - a default value if undefined #size-cells property as per libfdt
 *   implementation.
 ******************************************************************************/
static int fdt_get_node_parent_size_cells(int node)
{
	int parent;

	parent = fdt_parent_offset(fdt, node);
	if (parent < 0) {
		return -FDT_ERR_NOTFOUND;
	}

	return fdt_size_cells(fdt, parent);
}
#endif

/*******************************************************************************
 * This function return interrupts from node.
 ******************************************************************************/
int fdt_get_interrupt(int node, const fdt32_t **array, int *len, bool *extended)
{
	uint8_t status = fdt_get_status(node);

	*extended = false;

	switch (status) {
	case DT_SECURE:
		*array = fdt_getprop(fdt, node, "interrupts-extended", len);
		if (*array == NULL) {
			*array = fdt_getprop(fdt, node, "interrupts", len);
		} else {
			*extended = true;
		}
		break;

	default:
		*array = fdt_getprop(fdt, node, "secure-interrupts", len);
		break;
	}

	if (*array == NULL) {
		return -FDT_ERR_NOTFOUND;
	}

	return 0;
}

/*******************************************************************************
 * This function reads a value of a node property (generic use of fdt
 * library).
 * Returns value if success, and a default value if property not found.
 * Default value is passed as parameter.
 ******************************************************************************/
uint32_t fdt_read_uint32_default(int node, const char *prop_name,
				 uint32_t dflt_value)
{
	const fdt32_t *cuint;
	int lenp;

	cuint = fdt_getprop(fdt, node, prop_name, &lenp);
	if (cuint == NULL) {
		return dflt_value;
	}

	return fdt32_to_cpu(*cuint);
}

/*******************************************************************************
 * This function reads a series of parameters in a node property
 * (generic use of fdt library).
 * It reads the values inside the device tree, from property name and node.
 * The number of parameters is also indicated as entry parameter.
 * Returns 0 on success and a negative FDT error code on failure.
 * If success, values are stored at the third parameter address.
 ******************************************************************************/
int fdt_read_uint32_array(int node, const char *prop_name, uint32_t *array,
			  uint32_t count)
{
	const fdt32_t *cuint;
	int len;
	uint32_t i;

	cuint = fdt_getprop(fdt, node, prop_name, &len);
	if (cuint == NULL) {
		return -FDT_ERR_NOTFOUND;
	}

	if ((uint32_t)len != (count * sizeof(uint32_t))) {
		return -FDT_ERR_BADLAYOUT;
	}

	for (i = 0; i < ((uint32_t)len / sizeof(uint32_t)); i++) {
		*array = fdt32_to_cpu(*cuint);
		array++;
		cuint++;
	}

	return 0;
}

/*******************************************************************************
 * This function fills reg node info (base & size) with an index found by
 * checking the reg-names node.
 * Returns 0 on success and a negative FDT error code on failure.
 ******************************************************************************/
int fdt_get_reg_props_by_name(int node, const char *name, uintptr_t *base,
			      size_t *size)
{
	const fdt32_t *cuint;
	int index, len;

	assert((fdt_get_node_parent_address_cells(node) == 1) &&
	       (fdt_get_node_parent_size_cells(node) == 1));

	index = fdt_stringlist_search(fdt, node, "reg-names", name);
	if (index < 0) {
		return index;
	}

	cuint = fdt_getprop(fdt, node, "reg", &len);
	if (cuint == NULL) {
		return -FDT_ERR_NOTFOUND;
	}

	if ((index * (int)sizeof(uint32_t)) > len) {
		return -FDT_ERR_BADVALUE;
	}

	cuint += index << 1;
	if (base != NULL) {
		*base = fdt32_to_cpu(*cuint);
	}
	cuint++;
	if (size != NULL) {
		*size = fdt32_to_cpu(*cuint);
	}

	return 0;
}

/*******************************************************************************
 * This function gets the stdout path node.
 * It reads the value indicated inside the device tree.
 * Returns node offset on success and a negative FDT error code on failure.
 ******************************************************************************/
static int dt_get_stdout_node_offset(void)
{
	int node;
	const char *cchar;

	node = fdt_path_offset(fdt, "/secure-chosen");
	if (node < 0) {
		node = fdt_path_offset(fdt, "/chosen");
		if (node < 0) {
			return -FDT_ERR_NOTFOUND;
		}
	}

	cchar = fdt_getprop(fdt, node, "stdout-path", NULL);
	if (cchar == NULL) {
		return -FDT_ERR_NOTFOUND;
	}

	node = -FDT_ERR_NOTFOUND;
	if (strchr(cchar, (int)':') != NULL) {
		const char *name;
		char *str = (char *)cchar;
		int len = 0;

		while (strncmp(":", str, 1)) {
			len++;
			str++;
		}

		name = fdt_get_alias_namelen(fdt, cchar, len);

		if (name != NULL) {
			node = fdt_path_offset(fdt, name);
		}
	} else {
		node = fdt_path_offset(fdt, cchar);
	}

	return node;
}

/*******************************************************************************
 * This function gets the stdout pin configuration information from the DT.
 * And then calls the sub-function to treat it and set GPIO registers.
 * Returns 0 on success and a negative FDT error code on failure.
 ******************************************************************************/
int dt_set_stdout_pinctrl(void)
{
	int node;

	node = dt_get_stdout_node_offset();
	if (node < 0) {
		return -FDT_ERR_NOTFOUND;
	}

	return dt_set_pinctrl_config(node);
}

/*******************************************************************************
 * This function fills the generic information from a given node.
 ******************************************************************************/
void dt_fill_device_info(struct dt_node_info *info, int node)
{
	const fdt32_t *cuint;

	assert(fdt_get_node_parent_address_cells(node) == 1);

	cuint = fdt_getprop(fdt, node, "reg", NULL);
	if (cuint != NULL) {
		info->base = fdt32_to_cpu(*cuint);
	} else {
		info->base = 0;
	}

	cuint = fdt_getprop(fdt, node, "clocks", NULL);
	if (cuint != NULL) {
		cuint++;
		info->clock = (int)fdt32_to_cpu(*cuint);
	} else {
		info->clock = -1;
	}

	cuint = fdt_getprop(fdt, node, "resets", NULL);
	if (cuint != NULL) {
		cuint++;
		info->reset = (int)fdt32_to_cpu(*cuint);
	} else {
		info->reset = -1;
	}

	info->status = fdt_get_status(node);
}

/*******************************************************************************
 * This function retrieve the generic information from DT.
 * Returns node on success and a negative FDT error code on failure.
 ******************************************************************************/
int dt_get_node(struct dt_node_info *info, int offset, const char *compat)
{
	int node;

	node = fdt_node_offset_by_compatible(fdt, offset, compat);
	if (node < 0) {
		return -FDT_ERR_NOTFOUND;
	}

	dt_fill_device_info(info, node);

	return node;
}

/*******************************************************************************
 * This function gets the UART instance info of stdout from the DT.
 * Returns node on success and a negative FDT error code on failure.
 ******************************************************************************/
int dt_get_stdout_uart_info(struct dt_node_info *info)
{
	int node;

	node = dt_get_stdout_node_offset();
	if (node < 0) {
		return -FDT_ERR_NOTFOUND;
	}

	dt_fill_device_info(info, node);

	return node;
}

/*******************************************************************************
 * This function returns the node offset matching compatible string in the DT.
 * It is only valid for single instance peripherals (DDR, RCC, PWR, STGEN,
 * SYSCFG...).
 * Returns value on success, and error value on failure.
 ******************************************************************************/
int dt_get_node_by_compatible(const char *compatible)
{
	int node = fdt_node_offset_by_compatible(fdt, -1, compatible);

	if (node < 0) {
		INFO("Cannot find %s node in DT\n", compatible);
	}

	return node;
}

/*******************************************************************************
 * This function returns the node offset matching compatible string in the DT,
 * and also matching the reg property with the given address.
 * Returns value on success, and error value on failure.
 ******************************************************************************/
int dt_match_instance_by_compatible(const char *compatible, uintptr_t address)
{
	int node;

	for (node = fdt_node_offset_by_compatible(fdt, -1, compatible);
	     node != -FDT_ERR_NOTFOUND;
	     node = fdt_node_offset_by_compatible(fdt, node, compatible)) {
		const fdt32_t *cuint;

		assert(fdt_get_node_parent_address_cells(node) == 1);

		cuint = fdt_getprop(fdt, node, "reg", NULL);
		if (cuint == NULL) {
			continue;
		}

		if ((uintptr_t)fdt32_to_cpu(*cuint) == address) {
			return node;
		}
	}

	return -FDT_ERR_NOTFOUND;
}


/*******************************************************************************
 * This function gets DDR size information from the DT.
 * Returns value in bytes on success, and 0 on failure.
 ******************************************************************************/
uint32_t dt_get_ddr_size(void)
{
	int node;

	node = dt_get_node_by_compatible(DT_DDR_COMPAT);
	if (node < 0) {
		return 0;
	}

	return fdt_read_uint32_default(node, "st,mem-size", 0);
}

/*******************************************************************************
 * This function gets OPP table node from the DT.
 * Returns node offset on success and a negative FDT error code on failure.
 ******************************************************************************/
static int dt_get_opp_table_node(void)
{
	return dt_get_node_by_compatible(DT_OPP_COMPAT);
}

/*******************************************************************************
 * This function gets OPP parameters (frequency in KHz and voltage in mV) from
 * an OPP table subnode. Platform HW support capabilities are also checked.
 * Returns 0 on success and a negative FDT error code on failure.
 ******************************************************************************/
static int dt_get_opp_freqvolt_from_subnode(int subnode, uint32_t *freq_khz,
					    uint32_t *voltage_mv)
{
	const fdt64_t *cuint64;
	const fdt32_t *cuint32;
	uint64_t read_freq_64;
	uint32_t read_voltage_32;

	assert(freq_khz != NULL);
	assert(voltage_mv != NULL);

	cuint32 = fdt_getprop(fdt, subnode, "opp-supported-hw", NULL);
	if (cuint32 != NULL) {
		if (!stm32mp_supports_cpu_opp(fdt32_to_cpu(*cuint32))) {
			VERBOSE("Invalid opp-supported-hw 0x%x\n",
				fdt32_to_cpu(*cuint32));
			return -FDT_ERR_BADVALUE;
		}
	}

	cuint64 = fdt_getprop(fdt, subnode, "opp-hz", NULL);
	if (cuint64 == NULL) {
		VERBOSE("Missing opp-hz\n");
		return -FDT_ERR_NOTFOUND;
	}

	/* Frequency value expressed in KHz must fit on 32 bits */
	read_freq_64 = fdt64_to_cpu(*cuint64) / 1000ULL;
	if (read_freq_64 > (uint64_t)UINT32_MAX) {
		VERBOSE("Invalid opp-hz %llu\n", read_freq_64);
		return -FDT_ERR_BADVALUE;
	}

	cuint32 = fdt_getprop(fdt, subnode, "opp-microvolt", NULL);
	if (cuint32 == NULL) {
		VERBOSE("Missing opp-microvolt\n");
		return -FDT_ERR_NOTFOUND;
	}

	/* Millivolt value must fit on 16 bits */
	read_voltage_32 = fdt32_to_cpu(*cuint32) / 1000U;
	if (read_voltage_32 > (uint32_t)UINT16_MAX) {
		VERBOSE("Invalid opp-microvolt %u\n", read_voltage_32);
		return -FDT_ERR_BADVALUE;
	}

	*freq_khz = (uint32_t)read_freq_64;

	*voltage_mv = read_voltage_32;

	return 0;
}

/*******************************************************************************
 * This function parses OPP table in DT and finds the parameters for the
 * highest frequency supported by the HW platform.
 * If found, the new frequency and voltage values override the original ones.
 * Returns 0 on success and a negative FDT error code on failure.
 ******************************************************************************/
int dt_get_max_opp_freqvolt(uint32_t *freq_khz, uint32_t *voltage_mv)
{
	int node;
	int subnode;
	uint32_t freq = 0U;
	uint32_t voltage = 0U;

	assert(freq_khz != NULL);
	assert(voltage_mv != NULL);

	node = dt_get_opp_table_node();
	if (node < 0) {
		return node;
	}

	fdt_for_each_subnode(subnode, fdt, node) {
		uint32_t read_freq;
		uint32_t read_voltage;

		if (dt_get_opp_freqvolt_from_subnode(subnode, &read_freq,
						     &read_voltage) != 0) {
			continue;
		}

		if (read_freq > freq) {
			freq = read_freq;
			voltage = read_voltage;
		}
	}

	if ((freq == 0U) || (voltage == 0U)) {
		return -FDT_ERR_NOTFOUND;
	}

	*freq_khz = freq;
	*voltage_mv = voltage;

	return 0;
}

/*******************************************************************************
 * This function parses OPP table in DT and finds all parameters supported by
 * the HW platform.
 * If found, the corresponding frequency and voltage values are respectively
 * stored in @*freq_khz_array and @*voltage_mv_array.
 * Note that @*count has to be set by caller to the effective size allocated
 * for both tables. Its value is then replaced by the number of filled elements.
 * Returns 0 on success and a negative FDT error code on failure.
 ******************************************************************************/
int dt_get_all_opp_freqvolt(uint32_t *count, uint32_t *freq_khz_array,
			    uint32_t *voltage_mv_array)
{
	int node;
	int subnode;
	int idx = 0;

	assert(count != NULL);
	assert(freq_khz_array != NULL);
	assert(voltage_mv_array != NULL);

	node = dt_get_opp_table_node();
	if (node < 0) {
		return node;
	}

	fdt_for_each_subnode(subnode, fdt, node) {
		uint32_t read_freq;
		uint32_t read_voltage;

		if (dt_get_opp_freqvolt_from_subnode(subnode, &read_freq,
						     &read_voltage) != 0) {
			continue;
		}

		if (idx >= *count) {
			return -FDT_ERR_NOSPACE;
		}

		freq_khz_array[idx] = read_freq;
		voltage_mv_array[idx] = read_voltage;
		idx++;
	}

	if (idx == 0U) {
		return -FDT_ERR_NOTFOUND;
	}

	*count = idx;

	return 0;
}

/*******************************************************************************
 * This function gets PWR VDD regulator voltage information from the DT.
 * Returns value in microvolts on success, and 0 on failure.
 ******************************************************************************/
uint32_t dt_get_pwr_vdd_voltage(void)
{
	int node;
	const fdt32_t *cuint;

	node = dt_get_node_by_compatible(DT_PWR_COMPAT);
	if (node < 0) {
		return 0;
	}

	cuint = fdt_getprop(fdt, node, "vdd-supply", NULL);
	if (cuint == NULL) {
		return 0;
	}

	node = fdt_node_offset_by_phandle(fdt, fdt32_to_cpu(*cuint));
	if (node < 0) {
		return 0;
	}

	cuint = fdt_getprop(fdt, node, "regulator-min-microvolt", NULL);
	if (cuint == NULL) {
		return 0;
	}

	return fdt32_to_cpu(*cuint);
}

/*******************************************************************************
 * This function retrieves CPU regulator name from DT.
 * Returns string taken from supply node, NULL otherwise.
 ******************************************************************************/
const char *dt_get_cpu_regulator_name(void)
{
	int node;
	const fdt32_t *cuint;

	node = fdt_path_offset(fdt, "/cpus/cpu@0");
	if (node < 0) {
		return NULL;
	}

	cuint = fdt_getprop(fdt, node, "cpu-supply", NULL);
	if (cuint == NULL) {
		return NULL;
	}

	node = fdt_node_offset_by_phandle(fdt, fdt32_to_cpu(*cuint));
	if (node < 0) {
		return NULL;
	}

	return (const char *)fdt_getprop(fdt, node, "regulator-name", NULL);
}

/*******************************************************************************
 * This function retrieves board model from DT
 * Returns string taken from model node, NULL otherwise
 ******************************************************************************/
const char *dt_get_board_model(void)
{
	int node = fdt_path_offset(fdt, "/");

	if (node < 0) {
		return NULL;
	}

	return (const char *)fdt_getprop(fdt, node, "model", NULL);
}

/*******************************************************************************
 * This function gets GPIO bank PINCTRL node information from the DT.
 * Returns node value.
 ******************************************************************************/
int fdt_get_gpio_bank_pinctrl_node(unsigned int bank)
{
	switch (bank) {
	case GPIO_BANK_A ... GPIO_BANK_K:
		return fdt_path_offset(fdt, "/soc/pin-controller");
	case GPIO_BANK_Z:
		return fdt_path_offset(fdt, "/soc/pin-controller-z");
	default:
		panic();
	}
}

/*******************************************************************************
 * This function gets GPIOZ pin number information from the DT.
 * It also checks node consistency.
 ******************************************************************************/
int fdt_get_gpioz_nbpins_from_dt(void)
{
	int pinctrl_node;
	int pinctrl_subnode;

	pinctrl_node = fdt_get_gpio_bank_pinctrl_node(GPIO_BANK_Z);
	if (pinctrl_node < 0) {
		return -FDT_ERR_NOTFOUND;
	}

	fdt_for_each_subnode(pinctrl_subnode, fdt, pinctrl_node) {
		uint32_t bank_offset;
		const fdt32_t *cuint;

		if (fdt_getprop(fdt, pinctrl_subnode,
				"gpio-controller", NULL) == NULL) {
			continue;
		}

		cuint = fdt_getprop(fdt, pinctrl_subnode, "reg", NULL);
		if (cuint == NULL) {
			continue;
		}

		bank_offset = stm32_get_gpio_bank_offset(GPIO_BANK_Z);
		if (fdt32_to_cpu(*cuint) != bank_offset) {
			continue;
		}

		if (fdt_get_status(pinctrl_subnode) == DT_DISABLED) {
			return 0;
		}

		cuint = fdt_getprop(fdt, pinctrl_subnode, "ngpios", NULL);
		if (cuint == NULL) {
			return -FDT_ERR_NOTFOUND;
		}

		return (int)fdt32_to_cpu(*cuint);
	}

	return 0;
}
