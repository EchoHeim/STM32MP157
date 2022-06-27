/*
 * Copyright (c) 2017-2020, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <errno.h>

#include <libfdt.h>

#include <platform_def.h>

#include <common/debug.h>
#include <drivers/delay_timer.h>
#include <drivers/st/stm32_i2c.h>
#include <drivers/st/stm32mp_pmic.h>
#include <drivers/st/stpmic1.h>
#include <lib/mmio.h>
#include <lib/utils_def.h>

#define STPMIC1_LDO12356_OUTPUT_MASK	(uint8_t)(GENMASK(6, 2))
#define STPMIC1_LDO12356_OUTPUT_SHIFT	2
#define STPMIC1_LDO3_MODE		(uint8_t)(BIT(7))
#define STPMIC1_LDO3_DDR_SEL		31U

#define STPMIC1_BUCK_OUTPUT_SHIFT	2
#define STPMIC1_BUCK3_1V8		(39U << STPMIC1_BUCK_OUTPUT_SHIFT)

#define REGULATOR_MODE_STANDBY		8U

#define STPMIC1_DEFAULT_START_UP_DELAY_MS	1

#define CMD_GET_MIN_VOLTAGE		0U
#define CMD_CONFIG_BOOT_ON		1U
#define CMD_CONFIG_LP			2U

static struct i2c_handle_s i2c_handle;
static uint32_t pmic_i2c_addr;

static int dt_get_pmic_node(void)
{
	static int node = -FDT_ERR_BADOFFSET;

	if (node == -FDT_ERR_BADOFFSET) {
		node = dt_get_node_by_compatible("st,stpmic1");
	}

	return node;
}

int dt_pmic_status(void)
{
	static int status = -FDT_ERR_BADVALUE;
	int node;

	if (status != -FDT_ERR_BADVALUE) {
		return status;
	}

	node = dt_get_pmic_node();
	if (node <= 0) {
		status = -FDT_ERR_NOTFOUND;

		return status;
	}

	status = (int)fdt_get_status(node);

	return status;
}

static bool dt_pmic_is_secure(void)
{
	int status = dt_pmic_status();

	return (status >= 0) &&
		(status == DT_SECURE) &&
		(i2c_handle.dt_status == DT_SECURE);
}

static int dt_pmic_get_regulator_voltage(void *fdt, int node,
					 uint16_t *min_mv, uint16_t *max_mv)
{
	const fdt32_t *cuint;

	cuint = fdt_getprop(fdt, node, "regulator-min-microvolt", NULL);
	if (cuint == NULL) {
		return -FDT_ERR_NOTFOUND;
	}

	if (min_mv != NULL) {
		*min_mv = (uint16_t)(fdt32_to_cpu(*cuint) / 1000U);
	}

	cuint = fdt_getprop(fdt, node, "regulator-max-microvolt", NULL);
	if (cuint == NULL) {
		return -FDT_ERR_NOTFOUND;
	}

	if (max_mv != NULL) {
		*max_mv = (uint16_t)(fdt32_to_cpu(*cuint) / 1000U);
	}

	return 0;
}

static int pmic_config_boot_on(void *fdt, int node, const char *regu_name)
{
	uint16_t voltage = 0U;
	uint16_t voltage_min;
	uint16_t voltage_max;
	int status;
	int pmic_voltage;

	if ((fdt_getprop(fdt, node, "regulator-boot-on", NULL) == NULL) &&
	    (fdt_getprop(fdt, node, "regulator-always-on", NULL) == NULL)) {
		return 0;
	}

	if (fdt_getprop(fdt, node, "regulator-pull-down", NULL) != NULL) {

		status = stpmic1_regulator_pull_down_set(regu_name);
		if (status < 0) {
			return status;
		}
	}

	if (fdt_getprop(fdt, node, "st,mask-reset", NULL) != NULL) {

		status = stpmic1_regulator_mask_reset_set(regu_name);
		if (status < 0) {
			return status;
		}
	}

	if (dt_pmic_get_regulator_voltage(fdt, node, &voltage_min,
					  &voltage_max) < 0) {
		return 0;
	}

	pmic_voltage = stpmic1_regulator_voltage_get(regu_name);
	if (pmic_voltage < 0) {
		return pmic_voltage;
	}

	if ((uint16_t)pmic_voltage < voltage_min) {
		voltage = voltage_min;
	}

	if ((uint16_t)pmic_voltage > voltage_max) {
		voltage = voltage_max;
	}

	/* Only re-program voltage if not in the range provided in DT. */
	if (voltage != 0U) {
		status = stpmic1_regulator_voltage_set(regu_name, voltage);
		if (status < 0) {
			return status;
		}
	}

	if (!stpmic1_is_regulator_enabled(regu_name)) {
		status = stpmic1_regulator_enable(regu_name);
		if (status < 0) {
			return status;
		}
	}

	return 0;
}

#if defined(IMAGE_BL32)
static int pmic_config_lp(void *fdt, int node, const char *node_name,
			  const char *regu_name)
{
	int status;
	const fdt32_t *cuint;
	int regulator_state_node;

	status = stpmic1_powerctrl_on();
	if (status < 0) {
		return status;
	};

	/*
	 * First, copy active configuration (Control register) to
	 * PWRCTRL Control register, even if regulator_state_node
	 * does not exist.
	 */
	status = stpmic1_lp_copy_reg(regu_name);
	if (status < 0) {
		return status;
	}

	/* Then apply configs from regulator_state_node */
	regulator_state_node = fdt_subnode_offset(fdt, node, node_name);
	if (regulator_state_node <= 0) {
		return 0;
	}

	if (fdt_getprop(fdt, regulator_state_node, "regulator-on-in-suspend",
			NULL) != NULL) {
		status = stpmic1_lp_reg_on_off(regu_name, 1);
		if (status < 0) {
			return status;
		}
	}

	if (fdt_getprop(fdt, regulator_state_node, "regulator-off-in-suspend",
			NULL) != NULL) {
		status = stpmic1_lp_reg_on_off(regu_name, 0);
		if (status < 0) {
			return status;
		}
	}

	cuint = fdt_getprop(fdt, regulator_state_node,
			    "regulator-suspend-microvolt", NULL);
	if (cuint != NULL) {
		uint16_t voltage = (uint16_t)(fdt32_to_cpu(*cuint) / 1000U);

		status = stpmic1_lp_set_voltage(regu_name, voltage);
		if (status < 0) {
			return status;
		}
	}

	cuint = fdt_getprop(fdt, regulator_state_node, "regulator-mode", NULL);
	if (cuint != NULL) {
		if (fdt32_to_cpu(*cuint) == REGULATOR_MODE_STANDBY) {
			status = stpmic1_lp_set_mode(regu_name, 1);
			if (status < 0) {
				return status;
			}
		}
	}

	return 0;
}
#endif

static int pmic_operate(uint8_t command, const char *node_name,
			uint16_t *voltage_mv)
{
	int pmic_node, regulators_node, subnode;
	void *fdt;
	int ret = -EIO;

	if (fdt_get_address(&fdt) == 0) {
		return -ENOENT;
	}

	pmic_node = dt_get_pmic_node();
	if (pmic_node < 0) {
		return -ENOENT;
	}

	regulators_node = fdt_subnode_offset(fdt, pmic_node, "regulators");
	if (regulators_node < 0) {
		return -ENOENT;
	}

	fdt_for_each_subnode(subnode, fdt, regulators_node) {
		const char *regu_name = fdt_get_name(fdt, subnode, NULL);

		switch (command) {
		case CMD_GET_MIN_VOLTAGE:
			assert(node_name != NULL);
			assert(voltage_mv != NULL);

			if (strcmp(regu_name, node_name) != 0) {
				continue;
			}

			ret = dt_pmic_get_regulator_voltage(fdt, subnode,
							    voltage_mv, NULL);
			if (ret < 0) {
				return -ENXIO;
			}

			return ret;

		case CMD_CONFIG_BOOT_ON:
			ret = pmic_config_boot_on(fdt, subnode, regu_name);
			if (ret < 0) {
				return ret;
			}
			break;

#if defined(IMAGE_BL32)
		case CMD_CONFIG_LP:
			assert(node_name != NULL);

			ret = pmic_config_lp(fdt, subnode, node_name,
					     regu_name);
			if (ret < 0) {
				return ret;
			}
			break;
#endif

		default:
			return -EINVAL;
		}
	}

	return ret;
}

/*
 * Get PMIC and its I2C bus configuration from the device tree.
 * Return 0 on success, negative on error, 1 if no PMIC node is defined.
 */
static int dt_pmic_i2c_config(struct dt_node_info *i2c_info,
			      struct stm32_i2c_init_s *init)
{
	static int i2c_node = -FDT_ERR_NOTFOUND;

	if (i2c_node == -FDT_ERR_NOTFOUND) {
		void *fdt;
		int pmic_node;
		const fdt32_t *cuint;

		if (fdt_get_address(&fdt) == 0) {
			return -FDT_ERR_NOTFOUND;
		}

		pmic_node = dt_get_pmic_node();
		if (pmic_node < 0) {
			return 1;
		}

		cuint = fdt_getprop(fdt, pmic_node, "reg", NULL);
		if (cuint == NULL) {
			return -FDT_ERR_NOTFOUND;
		}

		pmic_i2c_addr = fdt32_to_cpu(*cuint) << 1;
		if (pmic_i2c_addr > UINT16_MAX) {
			return -FDT_ERR_BADVALUE;
		}

		i2c_node = fdt_parent_offset(fdt, pmic_node);
		if (i2c_node < 0) {
			return -FDT_ERR_NOTFOUND;
		}
	}

	dt_fill_device_info(i2c_info, i2c_node);
	if (i2c_info->base == 0U) {
		return -FDT_ERR_NOTFOUND;
	}

	return stm32_i2c_get_setup_from_fdt(i2c_node, init);
}

int pmic_configure_boot_on_regulators(void)
{
	return pmic_operate(CMD_CONFIG_BOOT_ON, NULL, NULL);
}

int pmic_set_lp_config(const char *node_name)
{
	return pmic_operate(CMD_CONFIG_LP, node_name, NULL);
}

int dt_pmic_find_supply(const char **supply_name, const char *regu_name)
{
	int pmic_node, regulators_node, subnode;
	void *fdt;

	if (fdt_get_address(&fdt) == 0) {
		return -FDT_ERR_NOTFOUND;
	}

	pmic_node = dt_get_pmic_node();
	if (pmic_node < 0) {
		return -FDT_ERR_NOTFOUND;
	}

	regulators_node = fdt_subnode_offset(fdt, pmic_node, "regulators");
	if (regulators_node < 0) {
		return -FDT_ERR_NOTFOUND;
	}

	fdt_for_each_subnode(subnode, fdt, regulators_node) {
		const char *name;

		name = fdt_getprop(fdt, subnode, "regulator-name", NULL);
		if ((name != NULL) &&
		    (strcmp(name, regu_name) == 0)) {
			*supply_name = fdt_get_name(fdt, subnode, NULL);
			return 0;
		}
	}

	return -FDT_ERR_NOTFOUND;
}

int pmic_set_regulator_min_voltage(const char *regu_name)
{
	int rc = -ENOENT;
	const char *supply_name;

	if (dt_pmic_find_supply(&supply_name, regu_name) == 0) {
		uint16_t min_mv;

		rc = pmic_operate(CMD_GET_MIN_VOLTAGE, supply_name, &min_mv);
		if (rc == 0) {
			rc = stpmic1_regulator_voltage_set(supply_name, min_mv);
		}
	}

	return rc;
}

bool initialize_pmic_i2c(void)
{
	int ret;
	struct dt_node_info i2c_info;
	struct i2c_handle_s *i2c = &i2c_handle;
	struct stm32_i2c_init_s i2c_init;

	ret = dt_pmic_i2c_config(&i2c_info, &i2c_init);
	if (ret < 0) {
		ERROR("I2C configuration failed %d\n", ret);
		panic();
	}

	if (ret != 0) {
		return false;
	}

	/* Initialize PMIC I2C */
	i2c->i2c_base_addr		= i2c_info.base;
	i2c->dt_status			= i2c_info.status;
	i2c->clock			= i2c_info.clock;
	i2c->i2c_state			= I2C_STATE_RESET;
	i2c_init.own_address1		= pmic_i2c_addr;
	i2c_init.addressing_mode	= I2C_ADDRESSINGMODE_7BIT;
	i2c_init.dual_address_mode	= I2C_DUALADDRESS_DISABLE;
	i2c_init.own_address2		= 0;
	i2c_init.own_address2_masks	= I2C_OAR2_OA2NOMASK;
	i2c_init.general_call_mode	= I2C_GENERALCALL_DISABLE;
	i2c_init.no_stretch_mode	= I2C_NOSTRETCH_DISABLE;
	i2c_init.analog_filter		= 1;
	i2c_init.digital_filter_coef	= 0;

	ret = stm32_i2c_init(i2c, &i2c_init);
	if (ret != 0) {
		ERROR("Cannot initialize I2C %x (%d)\n",
		      i2c->i2c_base_addr, ret);
		panic();
	}

	if (!stm32_i2c_is_device_ready(i2c, pmic_i2c_addr, 1,
				       I2C_TIMEOUT_BUSY_MS)) {
		ERROR("I2C device not ready\n");
		panic();
	}

	stpmic1_bind_i2c(i2c, (uint16_t)pmic_i2c_addr);

	return true;
}

static void register_non_secure_pmic(void)
{
	if (i2c_handle.i2c_base_addr == 0U) {
		return;
	}

	stm32mp_register_non_secure_periph_iomem(i2c_handle.i2c_base_addr);
}

static void register_secure_pmic(void)
{
	stm32mp_register_secure_periph_iomem(i2c_handle.i2c_base_addr);
}

static int pmic_regulator_enable(struct stm32mp_regulator *regu)
{
	void *fdt;
	const char *node_name;

	if (fdt_get_address(&fdt) == 0) {
		return -ENOENT;
	}

	node_name = fdt_get_name(fdt, fdt_node_offset_by_phandle(fdt, regu->id),
				 NULL);

	return stpmic1_regulator_enable(node_name);
}

static int pmic_regulator_disable(struct stm32mp_regulator *regu)
{
	void *fdt;
	const char *node_name;

	if (fdt_get_address(&fdt) == 0) {
		return -ENOENT;
	}

	node_name = fdt_get_name(fdt, fdt_node_offset_by_phandle(fdt, regu->id),
				 NULL);

	return stpmic1_regulator_disable(node_name);
}

static const struct stm32mp_regulator_ops pmic_regu_ops = {
	.enable = pmic_regulator_enable,
	.disable = pmic_regulator_disable,
};

bool is_pmic_regulator(struct stm32mp_regulator *regu)
{
	void *fdt;
	int parent_node;

	if (fdt_get_address(&fdt) == 0) {
		return false;
	}

	parent_node = fdt_parent_offset(fdt,
					fdt_node_offset_by_phandle(fdt,
								   regu->id));
	return (fdt_node_check_compatible(fdt, parent_node,
					  "st,stpmic1-regulators") == 0);
}

void bind_pmic_regulator(struct stm32mp_regulator *regu)
{
	regu->ops = &pmic_regu_ops;
}

void initialize_pmic(void)
{
	if (!initialize_pmic_i2c()) {
		VERBOSE("No PMIC\n");
		register_non_secure_pmic();
		return;
	}

	if (dt_pmic_is_secure()) {
		register_secure_pmic();
	} else {
		VERBOSE("PMIC is not secure-only hence assumed non secure\n");
		register_non_secure_pmic();
	}
}

void configure_pmic(void)
{
	if (pmic_configure_boot_on_regulators() < 0) {
		panic();
	};
}

#if DEBUG
void print_pmic_info_and_debug(void)
{
	unsigned long pmic_version;

	if (stpmic1_get_version(&pmic_version) != 0) {
		ERROR("Failed to access PMIC\n");
		panic();
	}

	INFO("PMIC version = 0x%02lx\n", pmic_version);
	stpmic1_dump_regulators();
}
#endif

int pmic_ddr_power_init(enum ddr_type ddr_type)
{
	bool buck3_at_1v8 = false;
	uint8_t read_val;
	int status;
	uint16_t buck2_mv;
	uint16_t ldo3_mv;

	if (pmic_operate(CMD_GET_MIN_VOLTAGE, "buck2", &buck2_mv) != 0) {
		return -EPERM;
	}

	switch (ddr_type) {
	case STM32MP_DDR3:
		/* Set LDO3 to sync mode */
		status = stpmic1_register_read(LDO3_CONTROL_REG, &read_val);
		if (status != 0) {
			return status;
		}

		read_val &= ~STPMIC1_LDO3_MODE;
		read_val &= ~STPMIC1_LDO12356_OUTPUT_MASK;
		read_val |= STPMIC1_LDO3_DDR_SEL <<
			    STPMIC1_LDO12356_OUTPUT_SHIFT;

		status = stpmic1_register_write(LDO3_CONTROL_REG, read_val);
		if (status != 0) {
			return status;
		}

		status = stpmic1_regulator_voltage_set("buck2", buck2_mv);
		if (status != 0) {
			return status;
		}

		status = stpmic1_regulator_enable("buck2");
		if (status != 0) {
			return status;
		}

		mdelay(STPMIC1_DEFAULT_START_UP_DELAY_MS);

		status = stpmic1_regulator_enable("vref_ddr");
		if (status != 0) {
			return status;
		}

		mdelay(STPMIC1_DEFAULT_START_UP_DELAY_MS);

		status = stpmic1_regulator_enable("ldo3");
		if (status != 0) {
			return status;
		}

		mdelay(STPMIC1_DEFAULT_START_UP_DELAY_MS);
		break;

	case STM32MP_LPDDR2:
	case STM32MP_LPDDR3:
		/*
		 * Set LDO3 to 1.8V
		 * Set LDO3 to bypass mode if BUCK3 = 1.8V
		 * Set LDO3 to normal mode if BUCK3 != 1.8V
		 */
		status = stpmic1_register_read(BUCK3_CONTROL_REG, &read_val);
		if (status != 0) {
			return status;
		}

		if ((read_val & STPMIC1_BUCK3_1V8) == STPMIC1_BUCK3_1V8) {
			buck3_at_1v8 = true;
		}

		status = stpmic1_register_read(LDO3_CONTROL_REG, &read_val);
		if (status != 0) {
			return status;
		}

		read_val &= ~STPMIC1_LDO3_MODE;
		read_val &= ~STPMIC1_LDO12356_OUTPUT_MASK;
		if (buck3_at_1v8) {
			read_val |= STPMIC1_LDO3_MODE;
		}

		status = stpmic1_register_write(LDO3_CONTROL_REG, read_val);
		if (status != 0) {
			return status;
		}

		if (pmic_operate(CMD_GET_MIN_VOLTAGE, "ldo3", &ldo3_mv) != 0) {
			return -EPERM;
		}

		status = stpmic1_regulator_voltage_set("ldo3", ldo3_mv);
		if (status != 0) {
			return status;
		}

		status = stpmic1_regulator_voltage_set("buck2", buck2_mv);
		if (status != 0) {
			return status;
		}

		status = stpmic1_regulator_enable("ldo3");
		if (status != 0) {
			return status;
		}

		mdelay(STPMIC1_DEFAULT_START_UP_DELAY_MS);

		status = stpmic1_regulator_enable("buck2");
		if (status != 0) {
			return status;
		}

		mdelay(STPMIC1_DEFAULT_START_UP_DELAY_MS);

		status = stpmic1_regulator_enable("vref_ddr");
		if (status != 0) {
			return status;
		}

		mdelay(STPMIC1_DEFAULT_START_UP_DELAY_MS);
		break;

	default:
		break;
	};

	return 0;
}
