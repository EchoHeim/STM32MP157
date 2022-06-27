// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) STMicroelectronics SA 2017
 *
 * Authors: Philippe Cornu <philippe.cornu@st.com>
 *          Yannick Fertre <yannick.fertre@st.com>
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>

/*** Manufacturer Command Set ***/
#define MCS_CMD_MODE_SW		0xFE /* CMD Mode Switch */
#define MCS_CMD1_UCS		0x00 /* User Command Set (UCS = CMD1) */
#define MCS_CMD2_P0		0x01 /* Manufacture Command Set Page0 (CMD2 P0) */
#define MCS_CMD2_P1		0x02 /* Manufacture Command Set Page1 (CMD2 P1) */
#define MCS_CMD2_P2		0x03 /* Manufacture Command Set Page2 (CMD2 P2) */
#define MCS_CMD2_P3		0x04 /* Manufacture Command Set Page3 (CMD2 P3) */

/* CMD2 P0 commands (Display Options and Power) */
#define MCS_STBCTR		0x12 /* TE1 Output Setting Zig-Zag Connection */
#define MCS_SGOPCTR		0x16 /* Source Bias Current */
#define MCS_SDCTR		0x1A /* Source Output Delay Time */
#define MCS_INVCTR		0x1B /* Inversion Type */
#define MCS_EXT_PWR_IC		0x24 /* External PWR IC Control */
#define MCS_SETAVDD		0x27 /* PFM Control for AVDD Output */
#define MCS_SETAVEE		0x29 /* PFM Control for AVEE Output */
#define MCS_BT2CTR		0x2B /* DDVDL Charge Pump Control */
#define MCS_BT3CTR		0x2F /* VGH Charge Pump Control */
#define MCS_BT4CTR		0x34 /* VGL Charge Pump Control */
#define MCS_VCMCTR		0x46 /* VCOM Output Level Control */
#define MCS_SETVGN		0x52 /* VG M/S N Control */
#define MCS_SETVGP		0x54 /* VG M/S P Control */
#define MCS_SW_CTRL		0x5F /* Interface Control for PFM and MIPI */

/* CMD2 P2 commands (GOA Timing Control) - no description in datasheet */
#define GOA_VSTV1		0x00
#define GOA_VSTV2		0x07
#define GOA_VCLK1		0x0E
#define GOA_VCLK2		0x17
#define GOA_VCLK_OPT1		0x20
#define GOA_BICLK1		0x2A
#define GOA_BICLK2		0x37
#define GOA_BICLK3		0x44
#define GOA_BICLK4		0x4F
#define GOA_BICLK_OPT1		0x5B
#define GOA_BICLK_OPT2		0x60
#define MCS_GOA_GPO1		0x6D
#define MCS_GOA_GPO2		0x71
#define MCS_GOA_EQ		0x74
#define MCS_GOA_CLK_GALLON	0x7C
#define MCS_GOA_FS_SEL0		0x7E
#define MCS_GOA_FS_SEL1		0x87
#define MCS_GOA_FS_SEL2		0x91
#define MCS_GOA_FS_SEL3		0x9B
#define MCS_GOA_BS_SEL0		0xAC
#define MCS_GOA_BS_SEL1		0xB5
#define MCS_GOA_BS_SEL2		0xBF
#define MCS_GOA_BS_SEL3		0xC9
#define MCS_GOA_BS_SEL4		0xD3

/* CMD2 P3 commands (Gamma) */
#define MCS_GAMMA_VP		0x60 /* Gamma VP1~VP16 */
#define MCS_GAMMA_VN		0x70 /* Gamma VN1~VN16 */

struct rm68200 {
	struct device *dev;
	struct drm_panel panel;
	struct gpio_desc *reset_gpio;
	struct regulator *supply;
	struct backlight_device *backlight;
	bool prepared;
	bool enabled;
};

static const struct drm_display_mode default_mode = {
	.clock = 51200,
	.hdisplay = 1024,
	.hsync_start =  1024+ 160,
	.hsync_end = 1024 + 160 + 70 ,
    .htotal = 1024 + 160 + 70 +160 ,
	.vdisplay = 600,
	.vsync_start = 600 + 12,
	.vsync_end = 600 + 12 + 20,
	.vtotal = 600 + 12 + 20 + 23,
	.vrefresh = 50,
	.flags = 0,
	.width_mm = 68,
	.height_mm = 122,
};

static inline struct rm68200 *panel_to_rm68200(struct drm_panel *panel)
{
	return container_of(panel, struct rm68200, panel);
}

static void rm68200_dcs_write_buf(struct rm68200 *ctx, const void *data,
				  size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int err;

	err = mipi_dsi_dcs_write_buffer(dsi, data, len);
	if (err < 0)
		DRM_ERROR_RATELIMITED("MIPI DSI DCS write buffer failed: %d\n",
				      err);
}

static void rm68200_dcs_write_cmd(struct rm68200 *ctx, u8 cmd, u8 value)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int err;

	err = mipi_dsi_dcs_write(dsi, cmd, &value, 1);
	if (err < 0)
		DRM_ERROR_RATELIMITED("MIPI DSI DCS write failed: %d\n", err);
}

#define dcs_write_seq(ctx, seq...)				\
({								\
	static const u8 d[] = { seq };				\
								\
	rm68200_dcs_write_buf(ctx, d, ARRAY_SIZE(d));		\
})

/*
 * This panel is not able to auto-increment all cmd addresses so for some of
 * them, we need to send them one by one...
 */
#define dcs_write_cmd_seq(ctx, cmd, seq...)			\
({								\
	static const u8 d[] = { seq };				\
	unsigned int i;						\
								\
	for (i = 0; i < ARRAY_SIZE(d) ; i++)			\
		rm68200_dcs_write_cmd(ctx, cmd + i, d[i]);	\
})

static void rm68200_init(struct rm68200 *ctx)
{

    dcs_write_seq(ctx, 0xB2, 0x10);
}

static int rm68200_disable(struct drm_panel *panel)
{
	struct rm68200 *ctx = panel_to_rm68200(panel);

	if (!ctx->enabled)
		return 0;

	backlight_disable(ctx->backlight);

	ctx->enabled = false;

	return 0;
}

static int rm68200_unprepare(struct drm_panel *panel)
{
	struct rm68200 *ctx = panel_to_rm68200(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int ret;

	if (!ctx->prepared)
		return 0;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret)
		DRM_WARN("failed to set display off: %d\n", ret);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret)
		DRM_WARN("failed to enter sleep mode: %d\n", ret);

	msleep(120);

	if (ctx->reset_gpio) {
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		msleep(20);
	}

	regulator_disable(ctx->supply);

	ctx->prepared = false;

	return 0;
}

static int rm68200_prepare(struct drm_panel *panel)
{
	struct rm68200 *ctx = panel_to_rm68200(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int ret;

	if (ctx->prepared)
		return 0;

	ret = regulator_enable(ctx->supply);
	if (ret < 0) {
		DRM_ERROR("failed to enable supply: %d\n", ret);
		return ret;
	}

	if (ctx->reset_gpio) {
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		msleep(20);
		gpiod_set_value_cansleep(ctx->reset_gpio, 0);
		msleep(100);
	}

	rm68200_init(ctx);

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret)
		return ret;

	msleep(125);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret)
		return ret;

	msleep(20);

	ctx->prepared = true;

	return 0;
}

static int rm68200_enable(struct drm_panel *panel)
{
	struct rm68200 *ctx = panel_to_rm68200(panel);

	if (ctx->enabled)
		return 0;

	backlight_enable(ctx->backlight);

	ctx->enabled = true;

	return 0;
}

static int rm68200_get_modes(struct drm_panel *panel)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(panel->drm, &default_mode);
	if (!mode) {
		DRM_ERROR("failed to add mode %ux%ux@%u\n",
			  default_mode.hdisplay, default_mode.vdisplay,
			  default_mode.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(panel->connector, mode);

	panel->connector->display_info.width_mm = mode->width_mm;
	panel->connector->display_info.height_mm = mode->height_mm;

	return 1;
}

static const struct drm_panel_funcs rm68200_drm_funcs = {
	.disable = rm68200_disable,
	.unprepare = rm68200_unprepare,
	.prepare = rm68200_prepare,
	.enable = rm68200_enable,
	.get_modes = rm68200_get_modes,
};

static int rm68200_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct rm68200 *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio)) {
		ret = PTR_ERR(ctx->reset_gpio);
		dev_err(dev, "cannot get reset GPIO: %d\n", ret);
		return ret;
	}

	ctx->supply = devm_regulator_get(dev, "power");
	if (IS_ERR(ctx->supply)) {
		ret = PTR_ERR(ctx->supply);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "cannot get regulator: %d\n", ret);
		return ret;
	}

	ctx->backlight = devm_of_find_backlight(dev);
	if (IS_ERR(ctx->backlight))
		return PTR_ERR(ctx->backlight);

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;

	dsi->lanes = 2;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_MODE_LPM | MIPI_DSI_CLOCK_NON_CONTINUOUS;

	drm_panel_init(&ctx->panel);
	ctx->panel.dev = dev;
	ctx->panel.funcs = &rm68200_drm_funcs;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "mipi_dsi_attach() failed: %d\n", ret);
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	return 0;
}

static int rm68200_remove(struct mipi_dsi_device *dsi)
{
	struct rm68200 *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

	return 0;
}

static const struct of_device_id raydium_rm68200_of_match[] = {
	{ .compatible = "raydium,rm68200" },
	{ }
};
MODULE_DEVICE_TABLE(of, raydium_rm68200_of_match);

static struct mipi_dsi_driver raydium_rm68200_driver = {
	.probe = rm68200_probe,
	.remove = rm68200_remove,
	.driver = {
		.name = "panel-raydium-rm68200",
		.of_match_table = raydium_rm68200_of_match,
	},
};
module_mipi_dsi_driver(raydium_rm68200_driver);

MODULE_AUTHOR("Philippe Cornu <philippe.cornu@st.com>");
MODULE_AUTHOR("Yannick Fertre <yannick.fertre@st.com>");
MODULE_DESCRIPTION("DRM Driver for Raydium RM68200 MIPI DSI panel");
MODULE_LICENSE("GPL v2");
