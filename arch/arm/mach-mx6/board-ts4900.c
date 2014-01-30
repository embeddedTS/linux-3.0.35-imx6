#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/memblock.h>
#include <linux/gpio.h>
#include <linux/ion.h>
#include <linux/etherdevice.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/spi/ads7846.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/iomux-mx6q.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ahci_sata.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_asrc.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"

static iomux_v3_cfg_t mx6q_ts4900_pads[] = {
	/* WIFI */
	MX6Q_PAD_SD1_CMD__USDHC1_CMD,
	MX6Q_PAD_SD1_CLK__USDHC1_CLK,
	MX6Q_PAD_SD1_DAT0__USDHC1_DAT0,
	MX6Q_PAD_SD1_DAT1__USDHC1_DAT1,
	MX6Q_PAD_SD1_DAT2__USDHC1_DAT2,
	MX6Q_PAD_SD1_DAT3__USDHC1_DAT3,
	MX6Q_PAD_ENET_RXD1__GPIO_1_26, // WIFI_IRQ

	/* SD Card */
	MX6Q_PAD_SD2_CMD__USDHC2_CMD,
	MX6Q_PAD_SD2_CLK__USDHC2_CLK,
	MX6Q_PAD_SD2_DAT0__USDHC2_DAT0,
	MX6Q_PAD_SD2_DAT1__USDHC2_DAT1,
	MX6Q_PAD_SD2_DAT2__USDHC2_DAT2,
	MX6Q_PAD_SD2_DAT3__USDHC2_DAT3,

	/* eMMC */
	MX6Q_PAD_SD3_CMD__USDHC3_CMD_50MHZ,
	MX6Q_PAD_SD3_CLK__USDHC3_CLK_50MHZ,
	MX6Q_PAD_SD3_DAT0__USDHC3_DAT0_50MHZ,
	MX6Q_PAD_SD3_DAT1__USDHC3_DAT1_50MHZ,
	MX6Q_PAD_SD3_DAT2__USDHC3_DAT2_50MHZ,
	MX6Q_PAD_SD3_DAT3__USDHC3_DAT3_50MHZ,
	MX6Q_PAD_SD3_RST__GPIO_7_8, // EMMC_RESET#

	/* DEBUG UART */
	MX6Q_PAD_SD3_DAT7__UART1_TXD,
	MX6Q_PAD_SD3_DAT6__UART1_RXD,

	/* COM2 */
	MX6Q_PAD_GPIO_7__UART2_TXD,
	MX6Q_PAD_GPIO_8__UART2_RXD,
	MX6Q_PAD_SD4_DAT6__UART2_CTS,

	/* COM3 - NC on P1 */
	/*MX6Q_PAD_SD4_CMD__UART3_TXD,
	MX6Q_PAD_EIM_D27__UART2_RXD,*/

	/* COM4 */
	MX6Q_PAD_KEY_COL0__UART4_TXD,
	MX6Q_PAD_KEY_ROW0__UART4_RXD,
	MX6Q_PAD_CSI0_DAT17__UART4_CTS,

	/* COM5 */
	MX6Q_PAD_KEY_COL1__UART5_TXD,
	MX6Q_PAD_KEY_ROW1__UART5_RXD,

	/* Audio */
	MX6Q_PAD_CSI0_DAT4__AUDMUX_AUD3_TXC, // AUD_CLK
	MX6Q_PAD_CSI0_DAT5__AUDMUX_AUD3_TXD, // AUD_TXD
	MX6Q_PAD_CSI0_DAT6__AUDMUX_AUD3_TXFS, // AUD_FRM
	MX6Q_PAD_CSI0_DAT7__AUDMUX_AUD3_RXD, // AUD_RXD

	/* CCM  */
	MX6Q_PAD_GPIO_0__CCM_CLKO,		/* SGTL500 sys_mclk */

	/* ECSPI1 */
	MX6Q_PAD_EIM_D17__ECSPI1_MISO,
	MX6Q_PAD_EIM_D18__ECSPI1_MOSI,
	MX6Q_PAD_EIM_D16__ECSPI1_SCLK,
	MX6Q_PAD_EIM_D19__GPIO_3_19,	/*SS1*/

	/* ECSPI2 */
	MX6Q_PAD_CSI0_DAT8__ECSPI2_SCLK,
	MX6Q_PAD_CSI0_DAT9__ECSPI2_MOSI,
	MX6Q_PAD_CSI0_DAT10__ECSPI2_MISO,
	MX6Q_PAD_CSI0_DAT11__ECSPI2_SS0,

	/* ENET */
	MX6Q_PAD_ENET_MDIO__ENET_MDIO,
	MX6Q_PAD_ENET_MDC__ENET_MDC,
	MX6Q_PAD_RGMII_TXC__ENET_RGMII_TXC,
	MX6Q_PAD_RGMII_TD0__ENET_RGMII_TD0,
	MX6Q_PAD_RGMII_TD1__ENET_RGMII_TD1,
	MX6Q_PAD_RGMII_TD2__ENET_RGMII_TD2,
	MX6Q_PAD_RGMII_TD3__ENET_RGMII_TD3,
	MX6Q_PAD_RGMII_TX_CTL__ENET_RGMII_TX_CTL,
	MX6Q_PAD_ENET_REF_CLK__ENET_TX_CLK,
	MX6Q_PAD_RGMII_RXC__ENET_RGMII_RXC,
	MX6Q_PAD_RGMII_RD0__ENET_RGMII_RD0,
	MX6Q_PAD_RGMII_RD1__ENET_RGMII_RD1,
	MX6Q_PAD_RGMII_RD2__ENET_RGMII_RD2,
	MX6Q_PAD_RGMII_RD3__ENET_RGMII_RD3,
	MX6Q_PAD_RGMII_RX_CTL__ENET_RGMII_RX_CTL,
	MX6Q_PAD_ENET_TX_EN__GPIO_1_28,		/* RGMII Interrupt */
	//MX6Q_PAD_DI0_PIN4__GPIO_4_20,		/* RGMII reset */

	/* DISPLAY */
	MX6Q_PAD_EIM_A19__GPIO_2_19, /* EN_LCD_3.3V */
	MX6Q_PAD_SD4_DAT1__PWM3_PWMO, /* Backlight */
	MX6Q_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK,
	MX6Q_PAD_DI0_PIN15__IPU1_DI0_PIN15,		/* DE */
	MX6Q_PAD_DI0_PIN2__IPU1_DI0_PIN2,		/* HSync */
	MX6Q_PAD_DI0_PIN3__IPU1_DI0_PIN3,		/* VSync */
	MX6Q_PAD_DISP0_DAT0__IPU1_DISP0_DAT_0,
	MX6Q_PAD_DISP0_DAT1__IPU1_DISP0_DAT_1,
	MX6Q_PAD_DISP0_DAT2__IPU1_DISP0_DAT_2,
	MX6Q_PAD_DISP0_DAT3__IPU1_DISP0_DAT_3,
	MX6Q_PAD_DISP0_DAT4__IPU1_DISP0_DAT_4,
	MX6Q_PAD_DISP0_DAT5__IPU1_DISP0_DAT_5,
	MX6Q_PAD_DISP0_DAT6__IPU1_DISP0_DAT_6,
	MX6Q_PAD_DISP0_DAT7__IPU1_DISP0_DAT_7,
	MX6Q_PAD_DISP0_DAT8__IPU1_DISP0_DAT_8,
	MX6Q_PAD_DISP0_DAT9__IPU1_DISP0_DAT_9,
	MX6Q_PAD_DISP0_DAT10__IPU1_DISP0_DAT_10,
	MX6Q_PAD_DISP0_DAT11__IPU1_DISP0_DAT_11,
	MX6Q_PAD_DISP0_DAT12__IPU1_DISP0_DAT_12,
	MX6Q_PAD_DISP0_DAT13__IPU1_DISP0_DAT_13,
	MX6Q_PAD_DISP0_DAT14__IPU1_DISP0_DAT_14,
	MX6Q_PAD_DISP0_DAT15__IPU1_DISP0_DAT_15,
	MX6Q_PAD_DISP0_DAT16__IPU1_DISP0_DAT_16,
	MX6Q_PAD_DISP0_DAT17__IPU1_DISP0_DAT_17,
	MX6Q_PAD_DISP0_DAT18__IPU1_DISP0_DAT_18,
	MX6Q_PAD_DISP0_DAT19__IPU1_DISP0_DAT_19,
	MX6Q_PAD_DISP0_DAT20__IPU1_DISP0_DAT_20,
	MX6Q_PAD_DISP0_DAT21__IPU1_DISP0_DAT_21,
	MX6Q_PAD_DISP0_DAT22__IPU1_DISP0_DAT_22,
	MX6Q_PAD_DISP0_DAT23__IPU1_DISP0_DAT_23,

	/* CAN 1 */
	MX6Q_PAD_KEY_COL2__CAN1_TXCAN,
	MX6Q_PAD_KEY_ROW2__CAN1_RXCAN,

	/* CAN 2 */
	MX6Q_PAD_KEY_COL4__CAN2_TXCAN,
	MX6Q_PAD_KEY_ROW4__CAN2_RXCAN,

	/* I2C 1 */
	MX6Q_PAD_EIM_D21__I2C1_SCL,
	MX6Q_PAD_EIM_D28__I2C1_SDA,

	/* I2C 2 */
	MX6Q_PAD_KEY_COL3__I2C2_SCL,
	MX6Q_PAD_KEY_ROW3__I2C2_SDA,

	/* FPGA */
	//MX6Q_PAD_GPIO_3__ANATOP_ANATOP_24M_OUT,

	/* USB */
	MX6Q_PAD_GPIO_1__USBOTG_ID,
	MX6Q_PAD_EIM_A16__GPIO_2_22, // EN_USB_5V

	/* GPIO */
	MX6Q_PAD_GPIO_2__GPIO_1_2, // Red LED
	MX6Q_PAD_GPIO_3__GPIO_1_3, // Green LED
	MX6Q_PAD_GPIO_4__GPIO_1_4,
	MX6Q_PAD_GPIO_5__GPIO_1_5,
	MX6Q_PAD_GPIO_6__GPIO_1_6,
	MX6Q_PAD_GPIO_9__GPIO_1_9,
	MX6Q_PAD_GPIO_16__GPIO_7_11,
	MX6Q_PAD_GPIO_17__GPIO_7_12,
	MX6Q_PAD_GPIO_19__GPIO_4_5,
	MX6Q_PAD_CSI0_MCLK__GPIO_5_19,
	MX6Q_PAD_CSI0_PIXCLK__GPIO_5_18,
	MX6Q_PAD_CSI0_VSYNC__GPIO_5_21,
	MX6Q_PAD_CSI0_DAT12__GPIO_5_30,
	MX6Q_PAD_CSI0_DAT13__GPIO_5_31,
	MX6Q_PAD_CSI0_DAT16__GPIO_6_2,
	MX6Q_PAD_EIM_OE__GPIO_2_25,
	MX6Q_PAD_EIM_CS1__GPIO_2_24,
	MX6Q_PAD_EIM_A18__GPIO_2_20,
	MX6Q_PAD_EIM_A20__GPIO_2_18,
	MX6Q_PAD_EIM_A21__GPIO_2_17,
	MX6Q_PAD_EIM_A22__GPIO_2_16,
	MX6Q_PAD_EIM_A23__GPIO_6_6,
	MX6Q_PAD_EIM_A24__GPIO_5_4,
	MX6Q_PAD_EIM_EB0__GPIO_2_28,
	MX6Q_PAD_EIM_EB1__GPIO_2_29,

	/* MUXBUS/GPIO */
	MX6Q_PAD_EIM_LBA__GPIO_2_27, /* BUS_ALE */
	MX6Q_PAD_EIM_RW__GPIO_2_26, /* BUS_DIR */
	MX6Q_PAD_EIM_CS0__GPIO_2_23, /* BUS_CS# */
	MX6Q_PAD_EIM_WAIT__GPIO_5_0, /* BUS_WAIT# */
	/* MUXBUS AD 00:15 */
	MX6Q_PAD_EIM_DA0__GPIO_3_0,
	MX6Q_PAD_EIM_DA1__GPIO_3_1,
	MX6Q_PAD_EIM_DA2__GPIO_3_2,
	MX6Q_PAD_EIM_DA3__GPIO_3_3,
	MX6Q_PAD_EIM_DA4__GPIO_3_4,
	MX6Q_PAD_EIM_DA5__GPIO_3_5,
	MX6Q_PAD_EIM_DA6__GPIO_3_6,
	MX6Q_PAD_EIM_DA7__GPIO_3_7,
	MX6Q_PAD_EIM_DA8__GPIO_3_8,
	MX6Q_PAD_EIM_DA9__GPIO_3_9,
	MX6Q_PAD_EIM_DA10__GPIO_3_10,
	MX6Q_PAD_EIM_DA11__GPIO_3_11,   /* AD7843 PENIRQ */
	MX6Q_PAD_EIM_DA12__GPIO_3_12,   /* AD7843 CS# */
	MX6Q_PAD_EIM_DA13__GPIO_3_13,   /* AD7843 DOUT (MISO) */
	MX6Q_PAD_EIM_DA14__GPIO_3_14,   /* AD7843 DIN (MOSI) */
	MX6Q_PAD_EIM_DA15__GPIO_3_15,   /* AD7843 CLK */

	/* Other */
	MX6Q_PAD_EIM_A17__GPIO_2_21, // OFF_BD_RESET#
};

#define TS4900_EN_USB_5V 	IMX_GPIO_NR(2, 22)
#define TS4900_ECSPI1_CS1	IMX_GPIO_NR(3, 19)

#define TS4900_GREEN_LED	IMX_GPIO_NR(1, 3)
#define TS4900_RED_LED		IMX_GPIO_NR(1, 2)
#define TS4900_MODE2		IMX_GPIO_NR(2, 26)
#define TS4900_BD_ID_DATA	IMX_GPIO_NR(2, 25)
#define TS4900_LCD_3P3_EN	IMX_GPIO_NR(2, 19)

#define TS8390_SPI_CLK		IMX_GPIO_NR(3, 15)
#define TS8390_SPI_MOSI		IMX_GPIO_NR(3, 14)
#define TS8390_SPI_MISO		IMX_GPIO_NR(3, 13)
#define TS8390_SPI_CSN		IMX_GPIO_NR(3, 12)
#define TS8390_PENDOWN 		IMX_GPIO_NR(3, 11)
#define TS8390_EN_SPKR 		IMX_GPIO_NR(5, 30)

#define IOMUX_OBSRV_MUX1_OFFSET	0x3c
#define OBSRV_MUX1_MASK			0x3f
#define OBSRV_MUX1_ENET_IRQ		0x9

static struct clk *sata_clk;
static int caam_enabled;

extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;
extern int epdc_enabled;
extern bool enet_to_gpio_6;

/* This shifts out the values from a hardwired 8-input lookup
 * to determine a unique baseboard id. */
static uint8_t detect_baseboard(void) 
{
	uint8_t id = 0;	
	int i;

	gpio_request(TS4900_RED_LED, "bbid-s0");
	gpio_request(TS4900_GREEN_LED, "bbid-s1");
	gpio_request(TS4900_MODE2, "bbid-s2");
	gpio_request(TS4900_BD_ID_DATA, "bbid-out");

	gpio_direction_output(TS4900_RED_LED, 0);
	gpio_direction_output(TS4900_GREEN_LED, 0);
	gpio_direction_output(TS4900_MODE2, 0);
	gpio_direction_input(TS4900_BD_ID_DATA);

	for(i = 0; i < 8; i++) {
		int in;
		if(i & 1) gpio_set_value(TS4900_RED_LED, 1);
		else gpio_set_value(TS4900_RED_LED, 0);

		if(i & 2) gpio_set_value(TS4900_GREEN_LED, 1);
		else gpio_set_value(TS4900_GREEN_LED, 0);

		if(i & 4) gpio_set_value(TS4900_MODE2, 1);
		else gpio_set_value(TS4900_MODE2, 0);
		
		udelay(10); // just for testing, should actually be ok at 2-3us

		in = gpio_get_value(TS4900_BD_ID_DATA);
		id = (id >> 1);
		if(in) id |= 0x80;
	}

	gpio_free(TS4900_RED_LED);
	gpio_free(TS4900_GREEN_LED);
	gpio_free(TS4900_MODE2);
	gpio_free(TS4900_BD_ID_DATA);

	return id;
}

// wifi
static const struct esdhc_platform_data mx6q_ts4900_sd1_data __initconst = {
	.always_present = 1,
	.keep_power_at_suspend = 1,
	.support_8bit = 0,
	.delay_line = 0,
	.wp_gpio = -EINVAL,
	.cd_gpio = -EINVAL,
	.cd_type = ESDHC_CD_PERMANENT,
};

// SD
static const struct esdhc_platform_data mx6q_ts4900_sd2_data __initconst = {
	.keep_power_at_suspend = 1,
	.support_8bit = 0,
	.delay_line = 0,
	.wp_gpio = -EINVAL,
	.cd_gpio = -EINVAL,
	.cd_type = ESDHC_CD_NONE,
};

// emmc
static const struct esdhc_platform_data mx6q_ts4900_sd3_data __initconst = {
	.keep_power_at_suspend = 1,
	.support_8bit = 0,
	.delay_line = 0,
	.wp_gpio = -EINVAL,
	.cd_gpio = -EINVAL,
	.cd_type = ESDHC_CD_PERMANENT,
};

static const struct anatop_thermal_platform_data
	mx6q_ts4900_anatop_thermal_data __initconst = {
		.name = "anatop_thermal",
};

static void imx6q_ts4900_usbotg_vbus(bool on)
{
	/*if (on)
		gpio_set_value(MX6Q_SABRELITE_USB_OTG_PWR, 1);
	else
		gpio_set_value(MX6Q_SABRELITE_USB_OTG_PWR, 0);*/
}

static inline void mx6q_ts4900_init_uart(void)
{
	imx6q_add_imx_uart(0, NULL);
	imx6q_add_imx_uart(1, NULL);
	imx6q_add_imx_uart(2, NULL);
	imx6q_add_imx_uart(3, NULL);
	imx6q_add_imx_uart(4, NULL);
}

static int mx6q_ts4900_fec_phy_init(struct phy_device *phydev)
{
	///* prefer master mode, disable 1000 Base-T capable */
	//phy_write(phydev, 0x9, 0x1c00);
//
	///* min rx data delay */
	//phy_write(phydev, 0x0b, 0x8105);
	//phy_write(phydev, 0x0c, 0x0000);
//
	///* max rx/tx clock delay, min rx/tx control delay */
	//phy_write(phydev, 0x0b, 0x8104);
	//phy_write(phydev, 0x0c, 0xf0f0);
	//phy_write(phydev, 0x0b, 0x104);

	return 0;
}

static struct fec_platform_data fec_data __initdata = {
	.init = mx6q_ts4900_fec_phy_init,
	.phy = PHY_INTERFACE_MODE_RGMII,
};

static int mx6q_ts4900_spi_cs[] = {
	TS4900_ECSPI1_CS1,
};

static const struct spi_imx_master mx6q_ts4900_spi_data __initconst = {
	.chipselect     = mx6q_ts4900_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx6q_ts4900_spi_cs),
};

static struct imxi2c_platform_data mx6q_ts4900_i2c_data = {
	.bitrate = 100000,
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("isl12022", 0x6f),
	},
};

static struct i2c_board_info ts8390_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("sgtl5000", 0x0a),
	},
};

static void __init imx6q_ts4900_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);
	/* disable external charger detect,
	 * or it will affect signal quality at dp .
	 */
	ret = gpio_request(TS4900_EN_USB_5V, "en-usb-5v");
	if (ret) {
		pr_err("failed to get GPIO TS4900_EN_USB_5V: %d\n",
			ret);
		return;
	}
	mxc_iomux_set_gpr_register(1, 13, 1, 1);
	gpio_direction_output(TS4900_EN_USB_5V, 1);
	gpio_free(TS4900_EN_USB_5V);

	mx6_set_otghost_vbus_func(imx6q_ts4900_usbotg_vbus);
	mx6_set_host1_vbus_func(imx6q_ts4900_usbotg_vbus);
}

/* HW Initialization, if return 0, initialization is successful. */
static int mx6q_ts4900_sata_init(struct device *dev, void __iomem *addr)
{
	u32 tmpdata;
	int ret = 0;
	struct clk *clk;

	sata_clk = clk_get(dev, "imx_sata_clk");
	if (IS_ERR(sata_clk)) {
		dev_err(dev, "no sata clock.\n");
		return PTR_ERR(sata_clk);
	}
	ret = clk_enable(sata_clk);
	if (ret) {
		dev_err(dev, "can't enable sata clock.\n");
		goto put_sata_clk;
	}

	/* Set PHY Paremeters, two steps to configure the GPR13,
	 * one write for rest of parameters, mask of first write is 0x07FFFFFD,
	 * and the other one write for setting the mpll_clk_off_b
	 *.rx_eq_val_0(iomuxc_gpr13[26:24]),
	 *.los_lvl(iomuxc_gpr13[23:19]),
	 *.rx_dpll_mode_0(iomuxc_gpr13[18:16]),
	 *.sata_speed(iomuxc_gpr13[15]),
	 *.mpll_ss_en(iomuxc_gpr13[14]),
	 *.tx_atten_0(iomuxc_gpr13[13:11]),
	 *.tx_boost_0(iomuxc_gpr13[10:7]),
	 *.tx_lvl(iomuxc_gpr13[6:2]),
	 *.mpll_ck_off(iomuxc_gpr13[1]),
	 *.tx_edgerate_0(iomuxc_gpr13[0]),
	 */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x07FFFFFD) | 0x0593A044), IOMUXC_GPR13);

	/* enable SATA_PHY PLL */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x2) | 0x2), IOMUXC_GPR13);

	/* Get the AHB clock rate, and configure the TIMER1MS reg later */
	clk = clk_get(NULL, "ahb");
	if (IS_ERR(clk)) {
		dev_err(dev, "no ahb clock.\n");
		ret = PTR_ERR(clk);
		goto release_sata_clk;
	}
	tmpdata = clk_get_rate(clk) / 1000;
	clk_put(clk);

#ifdef CONFIG_SATA_AHCI_PLATFORM
	ret = sata_init(addr, tmpdata);
	if (ret == 0)
		return ret;
#else
	usleep_range(1000, 2000);
	/* AHCI PHY enter into PDDQ mode if the AHCI module is not enabled */
	tmpdata = readl(addr + PORT_PHY_CTL);
	writel(tmpdata | PORT_PHY_CTL_PDDQ_LOC, addr + PORT_PHY_CTL);
	pr_info("No AHCI save PWR: PDDQ %s\n", ((readl(addr + PORT_PHY_CTL)
					>> 20) & 1) ? "enabled" : "disabled");
#endif

release_sata_clk:
	/* disable SATA_PHY PLL */
	writel((readl(IOMUXC_GPR13) & ~0x2), IOMUXC_GPR13);
	clk_disable(sata_clk);
put_sata_clk:
	clk_put(sata_clk);

	return ret;
}

#ifdef CONFIG_SATA_AHCI_PLATFORM
static void mx6q_ts4900_sata_exit(struct device *dev)
{
	clk_disable(sata_clk);
	clk_put(sata_clk);
}

static struct ahci_platform_data mx6q_ts4900_sata_data = {
	.init = mx6q_ts4900_sata_init,
	.exit = mx6q_ts4900_sata_exit,
};
#endif

static const struct flexcan_platform_data
	mx6q_ts4900_flexcan0_pdata __initconst = {};

static struct viv_gpu_platform_data imx6q_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M + SZ_64M - SZ_16M,
};

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static struct ipuv3_fb_platform_data ts4900_fb_data[] = {
	{
	.disp_dev = "lcd",
	.interface_pix_fmt = IPU_PIX_FMT_RGB24,
	.mode_str = "OKAYA-WVGA",
	.default_bpp = 24,
	.int_clk = true,
	.late_init = false,
	},{
	.disp_dev = "lcd",
	.interface_pix_fmt = IPU_PIX_FMT_RGB24,
	.mode_str = "800x600M-16@60",
	.default_bpp = 24,
	.int_clk = true,
	.late_init = false,
	},
};

static struct fsl_mxc_lcd_platform_data lcdif_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.default_ifmt = IPU_PIX_FMT_RGB24,
};

static struct imx_ipuv3_platform_data ipu_data[] = {
	{
	.rev = 4,
	.csi_clk[0] = "clko2_clk",
	}, {
	.rev = 4,
	.csi_clk[0] = "clko2_clk",
	},
};


static struct ion_platform_data imx_ion_data = {
	.nr = 1,
	.heaps = {
		{
		.type = ION_HEAP_TYPE_CARVEOUT,
		.name = "vpu_ion",
		.size = SZ_16M,
		},
	},
};


/*
static int ads7846_get_pendown_state(void)
{
	return !gpio_get_value(TS8390_PENDOWN);
}*/

static struct ads7846_platform_data ts8390_ads7846_platform_data __initdata = {
   
   .model      = 7843,
	.x_min			= 150,
	.x_max			= 3830,
	.y_min			= 190,
	.y_max			= 3830,
	.x_plate_ohms		= 400,
	.y_plate_ohms		= 400,	
	.vref_delay_usecs	= 100,
	.pressure_max	= 255,
	.debounce_max	= 20,
	.debounce_tol	= 6,
	.debounce_rep	= 3,
	.gpio_pendown	= TS8390_PENDOWN,
	
	//.get_pendown_state	= ads7846_get_pendown_state,
};

struct spi_gpio_platform_data ts8390_spi_pdata = {
	.sck		= TS8390_SPI_CLK,
	.mosi		= TS8390_SPI_MOSI,
	.miso		= TS8390_SPI_MISO,
	.num_chipselect	= 1,
};

static struct platform_device ts8390_spi_pdevice = {
	.name	= "spi_gpio",
	.id	= 1,
	.dev 	= {
		.platform_data	= &ts8390_spi_pdata,
	}
};

static struct spi_board_info ts8390_spi_devices[] __initdata = {
	{
		.modalias       = "ads7846",
		.max_speed_hz   = 1000,
		.bus_num        = 1,
		.platform_data  = &ts8390_ads7846_platform_data,
		.irq            = gpio_to_irq(TS8390_PENDOWN),
		.controller_data = (void*)TS8390_SPI_CSN,
	},
};


struct imx_vout_mem {
	resource_size_t res_mbase;
	resource_size_t res_msize;
};

static struct imx_vout_mem vout_mem __initdata = {
	.res_msize = SZ_128M,
};

static const struct pm_platform_data mx6q_ts4900_pm_data __initconst = {
	.name = "imx_pm",
};

static struct regulator_consumer_supply ts4900_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data ts4900_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(ts4900_vmmc_consumers),
	.consumer_supplies = ts4900_vmmc_consumers,
};

static struct fixed_voltage_config ts4900_vmmc_reg_config = {
	.supply_name		= "vmmc",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &ts4900_vmmc_init,
};


static struct regulator_consumer_supply ts4900_vcc_consumers[] = {	
	REGULATOR_SUPPLY("vcc", "spi1.0"),
};

static struct regulator_init_data ts4900_vcc_init = {   
	.num_consumer_supplies = ARRAY_SIZE(ts4900_vcc_consumers),
	.consumer_supplies = ts4900_vcc_consumers,
};

static struct fixed_voltage_config ts4900_vcc_reg_config = {
	.supply_name		= "vcc",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &ts4900_vcc_init,
};


static struct platform_device ts4900_vmmc_reg_devices[] = {
   {   
      .name	= "reg-fixed-voltage",
      .id	= 3,
      .dev	= {
         .platform_data = &ts4900_vmmc_reg_config,
      }
   }, {   
      .name	= "reg-fixed-voltage",
      .id	= 4,
      .dev	= {
         .platform_data = &ts4900_vcc_reg_config,
      }
   },
};


static struct regulator_consumer_supply sgtl5000_ts4900_consumer_vdda = {
	.supply = "VDDA",
	.dev_name = "1-000a",
};

static struct regulator_consumer_supply sgtl5000_ts4900_consumer_vddio = {
	.supply = "VDDIO",
	.dev_name = "1-000a",
};

static struct regulator_init_data sgtl5000_ts4900_vdda_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_ts4900_consumer_vdda,
};

static struct regulator_init_data sgtl5000_ts4900_vddio_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_ts4900_consumer_vddio,
};

static struct fixed_voltage_config sgtl5000_ts4900_vdda_reg_config = {
	.supply_name		= "VDDA",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &sgtl5000_ts4900_vdda_reg_initdata,
};

static struct fixed_voltage_config sgtl5000_ts4900_vddio_reg_config = {
	.supply_name		= "VDDIO",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &sgtl5000_ts4900_vddio_reg_initdata,
};

static struct platform_device sgtl5000_ts4900_vdda_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 0,
	.dev	= {
		.platform_data = &sgtl5000_ts4900_vdda_reg_config,
	},
};

static struct platform_device sgtl5000_ts4900_vddio_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 1,
	.dev	= {
		.platform_data = &sgtl5000_ts4900_vddio_reg_config,
	},
};

static struct platform_pwm_backlight_data mx6_ts4900_pwm3_backlight_data = {
	.pwm_id = 2,
	.max_brightness = 248,
	.dft_brightness = 248,
	.pwm_period_ns = 50000,
};

static struct mxc_dvfs_platform_data ts4900_dvfscore_data = {
	.reg_id = "VDDCORE",
	.soc_id	= "VDDSOC",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 80,
};


static struct mxc_audio_platform_data mx6_ts4900_audio_data;

static int mx6_ts4900_sgtl5000_init(void)
{
	struct clk *clko;
	struct clk *new_parent;
	int rate;

	clko = clk_get(NULL, "clko_clk");
	if (IS_ERR(clko)) {
		pr_err("can't get CLKO clock.\n");
		return PTR_ERR(clko);
	}
	new_parent = clk_get(NULL, "ahb");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko, 16000000);
	if (rate < 8000000 || rate > 27000000) {
		pr_err("Error:SGTL5000 mclk freq %d out of range!\n", rate);
		clk_put(clko);
		return -1;
	}

	mx6_ts4900_audio_data.sysclk = rate;
	clk_set_rate(clko, rate);
	clk_enable(clko);
	return 0;
}

static struct imx_ssi_platform_data mx6_ts4900_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct mxc_audio_platform_data mx6_ts4900_audio_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
	.init = mx6_ts4900_sgtl5000_init,
	.hp_gpio = -1,
};

static struct platform_device mx6_ts4900_audio_device = {
	.name = "imx-sgtl5000",
};

static int imx6q_init_sgtl5000audio(void)
{
	mxc_register_device(&mx6_ts4900_audio_device,
			    &mx6_ts4900_audio_data);
	imx6q_add_imx_ssi(1, &mx6_ts4900_ssi_pdata);

	platform_device_register(&sgtl5000_ts4900_vdda_reg_devices);
	platform_device_register(&sgtl5000_ts4900_vddio_reg_devices);

	return 0;
}

#ifdef CONFIG_ANDROID_RAM_CONSOLE
static struct resource ram_console_resource = {
	.name = "android ram console",
	.flags = IORESOURCE_MEM,
};

static struct platform_device android_ram_console = {
	.name = "ram_console",
	.num_resources = 1,
	.resource = &ram_console_resource,
};

static int __init imx6x_add_ram_console(void)
{
	return platform_device_register(&android_ram_console);
}
#else
#define imx6x_add_ram_console() do {} while (0)
#endif

static int __init caam_setup(char *__unused)
{
	caam_enabled = 1;
	return 1;
}
early_param("caam", caam_setup);

#define SNVS_LPCR 0x38
static void mx6_snvs_poweroff(void)
{

	void __iomem *mx6_snvs_base =  MX6_IO_ADDRESS(MX6Q_SNVS_BASE_ADDR);
	u32 value;
	value = readl(mx6_snvs_base + SNVS_LPCR);
	/*set TOP and DP_EN bit*/
	writel(value | 0x60, mx6_snvs_base + SNVS_LPCR);
}

static const struct imx_pcie_platform_data mx6_ts4900_pcie_data __initconst = {
#ifdef CONFIG_IMX_PCIE_EP_MODE_IN_EP_RC_SYS
	.type_ep	= 1,
#else
	.type_ep	= 0,
#endif
};

/*!
 * Board specific initialization.
 */
static void __init ts4900_board_init(void)
{
	struct clk *clko, *clko2;
	struct clk *new_parent;
	int rate, i;
	uint8_t baseboardid;
	struct platform_device *voutdev;

	mxc_iomux_v3_setup_multiple_pads(mx6q_ts4900_pads,
		ARRAY_SIZE(mx6q_ts4900_pads));

#ifdef CONFIG_FEC_1588
	/* Set GPIO_16 input for IEEE-1588 ts_clk and RMII reference clock
	 * For MX6 GPR1 bit21 meaning:
	 * Bit21:       0 - GPIO_16 pad output
	 *              1 - GPIO_16 pad input
	 */
	 mxc_iomux_set_gpr_register(1, 21, 1, 1);
#endif

	gp_reg_id = ts4900_dvfscore_data.reg_id;
	soc_reg_id = ts4900_dvfscore_data.soc_id;
	mx6q_ts4900_init_uart();
	imx6x_add_ram_console();

	baseboardid = detect_baseboard();
	printk(KERN_INFO "Baseboard ID: 0x%X\n", baseboardid);
	printk(KERN_INFO "Rev: %c\n", 'A' + ((baseboardid & 0xc0) >> 6));

	imx6q_add_vdoa();
	if((baseboardid & ~0xc0) == 0x2)
	{
		printk(KERN_INFO "Baseboard: TS-8390\n");

		
		// Enable LCD power
		gpio_request(TS4900_LCD_3P3_EN, "lcd-3p3-en");
		gpio_direction_output(TS4900_LCD_3P3_EN, 1);

		imx6q_add_ipuv3(0, &ipu_data[0]);
		imx6q_add_ipuv3(1, &ipu_data[1]);

		imx6q_add_ipuv3fb(0, &ts4900_fb_data[0]);
		imx6q_add_lcdif(&lcdif_data);

		voutdev = imx6q_add_v4l2_output(0);
		if (vout_mem.res_msize && voutdev) {
			dma_declare_coherent_memory(&voutdev->dev,
						    vout_mem.res_mbase,
						    vout_mem.res_mbase,
						    vout_mem.res_msize,
						    (DMA_MEMORY_MAP |
						     DMA_MEMORY_EXCLUSIVE));
		}

					
		// Add SPI GPIO/ads7846 for touchscreen
		platform_device_register(&ts8390_spi_pdevice);
		spi_register_board_info(ts8390_spi_devices,
			ARRAY_SIZE(ts8390_spi_devices));

		imx6q_init_sgtl5000audio();
		i2c_register_board_info(1, ts8390_i2c1_board_info,
			ARRAY_SIZE(ts8390_i2c1_board_info));

		// Enable Speaker
		gpio_request(TS8390_EN_SPKR, "spkr-en");
		gpio_direction_output(TS8390_EN_SPKR, 0);
	} else if ((baseboardid & ~0xc0) == 0xa)
	{
		printk(KERN_INFO "Baseboard: TS-8900\n");

		imx6q_add_lcdif(&lcdif_data);
		imx6q_add_ipuv3(0, &ipu_data[0]);
		imx6q_add_ipuv3(1, &ipu_data[0]);
		imx6q_add_ipuv3fb(0, &ts4900_fb_data[1]);
		// Enable LCD power
		gpio_request(TS4900_LCD_3P3_EN, "lcd-3p3-en");
		gpio_direction_output(TS4900_LCD_3P3_EN, 1);

		imx6q_init_sgtl5000audio();
		i2c_register_board_info(1, ts8390_i2c1_board_info,
			ARRAY_SIZE(ts8390_i2c1_board_info));

		// Enable Speaker
		gpio_request(TS8390_EN_SPKR, "spkr-en");
		gpio_direction_output(TS8390_EN_SPKR, 0);
	}

	if (1 == caam_enabled)
		imx6q_add_imx_caam();

	imx6q_add_imx_i2c(0, &mx6q_ts4900_i2c_data);
	imx6q_add_imx_i2c(1, &mx6q_ts4900_i2c_data);
	imx6q_add_imx_i2c(2, &mx6q_ts4900_i2c_data);
	i2c_register_board_info(0, mxc_i2c0_board_info,
			ARRAY_SIZE(mxc_i2c0_board_info));

	imx6q_add_anatop_thermal_imx(1, &mx6q_ts4900_anatop_thermal_data);

	if (enet_to_gpio_6)
		/* Make sure the IOMUX_OBSRV_MUX1 is set to ENET_IRQ. */
		mxc_iomux_set_specialbits_register(
			IOMUX_OBSRV_MUX1_OFFSET,
			OBSRV_MUX1_ENET_IRQ,
			OBSRV_MUX1_MASK);
	else
		fec_data.gpio_irq = -1;
	imx6_init_fec(fec_data);

	imx6q_add_pm_imx(0, &mx6q_ts4900_pm_data);

	/* Move sd4 to first because sd4 connect to emmc.
	   Mfgtools want emmc is mmcblk0 and other sd card is mmcblk1.
	*/
	imx6q_add_sdhci_usdhc_imx(1, &mx6q_ts4900_sd2_data); // sd
	imx6q_add_sdhci_usdhc_imx(2, &mx6q_ts4900_sd3_data); // emmc
	imx6q_add_sdhci_usdhc_imx(0, &mx6q_ts4900_sd1_data); // wifi
	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);
	imx6q_ts4900_init_usb();
	/* SATA is not supported by MX6DL/Solo */
	if (cpu_is_mx6q()) {
#ifdef CONFIG_SATA_AHCI_PLATFORM
		imx6q_add_ahci(0, &mx6q_ts4900_sata_data);
#else
		mx6q_ts4900_sata_init(NULL,
			(void __iomem *)ioremap(MX6Q_SATA_BASE_ADDR, SZ_4K));
#endif
	}
	imx6q_add_vpu();
	//imx6q_init_audio();
	platform_device_register(&ts4900_vmmc_reg_devices[0]);
	platform_device_register(&ts4900_vmmc_reg_devices[1]);
	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);

	imx6q_add_mxc_pwm(0);
	imx6q_add_mxc_pwm(1);
	imx6q_add_mxc_pwm(2);
	imx6q_add_mxc_pwm(3);
	imx6q_add_mxc_pwm_backlight(2, &mx6_ts4900_pwm3_backlight_data);

	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();

	imx6q_add_dvfs_core(&ts4900_dvfscore_data);

	imx6q_add_ion(0, &imx_ion_data,
		sizeof(imx_ion_data) + sizeof(struct ion_platform_heap));

	/*
	ret = gpio_request_array(mx6q_ts4900_flexcan_gpios,
			ARRAY_SIZE(mx6q_ts4900_flexcan_gpios));
	if (ret)
		pr_err("failed to request flexcan1-gpios: %d\n", ret);
	else
		imx6q_add_flexcan0(&mx6q_ts4900_flexcan0_pdata);
	*/

	clko2 = clk_get(NULL, "clko2_clk");
	if (IS_ERR(clko2))
		pr_err("can't get CLKO2 clock.\n");

	new_parent = clk_get(NULL, "osc_clk");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko2, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko2, 24000000);
	clk_set_rate(clko2, rate);
	clk_enable(clko2);

	/* Camera and audio use osc clock */
	clko = clk_get(NULL, "clko_clk");
	if (!IS_ERR(clko))
		clk_set_parent(clko, clko2);

	/* Register charger chips */
	pm_power_off = mx6_snvs_poweroff;
	imx6q_add_busfreq();

	/* Add PCIe RC interface support */
	imx6q_add_pcie(&mx6_ts4900_pcie_data);

	imx6_add_armpmu();
	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);

	// Default to green on, red off
	gpio_request(TS4900_GREEN_LED, "status-led");
	gpio_request(TS4900_RED_LED, "status-led");
	gpio_direction_output(TS4900_RED_LED, 0);
	gpio_direction_output(TS4900_GREEN_LED, 1);
	gpio_free(TS4900_RED_LED);
	gpio_free(TS4900_GREEN_LED);
}

extern void __iomem *twd_base;
static void __init ts4900_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART1_BASE_ADDR, uart_clk);
}

static struct sys_timer ts4900_timer = {
	.init   = ts4900_timer_init,
};

static void __init ts4900_reserve(void)
{
	phys_addr_t phys;
	int i, fb0_reserved = 0, fb_array_size;

	/*
	 * Reserve primary framebuffer memory if its base address
	 * is set by kernel command line.
	 */
	fb_array_size = ARRAY_SIZE(ts4900_fb_data);
	if (fb_array_size > 0 && ts4900_fb_data[0].res_base[0] &&
	    ts4900_fb_data[0].res_size[0]) {
		if (ts4900_fb_data[0].res_base[0] > SZ_2G)
			printk(KERN_INFO"UI Performance downgrade with FB phys address %x!\n",
			    ts4900_fb_data[0].res_base[0]);
		memblock_reserve(ts4900_fb_data[0].res_base[0],
				 ts4900_fb_data[0].res_size[0]);
		memblock_remove(ts4900_fb_data[0].res_base[0],
				ts4900_fb_data[0].res_size[0]);
		ts4900_fb_data[0].late_init = true;
		//ipu_data[ldb_data.ipu_id].bypass_reset = true;
		fb0_reserved = 1;
	}
	for (i = fb0_reserved; i < fb_array_size; i++)
		if (ts4900_fb_data[i].res_size[0]) {
			/* Reserve for other background buffer. */
			phys = memblock_alloc_base(ts4900_fb_data[i].res_size[0],
						SZ_4K, SZ_2G);
			memblock_remove(phys, ts4900_fb_data[i].res_size[0]);
			ts4900_fb_data[i].res_base[0] = phys;
		}

#ifdef CONFIG_ANDROID_RAM_CONSOLE
	phys = memblock_alloc_base(SZ_1M, SZ_4K, SZ_1G);
	memblock_remove(phys, SZ_1M);
	memblock_free(phys, SZ_1M);
	ram_console_resource.start = phys;
	ram_console_resource.end   = phys + SZ_1M - 1;
#endif

#if defined(CONFIG_MXC_GPU_VIV) || defined(CONFIG_MXC_GPU_VIV_MODULE)
	if (imx6q_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
					   SZ_4K, SZ_2G);
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
#endif

#if defined(CONFIG_ION)
	if (imx_ion_data.heaps[0].size) {
		phys = memblock_alloc(imx_ion_data.heaps[0].size, SZ_4K);
		memblock_remove(phys, imx_ion_data.heaps[0].size);
		imx_ion_data.heaps[0].base = phys;
	}
#endif

	if (vout_mem.res_msize) {
		phys = memblock_alloc_base(vout_mem.res_msize,
					   SZ_4K, SZ_1G);
		memblock_remove(phys, vout_mem.res_msize);
		vout_mem.res_mbase = phys;
	}
}

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	char *str;
	struct tag *t;
	int i = 0;
	struct ipuv3_fb_platform_data *pdata_fb = ts4900_fb_data;

	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			str = t->u.cmdline.cmdline;
			str = strstr(str, "fbmem=");
			if (str != NULL) {
				str += 6;
				pdata_fb[i++].res_size[0] = memparse(str, &str);
				while (*str == ',' &&
					i < ARRAY_SIZE(ts4900_fb_data)) {
					str++;
					pdata_fb[i++].res_size[0] = memparse(str, &str);
				}
			}
			/* GPU reserved memory */
			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpumem=");
			if (str != NULL) {
				str += 7;
				imx6q_gpu_pdata.reserved_mem_size = memparse(str, &str);
			}
			break;
		}
	}
}

/*
 * initialize __mach_desc_MX6Q_TS4900 data structure.
 */
MACHINE_START(TS4900, "Freescale i.MX 6Quad TS-4900 Board")
	/* Maintainer: Technologic Systems */
	.boot_params = MX6_PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx6_map_io,
	.init_irq = mx6_init_irq,
	.init_machine = ts4900_board_init,
	.timer = &ts4900_timer,
	.reserve = ts4900_reserve,
MACHINE_END
