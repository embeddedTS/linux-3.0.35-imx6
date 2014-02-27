/*
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef _BOARD_MX6DL_TS4900_H
#define _BOARD_MX6DL_TS4900_H
#include <mach/iomux-mx6dl.h>

static iomux_v3_cfg_t mx6dl_ts4900_pads[] = {
	/* WIFI */
	MX6DL_PAD_SD1_CMD__USDHC1_CMD,
	MX6DL_PAD_SD1_CLK__USDHC1_CLK,
	MX6DL_PAD_SD1_DAT0__USDHC1_DAT0,
	MX6DL_PAD_SD1_DAT1__USDHC1_DAT1,
	MX6DL_PAD_SD1_DAT2__USDHC1_DAT2,
	MX6DL_PAD_SD1_DAT3__USDHC1_DAT3,
	MX6DL_PAD_ENET_RXD1__GPIO_1_26, // WIFI_IRQ

	/* SD Card */
	MX6DL_PAD_SD2_CMD__USDHC2_CMD,
	MX6DL_PAD_SD2_CLK__USDHC2_CLK,
	MX6DL_PAD_SD2_DAT0__USDHC2_DAT0,
	MX6DL_PAD_SD2_DAT1__USDHC2_DAT1,
	MX6DL_PAD_SD2_DAT2__USDHC2_DAT2,
	MX6DL_PAD_SD2_DAT3__USDHC2_DAT3,
	MX6DL_PAD_EIM_EB0__GPIO_2_28, // EN_SD_POWER#

	/* eMMC */
	MX6DL_PAD_SD3_CMD__USDHC3_CMD_50MHZ,
	MX6DL_PAD_SD3_CLK__USDHC3_CLK_50MHZ,
	MX6DL_PAD_SD3_DAT0__USDHC3_DAT0_50MHZ,
	MX6DL_PAD_SD3_DAT1__USDHC3_DAT1_50MHZ,
	MX6DL_PAD_SD3_DAT2__USDHC3_DAT2_50MHZ,
	MX6DL_PAD_SD3_DAT3__USDHC3_DAT3_50MHZ,
	MX6DL_PAD_SD3_RST__GPIO_7_8, // EMMC_RESET#

	/* DEBUG UART */
	MX6DL_PAD_SD3_DAT7__UART1_TXD,
	MX6DL_PAD_SD3_DAT6__UART1_RXD,

	/* COM2 */
	MX6DL_PAD_GPIO_7__UART2_TXD,
	MX6DL_PAD_GPIO_8__UART2_RXD,
	MX6DL_PAD_SD4_DAT6__UART2_CTS,

	/* COM3 */
	MX6DL_PAD_EIM_D24__UART3_TXD,
	MX6DL_PAD_EIM_D25__UART3_RXD,

	/* COM4 */
	MX6DL_PAD_KEY_COL0__UART4_TXD,
	MX6DL_PAD_KEY_ROW0__UART4_RXD,
	MX6DL_PAD_CSI0_DAT17__UART4_CTS,

	/* COM5 */
	MX6DL_PAD_KEY_COL1__UART5_TXD,
	MX6DL_PAD_KEY_ROW1__UART5_RXD,

	/* Audio */
	MX6DL_PAD_CSI0_DAT4__AUDMUX_AUD3_TXC, // AUD_CLK
	MX6DL_PAD_CSI0_DAT5__AUDMUX_AUD3_TXD, // AUD_TXD
	MX6DL_PAD_CSI0_DAT6__AUDMUX_AUD3_TXFS, // AUD_FRM
	MX6DL_PAD_CSI0_DAT7__AUDMUX_AUD3_RXD, // AUD_RXD

	/* CCM  */
	MX6DL_PAD_GPIO_0__CCM_CLKO,		/* SGTL500 sys_mclk */

	/* ECSPI1 */
	MX6DL_PAD_EIM_D17__ECSPI1_MISO,
	MX6DL_PAD_EIM_D18__ECSPI1_MOSI,
	MX6DL_PAD_EIM_D16__ECSPI1_SCLK,
	MX6DL_PAD_EIM_D19__GPIO_3_19,	/*SS1*/

	/* ECSPI2 */
	MX6DL_PAD_CSI0_DAT8__ECSPI2_SCLK,
	MX6DL_PAD_CSI0_DAT9__ECSPI2_MOSI,
	MX6DL_PAD_CSI0_DAT10__ECSPI2_MISO,
	MX6DL_PAD_CSI0_DAT11__GPIO_5_29,
	MX6DL_PAD_CSI0_DAT16__GPIO_6_2,

	/* ENET */
	MX6DL_PAD_ENET_MDIO__ENET_MDIO,
	MX6DL_PAD_ENET_MDC__ENET_MDC,
	MX6DL_PAD_RGMII_TXC__ENET_RGMII_TXC,
	MX6DL_PAD_RGMII_TD0__ENET_RGMII_TD0,
	MX6DL_PAD_RGMII_TD1__ENET_RGMII_TD1,
	MX6DL_PAD_RGMII_TD2__ENET_RGMII_TD2,
	MX6DL_PAD_RGMII_TD3__ENET_RGMII_TD3,
	MX6DL_PAD_RGMII_TX_CTL__ENET_RGMII_TX_CTL,
	MX6DL_PAD_ENET_REF_CLK__ENET_TX_CLK,
	MX6DL_PAD_RGMII_RXC__ENET_RGMII_RXC,
	MX6DL_PAD_RGMII_RD0__ENET_RGMII_RD0,
	MX6DL_PAD_RGMII_RD1__ENET_RGMII_RD1,
	MX6DL_PAD_RGMII_RD2__ENET_RGMII_RD2,
	MX6DL_PAD_RGMII_RD3__ENET_RGMII_RD3,
	MX6DL_PAD_RGMII_RX_CTL__ENET_RGMII_RX_CTL,
	MX6DL_PAD_ENET_TX_EN__GPIO_1_28,		/* RGMII Interrupt */
	MX6DL_PAD_DI0_PIN4__GPIO_4_20,		/* RGMII reset */

	/* DISPLAY */
	MX6DL_PAD_EIM_A19__GPIO_2_19, /* EN_LCD_3.3V */
	MX6DL_PAD_SD4_DAT1__PWM3_PWMO, /* Backlight */
	MX6DL_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK,
	MX6DL_PAD_DI0_PIN15__IPU1_DI0_PIN15,		/* DE */
	MX6DL_PAD_DI0_PIN2__IPU1_DI0_PIN2,		/* HSync */
	MX6DL_PAD_DI0_PIN3__IPU1_DI0_PIN3,		/* VSync */
	MX6DL_PAD_DISP0_DAT0__IPU1_DISP0_DAT_0,
	MX6DL_PAD_DISP0_DAT1__IPU1_DISP0_DAT_1,
	MX6DL_PAD_DISP0_DAT2__IPU1_DISP0_DAT_2,
	MX6DL_PAD_DISP0_DAT3__IPU1_DISP0_DAT_3,
	MX6DL_PAD_DISP0_DAT4__IPU1_DISP0_DAT_4,
	MX6DL_PAD_DISP0_DAT5__IPU1_DISP0_DAT_5,
	MX6DL_PAD_DISP0_DAT6__IPU1_DISP0_DAT_6,
	MX6DL_PAD_DISP0_DAT7__IPU1_DISP0_DAT_7,
	MX6DL_PAD_DISP0_DAT8__IPU1_DISP0_DAT_8,
	MX6DL_PAD_DISP0_DAT9__IPU1_DISP0_DAT_9,
	MX6DL_PAD_DISP0_DAT10__IPU1_DISP0_DAT_10,
	MX6DL_PAD_DISP0_DAT11__IPU1_DISP0_DAT_11,
	MX6DL_PAD_DISP0_DAT12__IPU1_DISP0_DAT_12,
	MX6DL_PAD_DISP0_DAT13__IPU1_DISP0_DAT_13,
	MX6DL_PAD_DISP0_DAT14__IPU1_DISP0_DAT_14,
	MX6DL_PAD_DISP0_DAT15__IPU1_DISP0_DAT_15,
	MX6DL_PAD_DISP0_DAT16__IPU1_DISP0_DAT_16,
	MX6DL_PAD_DISP0_DAT17__IPU1_DISP0_DAT_17,
	MX6DL_PAD_DISP0_DAT18__IPU1_DISP0_DAT_18,
	MX6DL_PAD_DISP0_DAT19__IPU1_DISP0_DAT_19,
	MX6DL_PAD_DISP0_DAT20__IPU1_DISP0_DAT_20,
	MX6DL_PAD_DISP0_DAT21__IPU1_DISP0_DAT_21,
	MX6DL_PAD_DISP0_DAT22__IPU1_DISP0_DAT_22,
	MX6DL_PAD_DISP0_DAT23__IPU1_DISP0_DAT_23,

	/* CAN 1 */
	MX6DL_PAD_KEY_COL2__CAN1_TXCAN,
	MX6DL_PAD_KEY_ROW2__CAN1_RXCAN,

	/* CAN 2 */
	MX6DL_PAD_KEY_COL4__CAN2_TXCAN,
	MX6DL_PAD_KEY_ROW4__CAN2_RXCAN,

	/* I2C 1 */
	MX6DL_PAD_EIM_D21__I2C1_SCL,
	MX6DL_PAD_EIM_D28__I2C1_SDA,

	/* I2C 2 */
	MX6DL_PAD_KEY_COL3__I2C2_SCL,
	MX6DL_PAD_KEY_ROW3__I2C2_SDA,

	/* FPGA */
	MX6DL_PAD_GPIO_3__ANATOP_ANATOP_24M_OUT,

	/* USB */
	MX6DL_PAD_GPIO_1__USBOTG_ID,
	MX6DL_PAD_EIM_A16__GPIO_2_22, // EN_USB_5V

	/* GPIO */
	MX6DL_PAD_GPIO_2__GPIO_1_2, // Red LED
	MX6DL_PAD_EIM_CS1__GPIO_2_24, // Green LED
	MX6DL_PAD_GPIO_4__GPIO_1_4,
	MX6DL_PAD_GPIO_5__GPIO_1_5,
	MX6DL_PAD_GPIO_6__GPIO_1_6,
	MX6DL_PAD_GPIO_9__GPIO_1_9,
	MX6DL_PAD_GPIO_16__GPIO_7_11,
	MX6DL_PAD_GPIO_17__GPIO_7_12,
	MX6DL_PAD_GPIO_19__GPIO_4_5,
	MX6DL_PAD_CSI0_MCLK__GPIO_5_19,
	MX6DL_PAD_CSI0_PIXCLK__GPIO_5_18,
	MX6DL_PAD_CSI0_VSYNC__GPIO_5_21,
	MX6DL_PAD_CSI0_DAT12__GPIO_5_30,
	MX6DL_PAD_CSI0_DAT13__GPIO_5_31,
	MX6DL_PAD_CSI0_DAT16__GPIO_6_2,
	MX6DL_PAD_EIM_OE__GPIO_2_25,
	MX6DL_PAD_EIM_CS1__GPIO_2_24,
	MX6DL_PAD_EIM_A18__GPIO_2_20,
	MX6DL_PAD_EIM_A20__GPIO_2_18,
	MX6DL_PAD_EIM_A21__GPIO_2_17,
	MX6DL_PAD_EIM_A22__GPIO_2_16,
	MX6DL_PAD_EIM_A23__GPIO_6_6,
	MX6DL_PAD_EIM_A24__GPIO_5_4,
	MX6DL_PAD_EIM_EB0__GPIO_2_28,
	MX6DL_PAD_EIM_EB1__GPIO_2_29,
	MX6DL_PAD_EIM_BCLK__GPIO_6_31,

	/* MUXBUS/GPIO */
	MX6DL_PAD_EIM_LBA__GPIO_2_27, /* BUS_ALE */
	MX6DL_PAD_EIM_RW__GPIO_2_26, /* BUS_DIR */
	MX6DL_PAD_EIM_CS0__GPIO_2_23, /* BUS_CS# */
	MX6DL_PAD_EIM_WAIT__GPIO_5_0, /* BUS_WAIT# */
	/* MUXBUS AD 00:15 */
	MX6DL_PAD_EIM_DA0__GPIO_3_0,
	MX6DL_PAD_EIM_DA1__GPIO_3_1,
	MX6DL_PAD_EIM_DA2__GPIO_3_2,
	MX6DL_PAD_EIM_DA3__GPIO_3_3,
	MX6DL_PAD_EIM_DA4__GPIO_3_4,
	MX6DL_PAD_EIM_DA5__GPIO_3_5,
	MX6DL_PAD_EIM_DA6__GPIO_3_6,
	MX6DL_PAD_EIM_DA7__GPIO_3_7,
	MX6DL_PAD_EIM_DA8__GPIO_3_8,
	MX6DL_PAD_EIM_DA9__GPIO_3_9,
	MX6DL_PAD_EIM_DA10__GPIO_3_10,
	MX6DL_PAD_EIM_DA11__GPIO_3_11,   /* AD7843 PENIRQ */
	MX6DL_PAD_EIM_DA12__GPIO_3_12,   /* AD7843 CS# */
	MX6DL_PAD_EIM_DA13__GPIO_3_13,   /* AD7843 DOUT (MISO) */
	MX6DL_PAD_EIM_DA14__GPIO_3_14,   /* AD7843 DIN (MOSI) */
	MX6DL_PAD_EIM_DA15__GPIO_3_15,   /* AD7843 CLK */

	/* Other */
	MX6DL_PAD_EIM_A17__GPIO_2_21, // OFF_BD_RESET#
};

#endif