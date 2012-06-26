/*
 * arch/arm/mach-lpc32xx/ea3250.c
 *
 * Copyright (C) 2010 NXP Semiconductors
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/sysdev.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/spi/eeprom.h>
#include <linux/leds.h>
#include <linux/leds-pca9532.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/amba/bus.h>
#include <linux/amba/clcd.h>
#include <linux/amba/pl022.h>
#include <linux/amba/mmci.h>
#include <linux/kthread.h>
#include <mtd/mtd-abi.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/hardware.h>
#include <mach/platform.h>
#include <mach/gpio.h>
#include <mach/irqs.h>
#include <mach/board.h>
#include "common.h"

#include <linux/spi/ads7846.h>

#define I2C_PCA9532_ADDR 0x60
#define I2C_24LC256_ADDR 0x50

/*
 * Mapped GPIOLIB GPIOs
 */
#define	LED_GPIO	LPC32XX_GPIO(LPC32XX_GPIO_P2_GRP, 1)
#define	SPI0_CS_GPIO	LPC32XX_GPIO(LPC32XX_GPO_P3_GRP, 11)
#define	ADS_TS_GPIO	LPC32XX_GPIO(LPC32XX_GPIO_P3_GRP, 0)
#define	NAND_WP_GPIO	LPC32XX_GPIO(LPC32XX_GPO_P3_GRP, 19)
#define	LCD_CS_GPIO	LPC32XX_GPIO(LPC32XX_GPO_P3_GRP, 4)
#define	LCD_RS_GPIO	LPC32XX_GPIO(LPC32XX_GPO_P3_GRP, 5)
#define	BKL_POW_GPIO	LPC32XX_GPIO(LPC32XX_GPO_P3_GRP, 14)
#define	SSEL0_GPIO5	LPC32XX_GPIO(LPC32XX_GPIO_P3_GRP, 5)
#define	MMC_POWER_GPIO	LPC32XX_GPIO(LPC32XX_GPO_P3_GRP, 1)

/*
 * LCD controller functions
 */
#define SET_RS		(gpio_set_value(LCD_RS_GPIO, 1))
#define RESET_RS	(gpio_set_value(LCD_RS_GPIO, 0))

#define LCDB_PCA9532_I2C_ADDR       (0x64)
#define LCDB_CONFIG_EEPROM_I2C_ADDR (0x56)

/* PCA9532 registers*/
#define LCDB_PCA9532_INPUT0   0x00
#define LCDB_PCA9532_INPUT1   0x01
#define LCDB_PCA9532_PSC0     0x02
#define LCDB_PCA9532_PWM0     0x03
#define LCDB_PCA9532_PSC1     0x04
#define LCDB_PCA9532_PWM1     0x05
#define LCDB_PCA9532_LS0      0x06
#define LCDB_PCA9532_LS1      0x07
#define LCDB_PCA9532_LS2      0x08
#define LCDB_PCA9532_LS3      0x09
#define LCDB_PCA9532_AUTO_INC 0x10

#define LCDB_LS_MODE_ON     0x01
#define LCDB_LS_MODE_BLINK0 0x02
#define LCDB_LS_MODE_BLINK1 0x03

#define LCDB_CTRL_3V3     0x0001
#define LCDB_CTRL_5V      0x0002
#define LCDB_CTRL_DISP_EN 0x0010
#define LCDB_CTRL_BL_EN   0x0080
#define LCDB_CTRL_BL_C    0x0100
#define LCDB_EEPROM_WP    0x8000

#define LCDB_MAGIC 0xEA01CDAE

#define LCDB_NAME_BUF_SZ 30

static uint16_t blink0Shadow = 0;
static uint16_t blink1Shadow = 0;
static uint16_t ledStateShadow = 0;

typedef struct {
  uint32_t magic;        // magic number
  uint8_t  lcd_name[LCDB_NAME_BUF_SZ]; // LCD name
  uint8_t  lcd_mfg[LCDB_NAME_BUF_SZ];  // manufacturer name
  uint16_t lcdParamOff;  // offset to LCD parameters
  uint16_t initOff;      // offset to init sequence string
  uint16_t pdOff;        // offset to power down sequence string
  uint16_t tsOff;        // offset to touch parameters
  uint16_t end;          // end offset
} eaLcdbStore_t;

/* LCD display types */
typedef enum
{
  TFT = 0,      /* Panel type is standard TFT */
  ADTFT,        /* Panel type is advanced TFT */
  HRTFT,        /* Panel type is highly reflective TFT */
  MONO_4BIT,    /* Panel type is 4-bit mono */
  MONO_8BIT,    /* Panel type is 8-bit mono */
  CSTN          /* Panel type is color STN */
} LCD_PANEL_T;

/* Structure containing the parameters for the LCD panel */
typedef struct
{
  uint8_t           h_back_porch;         /* Horizontal back porch in
                                             clocks (minimum of 1) */
  uint8_t           h_front_porch;        /* Horizontal front porch in
                                             clocks (minimum of 1) */
  uint8_t           h_sync_pulse_width;   /* HSYNC pulse width in
                                             clocks (minimum of 1) */
  uint16_t          pixels_per_line;      /* Pixels per line (horizontal
                                             resolution) */
  uint8_t           v_back_porch;         /* Vertical back porch in
                                             clocks */
  uint8_t           v_front_porch;        /* Vertical front porch in
                                             clocks */
  uint8_t           v_sync_pulse_width;   /* VSYNC pulse width in
                                             clocks (minimum 1 clock) */
  uint16_t          lines_per_panel;      /* Lines per panel (vertical
                                             resolution) */
  uint8_t           invert_output_enable; /* Invert output enable, 1 =
                                             invert*/
  uint8_t           invert_panel_clock;   /* Invert panel clock, 1 =
                                             invert*/
  uint8_t           invert_hsync;         /* Invert HSYNC, 1 = invert */
  uint8_t           invert_vsync;         /* Invert VSYNC, 1 = invert */
  uint8_t           ac_bias_frequency;    /* AC bias frequency in
                                             clocks (minimum 1) */
  uint8_t           bits_per_pixel;       /* Maximum bits per pixel the
                                             display supports */
  uint32_t          optimal_clock;        /* Optimal clock rate (Hz) */
  LCD_PANEL_T     lcd_panel_type;       /* LCD panel type */
  uint8_t           dual_panel;           /* Dual panel, 1 = dual panel
                                             display */

  /* The following parameters are needed for ADTFT and HRTFT panels
     only. For all other panels, these should be programmed to 0 */
  uint8_t           hrtft_cls_enable;     /* HRTFT CLS enable flag, 1 =
                                             enable */
  uint8_t           hrtft_sps_enable;     /* HRTFT SPS enable flag, 1 =
                                             enable */
  uint8_t           hrtft_lp_to_ps_delay; /* HRTFT LP to PS delay in
                                             clocks */
  uint8_t           hrtft_polarity_delay; /* HRTFT polarity delay in
                                             clocks */
  uint8_t           hrtft_lp_delay;       /* HRTFT LP delay in clocks */
  uint8_t           hrtft_spl_delay;      /* HRTFT SPL delay in
                                             clocks */
  /* HRTFT SPL to CLKS delay */
  uint16_t          hrtft_spl_to_cls_delay;
} LCD_PARAM_T;

#define LCD_SEQUENCE_BUF_SZ  100
typedef struct
{
  char init_seq[LCD_SEQUENCE_BUF_SZ];
  char power_down_seq[LCD_SEQUENCE_BUF_SZ];
  char hardware_name[LCDB_NAME_BUF_SZ];
  char panel_name[LCDB_NAME_BUF_SZ];

  struct clcd_panel lcd_panel_from_eeprom;
} lpc32xx_lcd_parameters_t;

lpc32xx_lcd_parameters_t* lpc32xx_lcd_parameters = NULL;

static struct i2c_client *ea_i2c_video_client;

struct clcd_board lpc32xx_clcd_data;

/* SPI LCDC device structure */
struct spi_device *ea3250_spi_lcd_dev = NULL;

/*
 * Tick LED
 */
static struct gpio_led phy_leds[] = {
	{
		.name			= "led0",
		.gpio			= LED_GPIO,
		.active_low		= 1,
		.default_trigger	= "heartbeat",
	},
};

static struct gpio_led_platform_data led_data = {
	.leds = phy_leds,
	.num_leds = ARRAY_SIZE(phy_leds),
};

static struct platform_device lpc32xx_gpio_led_device = {
	.name			= "leds-gpio",
	.id			= -1,
	.dev.platform_data	= &led_data,
};

static struct pca9532_platform_data ea3250_leds = {
	.leds = {
	{ 	.type = PCA9532_TYPE_NONE }, /* Key 1 */
	{ 	.type = PCA9532_TYPE_NONE }, /* Key 2 */
	{ 	.type = PCA9532_TYPE_NONE }, /* Key 3 */
	{ 	.type = PCA9532_TYPE_NONE }, /* Key 4 */
	{ 	.type = PCA9532_TYPE_NONE }, /* MMC CD */
	{ 	.type = PCA9532_TYPE_NONE }, /* MMC WP */
	{ 	.type = PCA9532_TYPE_NONE }, /* not used */
	{ 	.type = PCA9532_TYPE_NONE }, /* not used */
	{	.name = "eabb:red:led1",
		.state = PCA9532_OFF,
		.type = PCA9532_TYPE_LED,
	},
	{	.name = "eabb:red:led2",
		.state = PCA9532_OFF,
		.type = PCA9532_TYPE_LED,
	},
	{	.name = "eabb:red:led3",
		.state = PCA9532_OFF,
		.type = PCA9532_TYPE_LED,
	},
	{	.name = "eabb:red:led4",
		.state = PCA9532_OFF,
		.type = PCA9532_TYPE_LED,
	},
	{	.name = "eabb:red:led5",
		.state = PCA9532_OFF,
		.type = PCA9532_TYPE_LED,
	},
	{	.name = "eabb:red:led6",
		.state = PCA9532_OFF,
		.type = PCA9532_TYPE_LED,
	},
	{	.name = "eabb:red:led7",
		.state = PCA9532_OFF,
		.type = PCA9532_TYPE_LED,
	},
	{	.name = "eabb:red:led8",
		.state = PCA9532_OFF,
		.type = PCA9532_TYPE_LED,
	}, },
	.psc = { 0, 0 },
	.pwm = { 0, 0 },
};

/*
 * AMBA SSP (SPI)
 */
static struct pl022_ssp_controller lpc32xx_ssp0_data = {
	.bus_id			= 0,
	.num_chipselect		= 2,
	.enable_dma		= 0,
};

static struct amba_device lpc32xx_ssp0_device = {
	.dev	= {
		.coherent_dma_mask	= ~0,
		.init_name		= "dev:ssp0",
		.platform_data		= &lpc32xx_ssp0_data,
	},
	.res				= {
		.start			= LPC32XX_SSP0_BASE,
		.end			= (LPC32XX_SSP0_BASE + SZ_4K - 1),
		.flags			= IORESOURCE_MEM,
	},
	.dma_mask			= ~0,
	.irq				= {IRQ_LPC32XX_SSP0, NO_IRQ},
};

/*
 * Touchscreen device
 */
/* Touch screen chip select function */
static void ea3250_spi_cs_set(u32 control)
{
	gpio_set_value(SPI0_CS_GPIO, (int) control);
}

/* Touch screen SPI parameters */
static struct pl022_config_chip spi0_chip_info = {
	.com_mode		= INTERRUPT_TRANSFER,
	.iface			= SSP_INTERFACE_MOTOROLA_SPI,
	.hierarchy		= SSP_MASTER,
	.slave_tx_disable	= 0,
	.rx_lev_trig		= SSP_RX_4_OR_MORE_ELEM,
	.tx_lev_trig		= SSP_TX_4_OR_MORE_EMPTY_LOC,
	.ctrl_len		= SSP_BITS_8,
	.wait_state		= SSP_MWIRE_WAIT_ZERO,
	.duplex			= SSP_MICROWIRE_CHANNEL_FULL_DUPLEX,
	.cs_control		= ea3250_spi_cs_set,
};

/* Touch screen interrupt status function */
static int ea3250_ads7846_pendown_state(void)
{
	u32 tmp = gpio_get_value(ADS_TS_GPIO);
	return (tmp == 0);
}

/* Touch screen platform data */
static struct ads7846_platform_data ea_ads7846_platform_data __initdata = {
	.model      = 7846,
	.debounce_max	= 10,
	.debounce_tol	= 3,
	.pressure_max	= 1024,
	.get_pendown_state = ea3250_ads7846_pendown_state,
	.wakeup = true,
};

/*
 * SPI based LCDC data
 */
/* LCDC chip select function */
static void ea3250_spi_lcdc_cs_set(u32 control)
{
	gpio_set_value(LCD_CS_GPIO, (int) control);
}

/* LCDC SPI parameters */
static struct pl022_config_chip spi0_chip_info1 = {
	.com_mode		= INTERRUPT_TRANSFER,
	.iface			= SSP_INTERFACE_MOTOROLA_SPI,
	.hierarchy		= SSP_MASTER,
	.slave_tx_disable	= 0,
	.rx_lev_trig		= SSP_RX_4_OR_MORE_ELEM,
	.tx_lev_trig		= SSP_TX_4_OR_MORE_EMPTY_LOC,
	.ctrl_len		= SSP_BITS_8,
	.wait_state		= SSP_MWIRE_WAIT_ZERO,
	.duplex			= SSP_MICROWIRE_CHANNEL_FULL_DUPLEX,
	.cs_control		= ea3250_spi_lcdc_cs_set,
};

/* SPI devices registration */
static int __init ea3250_spi_devices_register(void)
{
#if defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)
	static struct spi_board_info info[] = {
		{
			.modalias = "spidev",
			.max_speed_hz = 2500000,
			.bus_num = 0,
			.chip_select = 0,
			.controller_data = &spi0_chip_info,
		},
	};
#else
	struct spi_board_info info[] = {
		{
			.modalias      = "ads7846",
			.max_speed_hz  = 2500000,
			.chip_select   = 0,
			.irq           = IRQ_LPC32XX_GPIO_00,
			.platform_data = &ea_ads7846_platform_data,
			.controller_data = &spi0_chip_info,
		},
		{
			.modalias      = "ea3250_lcdc",
			.max_speed_hz  = 10000000,
			.chip_select   = 1,
			.controller_data = &spi0_chip_info1,
		},
	};
#endif

	/* Configure ADS TS INT GPIO pin as input */
	if (gpio_request(ADS_TS_GPIO, "ADS7846 TS INT"))
		return -EIO;
	if(gpio_direction_input(ADS_TS_GPIO))
		return -EIO;

	/* Configure LCDC CS GPIO pin */
	if (gpio_request(LCD_CS_GPIO, "LCDC CS"))
		return -EIO;
	if(gpio_direction_output(LCD_CS_GPIO, 1))
		return -EIO;

	return spi_register_board_info(info, ARRAY_SIZE(info));
}
arch_initcall(ea3250_spi_devices_register);

#if defined (CONFIG_FB_ARMCLCD)
/*
 * LCDC AMBA Driver Board Functions
 */
/*
 * Support for Embedded Artists 3.2 inch QVGA LCD panel
 */
static struct clcd_panel conn_lcd_panel = {
	.mode = {
		.name = "QVGA portrait",
		.refresh = 60,
		.xres = 240,
		.yres = 320,
		.pixclock = 121654,
		.left_margin = 28,
		.right_margin = 10,
		.upper_margin = 2,
		.lower_margin = 2,
		.hsync_len = 3,
		.vsync_len = 2,
		.sync = 0,
		.vmode = FB_VMODE_NONINTERLACED,
	},
	.width = -1,
	.height = -1,
	.tim2 = (TIM2_IVS | TIM2_IHS | TIM2_IPC),
	.cntl = (CNTL_BGR | CNTL_LCDTFT | CNTL_LCDVCOMP(1) |
			CNTL_LCDBPP16_565),
	.bpp = 16,
};


static int lpc32xx_clcd_setup(struct clcd_fb *fb)
{
	dma_addr_t dma;
	size_t panel_size = 3 * SZ_64K;
	struct clcd_panel* panel_to_use = &conn_lcd_panel;

    if (lpc32xx_lcd_parameters != NULL) {
      panel_to_use = &lpc32xx_lcd_parameters->lcd_panel_from_eeprom;
    }

	while (panel_size < ((panel_to_use->bpp / 8) * panel_to_use->mode.xres * panel_to_use->mode.yres)) {
	  panel_size += SZ_64K;
	}

	fb->fb.screen_base = dma_alloc_writecombine(&fb->dev->dev,
	    panel_size, &dma, GFP_KERNEL);
	if (!fb->fb.screen_base) {
		printk(KERN_ERR "CLCD: unable to map framebuffer\n");
		return -ENOMEM;
	}

	fb->fb.fix.smem_start = dma;
	fb->fb.fix.smem_len = panel_size;
	fb->panel = panel_to_use;

	if (gpio_request(SSEL0_GPIO5, "Unused GPIO5 input"))
		return -EIO;
	if(gpio_direction_input(SSEL0_GPIO5))
		return -EIO;

	/* Configure LCDC RS GPIO pin */
	if (gpio_request(LCD_RS_GPIO, "LCDC RS"))
		return -EIO;

	if(gpio_direction_output(LCD_RS_GPIO, 1))
		return -EIO;

	/* Configure LCDC Backlight GPIO pin */
	if (gpio_request(BKL_POW_GPIO, "LCDC BKL"))
		return -EIO;

	if(gpio_direction_output(BKL_POW_GPIO, 1)) {
		return -EIO;
	}

	return 0;
}

static int lpc32xx_clcd_mmap(struct clcd_fb *fb, struct vm_area_struct *vma)
{
	return dma_mmap_writecombine(&fb->dev->dev, vma,
															fb->fb.screen_base,
															fb->fb.fix.smem_start,
															fb->fb.fix.smem_len);
}

static void lpc32xx_clcd_remove(struct clcd_fb *fb)
{
	dma_free_writecombine(&fb->dev->dev, fb->fb.fix.smem_len,
			fb->fb.screen_base, fb->fb.fix.smem_start);
}

static int lcdb_eeprom_read(struct i2c_client *client, uint8_t* buf, uint16_t offset, uint16_t len)
{
  int i = 0;
  int ret;
  uint8_t off[2];

  off[0] = ((offset >> 8) & 0xff);
  off[1] = (offset & 0xff);

  ret = i2c_master_send(client, off, 2);
  if (ret != 2)
  {
    return -EIO;
  }
  for (i = 0; i < 0x2000; i++);
  ret = i2c_master_recv(client, buf, len);
  if (ret != len)
  {
    return -EIO;
  }
  return ret;
}

/*
 * Automatically detects the presence of the 4.3" and 7" displays
 * from Embedded Artists. This is done by reading the display
 * configuration from an EEPROM on the display board. If the
 * configuration is present it will be used to setup the display.
 * If the EEPROM is not found or the configuration is incorrect then
 * it is assumed that the display is the 3.2" display. The 3.2" display
 * is present on both the QVGA Base Board and as a separate display
 * board to use with the OEM Base Board.
 */
static int ea_i2c_display_eeprom_probe(struct i2c_client *client,
      const struct i2c_device_id *id)
{
  int result = 0;
  size_t size;
  eaLcdbStore_t store;
  LCD_PARAM_T lcdParams;

  do
  {
    /* Read parameter header */
    size = sizeof(store);
    result = lcdb_eeprom_read(client, (uint8_t*)&store, 0, size);
    if (result != size)
    {
      break;
    }
    if (store.magic != LCDB_MAGIC)
    {
      result = -ENODEV;
      break;
    }

    /* Found header with correct magic number, now read the LCD parameters */
    size = (store.initOff-store.lcdParamOff);
    result = lcdb_eeprom_read(client, (uint8_t*)&lcdParams, store.lcdParamOff, size);
    if (result != size)
    {
      break;
    }

    /* Allocate memory for display parameters that need to be kept */
    lpc32xx_lcd_parameters = NULL;
    size = sizeof(lpc32xx_lcd_parameters_t);
    lpc32xx_lcd_parameters = kmalloc(size, GFP_KERNEL);
    if (!lpc32xx_lcd_parameters) {
      result = -ENOMEM;
      break;
    }

    /* Read initialization sequence */
    size = (store.pdOff-store.initOff);
    BUG_ON(size > LCD_SEQUENCE_BUF_SZ);
    result = lcdb_eeprom_read(client, (uint8_t*)lpc32xx_lcd_parameters->init_seq, store.initOff, size);
    if (result != size)
    {
      break;
    }

    /* Read power down sequence */
    size = (store.tsOff-store.pdOff);
    BUG_ON(size > LCD_SEQUENCE_BUF_SZ);
    result = lcdb_eeprom_read(client, (uint8_t*)lpc32xx_lcd_parameters->power_down_seq, store.pdOff, size);
    if (result != size)
    {
      break;
    }

    memcpy(lpc32xx_lcd_parameters->hardware_name, store.lcd_name, LCDB_NAME_BUF_SZ);

    if (lcdParams.pixels_per_line == 800 && lcdParams.lines_per_panel == 480) {
      sprintf(lpc32xx_lcd_parameters->panel_name, "7 inch WVGA landscape");
    } else if (lcdParams.pixels_per_line == 480 && lcdParams.lines_per_panel == 272) {
      sprintf(lpc32xx_lcd_parameters->panel_name, "4.3 inch WVGA landscape");
    } else {
      sprintf(lpc32xx_lcd_parameters->panel_name, "%d x %d unknown", lcdParams.pixels_per_line, lcdParams.lines_per_panel);
    }
    lpc32xx_lcd_parameters->lcd_panel_from_eeprom.mode.name = lpc32xx_lcd_parameters->panel_name;
    lpc32xx_lcd_parameters->lcd_panel_from_eeprom.mode.refresh = 60;
    lpc32xx_lcd_parameters->lcd_panel_from_eeprom.mode.xres = lcdParams.pixels_per_line;
    lpc32xx_lcd_parameters->lcd_panel_from_eeprom.mode.yres = lcdParams.lines_per_panel;
    lpc32xx_lcd_parameters->lcd_panel_from_eeprom.mode.pixclock = KHZ2PICOS((lcdParams.optimal_clock / 1000)); //optimal_clock is in Hz
    lpc32xx_lcd_parameters->lcd_panel_from_eeprom.mode.left_margin = lcdParams.h_front_porch;
    lpc32xx_lcd_parameters->lcd_panel_from_eeprom.mode.right_margin = lcdParams.h_back_porch;
    lpc32xx_lcd_parameters->lcd_panel_from_eeprom.mode.upper_margin = lcdParams.v_front_porch;
    lpc32xx_lcd_parameters->lcd_panel_from_eeprom.mode.lower_margin = lcdParams.v_back_porch;
    lpc32xx_lcd_parameters->lcd_panel_from_eeprom.mode.hsync_len = lcdParams.h_sync_pulse_width;
    lpc32xx_lcd_parameters->lcd_panel_from_eeprom.mode.vsync_len = lcdParams.v_sync_pulse_width;
    lpc32xx_lcd_parameters->lcd_panel_from_eeprom.mode.sync = 0;
    lpc32xx_lcd_parameters->lcd_panel_from_eeprom.mode.vmode = FB_VMODE_NONINTERLACED;
    lpc32xx_lcd_parameters->lcd_panel_from_eeprom.width = -1;
    lpc32xx_lcd_parameters->lcd_panel_from_eeprom.height = -1;
    lpc32xx_lcd_parameters->lcd_panel_from_eeprom.tim2 = (TIM2_IVS | TIM2_IHS);
    lpc32xx_lcd_parameters->lcd_panel_from_eeprom.cntl = (CNTL_BGR | CNTL_LCDTFT | CNTL_LCDVCOMP(1) | CNTL_LCDBPP16_565);
    lpc32xx_lcd_parameters->lcd_panel_from_eeprom.bpp = 16;

    lpc32xx_clcd_data.name = lpc32xx_lcd_parameters->hardware_name;
    return 0;
  } while(0);

  if (lpc32xx_lcd_parameters != NULL)
  {
    kfree(lpc32xx_lcd_parameters);
    lpc32xx_lcd_parameters = NULL;
  }

  return result;
}

static int __devexit ea_i2c_display_eeprom_remove(struct i2c_client *client)
{
  if (lpc32xx_lcd_parameters != NULL)
  {
    kfree(lpc32xx_lcd_parameters);
    lpc32xx_lcd_parameters = NULL;
  }

  return 0;
}

static int ea_i2c_video_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
  ea_i2c_video_client = client;
  return 0;
}

static int __devexit ea_i2c_video_remove(struct i2c_client *client)
{
  ea_i2c_video_client = NULL;
  return 0;
}

static const struct i2c_device_id ea_i2c_video_id[] = {
    { "ea_i2c_video", 0 },
    { }
};

static struct i2c_driver ea_i2c_video_driver = {
	.driver = {
		.name	= "ea_i2c_video",
	},
	.probe		= ea_i2c_video_probe,
	.remove		= ea_i2c_video_remove,
	.id_table	= ea_i2c_video_id,
};

static const struct i2c_device_id ea_i2c_display_eeprom_id[] = {
    { "ea_i2c_disp_cfg", 0 },
    { }
};

static struct i2c_driver ea_i2c_display_eeprom_driver = {
  .driver = {
    .name = "ea_i2c_disp_cfg",
  },
  .probe    = ea_i2c_display_eeprom_probe,
  .remove   = ea_i2c_display_eeprom_remove,
  .id_table = ea_i2c_display_eeprom_id,
};

static void init_ea_i2c_video(void)
{
  i2c_add_driver(&ea_i2c_display_eeprom_driver);
  i2c_add_driver(&ea_i2c_video_driver);
}

/**
 * Helper function to set LED states
 */
static void setLsStates(uint16_t states, uint8_t* ls, uint8_t mode)
{
#define IS_LED_SET(bit, x) ( ( ((x) & (bit)) != 0 ) ? 1 : 0 )

    int i = 0;

    for (i = 0; i < 4; i++) {

        ls[i] |= ( (IS_LED_SET(0x0001, states)*mode << 0)
                | (IS_LED_SET(0x0002, states)*mode << 2)
                | (IS_LED_SET(0x0004, states)*mode << 4)
                | (IS_LED_SET(0x0008, states)*mode << 6) );

        states >>= 4;
    }
}

/**
 * Set LEDs (PCA9532 on LCD board)
 */
static void setLeds(void)
{
    uint8_t buf[5];
    uint8_t ls[4] = {0,0,0,0};
    uint16_t states = ledStateShadow;

    /* LEDs in On/Off state */
    setLsStates(states, ls, LCDB_LS_MODE_ON);

    /* set the LEDs that should blink */
    setLsStates(blink0Shadow, ls, LCDB_LS_MODE_BLINK0);
    setLsStates(blink1Shadow, ls, LCDB_LS_MODE_BLINK1);

    buf[0] = LCDB_PCA9532_LS0 | LCDB_PCA9532_AUTO_INC;
    buf[1] = ls[0];
    buf[2] = ls[1];
    buf[3] = ls[2];
    buf[4] = ls[3];
    i2c_master_send(ea_i2c_video_client, buf, 5);
}

/**
 * Set LED states (on or off).
 *
 * Params:
 *    [in]  ledOnMask  - The LEDs that should be turned on. This mask has
 *                       priority over ledOffMask
 *    [in]  ledOffMask - The LEDs that should be turned off.
 *
 */
static void lcdb_pca9532_setLeds (uint16_t ledOnMask, uint16_t ledOffMask)
{
    /* turn off leds */
    ledStateShadow &= (~(ledOffMask) & 0xffff);

    /* ledOnMask has priority over ledOffMask */
    ledStateShadow |= ledOnMask;

    /* turn off blinking */
    blink0Shadow &= (~(ledOffMask) & 0xffff);
    blink1Shadow &= (~(ledOffMask) & 0xffff);

    setLeds();
}

/**
 *  Set the blink period for PWM0. Valid values are 0 - 255 where 0
 *  means 152 Hz and 255 means 0.59 Hz. A value of 151 means 1 Hz.
 *
 * Params:
 *    [in]  period  - the period for pwm0
 *
 */
static void lcdb_pca9532_setBlink0Period(uint8_t period)
{
    uint8_t buf[2];

    buf[0] = LCDB_PCA9532_PSC0;
    buf[1] = period;
    i2c_master_send(ea_i2c_video_client, buf, 2);
}

/**
 * Set the duty cycle for PWM0. Valid values are 0 - 100. 25 means the LED
 * is on 25% of the period.
 *
 * Params:
 *    [in]  duty  - duty cycle
 *
 */
static void lcdb_pca9532_setBlink0Duty(uint8_t duty)
{
    uint8_t buf[2];
    uint32_t tmp = duty;
    if (tmp > 100) {
        tmp = 100;
    }

    tmp = (255 * tmp)/100;

    buf[0] = LCDB_PCA9532_PWM0;
    buf[1] = tmp;
    i2c_master_send(ea_i2c_video_client, buf, 2);
}

/**
 *  Set the LEDs that should blink with rate and duty cycle from PWM0.
 *  Blinking is turned off with pca9532_setLeds.
 *
 * Params:
 *    [in]  ledMask  - LEDs that should blink.
 *
 */
static void lcdb_pca9532_setBlink0Leds(uint16_t ledMask)
{
    blink0Shadow |= ledMask;
    setLeds();
}

/******************************************************************************
 *
 * Description:
 *    Enable/disable 3V3 signal
 *
 *****************************************************************************/
static void ea_lcdb_ctrl_3v3(uint32_t enable)
{
  if (enable) {
    lcdb_pca9532_setLeds(LCDB_CTRL_3V3, 0);
  } else {
    lcdb_pca9532_setLeds(0, LCDB_CTRL_3V3);
  }

}

/******************************************************************************
 *
 * Description:
 *    Enable/disable 5V signal
 *
 *****************************************************************************/
static void ea_lcdb_ctrl_5v(uint32_t enable)
{
  if (enable) {
    lcdb_pca9532_setLeds(LCDB_CTRL_5V, 0);
  } else {
    lcdb_pca9532_setLeds(0, LCDB_CTRL_5V);
  }
}

/******************************************************************************
 *
 * Description:
 *    Enable/disable display enable signal
 *
 *****************************************************************************/
static void ea_lcdb_ctrl_display(uint32_t enable)
{
  if (!enable) {
    lcdb_pca9532_setLeds(LCDB_CTRL_DISP_EN, 0);
  } else {
    lcdb_pca9532_setLeds(0, LCDB_CTRL_DISP_EN);
  }
}

/******************************************************************************
 *
 * Description:
 *    Set backlight contrast
 *
 * Params:
 *   [in] value - backlight value; valid values 0-100
 *
 *****************************************************************************/
static void ea_lcdb_ctrl_backlightContrast(uint32_t value)
{
  lcdb_pca9532_setBlink0Duty(100-value);
  lcdb_pca9532_setBlink0Period(0);
  lcdb_pca9532_setBlink0Leds(LCDB_CTRL_BL_C);
}


static void spiSend(u8 *buf, size_t len)
{
	BUG_ON(ea3250_spi_lcd_dev == NULL);
	spi_write(ea3250_spi_lcd_dev, buf, len);
}

static void writeToReg(u16 addr, u16 data)
{
	u8 buf[3];

	RESET_RS;
	buf[0] = 0x00;
	buf[1] = addr & 0xff;
	spiSend(buf, 2);

	SET_RS;
	buf[0] = data >> 8;
	buf[1] = data & 0xff;
	spiSend(buf, 2);

	RESET_RS;
	buf[0] = 0x00;
	buf[1] = 0x22;
	spiSend(buf, 2);
}

static void clcd_display_init(void)
{
	u32 tmp;

	/* setup MUX register to use SSP0 */
	__raw_writel(( _BIT(12) | _BIT(10) | _BIT(9) ), LPC32XX_GPIO_P_MUX_SET);
	tmp = __raw_readl(LPC32XX_GPIO_P_MUX_SET);

	writeToReg (0x00,0x0001);
	mdelay(20);
	writeToReg (0x03,0xA2A4);
	writeToReg (0x0C,0x0004);
	writeToReg (0x0D,0x0308);
	writeToReg (0x0E,0x3000);
	mdelay(50);
	writeToReg (0x1E,0x00AF);
	writeToReg (0x01,0x2B3F);
	writeToReg (0x02,0x0600);
	writeToReg (0x10,0x0000);
	writeToReg (0x07,0x0233);
	writeToReg (0x0B,0x0039);
	writeToReg (0x0F,0x0000);
	mdelay(50);

	writeToReg (0x30,0x0707);
	writeToReg (0x31,0x0204);
	writeToReg (0x32,0x0204);
	writeToReg (0x33,0x0502);
	writeToReg (0x34,0x0507);
	writeToReg (0x35,0x0204);
	writeToReg (0x36,0x0204);
	writeToReg (0x37,0x0502);
	writeToReg (0x3A,0x0302);
	writeToReg (0x3B,0x0302);

	writeToReg (0x23,0x0000);
	writeToReg (0x24,0x0000);

	writeToReg (0x48,0x0000);
	writeToReg (0x49,0x013F);
	writeToReg (0x4A,0x0000);
	writeToReg (0x4B,0x0000);

	writeToReg (0x41,0x0000);
	writeToReg (0x42,0x0000);

	writeToReg (0x44,0xEF00);
	writeToReg (0x45,0x0000);
	writeToReg (0x46,0x013F);
	mdelay(50);

	writeToReg (0x44,0xEF00);
	writeToReg (0x45,0x0000);
	writeToReg (0x4E,0x0000);
	writeToReg (0x4F,0x0000);
	writeToReg (0x46,0x013F);
}

/**
 * Convert a string to an integer
 *
 * @params str - the string to convert
 * @params len - length of the string (it might not be null terminated)
 */
static uint32_t str_to_uint(char* str, uint32_t len)
{
  uint32_t val = 0;

  while(len > 0 && *str <= '9' && *str >= '0') {
    val = (val * 10) + (*str - '0');
    str++;
    len--;
  }

  return val;
}

/**
 * Check if the sequence string version can be handled.
 *
 */
static int checkVersion(char* v, uint32_t len)
{
  uint32_t ver = str_to_uint(v, len);

  if (ver > 1) {
    return -1; //only supports v1
  }

  return 0;
}

/**
 * Execute a control request (PCA9532)
 */
static int execSeqCtrl(char* cmd, uint32_t len)
{

  switch (*cmd++) {
    // display enable
    case 'd':
      ea_lcdb_ctrl_display(*cmd == '1');
      break;

    // backlight contrast
    case 'c':
      ea_lcdb_ctrl_backlightContrast(str_to_uint(cmd, len));
      break;

    // 3v3 enable
    case '3':
      ea_lcdb_ctrl_3v3(*cmd == '1');
      break;

    // 5v enable
    case '5':
      ea_lcdb_ctrl_5v(*cmd == '1');
      break;

  }

  return 0;
}

static void lcd_processSequence(char* str)
{
  char* c = NULL;
  int result = 0;
  uint32_t len;

  BUG_ON(str == NULL);

  while(*str != '\0') {

    // skip whitespace
    while(*str == ' ') {
      str++;
    }

    c = str;

    // find end of command
    while(*str != ',' && *str != '\0') {
      str++;
    }

    len = (str-c);

    if (*str == ',') {
      str++;
    }

    switch (*c++) {

      case 'v':
        result = checkVersion(c, len-1);
        break;

      // sequence control command (pca9532)
      case 'c':
        execSeqCtrl(c, len-1);
        break;

      // delay
      case 'd':
        //execDelay(c, len-1);
        mdelay(str_to_uint(c, len-1));
        break;

      // open lcd (init LCD controller)
      case 'o':
//        if (lcdParams != NULL) {
//          *pDev = lcd_open(CLCDC, (int32_t)lcdParams);
//        }
//        else {
//          result = LCDB_RESULT_INVALID_ARG;
//        }
        break;
    }

    if (result != 0) {
      break;
    }
  }
}

static void lpc32xx_clcd_disable(struct clcd_fb *fb)
{
    /* Disable the backlight */
  if (lpc32xx_lcd_parameters != NULL)
  {
    lcd_processSequence(lpc32xx_lcd_parameters->power_down_seq);
  }
  else
  {
    gpio_set_value(BKL_POW_GPIO, 1);
  }
}

static void lpc32xx_clcd_enable(struct clcd_fb *fb)
{
  if (lpc32xx_lcd_parameters != NULL)
  {
    lcd_processSequence(lpc32xx_lcd_parameters->init_seq);
  }
  else
  {
    clcd_display_init();
    gpio_set_value(BKL_POW_GPIO, 0);
  }
}

struct clcd_board lpc32xx_clcd_data = {
    .name = "Default 3.2 inch LCD",
	.check = clcdfb_check,
	.decode = clcdfb_decode,
	.disable = lpc32xx_clcd_disable,
	.enable = lpc32xx_clcd_enable,
	.setup = lpc32xx_clcd_setup,
	.mmap = lpc32xx_clcd_mmap,
	.remove = lpc32xx_clcd_remove,
};

struct amba_device lpc32xx_clcd_device = {
	.dev = {
		.coherent_dma_mask = ~0,
		.init_name = "dev:clcd",
		.platform_data = &lpc32xx_clcd_data,
	},
	.res                            = {
		.start = LPC32XX_LCD_BASE,
		.end = (LPC32XX_LCD_BASE + SZ_4K - 1),
		.flags = IORESOURCE_MEM,
	},
	.dma_mask = ~0,
	.irq = {IRQ_LPC32XX_LCD, NO_IRQ},
};


/*
 * SPI LCDC Driver Probe function
 */
static int ea3250_spi_lcdc_probe(struct spi_device *spi)
{
	int err;

	spi->mode = SPI_MODE_0;
	ea3250_spi_lcd_dev = spi;

	/* SPI settings */
	err = spi_setup(spi);
	if (err < 0) {
		dev_err(&spi->dev, "Err in setting SPI \n");
		return err;
	}

	return 0;
}

/*
 *  * SPI LCDC Driver remove function
 *   * */
static int ea3250_spi_lcdc_remove(struct spi_device *spi)
{
	return 0;
}

static struct spi_driver ea3250_spi_lcdc_driver = {
	.driver = {
		.name   = "ea3250_lcdc",
		.bus    = &spi_bus_type,
		.owner  = THIS_MODULE,
	},
	.probe  = ea3250_spi_lcdc_probe,
	.remove = __devexit_p(ea3250_spi_lcdc_remove),
};

void __init ea3250_spi_lcdc_drv_init(void)
{
  if (lpc32xx_lcd_parameters == NULL)
  {
    spi_register_driver(&ea3250_spi_lcdc_driver);
  }
}
#endif //defined (CONFIG_FB_ARMCLCD)

#if defined (CONFIG_MMC_ARMMMCI)

static int card_inserted = 1;

static int card_detect_thread(void __iomem* d)
{
    int err = 0;
    struct i2c_adapter *adap;
    struct i2c_client *client;
    u8 data = 0;

    while (!kthread_should_stop()) {
        adap = i2c_get_adapter(0);

        if (!adap) {
            set_current_state(TASK_INTERRUPTIBLE);
            schedule_timeout(HZ);

            continue;
        }

        list_for_each_entry(client, &adap->userspace_clients, detected) {
            if (client->addr == I2C_PCA9532_ADDR) {

                i2c_put_adapter(adap);

                /* select input0 register */
                data = 0;
                err = i2c_master_send(client, (char*)&data ,1);

                /* read value from register */
                err = i2c_master_recv(client, (char*)&data, 1);

                /* LED4 input on PCA9532 is connected to card detect (active low) */
                card_inserted = ((data & 0x10) == 0);

                break;
            }
        }

        set_current_state(TASK_INTERRUPTIBLE);
        schedule_timeout(HZ);
    }

    return 0;
}

static struct task_struct *cd_thread;

static void card_detect_start(void)
{
    cd_thread = kthread_run(card_detect_thread, NULL, "card-detect");
    if (IS_ERR(cd_thread)) {
        printk(KERN_INFO "Failed to start card detect thread\n");
    }
}

static void card_detect_stop(void)
{
    kthread_stop(cd_thread);
}

/*
 * Returns !0 when card is removed, 0 when present
 */
unsigned int mmc_card_detect(struct device *dev)
{
	/*
	 * This function may be adapted to retrieve the actual 'card detect'
	 * status over the I2C bus, from PCA9532 pin 8 (LED4). For now, simply
	 * indicate that a card is always present.
	*/
	return card_inserted;
}

/*
 * Enable or disable SD slot power
 */
void mmc_power_enable(int enable)
{
	if (enable != 0) {
	    card_detect_start();

		/* active low */
		gpio_set_value(MMC_POWER_GPIO,0);
	}
	else {
	    card_detect_stop();

		gpio_set_value(MMC_POWER_GPIO,1);
	}
}

/*
 * Board specific MMC driver data
 */
struct mmci_platform_data lpc32xx_plat_data = {
	.ocr_mask	= MMC_VDD_30_31|MMC_VDD_31_32|MMC_VDD_32_33|MMC_VDD_33_34,
	.vdd_handler	= NULL,
	.capabilities	= MMC_CAP_4_BIT_DATA,

	/*
	 * Indicate no direct GPIO, so MMC driver will assume card is NOT
	 * write-protected
	 */
	.gpio_wp        = -ENOSYS,

	/*
	 * Indicate no direct GPIO, so MMC driver will call 'status' callback
	 * function
	 */
	.gpio_cd        = -ENOSYS,

	/*
	 * Callback function, used by MMC driver in case 'gpio_cd'
	 * equals -ENOSYS
	 */
	.status		= mmc_card_detect,
};

/*
 * SD card controller resources
 */
struct amba_device lpc32xx_mmc_device = {
	.dev				= {
		.coherent_dma_mask	= ~0,
		.init_name		= "dev:mmc0",
		.platform_data		= &lpc32xx_plat_data,
	},
	.res				= {
		.start			= LPC32XX_SD_BASE,
		.end			= (LPC32XX_SD_BASE + SZ_4K - 1),
		.flags			= IORESOURCE_MEM,
	},
	.dma_mask			= ~0,
	.irq				= {IRQ_LPC32XX_SD0, IRQ_LPC32XX_SD1},
};
#endif

/* AMBA based devices list */
static struct amba_device *amba_devs[] __initdata = {
	&lpc32xx_ssp0_device,
#if defined (CONFIG_FB_ARMCLCD)
	&lpc32xx_clcd_device,
#endif
#if defined (CONFIG_MMC_ARMMMCI)
	&lpc32xx_mmc_device,
#endif
};

/*
 * Register AMBA BUS Devices.
 * Call AMBA device restration after SPI driver probe(),
 * as LCD controller uses SPI driver for initialization
 */
static int __init ea3250_amba_devices_register(void)
{
	u32 i = 0;

	/* Add AMBA devices */
	for (i = 0; i < ARRAY_SIZE(amba_devs); i++) {
		struct amba_device *d = amba_devs[i];
		amba_device_register(d, &iomem_resource);
	}

	return 0;
}
device_initcall_sync(ea3250_amba_devices_register);

#if defined(CONFIG_MTD_NAND_SLC_LPC32XX)
/*
 *  * Board specific NAND setup data
 *   */
static int nandwp_enable(int enable)
{
	if (enable != 0)
		gpio_set_value(NAND_WP_GPIO,0);
	else
		gpio_set_value(NAND_WP_GPIO,1);

	return 1;
}
#define BLK_SIZE (2048 * 64)
static struct mtd_partition ea3250_nand_partition[] = {
	{
		.name = "ea3250-boot",
		.offset = 0,
		.size = (BLK_SIZE * 4),
		.mask_flags = MTD_WRITEABLE,
	},
	{
		.name = "ea3250-uboot",
		.offset = MTDPART_OFS_APPEND,
		.size = (BLK_SIZE * 6)
	},
	{
		.name = "ea3250-kernel",
		.offset = MTDPART_OFS_APPEND,
		.size = (BLK_SIZE * 32)
	},
	{
		.name = "ea3250-jffs2",
		.offset = MTDPART_OFS_APPEND,
		.size = MTDPART_SIZ_FULL
	},
};
static struct mtd_partition * ea3250_nand_partitions(int size, int *num_partitions)
{
	*num_partitions = ARRAY_SIZE(ea3250_nand_partition);
	return ea3250_nand_partition;
}
struct lpc32XX_nand_cfg lpc32xx_nandcfg =
{
	.wdr_clks = 14,
	.wwidth = 260000000,
	.whold = 104000000,
	.wsetup = 200000000,
	.rdr_clks = 14,
	.rwidth = 34666666,
	.rhold = 104000000,
	.rsetup = 200000000,
	.use_bbt = true,
	.polled_completion = false,
	.enable_write_prot = nandwp_enable,
	.partition_info = ea3250_nand_partitions,
};

/*
 *  * SLC NAND resources
 *   */
static struct resource slc_nand_resources[] = {
	[0] = {
		.start = LPC32XX_SLC_BASE,
		.end = LPC32XX_SLC_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_LPC32XX_FLASH,
		.end = IRQ_LPC32XX_FLASH,
		.flags = IORESOURCE_IRQ,
	},
};

static u64 lpc32xx_slc_dma_mask = 0xffffffffUL;
static struct platform_device lpc32xx_slc_nand_device = {
	.name = "lpc32xx-nand",
	.id = 0,
	.dev = {
		.platform_data = &lpc32xx_nandcfg,
		.dma_mask = &lpc32xx_slc_dma_mask,
		.coherent_dma_mask = ~0UL,
	},
	.num_resources = ARRAY_SIZE(slc_nand_resources),
	.resource = slc_nand_resources,
};
#endif

/*
 * Network Support
 */
static struct lpc_net_cfg lpc32xx_netdata =
{
	.phy_irq        = -1,
	.phy_mask       = 0xFFFFFFF0,
};

static struct resource net_resources[] = {
	[0] = {
		.start = LPC32XX_ETHERNET_BASE,
		.end = LPC32XX_ETHERNET_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},

	[1] = {
		.start = IRQ_LPC32XX_ETHERNET,
		.end = IRQ_LPC32XX_ETHERNET,
		.flags = IORESOURCE_IRQ,
	},
};

static u64 lpc32xx_mac_dma_mask = 0xffffffffUL;
static struct platform_device lpc32xx_net_device = {
	.name = "lpc-net",
	.id = 0,
	.dev = {
		.dma_mask = &lpc32xx_mac_dma_mask,
		.coherent_dma_mask = 0xffffffffUL,
		.platform_data = &lpc32xx_netdata,
	},
	.num_resources = ARRAY_SIZE(net_resources),
	.resource = net_resources,
};

/*
 * I2C devices support
 */
#if defined (CONFIG_LEDS_PCA9532) || defined (CONFIG_EEPROM_AT24) || defined (CONFIG_FB_ARMCLCD)
	static struct i2c_board_info __initdata ea3250_i2c_board_info [] = {
#if defined (CONFIG_LEDS_PCA9532)
		{
			I2C_BOARD_INFO("pca9532", I2C_PCA9532_ADDR),
			.platform_data = &ea3250_leds,
		},
#endif
#if defined (CONFIG_FB_ARMCLCD)
		{
			/* 8Kb Configuration EEPROM on display board */
			I2C_BOARD_INFO("ea_i2c_disp_cfg", LCDB_CONFIG_EEPROM_I2C_ADDR),
		},
		{
			I2C_BOARD_INFO("ea_i2c_video", LCDB_PCA9532_I2C_ADDR),
		},
#endif
#if defined (CONFIG_EEPROM_AT24)
		{
			I2C_BOARD_INFO("24c256", I2C_24LC256_ADDR),
		},
#endif
	};
#endif

static struct platform_device* ea3250_devs[] __initdata = {
	&lpc32xx_i2c0_device,
	&lpc32xx_i2c1_device,
	&lpc32xx_i2c2_device,
	&lpc32xx_watchdog_device,
	&lpc32xx_gpio_led_device,
	&lpc32xx_rtc_device,
	&lpc32xx_net_device,
#if defined(CONFIG_MTD_NAND_SLC_LPC32XX)
	&lpc32xx_slc_nand_device,
#endif
#if defined(CONFIG_USB_OHCI_HCD)
	&lpc32xx_ohci_device,
#endif
#if defined(CONFIG_USB_GADGET_LPC32XX)
	&lpc32xx_usbd_device,
#endif
	&lpc32xx_i2s_device,
	&lpc32xx_asoc_plat_device,
};


/*
 * Board specific functions
 */
void __init ea3250_board_init(void)
{
	u32 tmp;

	/* Intiliase GPIO */
	lpc32xx_gpio_init();

#if defined (CONFIG_MMC_ARMMMCI)
	/* Enable SD slot power */
	mmc_power_enable(1);
#endif

	/* Set SPI CS GPIO to output */
	gpio_request(SPI0_CS_GPIO, "spi0 cs");
	gpio_direction_output(SPI0_CS_GPIO, 1);

	/* Setup network interface for RMII mode */
	tmp = __raw_readl(LPC32XX_CLKPWR_MACCLK_CTRL);
	tmp &= ~LPC32XX_CLKPWR_MACCTRL_PINS_MSK;
	tmp |= LPC32XX_CLKPWR_MACCTRL_USE_RMII_PINS;
	__raw_writel(tmp, LPC32XX_CLKPWR_MACCLK_CTRL);

	/* Setup SLC NAND controller */
	__raw_writel(LPC32XX_CLKPWR_NANDCLK_SEL_SLC,
			LPC32XX_CLKPWR_NAND_CLK_CTRL);

	/* Setup LCD muxing to RGB565 */
	tmp = __raw_readl(LPC32XX_CLKPWR_LCDCLK_CTRL) &
		~(LPC32XX_CLKPWR_LCDCTRL_LCDTYPE_MSK |
				LPC32XX_CLKPWR_LCDCTRL_PSCALE_MSK);
	tmp |= LPC32XX_CLKPWR_LCDCTRL_LCDTYPE_TFT16;
	__raw_writel(tmp, LPC32XX_CLKPWR_LCDCLK_CTRL);

	/* Set up I2C pull levels */
	tmp = __raw_readl(LPC32XX_CLKPWR_I2C_CLK_CTRL);
	tmp |= LPC32XX_CLKPWR_I2CCLK_USBI2CHI_DRIVE |
		LPC32XX_CLKPWR_I2CCLK_I2C2HI_DRIVE;
	__raw_writel(tmp, LPC32XX_CLKPWR_I2C_CLK_CTRL);

	/* Enable DMA for I2S1 channel */
	tmp = __raw_readl(LPC32XX_CLKPWR_I2S_CLK_CTRL);
	tmp = LPC32XX_CLKPWR_I2SCTRL_I2S1_USE_DMA;
	__raw_writel(tmp, LPC32XX_CLKPWR_I2S_CLK_CTRL);

	/* Initalise Serial device */
	lpc32xx_serial_init();

	/*
	 * AMBA peripheral clocks need to be enabled prior to AMBA device
	 * detection or a data fault will occur, so enable the clocks
	 * here. However, we don't want to enable them if the peripheral
	 * isn't included in the image
	 */
	/* Initialise SSP clock */
	tmp = __raw_readl(LPC32XX_CLKPWR_SSP_CLK_CTRL);
	__raw_writel((tmp | LPC32XX_CLKPWR_SSPCTRL_SSPCLK0_EN),
			LPC32XX_CLKPWR_SSP_CLK_CTRL);

	/* Initialise LCD clock */
	tmp = __raw_readl(LPC32XX_CLKPWR_LCDCLK_CTRL);
	__raw_writel((tmp | LPC32XX_CLKPWR_LCDCTRL_CLK_EN),
			LPC32XX_CLKPWR_LCDCLK_CTRL);

	/* Enable SD card clock so AMBA driver will work correctly. The
	   AMBA driver needs the clock before the SD card controller
	   driver initializes it. The clock will turn off once the driver
	   has been initialized. */
	tmp = __raw_readl(LPC32XX_CLKPWR_MS_CTRL);
	tmp |= LPC32XX_CLKPWR_MSCARD_SDCARD_EN |
		LPC32XX_CLKPWR_MSCARD_MSDIO_PU_EN;
	__raw_writel(tmp, LPC32XX_CLKPWR_MS_CTRL);

	/* Disable UART5->USB transparent mode or USB won't work */
	tmp = __raw_readl(LPC32XX_UARTCTL_CTRL);
	tmp &= ~LPC32XX_UART_U5_ROUTE_TO_USB;
	__raw_writel(tmp, LPC32XX_UARTCTL_CTRL);

	/* Add platform devcies */
	platform_add_devices(ea3250_devs, ARRAY_SIZE(ea3250_devs));

	/* Test clock needed for UDA1380 initial init */
	__raw_writel(LPC32XX_CLKPWR_TESTCLK2_SEL_MOSC |
			LPC32XX_CLKPWR_TESTCLK_TESTCLK2_EN,
			LPC32XX_CLKPWR_TEST_CLK_SEL);

#if defined (CONFIG_LEDS_PCA9532) || defined (CONFIG_EEPROM_AT24) || defined (CONFIG_FB_ARMCLCD)
	i2c_register_board_info(0, ea3250_i2c_board_info,
			ARRAY_SIZE(ea3250_i2c_board_info));
#endif

	/* Register the I2C driver for LCD */
	init_ea_i2c_video();

	/* Register SPI driver for LCD */
	ea3250_spi_lcdc_drv_init();
}

static int __init lpc32xx_display_uid(void)
{
	u32 uid[4];

	lpc32xx_get_uid(uid);

	printk(KERN_INFO "LPC32XX unique ID: %08x%08x%08x%08x\n",
			uid[3], uid[2], uid[1], uid[0]);

	return 1;
}
arch_initcall(lpc32xx_display_uid);

MACHINE_START (EA3250, "Embedded Artists LPC3250 OEM board with the LPC3250 Microcontroller")
	/* Maintainer: Embedded Artists */
	.boot_params	= 0x80000100,
	.map_io		= lpc32xx_map_io,
	.init_irq	= lpc32xx_init_irq,
	.timer		= &lpc32xx_timer,
	.init_machine	= ea3250_board_init,
	MACHINE_END

/* For backwards compatibility with older bootloaders only */
MACHINE_START (LPC3XXX, "Embedded Artists LPC3250 OEM board with the LPC3250 Microcontroller")
	/* Maintainer: Embedded Artists */
	.boot_params	= 0x80000100,
	.map_io		= lpc32xx_map_io,
	.init_irq	= lpc32xx_init_irq,
	.timer		= &lpc32xx_timer,
	.init_machine	= ea3250_board_init,
	MACHINE_END

