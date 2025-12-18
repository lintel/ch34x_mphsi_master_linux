// SPDX-License-Identifier: GPL-2.0-only
#ifndef _CH34X_MPHSI_H
#define _CH34X_MPHSI_H

#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/gpio/machine.h>
#include <linux/i2c.h>
#include <linux/idr.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/usb.h>

#define spi_master spi_controller
#define spi_register_master spi_register_controller
#define spi_unregister_master spi_unregister_controller
#define spi_master_put spi_controller_put
#define SPI_MASTER_MUST_RX SPI_CONTROLLER_MUST_RX
#define SPI_MASTER_MUST_TX SPI_CONTROLLER_MUST_TX
#define spi_master_get_devdata spi_controller_get_devdata
#define spi_alloc_master spi_alloc_host

#define spi_device_get_master(spi) ((spi)->controller)
#define spi_device_get_chip_select(spi, idx) ((spi)->chip_select[idx])

#define CH34X_USBDEV (&(ch34x_dev->intf->dev))
#define DRIVER_AUTHOR "WCH"
#define DRIVER_ALIAS "spi/i2c/gpio: ch347/ch341"
#define DRIVER_DESC "USB to SPI/I2C/GPIO master driver for ch347/ch341, etc."

#define MAX_BUFFER_LENGTH 0x1000

#define CH341_CS_NUM 3

#define CH341_USB_MAX_BULK_SIZE 32
#define CH341_USB_MAX_INTR_SIZE 8

#define CH341_CMD_SPI_STREAM 0xA8 /* CH341 SPI command */
#define CH341_CMD_UIO_STREAM 0xAB /* UIO command */

#define CH341_CMD_UIO_STM_IN 0x00  /* UIO IN  command (D0~D7) */
#define CH341_CMD_UIO_STM_OUT 0x80 /* UIO OUT command (D0~D5) */
#define CH341_CMD_UIO_STM_DIR 0x40 /* UIO DIR command (D0~D5) */
#define CH341_CMD_UIO_STM_END 0x20 /* UIO END command */
#define CH341_CMD_UIO_STM_US 0xc0  /* UIO US  command */

#define CH341_SPI_MAX_NUM_DEVICES 3
#define CH341_SPI_BUS_NUM 0
#define CH341_SPI_MODE SPI_MODE_0
#define CH341_SPI_MIN_FREQ 400
#define CH341_SPI_MAX_FREQ 1e6
#define CH341_SPI_MIN_BITS_PER_WORD 4
#define CH341_SPI_MAX_BITS_PER_WORD 32

#define SCK_BIT (1 << 3)
#define MOSI_BIT (1 << 5)
#define MISO_BIT (1 << 7)

#define CH347_CS_NUM 2

#define CH347_USB_MAX_BULK_SIZE 510
#define CH347_USB_BULK_EPSIZE 512
#define CH347_USB_MAX_INTR_SIZE 512

#define CH347_MAX_GPIOS 8
#define CH347F_MPHSI_GPIOS 8
#define CH347T_MPHSI_GPIOS 3

#define CH347_CMD_SPI_STREAM 0xA8 /* CH347 SPI command */

#define CH347_SPI_MAX_NUM_DEVICES 2
#define CH347_SPI_BUS_NUM 0
#define CH347_SPI_MODE SPI_MODE_0
#define CH347_SPI_MIN_FREQ 218750
#define CH347_SPI_MAX_FREQ 60e6
#define CH347_SPI_MIN_BITS_PER_WORD 4
#define CH347_SPI_MAX_BITS_PER_WORD 32

#define USB20_CMD_HEADER 3
#define USB20_CMD_SPI_INIT 0xC0 /* SPI Init Command */
#define USB20_CMD_SPI_CONTROL                                                  \
	0xC1 /* SPI Control Command, used to control the SPI interface chip    \
		selection pin output high or low level and delay time */
#define USB20_CMD_SPI_RD_WR                                                    \
	0xC2 /* SPI general read and write command, used for SPI general read  \
		and write operation, and for short packet communication */
#define USB20_CMD_SPI_BLCK_RD                                                  \
	0xC3 /* SPI read data command in batch, is generally used for the      \
		batch data read operation. */
#define USB20_CMD_SPI_BLCK_WR                                                  \
	0xC4 /* SPI write data command in batch, is generally used for the     \
		batch data write operation. */
#define USB20_CMD_INFO_RD                                                      \
	0xCA /* Parameter acquisition command, used to obtain SPI interface    \
		related parameters, etc */
#define USB20_CMD_SPI_CLK_INIT 0xE1 /* System clock control command */
#define USB20_CMD_FUNC_SWITCH 0xE2  /* ch347f function switch command */

/*SPI CMD*/
#define SET_CS 0
#define CLR_CS 1
#define SPI_CS_ACTIVE 0x00
#define SPI_CS_DEACTIVE 0x01

/* SPI_Clock_Polarity */
#define SPI_CPOL_Low ((u16)0x0000)
#define SPI_CPOL_High ((u16)0x0002)

/* SPI_Clock_Phase */
#define SPI_CPHA_1Edge ((u16)0x0000)
#define SPI_CPHA_2Edge ((u16)0x0001)

/******************************************************/

#define I2C_SPEED_20K 0	 /* low rate 20KHz */
#define I2C_SPEED_50K 4	 /* 50KHz */
#define I2C_SPEED_100K 1 /* standard rate 100KHz */
#define I2C_SPEED_200K 5 /* 200KHz */
#define I2C_SPEED_400K 2 /* fast rate 400KHz */
#define I2C_SPEED_750K 3 /* high rate 750KHz */
#define I2C_SPEED_1M 6	 /* 1MHz */
#define I2C_SPEED_2M 7	 /* 2MHz */

#define CH341_CMD_I2C_STREAM 0xAA

#define CH341_CMD_I2C_STM_STA 0x74
#define CH341_CMD_I2C_STM_STO 0x75
#define CH341_CMD_I2C_STM_OUT 0x80
#define CH341_CMD_I2C_STM_IN 0xC0
#define CH341_CMD_I2C_STM_SET 0x60
#define CH341_CMD_I2C_STM_END 0x00
#define CH347_CMD_I2C_STM_MAX 0x3F
#define CH347_CMD_I2C_STRETCH_Y 0x15 /* I2C Clock Stretch enable */
#define CH347_CMD_I2C_STRETCH_N 0x16 /* I2C Clock Stretch disable */

#define GPIO_MODE_IN 0
#define GPIO_MODE_OUT 1

#define USB20_CMD_GPIO_CMD 0xCC /* GPIO Command */
#define CH347_GPIO_CNT 8

#define GPIO_ENABLE BIT(7)
#define GPIO_DIR_SET BIT(6)

#define ch34x_spi_maser_to_dev(m)                                              \
	*((struct ch34x_device **)spi_master_get_devdata(m))

#ifndef USB_DEVICE_INTERFACE_NUMBER
#define USB_DEVICE_INTERFACE_NUMBER(vend, prod, num)                           \
	.match_flags =                                                         \
	    USB_DEVICE_ID_MATCH_DEVICE | USB_DEVICE_ID_MATCH_INT_NUMBER,       \
	.idVendor = (vend), .idProduct = (prod), .bInterfaceNumber = (num)
#endif

enum ch34x_chip_type {
	CHIP_CH341 = 0,
	CHIP_CH347F = 1,
	CHIP_CH347T = 2,
};

struct ch341_pin_config {
	u8 pin;	    /* pin number of ch341a/b/f */
	char *name; /* pin name */
};

struct ch347_pin_config {
	u8 pin;	    /* pin number of ch347t */
	char *name; /* pin name */
	u8 gpioindex;
	u8 mode;
	bool hwirq;
};

#pragma pack(1)

/* SPI setting structure */
struct ch34x_spi_config {
	u8 mode;	    /* 0-3: SPI Mode0/1/2/3 */
	u8 clock;	    /* SPI clock divider value */
	u8 byte_order;	    /* 0: LSB, 1: MSB */
	u16 rw_interval;    /* SPI read and write interval, unit: us */
	u8 out_def;	    /* SPI output data by default while read */
	u16 cs;		    /* SPI chip select bits */
	u8 cs0_polar;	    /* 0：low active, 1：high active */
	u8 cs1_polar;	    /* 0：low active, 1：high active */
	u16 auto_de_cs;	    /* automatically undo the CS after operation */
	u16 active_delay;   /* delay time after setting CS, unit: us */
	u32 deactive_delay; /* delay time after canceling CS, unit: us */
};

/* SPI Init structure definition */
struct ch34x_spi_init {
	u16 direction;
	u16 mode;
	u16 datasize;
	u16 clock_pol;
	u16 clock_phase;
	u16 nss;
	u16 baudrate_scale;
	u16 first_bit;
	u16 crc_poly;
};

struct ch34x_stream_hw_config {
	struct ch34x_spi_init spi_init;
	u16 spi_rw_interval;
	u8 spi_out_def;
	u8 misc_cfg;
	u8 reserved[4];
};

#pragma pack()

/* device specific structure */
struct ch34x_device {
	struct mutex io_mutex;
	struct mutex ops_mutex;
	struct usb_device *usb_dev; /* usb device */
	struct usb_interface *intf; /* usb interface */

	struct usb_endpoint_descriptor *bulk_in;  /* usb endpoint bulk in */
	struct usb_endpoint_descriptor *bulk_out; /* usb endpoint bulk out */
	struct usb_endpoint_descriptor *intr_in; /* usb endpoint interrupt in */
	u8 bulk_out_endpointAddr;		 /*bulk output endpoint*/

	u8 *bulkin_buf;	 /* usb bulk in buffer */
	u8 *bulkout_buf; /* usb bulk out buffer */
	u8 *intrin_buf;	 /* usb interrupt in buffer */
	dma_addr_t bulkin_dma;
	dma_addr_t bulkout_dma;
	dma_addr_t intrin_dma;

	struct usb_anchor
	    submitted; /* in case we need to retract our submissions */
	int errors;
	spinlock_t err_lock;

	int windex;

	struct urb *intr_urb;

	struct spi_master *master;
	struct spi_device *slaves[CH341_SPI_MAX_NUM_DEVICES];
	int slave_num;
	bool last_cpol; /* last message CPOL */

	u8 gpio_mask;	 /* configuratoin mask defines IN/OUT pins */
	u8 gpio_io_data; /* current value of ch341 I/O register */

	enum ch34x_chip_type chiptype;
	u16 firmver;
	struct ch34x_stream_hw_config hwcfg;
	struct ch34x_spi_config spicfg;

	int id;
	struct platform_device *spi_pdev;

	/* only ch347 */
	struct gpio_chip gpio;
	u8 gpio_num;

	struct ch347_pin_config *gpio_pins[CH347_MAX_GPIOS];
	char *gpio_names[CH347_MAX_GPIOS]; /* pin names (gpio_num elements) */
	int gpio_irq_map[CH347_MAX_GPIOS]; /* GPIO to IRQ map (gpio_num
					      elements) */

	/* irq device description */
	struct irq_chip irq;		   /* chip descriptor for IRQs */
	u8 irq_num;			   /* number of pins with IRQs */
	int irq_base;			   /* base IRQ allocated */
	int irq_types[CH347_MAX_GPIOS];	   /* IRQ types (irq_num elements) */
	bool irq_enabled[CH347_MAX_GPIOS]; /* IRQ enabled flag (irq_num
					      elements) */
	int irq_gpio_map[CH347_MAX_GPIOS]; /* IRQ to GPIO pin map (irq_num
					    * elements)
					    */
	spinlock_t irq_lock;

	struct delayed_work work;

	/* i2c master */
	struct i2c_adapter adapter;
	bool i2c_init;
};

/* Global function prototypes */
extern int ch34x_usb_transfer(struct ch34x_device *ch34x_dev, int out_len,
			      int in_len);
extern bool ch347_func_switch(struct ch34x_device *ch34x_dev, int index);
extern int ch34x_mphsi_spi_probe(struct ch34x_device *ch34x_dev);
extern int ch34x_mphsi_spi_remove(struct ch34x_device *ch34x_dev);
extern int ch34x_spi_probe(struct ch34x_device *ch34x_dev);
extern void ch34x_spi_remove(struct ch34x_device *ch34x_dev);
extern int ch34x_mphsi_i2c_probe(struct ch34x_device *ch34x_dev);
extern void ch34x_mphsi_i2c_remove(struct ch34x_device *ch34x_dev);
extern int ch347_irq_check(struct ch34x_device *ch34x_dev, u8 irq);
extern int ch347_irq_probe(struct ch34x_device *ch34x_dev);
extern void ch347_irq_remove(struct ch34x_device *ch34x_dev);
extern int ch34x_mphsi_gpio_probe(struct ch34x_device *ch34x_dev);
extern void ch34x_mphsi_gpio_remove(struct ch34x_device *ch34x_dev);

#endif
