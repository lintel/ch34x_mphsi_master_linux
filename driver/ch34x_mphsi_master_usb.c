/*
 * USB to SPI/I2C/GPIO master driver for USB converter chip ch347/ch341, etc.
 *
 * Copyright (C) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * Author: WCH <tech@wch.cn>
 */

#include "ch34x_mphsi.h"

static DEFINE_IDA(ch34x_devid_ida);

/* table of devices that work with this driver */
static const struct usb_device_id ch34x_usb_ids[] = {
    {USB_DEVICE(0x1a86, 0x5512)}, /* CH341 NON-UART Mode */
    {USB_DEVICE_INTERFACE_NUMBER(0x1a86, 0x55de, 0x04)}, /* CH347F */
    {USB_DEVICE_INTERFACE_NUMBER(0x1a86, 0x55db, 0x02)}, /* CH347T Mode 1 */
    {}							 /* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, ch34x_usb_ids);

struct ch341_pin_config ch341_board_config[CH341_CS_NUM] = {
    {15, "cs0"},
    {16, "cs1"},
    {17, "cs2"},
};

struct ch347_pin_config ch347t_board_config[CH347T_MPHSI_GPIOS] = {
    {15, "gpio4", 4, GPIO_MODE_OUT, true},
    {2, "gpio6", 6, GPIO_MODE_IN, true},
    {13, "gpio7", 7, GPIO_MODE_OUT, true},
};

struct ch347_pin_config ch347f_board_config[CH347F_MPHSI_GPIOS] = {
    {17, "gpio0", 0, GPIO_MODE_IN, true},
    {18, "gpio1", 1, GPIO_MODE_OUT, true},
    {10, "gpio2", 2, GPIO_MODE_OUT, true},
    {9, "gpio3", 3, GPIO_MODE_OUT, true},
    {23, "gpio4", 4, GPIO_MODE_OUT, true},
    {24, "gpio5", 5, GPIO_MODE_IN, true},
    {25, "gpio6", 6, GPIO_MODE_OUT, true},
    {26, "gpio7", 7, GPIO_MODE_OUT, true},
};

extern struct spi_board_info ch341_spi_device_template;
extern struct spi_board_info ch347_spi_device_template;
extern struct spi_board_info ch34x_spi_devices[CH341_SPI_MAX_NUM_DEVICES];

static int ch34x_cfg_probe(struct ch34x_device *ch34x_dev)
{
	struct ch341_pin_config *ch341cfg;
	struct ch347_pin_config *ch347cfg;
	int i;

	if (!ch34x_dev)
		return -EINVAL;

	if (ch34x_dev->chiptype == CHIP_CH341) {
		/* set out: mosi/out2/sck/cs, in: miso */
		ch34x_dev->gpio_mask = 0x3f;
		for (i = 0; i < CH341_CS_NUM; i++) {
			ch341cfg = ch341_board_config + i;

			ch34x_spi_devices[ch34x_dev->slave_num] =
			    ch341_spi_device_template;
			ch34x_spi_devices[ch34x_dev->slave_num].bus_num =
			    CH341_SPI_BUS_NUM;
			ch34x_spi_devices[ch34x_dev->slave_num].mode =
			    CH341_SPI_MODE;
			ch34x_spi_devices[ch34x_dev->slave_num].chip_select =
			    ch341cfg->pin - 15;

			dev_info(CH34X_USBDEV, "output %s SPI slave with CS%d",
				 ch341cfg->name,
				 ch34x_spi_devices[ch34x_dev->slave_num]
				     .chip_select);

			ch34x_dev->slave_num++;
		}
	} else {
		if ((ch34x_dev->firmver >= 0x0341) ||
		    (ch34x_dev->chiptype == CHIP_CH347F)) {
			ch34x_dev->irq_num = 0;
			ch34x_dev->irq_base = 0;
		}

		for (i = 0; i < CH347_CS_NUM; i++) {
			ch34x_spi_devices[ch34x_dev->slave_num] =
			    ch347_spi_device_template;
			ch34x_spi_devices[ch34x_dev->slave_num].bus_num =
			    CH347_SPI_BUS_NUM;
			ch34x_spi_devices[ch34x_dev->slave_num].mode =
			    CH347_SPI_MODE;
			ch34x_spi_devices[ch34x_dev->slave_num].chip_select = i;

			dev_info(CH34X_USBDEV, "output SPI slave with CS%d",
				 ch34x_spi_devices[ch34x_dev->slave_num]
				     .chip_select);

			ch34x_dev->slave_num++;
		}
		if (ch34x_dev->chiptype == CHIP_CH347T) {
			for (i = 0; i < CH347T_MPHSI_GPIOS; i++) {
				ch347cfg = ch347t_board_config + i;
				ch34x_dev->gpio_names[ch34x_dev->gpio_num] =
				    ch347cfg->name;
				ch34x_dev->gpio_pins[ch34x_dev->gpio_num] =
				    ch347cfg;
				if (ch34x_dev->firmver >= 0x0341) {
					ch34x_dev->gpio_irq_map
					    [ch34x_dev->gpio_num] =
					    ch34x_dev->irq_num;
					ch34x_dev
					    ->irq_gpio_map[ch34x_dev->irq_num] =
					    ch34x_dev->gpio_num;
					dev_dbg(
					    CH34X_USBDEV, "%s %s irq=%d %s",
					    ch347cfg->mode == GPIO_MODE_IN
						? "input "
						: "output",
					    ch347cfg->name, ch34x_dev->irq_num,
					    ch347cfg->hwirq ? "(hwirq)" : "");
					ch34x_dev->irq_num++;
				} else {
					dev_dbg(CH34X_USBDEV, "%s %s",
						ch347cfg->mode == GPIO_MODE_IN
						    ? "input "
						    : "output",
						ch347cfg->name);
				}
				ch34x_dev->gpio_num++;
			}
		} else {
			for (i = 0; i < CH347F_MPHSI_GPIOS; i++) {
				ch347cfg = ch347f_board_config + i;
				ch34x_dev->gpio_names[ch34x_dev->gpio_num] =
				    ch347cfg->name;
				ch34x_dev->gpio_pins[ch34x_dev->gpio_num] =
				    ch347cfg;
				ch34x_dev->gpio_irq_map[ch34x_dev->gpio_num] =
				    ch34x_dev->irq_num;
				ch34x_dev->irq_gpio_map[ch34x_dev->irq_num] =
				    ch34x_dev->gpio_num;
				dev_dbg(CH34X_USBDEV, "%s %s irq=%d %s",
					ch347cfg->mode == GPIO_MODE_IN
					    ? "input "
					    : "output",
					ch347cfg->name, ch34x_dev->irq_num,
					ch347cfg->hwirq ? "(hwirq)" : "");
				ch34x_dev->irq_num++;
				ch34x_dev->gpio_num++;
			}
		}
	}
	return 0;
}

static void ch34x_cfg_remove(struct ch34x_device *ch34x_dev)
{
	if (!(ch34x_dev))
		return;

	return;
}

int ch34x_usb_transfer(struct ch34x_device *ch34x_dev, int out_len, int in_len)
{
	int retval = 0;
	int actual = 0;

	if (out_len != 0) {
		retval = usb_bulk_msg(
		    ch34x_dev->usb_dev,
		    usb_sndbulkpipe(ch34x_dev->usb_dev,
				    ch34x_dev->bulk_out_endpointAddr),
		    ch34x_dev->bulkout_buf, out_len, &actual, 2000);
		if (retval < 0) {
			dev_err(CH34X_USBDEV,
				"%s - failed submitting write msg, error %d\n",
				__func__, retval);
			return retval;
		}
	}

	if (in_len == 0)
		return out_len;

	memset(ch34x_dev->bulkin_buf, 0, MAX_BUFFER_LENGTH * 2);
	retval =
	    usb_bulk_msg(ch34x_dev->usb_dev,
			 usb_rcvbulkpipe(ch34x_dev->usb_dev,
					 usb_endpoint_num(ch34x_dev->bulk_in)),
			 ch34x_dev->bulkin_buf, in_len, &actual, 2000);

	return retval < 0 ? retval : actual;
}

/**
 * ch347_get_chipinfo - get chip information
 */
static bool ch347_get_chipinfo(struct ch34x_device *ch34x_dev)
{
	u8 *io = ch34x_dev->bulkout_buf;
	int len, i;
	bool ret = false;

	mutex_lock(&ch34x_dev->io_mutex);
	i = 0;
	io[i++] = USB20_CMD_INFO_RD;
	io[i++] = 1;
	io[i++] = 0;
	io[i++] = 0;
	len = i;

	if (ch34x_usb_transfer(ch34x_dev, len, 0) != len)
		goto exit;

	len = 4 + USB20_CMD_HEADER;

	if (ch34x_usb_transfer(ch34x_dev, 0, len) != len)
		goto exit;

	if (ch34x_dev->bulkin_buf[0] != USB20_CMD_INFO_RD)
		goto exit;

	ch34x_dev->firmver =
	    (ch34x_dev->bulkin_buf[4] << 8) | ch34x_dev->bulkin_buf[3];
	ret = true;

exit:
	mutex_unlock(&ch34x_dev->io_mutex);
	return ret;
}

/**
 * ch347_func_switch - switch functions of ch347f
 * @index: 0->spi cs1 enable, 1->jtag SRST enable, 2->spi slave interrupt
 * enable, 3->i2c enable
 */
bool ch347_func_switch(struct ch34x_device *ch34x_dev, int index)
{
	u8 *io = ch34x_dev->bulkout_buf;
	int len, i;
	bool ret = false;

	mutex_lock(&ch34x_dev->io_mutex);
	i = 0;
	io[i++] = USB20_CMD_FUNC_SWITCH;
	io[i++] = 8;
	io[i++] = 0;
	switch (index) {
	case 0:
		io[USB20_CMD_HEADER] = 0x81;
		break;
	case 1:
		io[USB20_CMD_HEADER + 1] = 0x81;
		break;
	case 2:
		io[USB20_CMD_HEADER + 1] = 0x82;
		break;
	case 3:
		io[USB20_CMD_HEADER + 2] = 0x81;
		io[USB20_CMD_HEADER + 3] = 0x81;
		break;
	default:
		break;
	}
	len = USB20_CMD_HEADER + 8;

	if (ch34x_usb_transfer(ch34x_dev, len, 0) != len)
		goto exit;

	len = 4;

	if (ch34x_usb_transfer(ch34x_dev, 0, len) != len)
		goto exit;

	if ((ch34x_dev->bulkin_buf[0] != USB20_CMD_FUNC_SWITCH) ||
	    (ch34x_dev->bulkin_buf[USB20_CMD_HEADER] != 0x00))
		goto exit;

	ret = true;

exit:
	mutex_unlock(&ch34x_dev->io_mutex);
	return ret;
}

static void ch34x_usb_complete_intr_urb(struct urb *urb)
{
	struct ch34x_device *ch34x_dev;
	u8 gpioindex;
	bool triggered;
	int i;
	u16 io_status;
	int gpiocount;

	if (!urb)
		return;
	ch34x_dev = urb->context;
	if (!ch34x_dev)
		return;

	if (!urb->status) {
		dev_dbg(CH34X_USBDEV, "urb status %d", urb->status);

		if (ch34x_dev->chiptype == CHIP_CH347F)
			gpiocount = CH347F_MPHSI_GPIOS;
		else
			gpiocount = CH347T_MPHSI_GPIOS;

		if (ch34x_dev->chiptype != CHIP_CH341) {
			for (i = 0; i < gpiocount; i++) {
				gpioindex = ch34x_dev->gpio_pins[i]->gpioindex;
				triggered =
				    ch34x_dev->intrin_buf[gpioindex + 3] &
				    BIT(3);
				dev_dbg(
				    CH34X_USBDEV,
				    "enable[%d]=%d gpioindex=%d triggered=%d",
				    i, ch34x_dev->irq_enabled[i], gpioindex,
				    triggered);
				if (ch34x_dev->irq_enabled[i] && triggered)
					ch347_irq_check(ch34x_dev, i);
			}
		} else {
			io_status = (ch34x_dev->intrin_buf[1] & 0x0F) << 8 |
				    ch34x_dev->intrin_buf[2];
			dev_dbg(CH34X_USBDEV, "ch341 io_status: 0x%04x",
				io_status);
		}

		usb_submit_urb(ch34x_dev->intr_urb, GFP_ATOMIC);
	}
}

static void ch34x_usb_free_device(struct ch34x_device *ch34x_dev)
{
	if (!ch34x_dev)
		return;

	if (ch34x_dev->intr_urb)
		usb_free_urb(ch34x_dev->intr_urb);

	if (ch34x_dev->bulkin_buf)
		usb_free_coherent(ch34x_dev->usb_dev, MAX_BUFFER_LENGTH * 2,
				  ch34x_dev->bulkin_buf, ch34x_dev->bulkin_dma);
	if (ch34x_dev->bulkout_buf)
		usb_free_coherent(ch34x_dev->usb_dev, MAX_BUFFER_LENGTH * 2,
				  ch34x_dev->bulkout_buf,
				  ch34x_dev->bulkout_dma);
	if (ch34x_dev->intrin_buf)
		usb_free_coherent(ch34x_dev->usb_dev, CH347_USB_MAX_INTR_SIZE,
				  ch34x_dev->intrin_buf, ch34x_dev->intrin_dma);

	if (ch34x_dev->intf)
		usb_set_intfdata(ch34x_dev->intf, NULL);

	usb_kill_anchored_urbs(&ch34x_dev->submitted);

	if (ch34x_dev->usb_dev)
		usb_put_dev(ch34x_dev->usb_dev);

	kfree(ch34x_dev);
}

static int ch34x_usb_probe(struct usb_interface *intf,
			   const struct usb_device_id *id)
{
	struct usb_device *usb_dev = usb_get_dev(interface_to_usbdev(intf));
	struct usb_endpoint_descriptor *endpoint;
	struct usb_host_interface *iface_desc;
	struct ch34x_device *ch34x_dev;
	int i, ret, length;

	dev_dbg(&intf->dev, "connect device");

	ch34x_dev = kzalloc(sizeof(struct ch34x_device), GFP_KERNEL);
	if (!ch34x_dev)
		return -ENOMEM;

	ch34x_dev->usb_dev = usb_dev;
	ch34x_dev->intf = intf;
	iface_desc = intf->cur_altsetting;

	dev_dbg(CH34X_USBDEV, "bNumEndpoints=%d",
		iface_desc->desc.bNumEndpoints);

	for (i = 0; i < iface_desc->desc.bNumEndpoints; i++) {
		endpoint = &iface_desc->endpoint[i].desc;

		dev_dbg(
		    CH34X_USBDEV, "    ->endpoint=%d type=%d dir=%d addr=%0x",
		    i, usb_endpoint_type(endpoint),
		    usb_endpoint_dir_in(endpoint), usb_endpoint_num(endpoint));

		if (usb_endpoint_is_bulk_in(endpoint)) {
			ch34x_dev->bulk_in = endpoint;
			ch34x_dev->bulkin_buf = usb_alloc_coherent(
			    ch34x_dev->usb_dev, MAX_BUFFER_LENGTH * 2,
			    GFP_KERNEL, &ch34x_dev->bulkin_dma);
			if (!ch34x_dev->bulkin_buf) {
				dev_err(CH34X_USBDEV,
					"could not allocate bulkin buffer");
				goto err_free_dev;
			}
		} else if (usb_endpoint_is_bulk_out(endpoint)) {
			ch34x_dev->bulk_out = endpoint;
			ch34x_dev->bulk_out_endpointAddr =
			    endpoint->bEndpointAddress;
			ch34x_dev->bulkout_buf = usb_alloc_coherent(
			    ch34x_dev->usb_dev, MAX_BUFFER_LENGTH * 2,
			    GFP_KERNEL, &ch34x_dev->bulkout_dma);
			if (!ch34x_dev->bulkout_buf) {
				dev_err(CH34X_USBDEV,
					"could not allocate bulkout buffer");
				goto err_free_dev;
			}
		} else if (usb_endpoint_xfer_int(endpoint)) {
			ch34x_dev->intr_in = endpoint;
			ch34x_dev->intrin_buf = usb_alloc_coherent(
			    ch34x_dev->usb_dev, CH347_USB_MAX_INTR_SIZE,
			    GFP_KERNEL, &ch34x_dev->intrin_dma);
			if (!ch34x_dev->intrin_buf) {
				dev_err(CH34X_USBDEV,
					"could not allocate intrin buffer");
				goto err_free_dev;
			}
		}
	}

	if (id->idProduct == 0x5512) {
		ch34x_dev->chiptype = CHIP_CH341;
		length = CH341_USB_MAX_INTR_SIZE;
	} else if (id->idProduct == 0x55de) {
		ch34x_dev->chiptype = CHIP_CH347F;
		length = CH347_USB_MAX_INTR_SIZE;
	} else {
		ch34x_dev->chiptype = CHIP_CH347T;
		length = CH347_USB_MAX_INTR_SIZE;
	}

	/* save the pointer to the new ch34x_device in USB interface device data
	 */
	usb_set_intfdata(intf, ch34x_dev);
	mutex_init(&ch34x_dev->io_mutex);
	spin_lock_init(&ch34x_dev->err_lock);
	init_usb_anchor(&ch34x_dev->submitted);

	if (ch34x_dev->chiptype != CHIP_CH341) {
		if (ch347_get_chipinfo(ch34x_dev) == false) {
			ret = -EPROTO;
			goto err_free_dev;
		}
		dev_dbg(CH34X_USBDEV, "Firmware version: 0x%x.",
			ch34x_dev->firmver);
	}

	ch34x_dev->id = ida_simple_get(&ch34x_devid_ida, 0, 0, GFP_KERNEL);
	if (ch34x_dev->id < 0) {
		ret = ch34x_dev->id;
		goto err_free_dev;
	}

	ret = ch34x_cfg_probe(ch34x_dev);
	if (ret < 0)
		goto err_remove_ida;

	ret = ch34x_mphsi_spi_probe(ch34x_dev);
	if (ret < 0)
		goto err_cfg_remove;

	ret = ch34x_spi_probe(ch34x_dev);
	if (ret < 0)
		goto err_spi_mphsi_remove;

	ret = ch34x_mphsi_i2c_probe(ch34x_dev);
	if (ret < 0)
		goto err_spi_remove;

	if ((ch34x_dev->firmver >= 0x0341) ||
	    (ch34x_dev->chiptype == CHIP_CH347F)) {
		ret = ch347_irq_probe(ch34x_dev);
		if (ret < 0)
			goto err_i2c_remove;
	}

	if (ch34x_dev->chiptype != CHIP_CH341) {
		ret = ch34x_mphsi_gpio_probe(ch34x_dev);
		if (ret < 0)
			goto err_irq_remove;
	}

	if (ch34x_dev->intr_in) {
		/* create URBs for handling interrupts */
		ch34x_dev->intr_urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!ch34x_dev->intr_urb) {
			dev_err(&intf->dev, "failed to alloc URB");
			ret = -ENOMEM;
			goto err_gpio_remove;
		}

		usb_fill_int_urb(
		    ch34x_dev->intr_urb, ch34x_dev->usb_dev,
		    usb_rcvintpipe(ch34x_dev->usb_dev,
				   usb_endpoint_num(ch34x_dev->intr_in)),
		    ch34x_dev->intrin_buf, length, ch34x_usb_complete_intr_urb,
		    ch34x_dev, ch34x_dev->intr_in->bInterval);

		usb_submit_urb(ch34x_dev->intr_urb, GFP_ATOMIC);
	}

	dev_info(CH34X_USBDEV,
		 "USB to SPI/I2C/GPIO adapter ch34x now attached.");

	return 0;

err_gpio_remove:
	ch34x_mphsi_gpio_remove(ch34x_dev);
err_irq_remove:
	if ((ch34x_dev->firmver >= 0x0341) ||
	    (ch34x_dev->chiptype == CHIP_CH347F))
		ch347_irq_remove(ch34x_dev);
err_i2c_remove:
	ch34x_mphsi_i2c_remove(ch34x_dev);
err_spi_remove:
	ch34x_spi_remove(ch34x_dev);
err_spi_mphsi_remove:
	ch34x_mphsi_spi_remove(ch34x_dev);
err_cfg_remove:
	ch34x_cfg_remove(ch34x_dev);
err_remove_ida:
	ida_simple_remove(&ch34x_devid_ida, ch34x_dev->id);
err_free_dev:
	ch34x_usb_free_device(ch34x_dev);
	return ret;
}

static void ch34x_draw_down(struct ch34x_device *ch34x_dev)
{
	int time;

	time = usb_wait_anchor_empty_timeout(&ch34x_dev->submitted, 1000);
	if (!time)
		usb_kill_anchored_urbs(&ch34x_dev->submitted);
}

static int ch34x_usb_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct ch34x_device *ch34x_dev = usb_get_intfdata(intf);

	if (!ch34x_dev)
		return 0;
	ch34x_draw_down(ch34x_dev);
	return 0;
}

static int ch34x_usb_resume(struct usb_interface *intf) { return 0; }

static int ch34x_pre_reset(struct usb_interface *intf)
{
	struct ch34x_device *ch34x_dev = usb_get_intfdata(intf);

	mutex_lock(&ch34x_dev->io_mutex);
	ch34x_draw_down(ch34x_dev);

	return 0;
}

static int ch34x_post_reset(struct usb_interface *intf)
{
	struct ch34x_device *ch34x_dev = usb_get_intfdata(intf);

	ch34x_dev->errors = -EPIPE;
	mutex_unlock(&ch34x_dev->io_mutex);

	return 0;
}

static void ch34x_usb_disconnect(struct usb_interface *intf)
{
	struct ch34x_device *ch34x_dev = usb_get_intfdata(intf);

	dev_info(CH34X_USBDEV, "ch34x adapter now disconnected");

	ch34x_mphsi_gpio_remove(ch34x_dev);
	if ((ch34x_dev->firmver >= 0x0341) ||
	    (ch34x_dev->chiptype == CHIP_CH347F))
		ch347_irq_remove(ch34x_dev);
	ch34x_mphsi_i2c_remove(ch34x_dev);
	ch34x_spi_remove(ch34x_dev);
	ch34x_mphsi_spi_remove(ch34x_dev);
	ch34x_cfg_remove(ch34x_dev);
	ida_simple_remove(&ch34x_devid_ida, ch34x_dev->id);
	ch34x_usb_free_device(ch34x_dev);
}

static struct usb_driver ch34x_usb_driver = {
    .name = "mphsi-ch34x",
    .id_table = ch34x_usb_ids,
    .probe = ch34x_usb_probe,
    .disconnect = ch34x_usb_disconnect,
    .suspend = ch34x_usb_suspend,
    .resume = ch34x_usb_resume,
    .pre_reset = ch34x_pre_reset,
    .post_reset = ch34x_post_reset,
};

module_usb_driver(ch34x_usb_driver);

MODULE_ALIAS(DRIVER_ALIAS);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
