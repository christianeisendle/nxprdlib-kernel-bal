/*
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
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/spi/bal_spi.h>
#include <linux/uaccess.h>

#define BALDEV_NAME "bal"
#define BALDEV_MAJOR			100
#define BALDEV_MINOR			0

#define BAL_MAX_BUF_SIZE		1024
#define BAL_BUSY_TIMEOUT_SECS		1

#define BAL_IOC_RW_MULTI_REG		4

#define BAL_HAL_HW_RC523		0
#define BAL_HAL_HW_RC663		1
#define BAL_HAL_HW_PN5180		2

struct bal_data {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;

	struct mutex		use_lock;
	bool			in_use;
	unsigned int		busy_pin;
	u8	*		buffer;

	struct spi_transfer xfers;

	unsigned long		HalType;
	unsigned long		MultiRegRW;
};


static struct bal_data bal;
static struct class *baldev_class;

static ssize_t
wait_for_busy_idle(void)
{
	unsigned long tmo;
	tmo = jiffies + (BAL_BUSY_TIMEOUT_SECS * HZ);
	while (gpio_get_value(bal.busy_pin) == 1) {
		if (time_after(jiffies, tmo)) {
			dev_err(&bal.spi->dev, "Timeout occured waiting for BUSY going low\n");
			return -EBUSY;
		}
	}
	return 0;
}

static ssize_t
baldev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	ssize_t			status = 0;

	uint8_t			readFlag = 0;

	size_t pos = 0;

	if (count > BAL_MAX_BUF_SIZE)
		return -EMSGSIZE;

	if(bal.HalType == BAL_HAL_HW_PN5180)
	{
		status = wait_for_busy_idle();

		if (0 == status)
			status = spi_read(bal.spi, bal.buffer, count);
		if (status < 0)
			return status;
		if (copy_to_user(buf, bal.buffer, count))
			return -EFAULT;
	}
	else
	{
		status = copy_from_user(bal.buffer, buf, count);


		if(bal.HalType == BAL_HAL_HW_RC523)
			readFlag = bal.buffer[0] & 0x80;

		if(bal.HalType == BAL_HAL_HW_RC663)
			readFlag = bal.buffer[0] & 0x01;

		/* Perform special multireg write only for RC663 */
		if(bal.MultiRegRW == 1 && bal.HalType == BAL_HAL_HW_RC663 && readFlag == 0)
		{
			while( pos < count )
			{
				bal.xfers.tx_buf = bal.buffer + pos;
				bal.xfers.rx_buf = bal.buffer + pos;
				bal.xfers.len = 2;

				status = spi_sync_transfer(bal.spi, &(bal.xfers), 1);
				if(status < 0)
					return status;

				pos += 2;
			}
		}
		else
		{
			bal.xfers.tx_buf = bal.buffer;
			bal.xfers.rx_buf = bal.buffer;
			bal.xfers.len = count;

			status = spi_sync_transfer(bal.spi, &(bal.xfers), 1);

			if(status)
				return status;

		}

		if(readFlag != 0)
			if (copy_to_user(buf, bal.buffer, count))
				return -EFAULT;
	}

	return count;
}


static ssize_t
baldev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	ssize_t status = 0;

	if (count > BAL_MAX_BUF_SIZE) {
		return -ENOMEM;
	}

	if(bal.HalType == BAL_HAL_HW_PN5180)
	{
		status = copy_from_user(bal.buffer, buf, count);
		if (status)
			return status;

		status = wait_for_busy_idle();
		if (status == 0)
			status = spi_write(bal.spi, bal.buffer, count);
	}
	else
		status = EBADE; //invalid exchange

	if (status < 0)
		return status;

	return count;
}


static int baldev_open(struct inode *inode, struct file *filp)
{
	mutex_lock(&bal.use_lock);
	if (bal.in_use) {
		dev_err(&bal.spi->dev, "Can only be opened once!\n");
		mutex_unlock(&bal.use_lock);
		return -EBUSY;
	}
	bal.buffer = (uint8_t *)kmalloc(BAL_MAX_BUF_SIZE, GFP_KERNEL | GFP_DMA);
	if (bal.buffer == NULL) {
		dev_err(&bal.spi->dev, "Unable to alloc memory!\n");
		mutex_unlock(&bal.use_lock);
		return -ENOMEM;
	}

	bal.in_use = true;
	mutex_unlock(&bal.use_lock);

	bal.xfers.tx_nbits = SPI_NBITS_SINGLE;
	bal.xfers.rx_nbits = SPI_NBITS_SINGLE;

	bal.xfers.bits_per_word = 8;
	bal.xfers.delay_usecs = 1;
	bal.xfers.speed_hz = bal.spi->max_speed_hz;

	nonseekable_open(inode, filp);
	try_module_get(THIS_MODULE);

	return 0;
}

static int baldev_release(struct inode *inode, struct file *filp)
{
	int status = 0;
	module_put(THIS_MODULE);
	kfree(bal.buffer);
	bal.in_use = false;
	return status;
}

static long
baldev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int status = -EINVAL;

	switch (cmd) {
		case BAL_IOC_RW_MULTI_REG:
			bal.MultiRegRW = arg;
			status = 0;
			break;
		default:
			break;
	}

	return status;
}

static const struct file_operations baldev_fops = {
	.owner =	THIS_MODULE,
	.write =	baldev_write,
	.read =		baldev_read,
	.open =		baldev_open,
	.unlocked_ioctl = baldev_ioctl,
	.release =	baldev_release,
	.llseek =	no_llseek,
};


#ifdef CONFIG_OF
static const struct of_device_id baldev_dt_ids[] = {
        { .compatible = "nxp,bal" },
        {},
};
MODULE_DEVICE_TABLE(of, baldev_dt_ids);
#endif


static int bal_spi_remove(struct spi_device *spi)
{
	dev_info(&spi->dev, "Removing SPI driver\n");
	device_destroy(baldev_class, bal.devt);
	return 0;
}

static int bal_spi_probe(struct spi_device *spi)
{
	struct device *dev;
	int value = -1;
	uint32_t * pValue = &value;
	int status = -1;

	dev_info(&spi->dev, "Probing BAL driver\n");
	mutex_init(&bal.use_lock);

	if (spi->dev.of_node) {

		status = of_property_read_u32_index(spi->dev.of_node, "NFC-reader-chip", 0, pValue);

		if(status == 0)
			switch(*pValue)
			{
				case 0: /* PN512 */
					dev_info(&spi->dev, "PN512 reader chip\n");
					bal.HalType = (unsigned long)*pValue;
					break;
				case 1: /* RC663 */
					dev_info(&spi->dev, "RC663 reader chip\n");
					bal.HalType = (unsigned long)*pValue;
					break;
				case 2: /* PN5180 */
				default :  /* If no other specified, PN5180 reader chip chosen as default by the BAL module. */
					bal.busy_pin = of_get_named_gpio(spi->dev.of_node, "busy-pin-gpio", 0);

					if (!gpio_is_valid(bal.busy_pin)) {
						dev_err(&spi->dev, "BUSY pin mapped to an invalid GPIO!\n");
						return -ENODEV;
					}
					gpio_direction_input(bal.busy_pin);

					dev_info(&spi->dev, "PN5180 reader chip\n");
					bal.HalType = (unsigned long)*pValue;
					break;
			}
		else { /* NFC-reader-chip property not read from the Device Tree. PN5180 is set byt this function as default HAL type. */
			dev_err(&spi->dev, "Parsing reader chip information from Device Tree FAILED!\n PN5180 is set by default.\n");

			bal.busy_pin = of_get_named_gpio(spi->dev.of_node, "busy-pin-gpio", 0);

			if (!gpio_is_valid(bal.busy_pin)) {
				dev_err(&spi->dev, "BUSY pin mapped to an invalid GPIO!\n");
				return -ENODEV;
			}
			gpio_direction_input(bal.busy_pin);
			bal.HalType = 2;
		}


	}
	else {
		struct bal_spi_platform_data * platform_data = spi->dev.platform_data;

		if (!platform_data) {
			dev_err(&spi->dev, "Platform data for BAL not specified!\n");
			return -ENODEV;
		}
		bal.busy_pin = platform_data->busy_pin;
	}

	bal.spi = spi;
	bal.devt = MKDEV(BALDEV_MAJOR, BALDEV_MINOR);
	bal.MultiRegRW = 0;

	dev = device_create(baldev_class, &spi->dev, bal.devt, &bal, "bal");
	if (IS_ERR(dev)) {
		dev_err(&spi->dev, "Error creating device!\n");
		return PTR_ERR(dev);
	}

	spi_set_drvdata(spi, &bal);
	return 0;
}

static struct spi_driver bal_spi_driver = {
		.driver = {
			.name		= "BAL_SPI_DRIVER",
			.owner		= THIS_MODULE,
			.of_match_table = of_match_ptr(baldev_dt_ids),
		},

		.probe		= bal_spi_probe,
		.remove		= bal_spi_remove,
	};

static int __init baldev_init(void)
{
	int status;
	baldev_class = class_create(THIS_MODULE, "bal");
	if (IS_ERR(baldev_class)) {
		return PTR_ERR(baldev_class);
	}
	status = register_chrdev(BALDEV_MAJOR, "bal", &baldev_fops);
	if (status < 0) {
		printk(KERN_ERR "Error registering character device for bal driver!\n");
		goto err_reg_dev;
	}
	status = spi_register_driver(&bal_spi_driver);
	if (status < 0) {
		printk(KERN_ERR "Error registering BAL driver structure at SPI core!\n");
		goto err_reg_drv;
	}
	return 0;

err_reg_drv:
	unregister_chrdev(BALDEV_MAJOR, BALDEV_NAME);
err_reg_dev:
	class_destroy(baldev_class);
	return status;
}
module_init(baldev_init);

static void __exit baldev_exit(void)
{
	spi_unregister_driver(&bal_spi_driver);
	class_destroy(baldev_class);
	unregister_chrdev(BALDEV_MAJOR, BALDEV_NAME);
}
module_exit(baldev_exit);

MODULE_AUTHOR("Christian Eisendle, <christian@eisendle.net>");
MODULE_DESCRIPTION("NXP RdLib BAL Kernel Module");
MODULE_LICENSE("GPL");
