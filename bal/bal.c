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
#define BAL_IOC_BUSY_PIN                0


struct bal_data {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;

	struct mutex		use_lock;
	bool			in_use;
	unsigned int		busy_pin;
	u8	*		buffer;
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
	
	if (count > BAL_MAX_BUF_SIZE)
		return -EMSGSIZE;
	
	status = wait_for_busy_idle();
	if (0 == status) {
		status = spi_read(bal.spi, bal.buffer, count);
	}

	if (copy_to_user(buf, bal.buffer, count)) {
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
	status = copy_from_user(bal.buffer, buf, count);
	if (status) {
		return status;
	}

	status = wait_for_busy_idle();

	if (0 == status) {

		if (status == 0) {
			status = spi_write(bal.spi, bal.buffer, count);
		}
	}

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
	bal.buffer = kmalloc(BAL_MAX_BUF_SIZE, GFP_KERNEL | GFP_DMA);
	if (bal.buffer == NULL) {
		dev_err(&bal.spi->dev, "Unable to alloc memory!\n");
		mutex_unlock(&bal.use_lock);
		return -ENOMEM;
	}
	bal.in_use = true;
	mutex_unlock(&bal.use_lock);

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
	if (cmd == BAL_IOC_BUSY_PIN) {
		if (gpio_is_valid(arg)) {
			status = gpio_request(arg, "BUSY pin");
			if (!status) {
				dev_info(&bal.spi->dev, "Setting BUSY pin to %d\n", (unsigned int)arg);
				gpio_free(bal.busy_pin);
				bal.busy_pin = (unsigned int)arg;
				gpio_direction_input(bal.busy_pin);
			}
		}

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
	dev_info(&spi->dev, "Probing BAL driver\n");
	mutex_init(&bal.use_lock);
	if (spi->dev.of_node) {
		bal.busy_pin = of_get_named_gpio(spi->dev.of_node, "busy-pin-gpio", 0);
	}
	else {
		struct bal_spi_platform_data * platform_data = spi->dev.platform_data;

		if (!platform_data) {
			dev_err(&spi->dev, "Platform data for BAL not specified!\n");
			return -ENODEV;
		}
		bal.busy_pin = platform_data->busy_pin;
	}
	if (!gpio_is_valid(bal.busy_pin)) {
		dev_err(&spi->dev, "BUSY pin mapped to an invalid GPIO!\n");
		return -ENODEV;
	}
	bal.spi = spi;
	bal.devt = MKDEV(BALDEV_MAJOR, BALDEV_MINOR);
	device_create(baldev_class, &spi->dev, bal.devt,
				    &bal, "bal");
	
	gpio_direction_input(bal.busy_pin);
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
	status = register_chrdev(BALDEV_MAJOR, "bal", &baldev_fops);
	printk(KERN_INFO "Registering character device /dev/bal. Status: %d\n", status);
	return spi_register_driver(&bal_spi_driver);
}
module_init(baldev_init);

static void __exit baldev_exit(void)
{
	spi_unregister_driver(&bal_spi_driver);
	class_destroy(baldev_class);
	unregister_chrdev(BALDEV_MAJOR, BALDEV_NAME);
}
module_exit(baldev_exit);

MODULE_AUTHOR("Christian Eisendle, <christian.eisendle@nxp.com>");
MODULE_DESCRIPTION("NXP RdLib BAL Kernel Module");
MODULE_LICENSE("GPL");
