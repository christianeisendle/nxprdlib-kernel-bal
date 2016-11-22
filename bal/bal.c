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
#include <linux/interrupt.h>

#define BALDEV_NAME "bal"
#define BALDEV_MAJOR			100
#define BALDEV_MINOR			0

#define BAL_IOCTL_MODE			(0x0U)

#define BAL_MAX_BUF_SIZE		1024
#define BAL_BUSY_TIMEOUT_SECS		3

#define BAL_MODE_NORMAL			(0x0U)
#define BAL_MODE_DWL			(0x1U)

#define BAL_DWL_DIRECTION_BYTE_WR       (0x7fU)
#define BAL_DWL_DIRECTION_BYTE_RD       (0xffU)

struct bal_data {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;

	struct mutex		use_lock;
	bool			in_use;
	unsigned int		busy_pin;
	int			busy_irq;
	struct completion	busy_done;
	bool			first_xfer;
	u8			*buffer;
	u8			mode;
};


static struct bal_data bal;
static struct class *baldev_class;

static irqreturn_t
bal_busy_isr(int irq, void *dev_id)
{
	complete(&bal.busy_done);
	return IRQ_HANDLED;
}

static int
bal_spi_sync_xfer(size_t count)
{
	struct spi_transfer t = {
		.rx_buf		= bal.buffer,
		.tx_buf		= bal.buffer,
		.len		= count,
	};

	return spi_sync_transfer(bal.spi, &t, 1);
}

static ssize_t
baldev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	ssize_t			status = 0;

	if (count > BAL_MAX_BUF_SIZE)
		return -EMSGSIZE;

	status = wait_for_completion_timeout(
		&bal.busy_done, (BAL_BUSY_TIMEOUT_SECS * HZ));
	if (status < 0) {
		dev_err(&bal.spi->dev, "Timeout waiting for BUSY\n");
		return status;
	}
	if (bal.mode == BAL_MODE_DWL)
		bal.buffer[0] = BAL_DWL_DIRECTION_BYTE_RD;
	status = bal_spi_sync_xfer(count);
	if (status < 0)
		return status;

	if (copy_to_user(buf, bal.buffer, count))
		return -EFAULT;
	return count;
}


static ssize_t
baldev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	ssize_t status = 0;

	if (count > BAL_MAX_BUF_SIZE)
		return -ENOMEM;
	if (copy_from_user(bal.buffer, buf, count))
		return -EFAULT;
	if ((bal.mode == BAL_MODE_NORMAL) &&
		!bal.first_xfer) {
		status = wait_for_completion_timeout(
			&bal.busy_done, (BAL_BUSY_TIMEOUT_SECS * HZ));
		if (status < 0) {
			dev_err(&bal.spi->dev, "Timeout waiting for BUSY\n");
			return status;
		}
	}
	bal.first_xfer = false;
	status = bal_spi_sync_xfer(count);
	if (status < 0)
		return status;

	return count;
}


static int baldev_open(struct inode *inode, struct file *filp)
{
	int status;

	mutex_lock(&bal.use_lock);
	if (bal.in_use) {
		dev_err(&bal.spi->dev, "Can only be opened once!\n");
		status = -EBUSY;
		goto err;
	}
	bal.buffer = kmalloc(BAL_MAX_BUF_SIZE, GFP_KERNEL | GFP_DMA);
	if (bal.buffer == NULL) {
		status = -ENOMEM;
		goto err;
	}
	bal.mode = BAL_MODE_NORMAL;
	status = request_threaded_irq(bal.busy_irq, bal_busy_isr,
							NULL,
							IRQF_TRIGGER_FALLING,
							"bal", &bal);
	if (status < 0) {
		dev_err(&bal.spi->dev, "Can't request IRQ %d!\n", bal.busy_irq);
		goto err;
	}
	bal.in_use = true;
	reinit_completion(&bal.busy_done);
	bal.first_xfer = true;
	mutex_unlock(&bal.use_lock);

	nonseekable_open(inode, filp);
	try_module_get(THIS_MODULE);

	return 0;
err:
	mutex_unlock(&bal.use_lock);
	return status;
}

static int baldev_release(struct inode *inode, struct file *filp)
{
	int status = 0;

	free_irq(bal.busy_irq, &bal);
	module_put(THIS_MODULE);
	kfree(bal.buffer);
	bal.in_use = false;
	return status;
}

static long
baldev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int status = 0;
	int flag;

	if (cmd == BAL_IOCTL_MODE) {
		bal.mode = (u8)arg;
		if (bal.mode == BAL_MODE_NORMAL)
			flag = IRQF_TRIGGER_FALLING;
		else
			flag = IRQF_TRIGGER_RISING;
		free_irq(bal.busy_irq, &bal);
		status = request_threaded_irq(bal.busy_irq, bal_busy_isr,
					NULL, flag, "bal", &bal);
		if (status < 0)
			dev_err(&bal.spi->dev,
				"Can't request IRQ %d!\n", bal.busy_irq);
	} else
		status = -EINVAL;

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

	dev_info(&spi->dev, "Probing BAL driver\n");
	mutex_init(&bal.use_lock);
	if (spi->dev.of_node) {
		bal.busy_pin =
			of_get_named_gpio(spi->dev.of_node, "busy-pin-gpio", 0);
	} else {
		struct bal_spi_platform_data *platform_data =
				spi->dev.platform_data;

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
	bal.busy_irq = gpio_to_irq(bal.busy_pin);
	if (bal.busy_irq < 0) {
		dev_err(&spi->dev, "BUSY pin GPIO can't be used as IRQ!\n");
		return bal.busy_irq;
	}
	bal.spi = spi;
	bal.devt = MKDEV(BALDEV_MAJOR, BALDEV_MINOR);
	init_completion(&bal.busy_done);

	dev = device_create(baldev_class, &spi->dev, bal.devt, &bal, "bal");
	if (IS_ERR(dev)) {
		dev_err(&spi->dev, "Error creating device!\n");
		return PTR_ERR(dev);
	}

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
	if (IS_ERR(baldev_class))
		return PTR_ERR(baldev_class);
	status = register_chrdev(BALDEV_MAJOR, "bal", &baldev_fops);
	if (status < 0)
		goto err_reg_dev;
	status = spi_register_driver(&bal_spi_driver);
	if (status < 0)
		goto err_reg_drv;
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
