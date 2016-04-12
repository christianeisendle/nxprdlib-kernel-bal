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

//#include <phbalReg.h>

#define BALDEV_NAME "bal"
#define BALDEV_MAJOR			100
#define BALDEV_MINOR			0

#define BAL_MAX_BUF_SIZE		1024
#define BAL_BUSY_TIMEOUT_SECS		1
#define BAL_IOC_BUSY_PIN                0
#define BAL_IOC_HAL_HW_TYPE		3
#define BAL_IOC_RW_MULTI_REG		4

struct bal_data {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;

	struct mutex		use_lock;
	bool			in_use;
	unsigned int		busy_pin;
	u8	*		buffer;
	u8	*		rxBuffer;
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

 static int __spi_validate_bits_per_word(struct spi_master *master, u8 bits_per_word)
 {
         if (master->bits_per_word_mask) {
                 /* Only 32 bits fit in the mask */
                 if (bits_per_word > 32)
                         return printk(KERN_ERR "!!!VALIDATION_ERROR!!! - %d\n", __LINE__);//-EINVAL;
                 if (!(master->bits_per_word_mask &
                                 SPI_BPW_MASK(bits_per_word)))
                         return printk(KERN_ERR "!!!VALIDATION_ERROR!!! - %d\n", __LINE__);//-EINVAL;
         }
 
        return 0;
 }

static int __spi_validate(struct spi_device *spi, struct spi_transfer *transfer)
{
         struct spi_master *master = spi->master;
         struct spi_transfer *xfer = transfer;
         int w_size;

		 printk("%s\n", __FUNCTION__);
 
         //if (list_empty(&message->transfers))
         //        return printk("!!!VALIDATION_ERROR!!! - %d\n", __LINE__);//-EINVAL;
 
         /* Half-duplex links include original MicroWire, and ones with
          * only one data pin like SPI_3WIRE (switches direction) or where
          * either MOSI or MISO is missing.  They can also be caused by
          * software limitations.
          */
         if ((master->flags & SPI_MASTER_HALF_DUPLEX)
                         || (spi->mode & SPI_3WIRE)) {
                 unsigned flags = master->flags;
 
                 /*list_for_each_entry(xfer, &message->transfers, transfer_list)*/ {
                         if (xfer->rx_buf && xfer->tx_buf)
						 {
							 printk(KERN_ERR "!!!VALIDATION_ERROR!!! - %d\n", __LINE__);//
							 return -EINVAL;
						 }                                 
                         if ((flags & SPI_MASTER_NO_TX) && xfer->tx_buf)
                         {
							 printk(KERN_ERR "!!!VALIDATION_ERROR!!! - %d\n", __LINE__);//
							 return -EINVAL;
						 }
                         if ((flags & SPI_MASTER_NO_RX) && xfer->rx_buf)
                         {
							 printk(KERN_ERR "!!!VALIDATION_ERROR!!! - %d\n", __LINE__);//
							 return -EINVAL;
						 }
                 }
         }
 
         /**
          * Set transfer bits_per_word and max speed as spi device default if
          * it is not set for this transfer.
          * Set transfer tx_nbits and rx_nbits as single transfer default
          * (SPI_NBITS_SINGLE) if it is not set for this transfer.
          */
         //message->frame_length = 0;
         {
                 //message->frame_length += xfer->len;
                 if (!xfer->bits_per_word)
                         xfer->bits_per_word = spi->bits_per_word;
 
                 if (!xfer->speed_hz)
                         xfer->speed_hz = spi->max_speed_hz;
                 if (!xfer->speed_hz)
                         xfer->speed_hz = master->max_speed_hz;
 
                 if (master->max_speed_hz &&
                     xfer->speed_hz > master->max_speed_hz)
                         xfer->speed_hz = master->max_speed_hz;
 
                 if (__spi_validate_bits_per_word(master, xfer->bits_per_word))
                         return -EINVAL;
 
                 /*
                  * SPI transfer length should be multiple of SPI word size
                  * where SPI word size should be power-of-two multiple
                  */
                 if (xfer->bits_per_word <= 8)
                         w_size = 1;
                 else if (xfer->bits_per_word <= 16)
                         w_size = 2;
                 else
                         w_size = 4;
 
                 /* No partial transfers accepted */
                 if (xfer->len % w_size)
                         return -EINVAL;
 
                 if (xfer->speed_hz && master->min_speed_hz &&
                     xfer->speed_hz < master->min_speed_hz)
				 {
                         printk(KERN_ERR "!!!VALIDATION_ERROR!!! - %d\n", __LINE__);
						 return -EINVAL;
				 }
 
                 if (xfer->tx_buf && !xfer->tx_nbits)
                         xfer->tx_nbits = SPI_NBITS_SINGLE;
                 if (xfer->rx_buf && !xfer->rx_nbits)
                         xfer->rx_nbits = SPI_NBITS_SINGLE;
                 /* check transfer tx/rx_nbits:
                  * 1. check the value matches one of single, dual and quad
                  * 2. check tx/rx_nbits match the mode in spi_device
                  */
                 if (xfer->tx_buf) {
                         if (xfer->tx_nbits != SPI_NBITS_SINGLE &&
                                 xfer->tx_nbits != SPI_NBITS_DUAL &&
                                 xfer->tx_nbits != SPI_NBITS_QUAD)
                                  {
									printk(KERN_ERR "!!!VALIDATION_ERROR!!! - %d\n", __LINE__);
									return -EINVAL;
								}
                         if ((xfer->tx_nbits == SPI_NBITS_DUAL) &&
                                 !(spi->mode & (SPI_TX_DUAL | SPI_TX_QUAD)))
                                  {
									printk(KERN_ERR "!!!VALIDATION_ERROR!!! - %d\n", __LINE__);
									 return -EINVAL;
								}
                         if ((xfer->tx_nbits == SPI_NBITS_QUAD) &&
                                 !(spi->mode & SPI_TX_QUAD))
                                  {
									printk(KERN_ERR "!!!VALIDATION_ERROR!!! - %d\n", __LINE__);
									 return -EINVAL;
									}
                 }
                 /* check transfer rx_nbits */
                 if (xfer->rx_buf) {
                         if (xfer->rx_nbits != SPI_NBITS_SINGLE &&
                                 xfer->rx_nbits != SPI_NBITS_DUAL &&
                                 xfer->rx_nbits != SPI_NBITS_QUAD)
                                  {
									printk(KERN_ERR "!!!VALIDATION_ERROR!!! - %d\n", __LINE__);
									return -EINVAL;
									}
                         if ((xfer->rx_nbits == SPI_NBITS_DUAL) &&
                                 !(spi->mode & (SPI_RX_DUAL | SPI_RX_QUAD)))
                                  {
									printk(KERN_ERR "!!!VALIDATION_ERROR!!! - %d\n", __LINE__);
									return -EINVAL;
									}
                         if ((xfer->rx_nbits == SPI_NBITS_QUAD) &&
                                 !(spi->mode & SPI_RX_QUAD))
                                  {
										printk(KERN_ERR "!!!VALIDATION_ERROR!!! - %d\n", __LINE__);
										return -EINVAL;
									}
                 }
         }
 
         //message->status = -EINPROGRESS;
 
         return 0;
}

static ssize_t
baldev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	ssize_t			status = 0;

	int i;
	ssize_t result;
	struct spi_transfer xfers;
		
	if (count > BAL_MAX_BUF_SIZE)
		return -EMSGSIZE;
	status = copy_from_user(bal.buffer, buf, count);
	
  //	struct spi_transfer {
  xfers.tx_buf = bal.buffer;
  xfers.rx_buf = bal.rxBuffer;
  xfers.len = count;
  //xfers.dma_addr_t tx_dma;
  xfers.rx_dma = bal.rxBuffer;
  //struct sg_table tx_sg;
  //struct sg_table rx_sg;

  xfers.tx_nbits = SPI_NBITS_SINGLE;
  xfers.rx_nbits = SPI_NBITS_SINGLE;

  xfers.bits_per_word = 8;
  xfers.delay_usecs = 0;
  //u32 speed_hz;
  //struct list_head transfer_list;
//}; 

  uint16_t flags = bal.spi->master->flags;
  printk(KERN_ERR "master flags = %04X\n", bal.spi->master->flags);
  printk(KERN_ERR "mode flags = %04X\n", bal.spi->mode);


  status = __spi_validate(bal.spi, &xfers);
  printk(KERN_ERR "status = %d\n", status);

	if(bal.HalType == 0x02)
	{
		status = wait_for_busy_idle();

		if (0 == status) {
			//printk(KERN_ERR "before READ filp=%d   buf=%02X,%02X,%02X,%02X  count=%d\n", filp, buf[0], buf[1], buf[2], buf[3], count);
			status = spi_read(bal.spi, bal.buffer, count);
			printk(KERN_ERR "READ");
		}
	}
	else
	{
		for(i = 0; i < count; i++) {
			printk(KERN_ERR "before READ buffer[%d] = %d", i, bal.buffer[i]);
			printk(KERN_ERR "before READ rxBuffer[%d] = %d", i, bal.rxBuffer[i]);
		}
		
		//for(i = 0; i < count; i++)
		//{
			//status = spi_write_then_read(bal.spi, bal.buffer, count,  bal.buffer, count);
		//	result = spi_w8r8(bal.spi, bal.buffer[i]);						
		//	bal.buffer[i] = (uint8_t)result;
		//}
		//bal.buffer[0] = 0x00; // reader expets the first byte in the reply buffer 00h

		//status = spi_sync_transfer (bal.spi, &xfers, 1);

		if(status != 0){
			printk(KERN_ERR "status = %d = %04X\n", status, status);
			return status;
		}

		for(i = 0; i < count; i++) {
			printk(KERN_ERR "after READ buffer[%d] = %d", i, bal.buffer[i]);
			printk(KERN_ERR "after READ rxBuffer[%d] = %d", i, bal.rxBuffer[i]);
		}
	
		result = 0;
	//if (copy_to_user(buf, &result, 1))
	//	return -EFAULT;
	//if (copy_to_user(buf+1, bal.buffer, count-1))
	//	return -EFAULT;
	if (copy_to_user(buf, bal.rxBuffer, count))
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

	printk(KERN_ERR "WR filp=%d   buf=%02X,%02X  count=%d\n", filp, buf[0], buf[1], count);

	if(bal.HalType == 2)
	{
		status = wait_for_busy_idle();
		if (status == 0)
				status = spi_write(bal.spi, bal.buffer, count);
	}
	else
		status = spi_write(bal.spi, bal.buffer, count);
		//status = spi_write_then_read(bal.spi, bal.buffer, count,  bal.buffer, count);

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
	bal.rxBuffer = (uint8_t *)kmalloc(BAL_MAX_BUF_SIZE, GFP_KERNEL | GFP_DMA);
	if (bal.rxBuffer == NULL) {
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
	//printk(KERN_ERR "%s : %d fp=%ld cmd=%ld arg=%lx\n", __FUNCTION__, __LINE__, (unsigned long int)filp, cmd, arg);
	printk(KERN_ERR "my ioctl\n");

	int status = 0; //-EINVAL;

	//if (cmd == BAL_IOC_BUSY_PIN) {
	//	if (gpio_is_valid(arg)) {
	//		status = gpio_request(arg, "BUSY pin");
	//		if (!status) {
	//			dev_info(&bal.spi->dev, "Setting BUSY pin to %d\n", (unsigned int)arg);
	//			gpio_free(bal.busy_pin);
	//				bal.busy_pin = (unsigned int)arg;
	//			gpio_direction_input(bal.busy_pin);
	//		}
	//	}
	//
	//}
	
	dev_info(&bal.spi->dev, "%s : %d fp=%ld cmd=%ld arg=%lx\n", __FUNCTION__, __LINE__, (unsigned long int)filp, cmd, arg);
	
	switch (cmd) {
		case BAL_IOC_BUSY_PIN:
			printk(KERN_ERR "busy pin\n");
			if (gpio_is_valid(arg)) {
				dev_info(&bal.spi->dev, "Requesting BUSY pin\n");
				printk(KERN_ERR "Requesting BUSY pin\n");
				status = gpio_request(arg, "BUSY pin");
				if (!status) {
					dev_info(&bal.spi->dev, "Setting BUSY pin to %d\n", (unsigned int)arg);
					gpio_free(bal.busy_pin);
					bal.busy_pin = (unsigned int)arg;
					gpio_direction_input(bal.busy_pin);
				}
			}
			//dev_info(&bal.spi->dev, "BAL_IOC_BUSY_PIN\n");
			printk(KERN_ERR "busy pin\n");
			break;
		case 1:
		case 2:
			printk(KERN_ERR "uneffective option\n");
			break;
		case BAL_IOC_HAL_HW_TYPE:
			bal.HalType = arg;
			status = 0;
			dev_info(&bal.spi->dev, "BAL_IOC_HAL_HW_TYPE - %d - %d\n", cmd, (unsigned int)arg);
			printk(KERN_INFO "HAL_HW_type %s - %d %u\n", __FUNCTION__, __LINE__, arg);
			break;
		case BAL_IOC_RW_MULTI_REG:
			bal.MultiRegRW = arg;
			status = 0;
			dev_info(&bal.spi->dev, "BAL_IOC_RW_MULTI_REG - %d - %d\n", cmd, (unsigned int)arg);
			printk(KERN_INFO "multiREG %s - %d  %u\n", __FUNCTION__, __LINE__, arg);
			break;
		default:
			printk(KERN_ERR "NO option - default\n");
			break;
//		case PHBAL_REG_CONFIG_HAL_HW_TYPE:
//			switch (arg) {
//				case PHBAL_REG_HAL_HW_RC523:
//					bal.wHalType = arg;
//					break;
//				case PHBAL_REG_HAL_HW_RC663:
//					bal.wHalType = arg;
//					break;
//				case PHBAL_REG_HAL_HW_PN5180:
//					bal.wHalType = arg;
//					break;
//				default:
//					return PH_ADD_COMPCODE(PH_ERR_INVALID_PARAMETER, PH_COMP_BAL);
//			}
//			break;
//
//		case PHBAL_CONFIG_RW_MULTI_REG:
//			bal.bMultiRegRW = arg;
//			break;
//
//		default:
//			return PH_ADD_COMPCODE(PH_ERR_UNSUPPORTED_PARAMETER, PH_COMP_BAL);
//			break;
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
	dev_info(&spi->dev, "Probing BAL kernel module driver\n");
        printk(KERN_ERR "BAL module MY printk %s\n", __TIME__);
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
