# NXP NFCRdLib Kernel BAL Module

## General
This is a linux kernel module for the NXP NFCRdLib BAL (Bus Abstraction Layer). 
It allows having the BAL within the Kernel while the NFCRdLib runs in user space. This may be required in order to redurce the latency introduced by unnecessary context switches when BAL also runs in user space.

## Build the Module
### Raspberry Pi
If you look into an out-of-the-box working version for raspberry pi, refer to https://github.com/christianeisendle/linux

### Generic
The module can be built out of the kernel tree ('extra' module). It requires the kernel sources checked out. The following example assumes that the kernel sources `/usr/src/linux-source` and this repo cloned to `/usr/src/nxprdlib-kernel-bal`.

```
cd /usr/src/linux-source
make modules_prepare
make M=/usr/src/nxprdlib-kernel-bal/bal 
```

The compiled module is available in `/usr/src/nxprdlib-kernel-bal/bal.ko`. If it should be installed within the current system/kernel then `sudo make M=/usr/src/nxprdlib-kernel-bal modules_install` can be called. Alternatively, the module can be manually copied to `/lib/modules/YOUR_KERNEL_VERSION/extra` and `depmod -a` needs to be issued.

**Note:** In order to compile the module the correct `Module.symvers` file (corresponding to the kernel in use) must be present in `/usr/src/linux-source`. There are two ways to get to this file:
  1. Build the kernel from scratch. This also generates the `Module.symvers`
  2. Get the `Module.symvers` for the kernel in use. On Debian/Ubuntu `Module.symvers` is part of the `linux-headers` package.

(See also http://askubuntu.com/questions/168279/how-do-i-build-a-single-in-tree-kernel-module). On Raspberry Pi the Module.symvers is available in this repository: https://github.com/raspberrypi/firmware/tree/master/extra (See also: https://www.raspberrypi.org/documentation/linux/kernel/building.md)
Like for `Module.symvers` a proper `.config` file must be available in `/usr/src/linux-source`. Also this is automatically generates when building the kernel from scratch. If supported, the running kernel configuration is also present in `/boot/config*` and just needs to be copied to `/usr/src/linux-source/.config`. 

A complete build example for Debian (assuming that `linux-source` and `linux-headers` package is already installed)

```
cd /usr/src/linux-source*/
cp /boot/config-* .config
cp /usr/src/linux-headers-`uname -r`/Module.symvers .
make modules_prepare
make M=/usr/src/nxprdlib-kernel-bal/bal
make M=/usr/src/nxprdlib-kernel-bal/bal modules_install
```

Raspberry Pi 2 Example:

```
cd /usr/src
git clone --depth=1 https://github.com/raspberrypi/linux
cd linux
KERNEL=kernel7
make bcm2709_defconfig
# Note: Module7.symvers must fit with the kernel version!
wget https://raw.githubusercontent.com/raspberrypi/firmware/master/extra/Module7.symvers
mv Module7.symvers Module.symvers
make modules_prepare
make M=/usr/src/nxprdlib-kernel-bal/bal
make M=/usr/src/nxprdlib-kernel-bal/bal modules_install
```
### SPI Board Info
In case device tree is not used the BAL module needs to be assigned to a spi_device within the spi_board_info struct of the platform specific code. In case of Raspberry Pi 2 the code is located within the kernel tree at `arch/arm/mach-bcm2709/bcm2709.c`

The existing bcm2708_spi_devices array needs to be adapted/modified in order to assign one SPI device with the BAL module instead of the spidev module. Example:

```
#include <linux/spi/bal_spi.h>

static struct bal_spi_platform_data balPlatformData = {
  .busy_pin = 25,
};

#ifdef CONFIG_BCM2708_SPIDEV
static struct spi_board_info bcm2708_spi_devices[] = {
	{
		.modalias = "bal",
		.max_speed_hz = 5000000,
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_0,
		.platform_data = &balPlatformData,
	}
#ifdef CONFIG_SPI_SPIDEV
	, {
		.modalias = "spidev",
		.max_speed_hz = 500000,
		.bus_num = 0,
		.chip_select = 1,
		.mode = SPI_MODE_0,
	}
#endif
};
#endif
```

`bal_spi.h` is present in this repo in `bal/include/linux/spi/bal_spi.h` and needs to be copied to the linux source tree under `include/linux/spi/`.
After applying these changes the Kernel needs to be recompiled.

## Load the Module
The module requires an SPI device to be associated with it. The SPI device needs to be connected with the module using device tree (`compatible` string has to be set to `"nxp,bal"`) or using SPI board info (see above).
An example device tree overlay can be found here: https://github.com/christianeisendle/linux/blob/rpi_4.1.19_bal/arch/arm/boot/dts/overlays/bal-overlay.dts

On Raspberry Pi's device tree it can be added like this:

```
cd /boot
sudo wget https://raw.githubusercontent.com/christianeisendle/linux/rpi_4.1.19_bal/arch/arm/boot/dts/overlays/bal-overlay.dts
# Note: dtc is part of the device-tree-compiler package: sudo apt-get install device-tree-compiler
sudo dtc -I dts -O dtb -o bal-overlay.dtb -@ bal-overlay.dts
sudo mv bal-overlay.dtb overlays/
sudo echo "dtoverlay=bal" >> config.txt
```

To load the module during boot, it should be added to `/etc/modules`:

```
echo "bal" >> /etc/modules
```

### Change ownership of `/dev/bal`
The device node `/dev/bal` is populated with ownership `root.root` and access rights set to `0660`. In order to change ownership or access rights persistently a rule to udevd needs to be added in order to perform appropriate actions when `/dev/bal` is populated or removed again. Following example for Raspberry Pi changes group of `/dev/bal` to `spi` where user `pi` is a member of.

```
echo "SUBSYSTEM==\"bal\", GROUP=\"spi\", MODE=\"0660\"" > /tmp/99-bal.rules
sudo mv /tmp/99-bal.rules /etc/udev/rules.d/
```
