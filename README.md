# NXP NFCRdLib Kernel BAL Module

## General
This is a linux kernel module for the NXP NFCRdLib BAL (Bus Abstraction Layer). 
It allows having the BAL within the Kernel while the NFCRdLib runs in user space. This may be required in order to redurce the latency introduced by unnecessary context switches when BAL also runs in user space

## Build the Module
The module can be built out of the kernel tree ('extra' module). It requires the kernel sources checked out. The following example assumes that the kernel sources `/usr/src/linux-source` and this repo cloned to `/usr/src/nxprdlib-kernel-bal`.

```
cd /usr/src/linux-source
make modules_prepare
make M=/usr/src/nxprdlib-kernel-bal
```

The compiled module is available in `/usr/src/nxprdlib-kernel-bal/bal.ko`. If it should be installed within the current system/kernel then `sudo make M=/usr/src/nxprdlib-kernel-bal modules_install` can be called. Alternatively, the module can be manually copied to `/lib/modules/YOUR_KERNEL_VERSION/extra` and `depmod -a` needs to be issued.

**Note:** In order to compile the module the correct `Module.symvers` file (corresponding to the kernel in use) must be present in `/usr/src/linux-source`. There are two ways to get to this file:
1. Build the kernel from scratch. This also generates the `Module.symvers`
2. Get the `Module.symvers` for the kernel in use. On Debian/Ubuntu `Module.symvers` is part of the `linux-headers` package. (See also http://askubuntu.com/questions/168279/how-do-i-build-a-single-in-tree-kernel-module). On Raspberry Pi the Module.symvers is available in this repository: https://github.com/raspberrypi/firmware/tree/master/extra (See also: https://www.raspberrypi.org/documentation/linux/kernel/building.md)
Like for `Module.symvers` a proper `.config` file must be available in `/usr/src/linux-source`. Also this is automatically generates when building the kernel from scratch. If supported, the running kernel configuration is also present in `/boot/config*` and just needs to be copied to `/usr/src/linux-source/.config`. 

A complete build example for Debian (assuming that `linux-source` and `linux-headers` package is already installed)

```
cd /usr/src/linux-source*/
cp /boot/config-* .config
cp /usr/src/linux-headers-`uname -r`/Module.symvers .
make modules_prepare
make M=/usr/src/nxprdlib-kernel-bal
make M=/usr/src/nxprdlib-kernel-bal modules_install
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
make M=/usr/src/nxprdlib-kernel-bal
make M=/usr/src/nxprdlib-kernel-bal modules_install
```

## Load the Module
The module requires an SPI device to be associated with it. The SPI device needs to be connected with the module using device tree (compatible tag has to be set to "nxp,bal") or using SPI board info (not supported yet).
An example device tree overlay can be found here: https://github.com/christianeisendle/linux/blob/rpi_4.1.19_bal/arch/arm/boot/dts/overlays/bal-overlay.dts

On Raspberry Pi's device tree it can be added like this:

```
cd /boot
sudo wget https://raw.githubusercontent.com/christianeisendle/linux/rpi_4.1.19_bal/arch/arm/boot/dts/overlays/bal-overlay.dts
# Note: dtc is part of the device-tree-compiler package: sudo apt-get install device-tree-compiler
sudo dtc -I dts -O dtb -o bal-overlay.dtb bal-overlay.dts
sudo mv bal-overlay.dtb overlays/
sudo echo "dtoverlay=bal" >> config.txt
```

To load the module during boot, it should be added to `/etc/modules`:

```
echo "bal" >> /etc/modules
```

If loaded successfully, `/dev/bal` should have been populated by udevd. 
