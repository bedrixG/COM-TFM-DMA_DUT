# Set up env

```bash
source ~/../tools/Xilinx/Petalinux/v2024.1/settings.sh
```

# Create project
```
petalinux-create -t project -n dma-throughput-measure --template zynqMP
```

edit *project-spec/meta-user/recipes-bsp/device-tree/files/system-user.dtsi*
```plain text
/include/ "system-conf.dtsi"
/ {
};


/* SD */
&sdhci1{
disable-wp;
no-1-8-v;
};

/* USB */

&dwc3_0{
status="okay";
dr_mode="host";
};
```

# Build
```bash
petalinux-build
# clean build: petalinux-build -x mrproper
# permissions error: sudo chown -R $USER:"$(id -gn)" images/linux
```

# Generate BOOT file
```bash
petalinux-package --boot --fsbl zynqmp_fsbl.elf --fpga system.bit --u-boot u-boot.elf
sudo tar zxvf rootfs.tar.gz
```

# SD setup
Get bin files from server
```bash
scp 'rbedrinana@10.5.5.96:~/Projects/dma-throughput-measure/images/linux/{BOOT.BIN,boot.scr,image.ub,rootfs.tar.gz}' .
```

Copy bin files to SD
```bash
#sudo rm -r /media/rbedrinana/root/*
#sudo rm -r /media/rbedrinana/boot/*
sudo cp BOOT.BIN image.ub boot.scr /media/rbedrinana/boot/
sudo tar zxvf rootfs.tar.gz -C /media/rbedrinana/root/
sync
```
