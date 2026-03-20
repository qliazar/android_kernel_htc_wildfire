export ARCH=arm && export SUBARCH=arm
export CROSS_COMPILE=$(pwd)/gcc/bin/arm-eabi-
make -j4 O=out buzz_defconfig
make -j4 O=out
mv $(pwd)/out/arch/arm/boot/zImage dist/zImage
mv $(pwd)/out/drivers/net/wireless/bcm4329/bcm4329.ko dist/bcm4329.ko
