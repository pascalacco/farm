#!/bin/bash
DOWNLOAD_DIR=./downloads

echo "Looking if there is a "$DOWNLOAD_DIR" ..."
if [ ! -d $DOWNLOAD_DIR ]
then
    echo " OK I am creating downloads directory"
    mkdir ${DOWNLOAD_DIR}
else
    echo " OK"
fi

source ./scripts/get_downloads.sh

GCC_ARM="GCC_ARM"
GCC_ARM_TAR="gcc-arm-none-eabi-4_9-2015q3-20150921-linux.tar.bz2"
GCC_ARM_HTML="https://launchpad.net/gcc-arm-embedded/4.9/4.9-2015-q3-update/+download/"${GCC_ARM_TAR}
GCC_ARM_DIR="gcc-arm-none-eabi-4_9-2015q3"
GCC_ARM_CHECK="bin/arm-none-eabi-gcc"
get_it $GCC_ARM $GCC_ARM_TAR $GCC_ARM_HTML $GCC_ARM_DIR $GCC_ARM_CHECK

OPENOCD="Open_OCD"
OPENOCD_TAR="gnuarmeclipse-openocd-debian64-0.10.0-201601101000-dev.tgz"
OPENOCD_HTML="https://github.com/gnuarmeclipse/openocd/releases/download/gae-0.10.0-20160110/"${OPENOCD_TAR}
OPENOCD_DIR="openocd"
OPENOCD_CHECK="0.10.0-201601101000-dev/bin/openocd"
get_it $OPENOCD $OPENOCD_TAR $OPENOCD_HTML $OPENOCD_DIR $OPENOCD_CHECK

QEMU="Qemu"
QEMU_TAR="gnuarmeclipse-qemu-debian64-2.4.50-201510290935-dev.tgz"
QEMU_HTML="https://github.com/gnuarmeclipse/qemu/releases/download/gae-2.4.50-20151029/"${QEMU_TAR}
QEMU_DIR="qemu"
QEMU_CHECK="2.4.50-201510290935-dev/bin/qemu-system-gnuarmeclipse"
get_it $QEMU $QEMU_TAR $QEMU_HTML $QEMU_DIR $QEMU_CHECK


ECLIPSE="Eclipse_Luna"
ECLIPSE_TAR=eclipse-cpp-luna-SR2-linux-gtk-x86_64.tar.gz
ECLIPSE_HTML="http://www.eclipse.org/downloads/download.php?file=/technology/epp/downloads/release/luna/SR2/eclipse-cpp-luna-SR2-linux-gtk-x86_64.tar.gz&r=1"
ECLIPSE_DIR="eclipse"
ECLIPSE_CHECK="eclipse"
get_it $ECLIPSE $ECLIPSE_TAR $ECLIPSE_HTML $ECLIPSE_DIR $ECLIPSE_CHECK


echo "_______________________"
echo "install  arm toolchain"
echo "_______________________"

echo "install dependencies with apt-get with sudo rights:"
sudo apt-get -y install lib32z1 lib32ncurses5 lib32bz2-1.0

if [ ! -d /usr/local/$GCC_ARM_DIR ]
then
    echo "symlink the usr/local to this place for eclipse compatibility"
    sudo ln -sf `pwd`/$GCC_ARM_DIR /usr/local/$GCC_ARM_DIR
fi

echo "_______________________"
echo "check if arm toolchain is working"
echo "_______________________"

/usr/local/$GCC_ARM_DIR/bin/arm-none-eabi-gcc --version


echo "_______________________"
echo "install OpenOcd"
echo "_______________________"

if [ ! -d /opt/gnuarmeclipse ]
then
  sudo mkdir -p /opt/gnuarmeclipse
fi

if [ ! -d /opt/gnuarmeclipse/$OPENOCD_DIR ]
then
    echo "symlink the /opt/gnuarmeclipse to this place for eclipse compatibility"  
    sudo ln -fs `pwd`/$OPENOCD_DIR /opt/gnuarmeclipse/$OPENOCD_DIR
fi

echo "Adding udev rules for USB jtag"
sudo cp ./${OPENOCD_DIR}/0.10.0-201601101000-dev/contrib/99-openocd.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules


echo "_______________________"
echo "check if openocd is working"
echo "_______________________"
/opt/gnuarmeclipse/openocd/0.10.0-201601101000-dev/bin/openocd --version

echo "To test it connect a Nucleo Board to the PC, grab the USB with the virtual machine (Devices -> USB) and then enter the folowwing commands on the terminal I just opened : "
gnome-terminal

echo "cd farm"
echo "./openocd/0.10.0-201601101000-dev/bin/openocd -f openocd/0.10.0-201601101000-dev/scripts/board/st_nucleo_f103rb.cfg"

read -p "press a key o continue"

echo "if it works ST led is blinking red/green and program outputs target voltage."

read -p "press a key o continue"

echo "_______________________"
echo "install Qemu Emulator"
echo "_______________________"


if [ ! -d /opt/gnuarmeclipse/$QEMU_DIR ]
then
    echo "symlink /opt/gnuarmeclipse to this place for eclipse compatibility"  
    sudo ln -s `pwd`/$QEMU_DIR /opt/gnuarmeclipse/$QEMU_DIR
fi

echo "_______________________"
echo "check if Qemu is working"
echo "_______________________"

/opt/gnuarmeclipse/qemu/2.4.50-201510290935-dev/bin/qemu-system-gnuarmeclipse --version

echo "_______________________"
echo "install Java jdk       "
echo "_______________________"

sudo apt-get -y install default-jdk

echo "_______________________"
echo "Checking Java           "
echo "_______________________"
java -version



echo "_______________________"
echo "install Eclipse for C++"
echo "_______________________"

if [ ! -d /usr/local/$ECLIPSE_DIR ]
then
    echo "symlink "`pwd`/$ECLIPSE_DIR" to /usr/local/"$ECLIPSE_DIR" for eclipse compatibility"  
    sudo ln -sf `pwd`/$ECLIPSE_DIR /usr/local/$ECLIPSE_DIR
fi


echo "_______________________"
echo "check if Eclipse is working"
echo "_______________________"
./eclipse/eclipse -data ./workspace &

echo "_______________________"
echo "launching install page of gnu arm eclipse "
echo "_______________________"
firefox http://gnuarmeclipse.github.io/plugins/install/ &


echo "___________________________________"
echo "please drag the install icon of web page and drop it on eclipse window"
echo "then follow install process"
echo "___________________________________"

read -p "Press a key to continue"


echo "___________________________________"
echo "Now manualy configure eclipse has shown"
echo " This web page "
echo "___________________________________"

firefox http://gnuarmeclipse.github.io/eclipse/workspace/preferences/

echo "___________________________________"
echo "FINISHED !                         "
echo "___________________________________"

