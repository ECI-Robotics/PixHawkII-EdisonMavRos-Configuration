#!/bin/bash

# Este script esta adaptado de la version Jubilinux Stretch para la raspi. Con el fin de instalar ROS en la intel edison.
# Creado por Cesar Augusto Vega Fernández
# http://wiki.ros.org/kinetic/Installation/Source

# Configuración inicial
# sudo nano /etc/network/interfaces 
# Descomentar wlan0 y cambiar por la configuración de internet
# wpa-ssid 599506255173
# wpa-psk 450835795026
# Guardar y salir 
# sudo ifup wlan0 
# creaar un archivo, para copiar el contenido de este documento
# dar permisos de ejecucion chmod +x "nombre de archivo"
# ejecutar ./"nombre de archivo"

if [ `whoami` == "root" ]; then 
  echo "Do not run this as root!"
  exit 1
fi

echo "*** Update the OS ***"
sudo apt-get -y update

echo "*** Install required OS packages ***"
sudo apt-get -y install python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential python-setuptools python-pip python-yaml python-argparse python-distribute python-docutils python-dateutil python-six libpoco-dev

sudo apt --fix-broken install -y

echo "*** Install required ROS packages ***"
sudo pip install -U rosdep rosinstall_generator wstool rosinstall

sudo pip install --upgrade setuptools

echo "*** ROSDEP ***"
sudo rosdep init

rosdep update

mkdir ~/ros_catkin_ws

cd ~/ros_catkin_ws

echo "*** rosinstall ***"
rosinstall_generator ros_comm mavros tf laser_geometry --rosdistro kinetic --deps --wet-only --tar > kinetic-ros_comm-wet.rosinstall

echo "*** wstool ***"
sudo wstool init -j1 src kinetic-ros_comm-wet.rosinstall
while [ $? != 0 ]; do
  echo "*** wstool - download failures, retrying ***"
  sudo wstool update -t src -j1
done

echo "*** rosdep install - Errors at the end are normal ***"
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y

echo "*** move share ***"
sudo mv /home/.rootfs/usr/share /
sudo ln -s /share /home/.rootfs/usr/share

sudo mv /home/.rootfs/usr/include /
sudo ln -s /include /home/.rootfs/usr/include

sudo mv /home/.rootfs/usr/local /
sudo ln -s /local /home/.rootfs/usr/local

sudo apt-get -y update

sudo apt-get clean

echo “******************************************************************”
echo “About to start some heavy building. Go have a looong coffee break.”
echo “******************************************************************”

echo "*** Building ROS ***"
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/kinetic -j1

echo "*** Updating .profile and .bashrc ***"
echo "source /opt/ros/kinetic/setup.bash" >> ~/.profile
source ~/.profile

echo "source /home/edison/ros_catkin_ws/devel_isolated/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

cd ~/ros_catkin_ws

echo ""
echo "*** FINISHED! ***"
