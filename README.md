
# INSTALACIÓN ROS EN JUBILINUX

## 1. Introducción

En el siguiente documento se aplica el proceso de instalación de **ROS** con **mavros** en el sistema operativo de **Jubilinux** para más información sobre este sistema operativo ([Jubilinux](http://www.jubilinux.org/)).

## 2. Prerequisitos

### 2.1 Dependencias

En primer lugar, es necesario instalar algunas librerías de python para ser usadas por ROS.

> sudo apt-get -y install python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential python-setuptools python-pip python-yaml python-argparse python-distribute python-docutils python-dateutil python-six libpoco-dev

Una vez instalado las librerías, se usa pip de python para la instalación de otras dependencias de las distribuciones del ROS

> sudo pip install -U rosdep rosinstall_generator wstool rosinstall
> 
> sudo pip install --upgrade setuptools


### 2.2 INICIALIZACION DE ROSDEP

Ahora se procede a la iniciación y actualización de ROS con sus diferentes distribuciones

> sudo rosdep init
> 
> rosdep update

## 3. INSTALCION 

Ahora, descargaremos y crearemos ROS Kinetic.

### 3.1 Crear el espacio de trabajo (catkin Workspace)

Creamos el sitio de trabajo donde se instalar el ROS

> mkdir ~/ros_catkin_ws
> 
> cd ~/ros_catkin_ws

Se generan los paquetes de ROS Kinetic para este caso las cuales serán instaladas y así mismo se actualizan utilizando herramientas de wstool

> rosinstall_generator ros_comm mavros tf laser_geometry --rosdistro kinetic --deps --wet-only --tar > kinetic-ros_comm-wet.rosinstall
> 
> sudo wstool init -j1 src kinetic-ros_comm-wet.rosinstall
> 
> sudo wstool update -t src -j1

### 3.2 Resolver dependencias

A continuación, se buscan las librerías faltantes dado los paquetes de ROS que solicitamos anteriormente (ros_comm, mavros, tf, laser_geometry) para esto se usará rosdep

> rosdep install --from-paths src --ignore-src --rosdistro kinetic -y

### 3.3 Liberando espacio

Antes de proceder con la instalación de los paquetes de ROS, dado el poco espacio con el que cuenta la Intel edison, debemos hacer una liberación de espacio (esto puede no ser necesario en todos los casos, dada la configuración por defecto con el que cuente el sistema operativo)

* Mover carpeta share
> sudo mv /home/.rootfs/usr/share /
> 
> sudo ln -s /share /home/.rootfs/usr/share

* Mover carpeta include
> sudo mv /home/.rootfs/usr/include /
> 
> sudo ln -s /include /home/.rootfs/usr/include

* Mover carpeta local
> sudo mv /home/.rootfs/usr/local /
> 
> sudo ln -s /local /home/.rootfs/usr/local

Para liberar aun mayor espacio se borran los archivos temporales de instalaciones anteriores

> sudo apt-get clean

### 3.4 Construir el espacio de trabajo

Estando en la carpeta de trabajo de ROS procedemos a la instalación de esté, en este paso la instalación es demorada dada el número de paquetes que se van a instalar

> sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/kinetic -j1

## 4. Agregar paquetes creados
 
Una vez finalizada la instalación, hay que agregar las direcciones de las carpetas para que el sistema tome los paquetes generadas

* Agregar al profile
> echo "source /opt/ros/kinetic/setup.bash" >> ~/.profile
> 
> source ~/.profile

* Agregar al bash
> echo "source /home/edison/ros_catkin_ws/devel_isolated/setup.bash" >> ~/.bashrc
> 
> echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
> 
> source ~/.bashrc

## 5. Instalación de librerías extras MAVROS

Una vez se tenga instalado el ROS con sus extensiones, se debe instalar una librería adicional la cual se puede encontrar [aquí](https://github.com/mavlink/mavros/blob/master/mavros/scripts/install_geographiclib_datasets.sh). Para realizar la instalación de este script descarguelo y después debe dar permisos de ejecución.
> sudo chmod +x install_geographiclib_datasets.sh

Y a continuacion ejecutarlo

> ./install_geographiclib_datasets.sh

## 6. EJECUTAR MAVROS

Una vez terminada la instalación puede ejecutar todo en conjunto y observar los tópicos de comunicación que genera mavros
>roslaunch mavros px4.launch

Para observar los topicos generados por el mavros
> rostopic list

**Nota:** Es importante tener en cuenta que los puertos de comunicación entre la pixhawk y la Intel Edison, para esta guía de instalación ya debe están conectados y para dar permisos de comunicación entre las placas debe ejecutar
>sudo chmod 666 MFD1

Teniendo en cuenta que el puerto MFD1 es el serial de comunicación de la Intel Edison.

# INSTALACIÓN Y CONFIGURACIÓN RPLIDAR 360°
## 1. Instalación 

Para la uso del rplidar, se debe tener el SDK instalado, en la pagina de ROS podemos encontrar un paquete completo con todo lo que se necesita para el reconocimiento y visualizan de la nube de punto generada por el radar, la cual podemos encontrar [aquí](https://github.com/robopeak/rplidar_ros).

Cuando se descarga el paquete completo, debemos agregarlo al espacio de trabajo creado para proyectos en ROS (catkin_ws) y construir (catkin_make) el proyecto para que quede agregado a las librerías de ROS y poder ejecutarlo, para mayor información visite este [link](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

Para ejecutar el proyecto debemos tener conectado el láser para su correcta ejecución y con el comando
>roslaunch rplidar_ros rplidar.launch

Se empieza a generar la nube de puntos, esta puede ser visualizada gráficamente con GAZEBO.

## 2. Configuración

La primera confuiguracion que se debe realizar es cambiar el nombre del frame que usa el rplidar.launch, el cual por defecto se nombra laser, aqui se debe cambiar por el nombre que usted tenga de base para su robot.

La segunda confuiguracuion que se debe realizar s la apertuira del puerto de comunicacion entre la Intel Edison y el laser, para esto por defecto el puerto que se asigna es el USB0
>sudo chmod 666 USB0

# PLEXIL
## 1. Instalación 
Par la instalación del PLEXIL se debe descargar los fuentes  [aquí](https://sourceforge.net/projects/plexil/). Una vez descargado el software se debe descomprimir
>tar xzf plexil-x.x.x.tar.gz 

Para la instalacion se crear un $PLEXIL_HOME, el cual se crea en el bash
>echo "PLEXIL_HOME=/ruta/de/los/fuentes" >> ~/.bashrc

Ya creado el home, continuamos con
>cd $PLEXIL_HOME
>make
 
# Referencias
Autopilot, P. (2018). MAVROS (MAVLink on ROS) · PX4 Developer Guide. [online] Dev.px4.io. Available at: https://dev.px4.io/en/ros/mavros_installation.html [Accessed 30 Jan. 2018].

GitHub. (2018). UAVenture/ros-setups. [online] Available at: https://github.com/UAVenture/ros-setups/tree/master/intel-edison [Accessed 30 Jan. 2018].

Wiki.ros.org. (2018). ROS/Installation - ROS Wiki. [online] Available at: http://wiki.ros.org/ROS/Installation [Accessed 30 Jan. 2018].

Wiki.ros.org. (2018). wiki/edison - ROS Wiki. [online] Available at: http://wiki.ros.org/wiki/edison [Accessed 30 Jan. 2018].

GitHub. (2018).  _robopeak/rplidar_ros_. \[online\] Available at: https://github.com/robopeak/rplidar_ros \[Accessed 12 Mar. 2018\].

Plexil.sourceforge.net. (2018).  _Installation - plexil_. \[online\] Available at: http://plexil.sourceforge.net/wiki/index.php/Installation \[Accessed 12 Mar. 2018\].

Wiki.ros.org. (2018).  _catkin/Tutorials/create\_a\_workspace - ROS Wiki_. \[online\] Available at: http://wiki.ros.org/catkin/Tutorials/create\_a\_workspace \[Accessed 12 Mar. 2018\].