# USB-ISS_minimu9_and_buttons-ROS-Package

## Descripción general
Tutorial de conexión y utilización dentro de ROS en Ubuntu del módulo USB-ISS conectado al Pololu MinIMU-9 v2 (Acelerómetro, Magnetómetro y Giroscopio) y 2 push buttons con el propósito de la utilizar los datos de salida en programas más avanzados.

## Pre-requisitos
El usuario deberá tener correctamente instalado ROS, GIT, así como la carpeta de `catkin_ws` inicializada y los siguientes componentes:

* Pololu MinIMU-9 v2 o versiones más actualizadas, por ejemplo: [MinIMU-9 v5](https://www.pololu.com/product/2738) (Al ser una versión diferente a la utilizada, se requiere leer el apartado de [Compatibilidades](#compatibilidades))
* Módulo de comunicación USB-ISS (Puede ser comprado de [México](https://store.robodacta.mx/interfaces-y-programadores/interfaces/interfaz-usb-iss/) o [Estados Unidos](https://www.robotshop.com/en/devantect-usb-to-i2c-spi-serial-interface.html))
* Cable [USB A macho a B macho](https://www.trossenrobotics.com/store/p/6611-USB-A-Male-to-B-Male-6ft-Cable.aspx)
* Dos [Push-buttons](https://www.sparkfun.com/products/8605)
* Dos [resistencias de 1K Ohms](https://www.sparkfun.com/products/14492)

## Uso e información general
Este proyecto es una extensión a ROS de un repositorio anterior [USB-ISS_minimu9_and_buttons](https://github.com/JuanCarlos-MA/USB-ISS_minimu9_and_buttons/edit/master/README.md) en dónde se utilzaron todos los componentes que se encuentran en el apartado de [Pre-requisitos](#pre-requisitos), es por ello que si requiere saber más imformación sobre cualqueira de ellos, su diagrama de conexión física y/o compatibilidades de los productos es altamente recomendable leerlo a detalle.

## Proceso de utilización

Al conectar el USB a la computadora con Ubuntu, es necesario verificar que el dispositivo aparece como algún puerto ACM, pj. `/dev/ttyACM0`, así como es de suma importancia habilitar los permisos de lectura de dicho dispositivo para ello se necesita escribir en una linea de comandos:
```
sudo chmod a+rw /dev/ttyACM0 
```
Para instalar el paquete de este repositorio llamado: `minimu-9`, se deben ejecutar en una terminal las siguientes instrucciones:

```
cd ~/catkin_ws/src
git clone https://github.com/JuanCarlos-MA/USB-ISS_minimu9_and_buttons-ROS-Package.git
cd ~/catkin_ws
catkin build
source devel/setup.bash
```
Ya terminado el proceso, deberá de abrir dos Terminales nuevas y ejecutar los siguientes comandos:

* Terminal 1
```
roscore
```
* Terminal 2
```
rosrun minimu-9 minimu9
```

Al haber corrido el último comando se deberá mostrar una salida similar a la siguiente:
```
USB-ISS Module ID: 7 
USB-ISS Software v: 6 

 
 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   DATOS DE LOS SENSORES   >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 

    Acelerómetro			    Magnetómetro		              Giroscopio		  Botones 

  X        Y        Z			 X        Y        Z			 X        Y        Z		  B1   B2 
-0.016  -0.031  -1.094 			209	-1121 	-864			157	-70       41		 OFF   OFF
 
0.000  -0.016  -1.078 			209	-1121 	-865			151	-80       80		 OFF   OFF
 
0.000  -0.031  -1.078 			210	-1121 	-863			147	-92       51		 OFF   OFF
 
-0.016  -0.016  -1.094 			209	-1121 	-865			126	-86       58		 ON   ON
 
-0.016  -0.016  -1.078 			210	-1121 	-864			143	-111       82		 OFF   ON
```

Esto demuestra que funciona correctamente, para observar una salida del programa más completo, se adjuntó un archivo .txt, llamado [ejemplo-salida.txt](ejemplo-salida.txt).

## Autor

**Juan Carlos Martínez Aguilar** *Estancia de Investigación en Robots Humanoides*

## Referencias
1. ROS.org, "ROS Tutorials," 23 Decemeber 2019. [Online]. Available: http://wiki.ros.org/ROS/Tutorials. [Accessed 16 January 2020].
