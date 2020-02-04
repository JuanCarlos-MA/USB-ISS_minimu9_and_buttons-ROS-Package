# USB-ISS_minimu9_and_buttons-ROS-Package

## Descripción general
Tutorial de conexión y utilización dentro de ROS en Ubuntu del módulo USB-ISS conectado al Pololu MinIMU-9 v2 (Acelerómetro, Magnetómetro y Giroscopio) y 2 push buttons con el propósito de la utilizar los datos de salida en programas más avanzados.

## Pre-requisitos
El usuario deberá tener correctamente instalado ROS, GIT, así como los siguientes componentes:

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

cd ~
git clone
cd ~/minimu-9
catkin build
source devel/setup.bash


