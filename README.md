# Guía para comenzar con ROS 2 (Jazzy Jalisco) en ubuntu 24.04.1

En esta guía se han recopilado los pasos más importantes para la instalación, configuración, herramientas y conceptos de ROS 2.

A continuación, se muestra el contenido de este repositorio:

0. [Descripción de ROS 2](#0-Descripción-de-ros-2)
   - [Conceptos](#-conceptos)
1. [Instalación de ROS 2 (Jazzy Jalisco)](#1-instalación-de-ros-2-jazzy-jalisco)
   - [Habilitar los repositorios requeridos](#habilitar-los-repositorios-requeridos)
   - [Instalar ROS 2](#instalar-ros-2)
2. [ Configurar el entorno](#2-configurar-el-entorno)
   - [Hacer un source al entorno de ROS 2 (Underlay)](#hacer-un-source-al-entorno-de-ros-2-underlay)
   - [Verificar las variables del entorno](verificar-las-variables-del-entorno)
   - [Establecer la variable ``ROS_DOMAIN_ID``](#establecer-la-variable-ros_domain_id)
3. [Uso de ``colcon`` para construir paquetes](#3-uso-de-colcon-para-construir-paquetes)
   - [Instalar colcon](#instalar-colcon)
   - [Configurar ``colcon_cd``](#configurar-colcon_cd)
4. [Crear un workspace](#4-crear-un-workspace)
   - [Clonar un paquete de muestra](#clonar-un-paquete-de-muestra)
   - [Resolver dependencias](#resolver-dependencias)
   - [Construir el workspace con ``colcon``](#construir-el-workspace-con-colcon)
   - [Hacer un source al workspace (Overlay)](#hacer-un-source-al-workspace-overlay)
5. [Crear un package](#5-crear-un-package)
   - [Construir un package](#construir-un-package)
   - [Usar el package](#usar-el-package)
6. [Herramientas esenciales en ROS 2](#6-herramientas-esenciales-en-ros-2)
7. [Comandos esenciales en ROS 2](#7-comandos-esenciales-en-ros-2)


## 0. Descripción de ROS 2 

**ROS** es un middleware (software) con el que diferentes paquetes se comunican entre sí, además es un lenguaje *agnóstico* (no depende de una lenguaje específico)

**Objetivos de ROS** 

1. Prover un standard para aplicaciones robóticas
2. Usarse en  cualquier robot
3. Separación de código, proveer herramientas de comunicación y librerías de Plug&Play 
   

### Conceptos

**Nodos:** Subprogramas de alguna aplicación, responsables de una sola una cosa, se comunican entre ellos a través de tópicos servicios y parámetros.

**Tópicos:** Es un bus con nombre sobre el cual los nodos intercambian mensajes


## 1. Instalación de ROS 2 (Jazzy Jalisco)

Los siguientes paquetes de instalación estan actualmente disponibles para Ubuntu Noble (24.04.1)

### Habilitar los repositorios requeridos

Es necesario agregar el repositorio apt de ROS 2 a su sistema, para ello, en una terminal ejecute la siguiente instrucción:

```
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Por consiguiente, es necesario agregar la llave ROS 2 GPG, para esto, en la misma terminal ejecute lo siguiente:

```
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Ahora agregue el repositorio a la lista source:

```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Además, es necesario instalar algunas herramientas de desarrollo para construir packages en ROS 2, para ello, ejecute lo siguiente:

```
sudo apt update && sudo apt install ros-dev-tools
```

### Instalar ROS 2

Primero, es necesario actualizar el repositorio apt después de haber realizado la configuración anterior, para esto, en la misma terminal ejecute el siguiente comando:

```
sudo apt update
```

Posteriormente, ejecute el siguiente comando para mantener su sistema completamente actualizado:

```
sudo apt update
```

Por último, ejecute lo siguiente para instalar la versión de escritorio (recomendada) 

```
sudo apt install ros-jazzy-desktop
```



## 2. Configurar el entorno

### Hacer un source al entorno de ROS 2 (Underlay)

Para acceder a los comandos y herramientas de ROS 2 cada vez que necesite correr un paquete, ejecute lo siguiente en una terminal:

```
source /opt/ros/jazzy/setup.bash
```

   - ### Nota: Agregar el comando anterior al startup script
     Lo más común para usuarios de ROS 2 para evitar lo anterior, es agregar el mismo comando al startup script (``.bashrc``), para ello, en una nueva terminal ejecute la siguiente instrucción:
     
     ```
     echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
     ```

### Verificar las variables del entorno

Para verificar que el entorno para operar ROS 2 está apropiadamente configurado, ejecute el siguiente comando en una nueva terminal:
```
printenv | grep -i ROS
```
Revise que las variables ``ROS_DISTRO`` y ``ROS_VERSION`` están configuradas, en la terminal se debe apreciar lo siguiente: 

``
  ROS_VERSION=2
`` 

`` 
  ROS_PYTHON_VERSION=3
``

`` 
  ROS_DISTRO={DISTRO}
``

### Establecer la variable ``ROS_DOMAIN_ID`` 

Para evitar incovenientes entre usuarios de ROS 2 conectados a una misma red, es necesario establecer un valor entre ``0`` y ``101`` para la variable ``ROS_DOMAIN_ID``. Para ello, ejecute el siguiente comando estableciendo un valor en ``<your_domain_id>``
```
echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bashrc
```

## 3. Uso de ``colcon`` para construir paquetes

El comando ``colcon`` es de suma importancia en el ambiente de ROS 2, es una iteración de las herramientas de compilación.

### Instalar colcon 
```
sudo apt install python3-colcon-common-extensions
```

### Configurar ``colcon_cd`` 
El comando ``colcon_cd``  permite dirigirse rápidamente a la raíz de la carpeta donde está el paquete creado desde la terminal, para ello, ejecute los siguientes comandos, los cuales son agregados al archivo ``.bashrc``

```
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
```

```
echo "export _colcon_cd_root=/opt/ros/jazzy/" >> ~/.bashrc
```

Para probar el funcionando del comando ``colcon_cd``, ejecute la siguiente instrucción cambiando ``some_ros_package`` por el nombre de su package

```
colcon_cd some_ros_package
```

## 4. Crear un workspace

> Un workspace es un directorio que contiene los paquetes de ROS 2, se recomienda crear un nuevo directorio para cada workspace.

Para ello, primero elija el nombre del directorio, por ejemplo: ``ros2_ws``, posteriormente, ejecute el siguiente comando en una terminal para crear el workspace. 

```
mkdir -p ~/ros2_ws/src
```

El siguiente comando navega hacia la subcarpeta ``src`` dentro de la carpeta ``ros2_ws`` 

```
cd ~/ros2_ws
```

En este punto, la carpeta ``ros2_ws`` contiene solo una subcarpeta vacia ``src``, otra buena práctica es colocar nuestros paquetes dentro de esta subcarpeta.

### Clonar un paquete de muestra 

En la misma terminal, asegúrese de estar en el directorio ``ros2_ws/src`` antes de clonar el siguiente paquete, para esto, ejecute el siguiente comando:

```
git clone https://github.com/ros/ros_tutorials.git -b jazzy
```
Ahora el paquete ``ros_tutorials`` ha sido clonado en el workspace (``ros2_ws``). 

Hasta el momento se ha rellenado el workspace creado con un paquete de ejemplo, pero aún no es funcional, es necesario resolver las dependencias y luego construir el workspace con el comando ``colcon build``.

### Resolver dependencias

Para usar el comando ``rosdep`` es necesario asegurarse que el sistema está actualizado antes de instalar nuevos paquetes. Para ello, es necesario ejecutar los siguientes comandos en una nueva terminal:

```
sudo apt upgrade
```

```
sudo rosdep init
```

```
rosdep update
```

```
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"
```

Al finalizar, cierre la terminal.

Ahora en una nueva terminal diríjase a la raíz del workspace (``ros2_ws``) con el siguiente comando:

```
cd ~/ros2_ws/
```

Por consiguiente, ejecute lo siguiente:

```
rosdep install -i --from-path src --rosdistro jazzy -y
```

Si todo está bien con tus dependencias, la terminal retornará lo siguiente:

``
#All required rosdeps installed successfully
`` 

### Construir el workspace con ``colcon``

En la misma terminal en la raíz del workspace (``ros2_ws``) construya sus paquetes con el siguiente comando:

```
colcon build
```
La terminal retornará algo similar a esto:

``
Starting >>> turtlesim
Finished <<< turtlesim [5.49s]
`` 

``
Finished <<< turtlesim [5.49s]
`` 

``
Summary: 1 package finished [5.58s]
`` 

Ahora ejecute el siguiente comando en la raíz del workspace (``~/ros2_ws``)

```
ls
```

Si todo está bien, en la terminal verá las siguientes carpetas que ``colcon`` ha creado:

``
build  install  log  src
`` 

### Hacer un source al workspace (Overlay)

Cada vez que los archivos creados en un paquete dentro de nuestro workspace son modificados o actualizados, es necesario hacer un source (ejecución) a nuestro workspace. 

Para esto, en una nueva terminal diríjase a la raíz del workspace

```
cd ~/ros2_ws
``` 

En la raíz, haga un source al workspace

```
source install/local_setup.bash
```

   - ### Nota: Agregar el comando anterior al startup script
     Mayormente cuando se trabaja con un solo workspace, es más fácil agregar el mismo comando al startup script (``.bashrc``), para ello, en una nueva terminal ejecute lo siguiente.
     
     ```
     echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
     ```
     
     Cuando se trabaja con 2 o más workspaces distintos, lo mejor es hacer el source al workspace requerido o modificar el comando anterior dependiendo del workspace elegido. La modificación se puede realizar accediendo al startup script, con el siguiente comando:
     
     ```
     gedit .bashrc
     ```
       
Por último, para probar que todo está bien, ejecute el package ``turtlesim`` del repositorio clonado, con la siguiente instrucción:

```
 ros2 run turtlesim turtlesim_node
```   


## 5 Crear un package

### C++
Un package es una forma de agrupar nuestros archivos (scripts, ejecutables, archivos de configuración) en ROS 2, si necesitas compartir tu código con otros, en necesario organizarlo en un package.

Para crear un package diríjase a la subcarpeta ``src`` de su workspace

```
cd ~/ros2_ws/src
```

La sintaxis del comando para crear un un nuevo package con C++ en ROS 2 es:

``
ros2 pkg create --build-type ament_cmake --license Apache-2.0 --node-name <node_name> <package_name>
`` 

Donde ``<package_name>``es el nombre del package y ``node_name`` es el nombre del nodo. Para ello, ejecute el comando anterior nombrando al package como ``my_package`` y el nodo como ``my_node``

```
ros2 pkg create --build-type ament_cmake --license Apache-2.0 --node-name my_node my_package
```

Template para un nodo en C++

```
#include "rclcpp/rclcpp.hpp"

class MyCustomNode : public rclcpp::Node // MODIFY NAME
{
public:
    MyCustomNode() : Node("node_name") // MODIFY NAME
    {
    }

private:
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyCustomNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

Una vez creado el nodo es necesario agregar las dependencias (librerias) al archivo package.xml, utilizando la siguiente sintaxis:

```
<depend>rclcpp</depend>
```

Donde ``rclcpp`` es un ejemplo de dependencia

Por último tambien agregue las dependencias y el ejecutable al archivo CMakeLists.txt, utilizando la siguiente sintaxis

```
# find dependencies
find_package(rclcpp REQUIRED)

#Añadir el ejecutable (ejemplo) y la ruta para guardarlo en la carpeta install
#-------------------------------------------------------------------
add_executable(cpp_node src/my_first_node.cpp)
ament_target_dependencies(cpp_node rclcpp) #Name of executable dependencies

install(TARGETS
  cpp_node
  DESTINATION lib/${PROJECT_NAME}
)
#-------------------------------------------------------------------
```


### Python 



Template para un nodo en Python
```
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class MyCustomNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("node_name") # MODIFY NAME


def main(args=None):
    rclpy.init(args=args)
    node = MyCustomNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

Crear un nuevo nodo en un paquete de Python
```
touch <name_of_node> 
```
Posteriormente con el siguiente comando se crea el archivo ejecutable 
```
chmod +x <name_of_node> 
```

### Construir un package

En una nueva terminal, diríjase a la raíz de su workspace

```
cd ~/ros2_ws
``` 

Ahora construya su package con el siguiente comando:

```
colcon build
```

   - ### Nota: Construir un solo package 
     La próxima vez que se tenga muchos packages creados, es posible construir un solo paquete seleccionado, para esto, en la siguiente instrucción, cambie ``my_package`` por el nombre de su paquete 
     
     ```
     colcon build --packages-select my_package
     ```

### Usar el package

Para correr el ejecutable creado durante la creación del package, ejecute el siguiente comando:

```
ros2 run my_package my_node
```

## 6. Herramientas esenciales en ROS 2

1. Instalar Visual studio Code
   ```
   sudo snap install code --classic
   ```
   Posteriormente se puede instalar los siguientes extensiones dentro de Visual Studio Code
   
   A] ROS ((Microsoft v0.9.2)
   B] CMake (by twxs)
   C] C/C++ (Microsoft)
   D] Python (Microsoft)

2. Instalar gedit
   ```
   sudo apt install gedit
   ```
   
. Instalar RQT (Herramienta de interfaz Gráfica)
Ejecute en una terminal la siguientes instrucciones para instalar ``rqt`` y sus plugins

```
sudo apt update
```
```
sudo apt install '~nros-jazzy-rqt*'
```

## 7. Comandos esenciales en ROS 2

Abrir y editar el archivo .bashrc
```
gedit .bashrc
```
Instalar app terminator

```
sudo apt install terminator
```

Visualizar los nodos y topicos disponibles
```
ros2 node list
ros2 topic list
```

Renombrar un nodo en el tiempo de ejecución

```
ros2 run <name_of_pkg> <name_of_executable> --ros-args -r __node:=<rename_of_node>
```

Actualización automática del nodo sin necesidad de estar compilando cada vez que se hace una modificación (Solo para paquetes creados con Python)
```
colcon build --packgages-select <name_of_pkg> --symlink-install
```

Abrir rqt graph para visualizar el nombre de los nodos y tópicos activos
```
rqt_graph
```



### 8. Descripción de los comandos en ubuntu

**source:** Se utiliza para ejecutar un script o un archivo de configuración








