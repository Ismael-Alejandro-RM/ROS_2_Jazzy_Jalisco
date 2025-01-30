# Guía para comenzar con ROS 2 (Jazzy Jalisco) en ubuntu 24.04.1

En esta guía se han recopilado los pasos importantes para la instalación y configuración de ROS 2. A continuación se muestra el contenido de este repositorio:

- [1. Instalación de ROS 2 (Jazzy Jalisco)](#instalación-de-ros-2)
- [2. Configurar el entorno)](#2. Configurar el entorno)
- [Crear un package](#crear-un-package)



## 1. Instalación de ROS 2 (Jazzy Jalisco)


## 2. Configurar el entorno

### 2.1. Hacer un source al entorno de ROS 2 (underlay)

Para tener acceso a los comandos y herramientas de ROS 2, cada vez que se requiera ejecutar un paquete es necesario en una terminal correr lo siguiente: 
```
source /opt/ros/jazzy/setup.bash
```

   - ### Nota: Agregar el comando anterior al startup script
     Lo más comun para usuarios de ROS 2 para evitar lo anterior, cada vez que se quiere ejecutar un paquete, es agregar el mismo comando al startup script (``.bashrc``), para ello en una nueva terminal ejecute el siguiente comando:
     
     ```
     echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
     ```

### 2.2. Verificar las variables del entorno

Para verificar que el entorno para operar ROS 2 está apropiadamente configurado, en una terminal ejecute el siguiente comando:
```
printenv | grep -i ROS
```
Revisé que las variables ``ROS_DISTRO`` and ``ROS_VERSION`` están configuradas, en la terminal se debe apreciar lo siguiente: 

``
  ROS_VERSION=2
`` 

`` 
  ROS_PYTHON_VERSION=3
``

`` 
  ROS_DISTRO={DISTRO}
``

### 2.3. Establecer la variable ``ROS_DOMAIN_ID``

Para evitar incovenientes entre usuarios de ROS 2 conectados a una misma red, es necesario establecer un valor entre ``0`` y ``101``. Para ello, ejecute el siguiente comando estableciendo un valor en ``<your_domain_id>``
```
echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bashrc
```


## 3. Uso de ``colcon`` para construir paquetes

El comando ``colcon`` es de suma importancia en el ambiente de ROS 2, es una iteración de las herramientas de compilación.

### 3.1. Instalar colcon 
```
sudo apt install python3-colcon-common-extensions
```

### 3.2. Configurar ``colcon_cd`` 
El comando ``colcon_cd``  permite dirigirse rápidamente a la raiz de la carpeta donde está el paquete creado desde la terminal, para ello ejecute los siguientes comandos, los cuales son agregados al archivo .bashrc

```
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
```

```
echo "export _colcon_cd_root=/opt/ros/jazzy/" >> ~/.bashrc
```

Para probar el funcionando del comando ``colcon_cd`` ejecute la siguiente instrucción cambiando ``some_ros_package`` por el nombre de su paquete:

```
colcon_cd some_ros_package
```

## 4. Crear un workspace

Un workspace es un directorio que contiene los paquetes de ROS 2, se recomienda crear un nuevo directorio para cada workspace.

### 4.1. Crear un nuevo workspace

Para ello, primero elija el nombre del directorio (por ejemplo ``ros2_ws``), posteriormente ejecute el siguiente comando en una terminal para crear el workspace. 

```
mkdir -p ~/ros2_ws/src
```

El siguiente comando navega hacia la subcarpeta ``src`` dentro de la carpeta ``ros2_ws`` 

```
cd ~/ros2_ws
```

En este punto, la carpeta ``ros2_ws`` contiene solo una subcarpeta vacia ``src``, otra buena práctica es colocar nuestros paquetes dentro de esta subcarpeta.

### 4.2. Clonar un repositorio de muestra 

En la mima terminal, asegurese de estar en el directorio ``ros2_ws/src`` antes de clonar el siguiente paquete, para ello ejecute el siguiente comando:

```
git clone https://github.com/ros/ros_tutorials.git -b jazzy
```
Ahora el paquete ``ros_tutorials`` ha sido clonado en el workspace (``ros2_ws``). 

Hasta el momento se ha rellenado el workspace creado con un paquete de ejemplo, pero aún no es funcional, es necesario resolver las dependencias y luego construir el workspace con el comando ``colcon build``.

### 4.3. Resolver dependencias

Para usar el comando ``rosdep`` es necesario asegurar que tu sistema está actualizado antes de instalar nuevos paquetes. Para ello, es necesario ejecutar los siguientes comandos en una nueva terminal:

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

Al finalizar cierre la termial.

Ahora en una nueva terminal dirigete a la raíz del workspace (``ros2_ws``) con el siguiente comando:

```
cd ~/ros2_ws/
```

Por consiguiente, ejecuta lo siguiente:

```
rosdep install -i --from-path src --rosdistro jazzy -y
```

Si todo está bien con tus dependencias, en la terminal retornará lo siguiente

``
#All required rosdeps installed successfully
`` 

### 4.4 Construir el workspace con ``colcon``

En la misma terminal en la raiz del workspace (``ros2_ws``) construyá sus paquetes con el siguiente comando:

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

Si todo está bien en la terminal verá las nuevas carpetas que ``colcon``ha creado:

``
build  install  log  src
`` 

### 4.5 Hacer un source al workspace (Overlay)
Cada vez que los archivos creados en un paquete dentro de nuestro workspace son modificados o actualizados es necesario hacer un source (ejecutar) a nuestro workspace. 

Para esto, en una nueva terminal digirasé a la raíz del workspace

``
cd ~/ros2_ws
`` 
En la ráiz, haga un source al workspace:

``
source install/local_setup.bash
``

   - ### Nota: Agregar el comando anterior al startup script
     Mayormente cuando se trabaja con un solo workspace es más facil agregar el mismo comando al startup script (``.bashrc``), para ello en una nueva terminal ejecute lo siguiente.
     
     ```
     echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
     ```
     
     Cuando se trabaja con 2 o más workspaces distintos, lo mejor es hacer el source al workspace requerido o modificar el comando anterior dependiente del workspace elegido. La modificación se puede realizar accediendo al startup script, con el ||         siguiente comando:
     
     ```
     gedit .bashrc
     ```
       
Por último, para probar que todo está bien, ejecute el package ``turtlesim``, del repositorio clonado, con la siguiente instrucción:

```
 ros2 run turtlesim turtlesim_node
```   


## 5. Crear un package

### 5.1. Crear un nuevo package

Un package es una forma de agrupar nuestros archivos (scripts, ejecutables, archivos de configuración) en ROS 2, si necesitas compartir tu código con otros, en necesario organizarlo en un package.


Para crear un package dirigasé a la subcarpeta ``src`` del workspace

```
cd ~/ros2_ws/src
```

La sintaxis del comando para crear un un nuevo package en ROS 2 es:

``
ros2 pkg create --build-type ament_cmake --license Apache-2.0 --node-name <node_name> <package_name>
`` 

Donde ``<package_name>``es el nombre del package y ``node_name`` es el nombre del nodo. Para ello, ejecutemos el comando anterior nombrando al package como ``my_package`` y el nodo como: ``my_node``

```
ros2 pkg create --build-type ament_cmake --license Apache-2.0 --node-name my_node my_package
```

### 5.2. Construir un package

En una nueva terminal, dirigasé a la raíz de su workspace

```
cd ~/ros2_ws
``` 

Ahora construya su paquete con el siguiente comando:

```
colcon build
```

   - ### Nota: Construir un solo package 
     La proxima vez, que se tenga muchos packages creados, es posible construir un solo paquete seleccionado, para         ello, cambie ``my_package`` por el nombre de su paquete, en la siguiente instrucción:
     
     ```
     colcon build --packages-select my_package
     ```

### 5.3. Usar el package

Para correr el ejecutable creado duante la creación del package, ejecute el siguiente comando:

```
ros2 run my_package my_node
```

## 6. Herramientas esenciales en ROS 2

### 6.1 Instalación de rqt (Herramienta de interfaz gráfica)
Ejecute en una terminal la siguientes instrucciones para instalar ``rqt`` y sus plugins

```
sudo apt update
```
```
sudo apt install '~nros-jazzy-rqt*'
```








