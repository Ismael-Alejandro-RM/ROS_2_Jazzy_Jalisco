# Guía rápida para utilizar ROS 2 (Jazzy Jalisco) en ubuntu 24.04.1

Antes de continuar con la configuración es necesario realizar los pasos para instalar ROS 2
## 1. Configurando el entorno

### 1.1. Hacer un source a los archivos de instalación

Para tener acceso a los comandos de ROS, cada vez que se abre una terminal es necesario ejecutar lo siguiente: 
```
source /opt/ros/jazzy/setup.bash
```

   - ### Nota: Agregar el comando anterior al startup script
     Para evitar lo anterior cada vez que se abre una terminal, es necesario agregar el mismo comando al startup script (``.bashrc``), para ello en una nueva terminal ejecute el siguiente comando:
     
     ```
     echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
     ```

### 1.2. Verificar las variables del entorno

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

### 1.4. Establecer la variable ``ROS_DOMAIN_ID``

Para evitar incovenientes entre usuarios de ROS 2 conectados a una misma red, es necesario establecer un valor entre ``0`` y ``101``. Para ello, ejecute el siguiente comando estableciendo un valor en ``<your_domain_id>``
```
echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bashrc
```


## 2. Uso de ``colcon`` para construir paquetes

### 2.1. Instalar colcon 
```
sudo apt install python3-colcon-common-extensions
```

### configurar ``colcon_cd`` 
El comando ``colcon_cd``  permite dirigirse rápidamente a la raiz de la carpeta donde está el paquete creado desde la terminal, para ello ejecute los siguientes comandos, los cuales son agregados al archivo .bashrc

```
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
```

```
echo "export _colcon_cd_root=/opt/ros/jazzy/" >> ~/.bashrc
```


## 3. Crear un workspace

Un workspace es un directorio que contiene los paquetes de ROS 2
Primero es necesario crear un carpeta (``ros2_ws``) para contener el workspace, ejecutando los siguientes comandos:
```
mkdir -p ~/ros2_ws/src
```

```
cd ~/ros2_ws
```

En este punto, la carpeta ``ros2_ws`` contiene solo una subcarpeta vacia ``src``

### 3.1. Clonar un repositorio de muestra
En el directorio ``ros2_ws/src`` ejecute el siguiente comando:

```
git clone https://github.com/ros/ros_tutorials.git -b jazzy
```
Ahora el paquete ``ros_tutorials`` ha sido clonado en el workspace (``ros2_ws``). 
Hasta el momento se ha rellado el workspace crreado con un paquete de ejemplo, pero aún no es funcional, es necesario resolver las dependencias y luego construir el workspace.

#### Resolver dependencias

Para usar el comando rosdep es necesario asegurar que tu sistema está actualizado antes de instalar nuevos paquetes.

para ello es necesario ejecutar los siguientes comandos en una nueva terminal:

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

Ahora en la terminal anterior dirigete a la raíz del workspace (``ros2_ws``) con el siguiente comando:

```
cd ..
```

Por consiguiente, ejecuta lo siguiente:

```
git clone https://github.com/ros/ros_tutorials.git -b jazzy
```

Si todo está bien con tus dependencias, en la terminal retornará lo siguiente

``
#All required rosdeps installed successfully
`` 

## 3.2 Costruir el workspace con ``colcon``

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




## Herramientas esenciales en ROS 2

### 2.1. Instalación de rqt (Herramienta de interfaz gráfica)
Ejecute una terminal para instalar ``rqt`` y sus plugins

```
sudo apt update
sudo apt install '~nros-jazzy-rqt*'
```


###  Manual de los comandos más usados en Linux

**source:** Lee y ejecuta comandos de un archivo

**echo:** Añade un string a un fichero (echo " " >> prueba.txt)






