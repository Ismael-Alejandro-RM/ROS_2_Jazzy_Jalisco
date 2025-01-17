# Guía rápida para utilizar ROS 2 (Jazzy Jalisco) en ubuntu 24.04.1

Antes de continuar con la configuración es necesario los pasos para instalar ROS 2
## 1. Configurando el entorno

 ### 1.1. Obtener los archivos de instalación

**source:** Lee y ejecuta comandos de un archivo

Para tener acceso a los comandos de ROS, cada vez que se abre una terminal es necesario ejecutar lo siguiente: 
```
source /opt/ros/jazzy/setup.bash
```

### 1.2 Agregar el source al startup script
Añadir un string a un fichero

echo " " >> prueba.txt

Para evitar lo anterior cada vez que se abre una terminal, es necesario agregar el mismo comando al startup script

```
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```
### 1.3. Verificar las variables del entorno

Para verificar que el entorno para operar ROS 2 está apropiadamente configurado, ejecute el siguiente comando:
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

Para evitar incovenientes entre usuarios de ROS 2 conectados a una misma red, es necesario establecer un valor entre 0 y 101. Para ello, ejecute el siguiente comando estableciendo un valor en ``<your_domain_id>``
```
echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bashrc
```

## 2. Herramientas esenciales en ROS 2

### 2.1. Instalación de rqt (Herramienta de interfaz gráfica)
Ejecute una terminal para instalar ``rqt`` y sus plugins

```
sudo apt update
sudo apt install '~nros-jazzy-rqt*'
```





