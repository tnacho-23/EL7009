# EL7009
Tareas y archivos para el curso EL7009 - Robotica Movil


## Cheat Sheet


### Run
- Dar permisos para GUIs (correr cada vez que se reinicia el computador)
```
xhost +local:docker
```

- Correr por defecto
```
sudo docker run -it  --gpus all --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="/home/ignacio/EL7009:/home/EL7009:rw" ros2_jazzy_devel
```


- Correr y que el contenedor se apague al cerrarlo
```
sudo docker run -it --rm --gpus all --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="/home/ignacio/EL7009:/home/EL7009:rw" ros2_jazzy_devel
```

- Abrir una nueva terminal de un contenedor corriendo
```
sudo docker exec -it -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 rhopeful_keldysh bash
```


### Utils
- Borrar todas las imágenes 
```
sudo docker rmi $(sudo docker images -q) -f
```

- Borrar todos los contenedores detenidos
```
sudo docker rm $(sudo docker ps -a -q)
```

- Borrar todos los contenedores 
```
sudo docker rm -f $(sudo docker ps -a -q)
```

- Ver contenedores activos
```
sudo docker ps
```

- Ver todos los contenedores (activos y detenidos)
```
sudo docker ps -a
```
