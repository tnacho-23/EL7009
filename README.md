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
docker run -it  --gpus all --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="/home/ignacio/EL7009:/home/EL7009:rw" ros2_jazzy_devel
```

- Correr y que el contenedor se apague al cerrarlo
```
docker run -it --rm --gpus all --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="/home/ignacio/EL7009:/home/EL7009:rw" ros2_jazzy_devel
```

- Abrir una nueva terminal de un contenedor corriendo
```
docker exec -it -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 rhopeful_keldysh bash
```


### Utils
- Borrar todas las imágenes 
```
sudo docker rmi $(sudo docker images -q) -f
```
- Abrir empty world en gazebo
```
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=empty.sdf
```