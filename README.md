# OpneCV Projects
This project contains samples of the basic routine in OpneCV.



Build the docker image:
```
docker build -t myopencv-image .
```

Run the container:
```
docker run --name ubuntu-opencv --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"  -v opencv-src:/home/opencv-src -v /home/behnam/Workspace/OpenCVProjects:/home/OpenCVProjects   -it myopencv-image
```

Bring the GUI via X Window System

```
xhost +local:`docker inspect --format='{{ .Config.Hostname }}' $containerId`
```

