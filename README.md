# OpenCV Projects



![alt text](https://img.shields.io/badge/license-BSD-blue.svg)  
This project contains my Computer Vinson Projects with OpenCV.



## Building and Installation
### 1. Building the Image
There is docker file for this project where contains all dependencies and you build the image with :   

`docker build -t myopencv_image .`

### 2. Creating the container
Create a container where you mount the checkout code into your container: 

`docker run --name <continer-name> -v <checked-out-path-on-host>:<path-in-the-container> -it <docker-image-name>`

for instance:

`docker run --name myopencv_container -v /home/behnam/workspace/OpenCVProjects:/OpenCVProjects -it myopencv-image`

### 3. Starting an existing container
If you have already created a container from the docker image, you can start it with:

`docker start -i myopencv_container`

### 4. Removing  unnecessary images and containers
You can remove unnecessary images and containers by:

`docker image prune -a`

`docker container prune` 


### GUI application with docker
1. You need to run:

`docker run --name myopencv_container -v /home/behnam/workspace/OpenCVProjects:/OpenCVProjects --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"  -it myopencv-image`

2. On the host run the following (every time you run your container):

`export containerId=$(docker ps -l -q)`

<code>  xhost +local: docker inspect --format='{{ .Config.Hostname }}' $containerId </code>


read more [here](https://ros-developer.com/2017/11/08/docker/)



# [Index](#)
[Affine Transformation](src/affine_transformation.cpp)    
[Basic Operations](src/basic_operations.cpp)    
[Blob Detection](src/blob_detection.cpp)    
[Corner Detection](src/corner_detection.cpp)    
[Edge Detection](docs/edge_detection.md)    
[Feature Description](src/feature_description.cpp)    
[Fundamental Matrix_estimation](src/fundamental_matrix_estimation.cpp)    
[Histogram Analysis](docs/histogram_analysis.md)    
[Homogenious_conversion](src/homogenious_conversion.cpp)    
[Homography](src/homography.cpp)    
[Hough Transform](src/hough_transform.cpp)    
[ICP](src/icp.cpp)    
[Image Moments](src/image_moments.cpp)    
[Morphological Transformation](src/morphological_transformation.cpp)    
[PCA](src/pca.cpp)    
[Perspective-n-point](src/perspective-n-point.cpp)    
[Triangulation](src/triangulation.cpp)  
[Structured Light Range Finding](src/structured_light_range_finding.cpp)  
[2D Image Points to 3D World Unit Vector](src/2d_image_point_to_3d_world_unit_vector.cpp)  

Refs: [1](https://www.youtube.com/channel/UCf0WB91t8Ky6AuYcQV0CcLw/videos),[2](https://github.com/spmallick/learnopencv/blob/master/README.md), [3](http://graphics.cs.cmu.edu/courses/15-463/), [4](https://www.tangramvision.com/blog/camera-modeling-exploring-distortion-and-distortion-models-part-i)  
