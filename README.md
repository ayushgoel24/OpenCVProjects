# OpenCV Projects



![alt text](https://img.shields.io/badge/license-BSD-blue.svg) ![build workflow](https://github.com/behnamasadi/OpenCVProjects/actions/workflows/docker-image.yml/badge.svg)  

This project contains my Computer Vinson Projects with OpenCV.



## Building and Installation
### 1. Building the Image
There is docker file for this project where contains all dependencies and you build the image with :   

`docker build -t myopencv_image:latest .`

### 2. Creating the container
Create a container where you mount the checkout code into your container: 

`docker run --name <continer-name> -v <checked-out-path-on-host>:<path-in-the-container> -it <docker-image-name>`

for instance:

`docker run --name myopencv_container -v /home/behnam/workspace/OpenCVProjects:/OpenCVProjects -it myopencv_image`

### 3. Starting an existing container
If you have already created a container from the docker image, you can start it with:

`docker start -i myopencv_container`

### 4. Removing  unnecessary images and containers
You can remove unnecessary images and containers by:

`docker image prune -a`

`docker container prune` 


### GUI application with docker
1. You need to run:

`docker run --name myopencv_container -v /home/behnam/workspace/OpenCVProjects:/OpenCVProjects --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"  -it myopencv_image`

2. On the host run the following (every time you run your container):

<code>export containerId=$(docker ps -l -q)  
<code>  xhost +local: docker inspect --format='{{ .Config.Hostname }}' $containerId </code>


read more [here](https://ros-developer.com/2017/11/08/docker/)



# [Computer Vision](#)

[Edge Detection](docs/edge_detection.md)  
[Histogram Analysis](docs/histogram_analysis.md)  
[Pinhole Camera Model and Projection](docs/projection_camera_intrinsic.md)  
[Direct Linear Transformation](docs/direct_linear_transformation.md)  
[Zhang's Camera Calibration Algorithm](docs/zhang_camera_calibration_algorithm.md)  
[Affine Transformation](docs/affine_transformation.md)  
[Perspective Transformation](docs/perspective_transform.md)  
[Homography Transformation](docs/homography.md)  
[Lense Distortion Modeling, Image Undistortion](docs/lense_distortion.md)  
[Triangulation](docs/triangulation.md)  
[Epipolar Geometry](docs/epipolar_geometry.md)  
[Essential Matrix](docs/essential_matrix.md)  
[Fundamental Matrix](docs/fundamental_matrix.md)  
[Image Rectification](docs/image_rectification.md)  
[Correspondence Problem](docs/correspondence_problem.md)  
[Stereo Vision](docs/stereo_vision.md)  
[Photogrammetry](docs/photogrammetry.md)  
[Structure From Motion](docs/structure_from_motion.md)  
[Parallax](docs/parallax.md)  



[Optical Flow](docs/optical_flow.md)  
[Visual Odometry](visual_odometry.md)  
[Ego Motion](docs/ego-motion.md)  
[Laser Triangulation](docs/laser_triangulation.md)  

# [OpenCV API](#)
[Basic Operations](src/basic_operations.cpp)  
[Coordinate System and Points](src/coordinate_system_points.cpp)  
[File Storage IO](src/file_storage_io.cpp)  
[Blob Detection](src/blob_detection.cpp)  
[Corner Detection](src/corner_detection.cpp)  
[Feature Detection](src/feature_detection.cpp)  
[Feature Description](src/feature_description.cpp)  
[Image Moments](src/image_moments.cpp)  
[PCA Principal Component Analysis](src/pca.cpp)  
[Morphological Transformation](src/morphological_transformation.cpp)  
[Hough Transform](src/hough_transform.cpp)  
[Homogeneous Conversion](src/homogeneous_conversion.cpp)  
[Camera Calibration](src/virtual_camera_calibration.cpp)  
[Projection Matrix](src/projection_camera_intrinsic.cpp)  
[Camera Intrinsic](src/projection_camera_intrinsic.cpp)  
[Perspective-n-point](src/perspective-n-point.cpp)  
[3D World Unit Vector](src/projection_camera_intrinsic.cpp)  
[Essential Matrix Estimation](src/essential_matrix_estimation.cpp)  
[Fundamental Matrix Estimation](src/fundamental_matrix_estimation.cpp)  
[Rodrigue, Rotation Matrices](src/rodrigue_rotation_matrices.cpp)  
[Tracking Objects by Color](src/tracking_objects_by_color.cpp)  
[Image Remaping](src/remap.cpp)
[Undistorting Images](src/undistorting_images.cpp)  
[Rectifying Images](src/rectifying_images.cpp)  
[Triangulation](src/triangulation.cpp)  
[ICP Iterative Closest Point](src/icp.cpp)  
[Structured Light Range Finding](src/structured_light_range_finding.cpp)  



Refs: [1](https://www.youtube.com/channel/UCf0WB91t8Ky6AuYcQV0CcLw/videos),[2](https://github.com/spmallick/learnopencv/blob/master/README.md),[3](http://graphics.cs.cmu.edu/courses/15-463/),[4](https://www.tangramvision.com/blog/camera-modeling-exploring-distortion-and-distortion-models-part-i)  
