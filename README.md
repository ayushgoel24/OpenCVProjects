# OpenCV Projects
This project contains my Computer Vinson Projects with OpenCV.

# Installation

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

# Index
[Affine Transformation](src/affine_transformation.cpp)
[Basic Operations](src/basic_operations.cpp)    
[Blob Detection](src/blob_detection.cpp)    
[Corner Detection](src/corner_detection.cpp)    
[Edge Detection](src/edge_detection.cpp)    
[Feature Description](src/feature_description.cpp)    
[Fundamental Matrix_estimation](src/fundamental_matrix_estimation.cpp)    
[Histogram Analysis](src/histogram_analysis.cpp)    
[Homogenious_conversion](src/homogenious_conversion.cpp)    
[Homography](src/homography.cpp)    
[Hough Transform](src/hough_transform.cpp)    
[ICP](src/icp.cpp)    
[Image Moments](src/image_moments.cpp)    
[Morphological Transformation](src/morphological_transformation.cpp)    
[PCA](src/pca.cpp)    
[Perspective-n-point](src/perspective-n-point.cpp)    

