# Graphical Projection

There are two graphical projection categories:

- parallel projection
- perspective projection


<img src="images/comparison_of_graphical_projections.svg" width="500" height="500" />



# Pinhole Camera Model

the coordinates  of point <img src="https://latex.codecogs.com/svg.image?Q(x,y)" title="https://latex.codecogs.com/svg.image?Q(x,y)" /> depend on the coordinates of point <img src="https://latex.codecogs.com/svg.image?P(X_w,Y_w,Z_w)" title="https://latex.codecogs.com/svg.image?P(X_w,Y_w,Z_w)" /> 

- <img src="https://latex.codecogs.com/svg.image?\frac{-y}{Y_w}=\frac{f}{Z_w}" title="https://latex.codecogs.com/svg.image?\frac{-y}{Y_w}=\frac{f}{Z_w}" />

- <img src="https://latex.codecogs.com/svg.image?\frac{-x}{X_w}=\frac{f}{Z_w}" title="https://latex.codecogs.com/svg.image?\frac{-x}{X_w}=\frac{f}{Z_w}" />


![Pinhole](images/Pinhole.svg)
![Pinhole2](images/Pinhole2.svg)



Refs: [1](https://en.wikipedia.org/wiki/Pinhole_camera_model#Geometry),
[2](https://ksimek.github.io/2013/08/13/intrinsic/),


# Rotated Image and the Virtual Image Plane

The mapping from 3D to 2D coordinates described by a pinhole camera is a perspective projection followed by a `180°` rotation in the image plane. This corresponds to how a real pinhole camera operates; the resulting image is rotated `180°` and the relative size of projected objects depends on their distance to the focal point and the overall size of the image depends on the distance f between the image plane and the focal point. In order to produce an unrotated image, which is what we expect from a camera we Place the image plane so that it intersects the <img src="https://latex.codecogs.com/svg.image?Z" title="https://latex.codecogs.com/svg.image?Z" /> axis at `f` instead of at `-f` and rework the previous calculations. This would generate a virtual (or front) image plane which cannot be implemented in practice, but provides a theoretical camera which may be simpler to analyse than the real one.


# Camera Resectioning and Projection Matrix 

Projection refers to the pinhole camera model, a camera matrix <img src="https://latex.codecogs.com/svg.image?\text{M}" title="https://latex.codecogs.com/svg.image?\text{M}" /> is used to denote a projective mapping from world coordinates to pixel coordinates.



<img src="https://latex.codecogs.com/svg.image?{\displaystyle&space;z_{c}{\begin{bmatrix}u\\v\\1\end{bmatrix}}=K\,{\begin{bmatrix}R&T\end{bmatrix}}{\begin{bmatrix}X_{w}\\Y_{w}\\Z_{w}\\1\end{bmatrix}}=M{\begin{bmatrix}X_{w}\\Y_{w}\\Z_{w}\\1\end{bmatrix}}}" title="https://latex.codecogs.com/svg.image?{\displaystyle z_{c}{\begin{bmatrix}u\\v\\1\end{bmatrix}}=K\,{\begin{bmatrix}R&T\end{bmatrix}}{\begin{bmatrix}X_{w}\\Y_{w}\\Z_{w}\\1\end{bmatrix}}=M{\begin{bmatrix}X_{w}\\Y_{w}\\Z_{w}\\1\end{bmatrix}}}" />




<img src="https://latex.codecogs.com/svg.image?[u\&space;v\&space;1]^{T}" title="https://latex.codecogs.com/svg.image?[u\ v\ 1]^{T}" /> represent a 2D point position in pixel coordinates and <img src="https://latex.codecogs.com/svg.image?[X_{w}\&space;Y_{w}\&space;Z_{w}\&space;1]^{T}" title="https://latex.codecogs.com/svg.image?[X_{w}\ Y_{w}\ Z_{w}\ 1]^{T}" /> represent a 3D point position in world coordinates.


<img src="https://latex.codecogs.com/svg.image?{\displaystyle&space;K={\begin{bmatrix}f_{x}&\gamma&space;&c_{x}&0\\0&f_{y}&c_{y}&0\\0&0&1&0\end{bmatrix}}}" title="https://latex.codecogs.com/svg.image?{\displaystyle K={\begin{bmatrix}f_{x}&\gamma &c_{x}&0\\0&f_{y}&c_{y}&0\\0&0&1&0\end{bmatrix}}}" />



- <img src="https://latex.codecogs.com/svg.image?mx=\frac{1}{\text{Width}}=\frac{&space;\text{Number&space;of&space;Pixel&space;In&space;Width}}{\text{Width&space;of&space;Sensor}&space;&space;}" title="https://latex.codecogs.com/svg.image?mx=\frac{1}{\text{Width}}=\frac{ \text{Number of Pixel In Width}}{\text{Width of Sensor} }" />    





- <img src="https://latex.codecogs.com/svg.image?my=\frac{1}{\text{Height}}=\frac{&space;\text{Number&space;of&space;Pixel&space;In&space;Height}}{\text{Height&space;of&space;Sensor}}" title="https://latex.codecogs.com/svg.image?my=\frac{1}{\text{Height}}=\frac{ \text{Number of Pixel In Height}}{\text{Height of Sensor}}" />    





- <img src="https://latex.codecogs.com/svg.image?cy=\frac{&space;\text{Number&space;of&space;Pixel&space;In&space;Height}}{\text{2}}" title="https://latex.codecogs.com/svg.image?cy=\frac{ \text{Number of Pixel In Height}}{\text{2}}" />    




- <img src="https://latex.codecogs.com/svg.image?cx=\frac{&space;\text{Number&space;of&space;Pixel&space;In&space;Width}}{\text{2}}" title="https://latex.codecogs.com/svg.image?cx=\frac{ \text{Number of Pixel In Width}}{\text{2}}" />    

 
 In OpenCV, `Point(x,y)` and in the in the following figure <img src="https://latex.codecogs.com/svg.image?\text{x&space;or&space;u}" title="https://latex.codecogs.com/svg.image?\text{x or u}" /> means <img src="https://latex.codecogs.com/svg.image?\text{column}" title="https://latex.codecogs.com/svg.image?\text{column}" /> and <img src="https://latex.codecogs.com/svg.image?\text{y&space;or&space;v}" title="https://latex.codecogs.com/svg.image?\text{y or v}" /> means  <img src="https://latex.codecogs.com/svg.image?\text{row}" title="https://latex.codecogs.com/svg.image?\text{row}" /> (reverse of what is in the picture!)


```cpp
Point(x,y) represent (column,row)

mat.at<type>(row,column) or mat.at<type>(cv::Point(x,y))
to access the same point if x=column and y=row


0/0---column--->
 |
 |
row
 |
 |
 v
```



```cpp 

std::vector<cv::Point3d> objectpoints;
objectpoints.push_back(cv::Point3d(8,-4,2));
objectpoints.push_back(cv::Point3d(4,-2,1));
objectpoints.push_back(cv::Point3d(0,0,1));

cv::projectPoints(objectpoints, cameraRotation, cameraTranslation, cameraMatrix, distortionCoefficient, projectedPoints);
std::cout<< "projected point in camera" <<std::endl;
for(const auto p:projectedPoints)
	std::cout<<"column:" <<p.x <<"," <<"row:"<<p.y <<std::endl;
```

![pinhole_camera_model](images/pinhole_camera_model.png)



# 3D World Unit Vector

Refs: [1](https://stackoverflow.com/questions/12977980/in-opencv-converting-2d-image-point-to-3d-world-unit-vector),
[2](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html),
[3](https://stackoverflow.com/questions/44888119/c-opencv-calibration-of-the-camera-with-different-resolution),
[4](https://docs.opencv.org/3.2.0/da/d54/group__imgproc__transform.html#ga55c716492470bfe86b0ee9bf3a1f0f7e),
[5](https://www.mathematik.uni-marburg.de/~thormae/lectures/graphics1/graphics_6_1_eng_web.html#1)











