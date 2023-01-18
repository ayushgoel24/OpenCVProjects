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

<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?mx=\frac{&space;\text{Number&space;of&space;Pixel&space;In&space;Width}}{\text{Width&space;of&space;Sensor}}=\frac{1}{\text{Width&space;of&space;Pixel}" title="https://latex.codecogs.com/svg.image?mx=\frac{ \text{Number of Pixel In Width}}{\text{Width of Sensor}}=\frac{1}{\text{Width of Pixel}" />

<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?my=\frac{&space;\text{Number&space;of&space;Pixel&space;In&space;Heigh}}{\text{Height&space;of&space;Sensor}}=\frac{1}{\text{Height&space;of&space;Pixel}" title="https://latex.codecogs.com/svg.image?my=\frac{ \text{Number of Pixel In Heigh}}{\text{Height of Sensor}}=\frac{1}{\text{Height of Pixel}" />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?cy=\frac{&space;\text{Number&space;of&space;Pixel&space;In&space;Height}}{\text{2}}" title="https://latex.codecogs.com/svg.image?cy=\frac{ \text{Number of Pixel In Height}}{\text{2}}" />    


<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?cx=\frac{&space;\text{Number&space;of&space;Pixel&space;In&space;Width}}{\text{2}}" title="https://latex.codecogs.com/svg.image?cx=\frac{ \text{Number of Pixel In Width}}{\text{2}}" />    

<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?f_x=f\times&space;m_x" title="https://latex.codecogs.com/svg.image?f_x=f\times m_x" />


<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?f_y=f\times&space;m_y" title="https://latex.codecogs.com/svg.image?f_y=f\times m_y" />



<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?\text{column}&space;=f_x\frac{X}{Z}&space;&plus;cx" title="https://latex.codecogs.com/svg.image?\text{column} =f_x\frac{X}{Z} +cx" />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\text{&space;row}&space;=f_y\frac{Y}{Z}&space;&plus;cy" title="https://latex.codecogs.com/svg.image?\text{ row} =f_y\frac{Y}{Z} +cy" />
<br/>
<br/>

# Example of Projection 


<img src="https://latex.codecogs.com/svg.image?\text{Number&space;of&space;Pixel&space;In&space;Width=640}" title="https://latex.codecogs.com/svg.image?\text{Number of Pixel In Width=640}" />
<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?\text{Number&space;of&space;Pixel&space;In&space;Height=480}&space;" title="https://latex.codecogs.com/svg.image?\text{Number of Pixel In Height=480} " />



<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?\text{Height%20of%20Sensor=10%20mm}" title="https://latex.codecogs.com/svg.image?\text{Height of Sensor=10 mm}" />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\text{Width&space;of&space;Sensor=10&space;mm}&space;" title="https://latex.codecogs.com/svg.image?\text{Width of Sensor=10 mm} " />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\text{f=0.1}&space;" title="https://latex.codecogs.com/svg.image?\text{f=0.1} " />
<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?\text{Points&space;in&space;Camera&space;Coordinate=}\begin{bmatrix}&space;0&&space;2&space;&&space;1&space;&&space;&space;2&&space;3&space;&&space;2&&space;2\\&space;0&&space;1&space;&&space;2&space;&&space;&space;2&space;&&space;2&&space;3&space;&&space;4\\&space;0&&space;1&space;&&space;1&space;&&space;&space;1&&space;1&&space;1&space;&&space;1\\\end{bmatrix}&space;&space;" title="https://latex.codecogs.com/svg.image?\text{Points in Camera Coordinate=}\begin{bmatrix} 0& 2 & 1 & 2& 3 & 2& 2\\ 0& 1 & 2 & 2 & 2& 3 & 4\\ 0& 1 & 1 & 1& 1& 1 & 1\\\end{bmatrix} " />


<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?cy=\frac{480}{2}=240" title="https://latex.codecogs.com/svg.image?cy=\frac{480}{2}=240" />
<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?cx=\frac{640}{2}=320" title="https://latex.codecogs.com/svg.image?cx=\frac{640}{2}=320" />
<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?fx=0.1&space;\times&space;\frac{640}{10}=6.4" title="https://latex.codecogs.com/svg.image?fx=0.1 \times \frac{640}{10}=6.4" />

<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?fy=0.1&space;\times&space;\frac{240}{10}=2.4" title="https://latex.codecogs.com/svg.image?fy=0.1 \times \frac{240}{10}=2.4" />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?K=\begin{bmatrix}6.4&space;&&space;0&space;&space;&&space;320&space;&space;\\0,&space;&&space;4.8&space;&&space;240&space;\\0&space;&space;&&space;0&space;&space;&&space;1&space;\\\end{bmatrix}&space;" title="https://latex.codecogs.com/svg.image?K=\begin{bmatrix}6.4 & 0 & 320 \\0, & 4.8 & 240 \\0 & 0 & 1 \\\end{bmatrix} " />


<br/>
<br/>


projected pints in camera:

<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?\text{Projectd&space;Points&space;Camera&space;Plane=}\begin{bmatrix}\text{column&space;}\\\text{row&space;}\\1\end{bmatrix}&space;=\begin{bmatrix}320&space;&&space;332.8&space;&&space;326.4&space;&&space;332.8&space;&&space;339.2&space;&&space;332.8&space;&&space;332.8\\&space;240&space;&&space;244.8&space;&&space;249.6&space;&&space;249.6&space;&&space;249.6&space;&&space;254.4&space;&&space;259.2\\&space;1&space;&&space;1&space;&&space;1&space;&&space;1&space;&&space;1&space;&&space;1&space;&&space;1\\\end{bmatrix}&space;&space;" title="https://latex.codecogs.com/svg.image?\text{Projectd Points Camera Plane=}\begin{bmatrix}\text{column }\\\text{row }\\1\end{bmatrix} =\begin{bmatrix}320 & 332.8 & 326.4 & 332.8 & 339.2 & 332.8 & 332.8\\ 240 & 244.8 & 249.6 & 249.6 & 249.6 & 254.4 & 259.2\\ 1 & 1 & 1 & 1 & 1 & 1 & 1\\\end{bmatrix} " />

<br/>
<br/>

![pinhole_camera_model](images/pinhole_camera_model.png)

<br/>
<br/>

<img src="images/image_0.100000_.jpg">


# OpenCV API

In OpenCV, `Point(x=coulmn,y=row)`. For instance the point in the following image can be accessed with

```cpp
    X                      
    --------column---------►
    | Point(0,0) Point(1,0) Point(2,0) Point(3,0)
    | Point(0,1) Point(1,1) Point(2,1) Point(3,1)
    | Point(0,2) Point(1,2) Point(2,2) Point(3,2)
  y |
   row
    |
    |
    ▼

```
However if you access an image directly, the order is <img src="https://latex.codecogs.com/svg.image?\text{(row,&space;column)}" title="https://latex.codecogs.com/svg.image?\text{(row, column)}" />

```cpp
    X                      
    --------column---------►
    | mat.at<type>(0,0) mat.at<type>(0,1) mat.at<type>(0,2) mat.at<type>(0,3)
    | mat.at<type>(1,0) mat.at<type>(1,1) mat.at<type>(1,2) mat.at<type>(1,3)
    | mat.at<type>(2,0) mat.at<type>(2,1) mat.at<type>(2,2) mat.at<type>(2,3)
  y |
   row
    |
    |
    ▼
```    


So the following will return the same value:



```cpp
mat.at<type>(row,column) 
mat.at<type>(cv::Point(column,row))
```
For instance:
```cpp
std::cout<<static_cast<unsigned>(img.at<uchar>(row,column))    <<std::endl;
std::cout<<static_cast<unsigned>(img.at<uchar>( cv::Point(column,row))     )<<std::endl;
```








# 3D World Unit Vector

Refs: [1](https://stackoverflow.com/questions/12977980/in-opencv-converting-2d-image-point-to-3d-world-unit-vector),
[2](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html),
[3](https://stackoverflow.com/questions/44888119/c-opencv-calibration-of-the-camera-with-different-resolution),
[4](https://docs.opencv.org/3.2.0/da/d54/group__imgproc__transform.html#ga55c716492470bfe86b0ee9bf3a1f0f7e),
[5](https://www.mathematik.uni-marburg.de/~thormae/lectures/graphics1/graphics_6_1_eng_web.html#1)






# Resizing Image Effect on the Camera Intrinsic Matrix




