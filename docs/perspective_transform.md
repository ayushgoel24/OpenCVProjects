# Perspective projection

Perspective projection or perspective transformation is a linear projection where three dimensional objects are projected on a picture plane. This has the effect that distant objects appear smaller than nearer objects. Lines which are parallel in nature (that is, meet at the point at infinity) appear to intersect in the projected image. 
perspective transformation is change of view point. perspective preserve collinearity and incidence (straight lines will remain straight) but parallelism , length and angle may not preserved.



depending on the orientation of the projection plane towards the axes of the depicted object, perspective projection is categorized into **one-point**, **two-point** and **three-point** perspective.

## One Point Perspective
In one point perspective all lines recede toward one vanishing point.

<img src="images/one-point-perspective.svg" width="400" height="400"/>  

## Two Point Perspective
In two point perspective, lines converge on two vanishing points.
 
<img src="images/two-point-perspective.svg" width="400" height="400"/>  


## Three Point Perspective
In three point perspective all lines recede toward one of the three vanishing points.

<img src="images/three-point-perspective.svg" width="400" height="400"/>  

image courtesy: [1](https://en.wikipedia.org/wiki/3D_projection)



# Weak perspective projection

# Perspective Transformation Matrix

Affine Transformation Matrix had the following form:
<br/>

<img src="https://latex.codecogs.com/svg.image?\displaystyle&space;{\begin{bmatrix}M_{11}&space;&M_{12}&space;&M_{13}&space;\\M_{21}&space;&M_{22}&space;&M_{23}&space;\\0&space;&0&1\end{bmatrix}}" title="https://latex.codecogs.com/svg.image?\displaystyle {\begin{bmatrix}M_{11} &M_{12} &M_{13} \\M_{21} &M_{22} &M_{23} \\0 &0&1\end{bmatrix}}" />  

Perspective Transformation Matrix has the following form:

<img src="https://latex.codecogs.com/svg.image?\displaystyle&space;{\begin{bmatrix}M_{11}&space;&M_{12}&space;&M_{13}&space;\\M_{21}&space;&M_{22}&space;&M_{23}&space;\\M_{31}&space;&M_{32}&1\end{bmatrix}}" title="https://latex.codecogs.com/svg.image?\displaystyle {\begin{bmatrix}M_{11} &M_{12} &M_{13} \\M_{21} &M_{22} &M_{23} \\M_{31} &M_{32}&1\end{bmatrix}}" />  


<br/>
This is the rotation part:
<br/>

<img src="https://latex.codecogs.com/svg.image?\displaystyle&space;{\begin{bmatrix}M_{11}&space;&M_{12}&space;&space;\\M_{21}&space;&M_{22}&space;&space;\end{bmatrix}}" title="https://latex.codecogs.com/svg.image?\displaystyle {\begin{bmatrix}M_{11} &M_{12} \\M_{21} &M_{22} \end{bmatrix}}" />


<br/>
This is the translation part:
<br/>


<img src="https://latex.codecogs.com/svg.image?\displaystyle&space;{\begin{bmatrix}M_{13}&space;\\M_{23}&space;&space;&space;\end{bmatrix}}" title="https://latex.codecogs.com/svg.image?\displaystyle {\begin{bmatrix}M_{13} \\M_{23} \end{bmatrix}}" />


<br/>
This is the projection part:
<br/>

<img src="https://latex.codecogs.com/svg.image?\displaystyle&space;{\begin{bmatrix}M_{31}&space;&&space;M_{32}&space;&space;&space;\end{bmatrix}}" title="https://latex.codecogs.com/svg.image?\displaystyle {\begin{bmatrix}M_{31} & M_{32} \end{bmatrix}}" />





## Effect of Affine vs Perspective Transformation
<br/>

<img src="images/affine_perspective_transformation_matrix.svg" />


# OpenCV Perspective function
The function `cv::warpPerspective` transforms the source image using the specified matrix:


<img src="https://latex.codecogs.com/svg.image?\texttt{dst}&space;(x,y)&space;=&space;\texttt{src}&space;\left&space;(&space;\frac{M_{11}&space;x&space;&plus;&space;M_{12}&space;y&space;&plus;&space;M_{13}}{M_{31}&space;x&space;&plus;&space;M_{32}&space;y&space;&plus;&space;M_{33}}&space;,&space;\frac{M_{21}&space;x&space;&plus;&space;M_{22}&space;y&space;&plus;&space;M_{23}}{M_{31}&space;x&space;&plus;&space;M_{32}&space;y&space;&plus;&space;M_{33}}&space;\right&space;)" title="https://latex.codecogs.com/svg.image?\texttt{dst} (x,y) = \texttt{src} \left ( \frac{M_{11} x + M_{12} y + M_{13}}{M_{31} x + M_{32} y + M_{33}} , \frac{M_{21} x + M_{22} y + M_{23}}{M_{31} x + M_{32} y + M_{33}} \right )" />
