# Affine Transformation
An affine transformation is a geometric transformation that preserves **lines** and **parallelism** but not necessarily distances and angles.
Examples of affine transformations include translation, scaling, homothety, similarity, reflection, rotation, shear mapping, and compositions of them in any combination and sequence.

# Affine Map
A generalization of an affine transformation is an affine map. an affine map is the composition of two functions: a translation and a linear map.
 if the linear map is represented as a multiplication by an invertible matrix <img src="https://latex.codecogs.com/svg.image?A" title="https://latex.codecogs.com/svg.image?A" /> and the translation as the addition of a vector <img src="https://latex.codecogs.com/svg.image?b" title="https://latex.codecogs.com/svg.image?b" />, an affine map <img src="https://latex.codecogs.com/svg.image?f" title="https://latex.codecogs.com/svg.image?f" /> acting on a vector <img src="https://latex.codecogs.com/svg.image?X" title="https://latex.codecogs.com/svg.image?X" />  can be represented as

<img src="https://latex.codecogs.com/svg.image?{\displaystyle&space;\mathbf&space;{y}&space;=f(\mathbf&space;{x}&space;)=A\mathbf&space;{x}&space;&plus;\mathbf&space;{b}&space;.}" title="https://latex.codecogs.com/svg.image?{\displaystyle \mathbf {y} =f(\mathbf {x} )=A\mathbf {x} +\mathbf {b} .}" />

# Affine Transformation Matrix
Using an augmented matrix, we can represent both the translation and the linear map using a single matrix multiplication. 

<img src="https://latex.codecogs.com/svg.image?{\displaystyle&space;{\begin{bmatrix}\mathbf&space;{y}&space;\\1\end{bmatrix}}=\left[{\begin{array}{ccc|c}&A&&\mathbf&space;{b}&space;\\0&\cdots&space;&0&1\end{array}}\right]{\begin{bmatrix}\mathbf&space;{x}&space;\\1\end{bmatrix}}}" title="https://latex.codecogs.com/svg.image?{\displaystyle {\begin{bmatrix}\mathbf {y} \\1\end{bmatrix}}=\left[{\begin{array}{ccc|c}&A&&\mathbf {b} \\0&\cdots &0&1\end{array}}\right]{\begin{bmatrix}\mathbf {x} \\1\end{bmatrix}}}" />


## 1. Scaling
A scaling can be represented by a scaling matrix:

<img src="https://latex.codecogs.com/svg.image?{\displaystyle&space;{\begin{bmatrix}c_{x}&0&0\\0&c_{y}&0\\0&0&1\end{bmatrix}}}" title="https://latex.codecogs.com/svg.image?{\displaystyle {\begin{bmatrix}c_{x}&0&0\\0&c_{y}&0\\0&0&1\end{bmatrix}}}" />

 
## 2. Translation

<img src="https://latex.codecogs.com/svg.image?{\displaystyle&space;{\begin{bmatrix}1&0&v_{x}\\0&1&v_{y}\\0&0&1\end{bmatrix}}}" title="https://latex.codecogs.com/svg.image?{\displaystyle {\begin{bmatrix}1&0&v_{x}>0\\0&1&v_{y}=0\\0&0&1\end{bmatrix}}}" />



## 3. Reflection

<img src="https://latex.codecogs.com/svg.image?{\displaystyle&space;{\begin{bmatrix}-1&0&0\\0&1&0\\0&0&1\end{bmatrix}}}" title="https://latex.codecogs.com/svg.image?{\displaystyle {\begin{bmatrix}-1&0&0\\0&1&0\\0&0&1\end{bmatrix}}}" />



## 4. Rotate

<img src="https://latex.codecogs.com/svg.image?{\displaystyle&space;{\begin{bmatrix}\cos(\theta&space;)&-\sin(\theta&space;)&0\\\sin(\theta&space;)&\cos(\theta&space;)&0\\0&0&1\end{bmatrix}}}" title="https://latex.codecogs.com/svg.image?{\displaystyle {\begin{bmatrix}\cos(\theta )&-\sin(\theta )&0\\\sin(\theta )&\cos(\theta )&0\\0&0&1\end{bmatrix}}}" />


## 5. Shear

<img src="https://latex.codecogs.com/svg.image?{\displaystyle&space;{\begin{bmatrix}1&c_{x}&0\\c_{y}&1&0\\0&0&1\end{bmatrix}}}" title="https://latex.codecogs.com/svg.image?{\displaystyle {\begin{bmatrix}1&c_{x}&0\\c_{y}&1&0\\0&0&1\end{bmatrix}}}" />


## OpenCV Affine Transformation
In OpenCV you can call `cv::warpAffine` to apply an affine transformation to an image. The function warpAffine transforms the source image using the specified matrix:

<img src="https://latex.codecogs.com/svg.image?dst&space;(x,y)&space;=&space;src&space;(&space;M&space;_{11}&space;x&space;&plus;&space;M&space;_{12}&space;y&space;&plus;&space;M&space;_{13},&space;M&space;_{21}&space;x&space;&plus;&space;M&space;_{22}&space;y&space;&plus;&space;M&space;_{23})" title="https://latex.codecogs.com/svg.image?dst (x,y) = src ( M_{11} x + M_{12} y + M_{13}, M_{21} x + M_{22} y + M_{23})" />

As you can see the relation between input and output is completely linear. When you apply affine transform on an image, the size of output image might be different and you have to specify the size

`cv::warpAffine( src, dst,  affine_mat, cv::Size2d(width,heigh) );`

For Scaling, you can call `cv::resize`

# Finding Affine Transform


`cv::getAffineTransform` and `cv::estimateAffine3D` will find the affine transformation between source and destination 

[code](../src/affine_transformation.cpp)

