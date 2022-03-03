<style>body {text-align: left}</style>
<div style="text-align: left">

# Edge Detection
Rapid changes in image intensity in small window caused by:


![what_is_edge](images/what_is_edge.jpg)



- Position
- Magnitude
- Orientation


# Image Coordinate System

The **x-coordinate** is defined here as increasing in the **"right"-direction**, and the **y-coordinate** is defined as increasing in the **"down"-direction**.

![Image Coordinate System](https://latex.codecogs.com/svg.latex?{\displaystyle \begin{bmatrix}
x,y   &  x+1,y   & ... & x+n,y  \\
x,y+1 &  x+2,y+1  & ... & x+n,y+1  \\
\vdots &  &  \ddots & \vdots \\ 
 x,y+m  & x+2,y+m  & ... & x+n,y+m  \\
\end{bmatrix} })







# Edge Detection Using First Derivative, Gradient Operator

First derivative edge detection provides both location and strength

![gradients_1d](images/gradients_1d.jpg)

<!-- 
![gradients_operator](images/gradients_operator.jpg)
-->




<img src="https://latex.codecogs.com/svg.latex?
{\displaystyle }\nabla I=\begin{bmatrix}
\frac{\partial I}{\partial x} & , 0 \\\end{bmatrix}" />
![gradients_1d](images/edge_x.jpg)





<img src="https://latex.codecogs.com/svg.latex?
{\displaystyle \nabla I=\begin{bmatrix}
0& , \frac{\partial I}{\partial y} \\\end{bmatrix} }" />
![gradients_1d](images/edge_y.jpg)




<img src="https://latex.codecogs.com/svg.latex?
{\displaystyle \nabla I=\begin{bmatrix}
\frac{\partial I}{\partial x} & , \frac{\partial I}{\partial y} \\\end{bmatrix} }" />
![gradients_1d](images/edge_x_y.jpg)





The simplest approach is to use central differences:


<img src="https://latex.codecogs.com/svg.latex?
{\displaystyle {\begin{aligned}L_{x}(x,y)&=-{\frac {1}{2}}L(x-1,y)+0\cdot L(x,y)+{\frac {1}{2}}\cdot L(x+1,y)\\[8pt]L_{y}(x,y)&=-{\frac {1}{2}}L(x,y-1)+0\cdot L(x,y)+{\frac {1}{2}}\cdot L(x,y+1),\end{aligned}}}" />



<img src="https://latex.codecogs.com/svg.latex?|\nabla L|={\sqrt {L_{x}^{2}+L_{y}^{2}}}" />


while the gradient orientation can be estimated as:

<img src="https://latex.codecogs.com/svg.latex?{\displaystyle \theta =\operatorname {atan2} (L_{y},L_{x}).}" />

- good localization

- noise sensitive


## Roberts cross
Gradient of an image through discrete differentiation which is achieved by computing the sum of the squares of the differences between **diagonally** adjacent pixels.

we convolve the original image, with the following two kernels:


<img src="https://latex.codecogs.com/svg.latex?{\displaystyle {\begin{bmatrix}+1&0\\0&-1\\\end{bmatrix}}\quad {\mbox{and}}\quad {\begin{bmatrix}0&+1\\-1&0\\\end{bmatrix}}.}{" />


<!-- 
<img src="https://latex.codecogs.com/svg.latex?" />
-->


Let 
<img src="https://latex.codecogs.com/svg.latex?{\displaystyle I(x,y)}" /> 
be a point in the original image and 
<img src="https://latex.codecogs.com/svg.latex?{\displaystyle G_{x}(x,y)}" /> 
be a point in an image formed by convolving with the first kernel and 
<img src="https://latex.codecogs.com/svg.latex?{\displaystyle G_{y}(x,y)}" />
 be a point in an image formed by convolving with the second kernel. The gradient can then be defined as:

<img src="https://latex.codecogs.com/svg.latex?{\displaystyle \nabla I(x,y)=G(x,y)={\sqrt {G_{x}^{2}+G_{y}^{2}}}.}" />


direction of the gradient:

<img src="https://latex.codecogs.com/svg.latex?{\displaystyle \Theta (x,y)=\arctan {\left({\frac {G_{y}(x,y)}{G_{x}(x,y)}}\right)}-{\frac {3\pi }{4}}.}" />




## Prewitt

<img src="https://latex.codecogs.com/svg.latex?{\displaystyle \mathbf {G_{x}} ={\begin{bmatrix}+1&0&-1\\+1&0&-1\\+1&0&-1\end{bmatrix}}*\mathbf {A} \quad {\mbox{and}}\quad \mathbf {G_{y}} ={\begin{bmatrix}+1&+1&+1\\0&0&0\\-1&-1&-1\end{bmatrix}}*\mathbf {A} }" />

where  <img src="https://latex.codecogs.com/svg.latex?{\displaystyle *}" /> here denotes the 2-dimensional convolution operation.

Prewitt it is a separable filter (can be decomposed as the products of an averaging and a differentiation kernel), therefore <img src="https://latex.codecogs.com/svg.latex?{\displaystyle \mathbf {G_{x}} }" /> can be written as:

<img src="https://latex.codecogs.com/svg.latex?{\displaystyle {\begin{bmatrix}+1&0&-1\\+1&0&-1\\+1&0&-1\end{bmatrix}}={\begin{bmatrix}1\\1\\1\end{bmatrix}}{\begin{bmatrix}+1&0&-1\end{bmatrix}}}"/>


## Sobel

<img src="https://latex.codecogs.com/svg.latex?
{\displaystyle \mathbf {G} _{x}={\begin{bmatrix}+1&0&-1\\+2&0&-2\\+1&0&-1\end{bmatrix}}*\mathbf {A} \quad {\mbox{and}}\quad \mathbf {G} _{y}={\begin{bmatrix}+1&+2&+1\\0&0&0\\-1&-2&-1\end{bmatrix}}*\mathbf {A} }"/>

Sobel is a separable filter (can be decomposed as the products of an **averaging and a differentiation kernel**), therefore


<img src="https://latex.codecogs.com/svg.latex?
{\displaystyle \mathbf {G} _{x}={\begin{bmatrix}1\\2\\1\end{bmatrix}}*\left({\begin{bmatrix}+1&0&-1\end{bmatrix}}*\mathbf {A} \right)\quad {\mbox{and}}\quad \mathbf {G} _{y}={\begin{bmatrix}+1\\0\\-1\end{bmatrix}}*\left({\begin{bmatrix}1&2&1\end{bmatrix}}*\mathbf {A} \right)}"/>


Refs: [1](https://www.youtube.com/watch?v=G8yp6f9V_6c)

# Edge Detection Using Second Gradient
differential approach of detecting zero-crossings

![gradients_operator](images/laplacian.jpg)

Second-order derivatives can be computed from the scale space representation <img src="https://latex.codecogs.com/svg.latex?{\displaystyle L}"/> according to:


<img src="https://latex.codecogs.com/svg.latex?
{\displaystyle {\begin{aligned}L_{xx}(x,y)&=L(x-1,y)-2L(x,y)+L(x+1,y),\\[6pt]L_{yy}(x,y)&=L(x,y-1)-2L(x,y)+L(x,y+1).\end{aligned}}}"/>

corresponding to the following filter masks:

<img src="https://latex.codecogs.com/svg.latex?{\displaystyle L_{xx}={\begin{bmatrix}1&-2&1\end{bmatrix}}L\quad {\text{and}}\quad {\text{and}}\quad L_{yy}={\begin{bmatrix}1\\-2\\1\end{bmatrix}}L.}"/>


- Edges are zero crossing in Laplacian image
- Laplacian doesn't provide direction of edges

# Difference of Gaussians (DoG) 
Convolution Properties: Associativity

<img src="https://latex.codecogs.com/svg.latex?{\displaystyle f*(g*h)=(f*g)*h}" /> 


First derivative is linear operation, Gaussian smoothing is also linear operation so we can find the first derivative of Gaussian 

# Laplacian of Gaussian (LoG)

# Canny Edge

# Gabor Filter


# Separable Filter

A 2-dimensional convolution operation is separated into two 1-dimensional filters. 

 This reduces the computational costs on an <img src="https://latex.codecogs.com/svg.latex?{\displaystyle N\times M}" />  image with a
 <img src="https://latex.codecogs.com/svg.latex?{\displaystyle m\times n}" /> filter from  <img src="https://latex.codecogs.com/svg.latex?{\displaystyle {\mathcal {O}}(M\cdot N\cdot m\cdot n)}" /> down to <img src="https://latex.codecogs.com/svg.latex?{\displaystyle {\mathcal {O}}(M\cdot N\cdot (m+n))}" />
   

Examples:

1. Smoothing filter:

<img src="https://latex.codecogs.com/svg.latex?{\displaystyle {\frac {1}{3}}{\begin{bmatrix}1\\1\\1\end{bmatrix}}*{\frac {1}{3}}{\begin{bmatrix}1&1&1\end{bmatrix}}={\frac {1}{9}}{\begin{bmatrix}1&1&1\\1&1&1\\1&1&1\end{bmatrix}}}" />

2. Weighted smoothing filter:

<img src="https://latex.codecogs.com/svg.latex?{\frac {1}{4}}{\begin{bmatrix}1\\2\\1\end{bmatrix}}*{\frac {1}{4}}{\begin{bmatrix}1&2&1\end{bmatrix}}={\frac {1}{16}}{\begin{bmatrix}1&2&1\\2&4&2\\1&2&1\end{bmatrix}}" />


3. Sobel operator:

<img src="https://latex.codecogs.com/svg.latex?{\displaystyle {\begin{bmatrix}1\\2\\1\end{bmatrix}}*{\begin{bmatrix}1&0&-1\end{bmatrix}}={\begin{bmatrix}1&0&-1\\2&0&-2\\1&0&-1\end{bmatrix}}}" />



# Convolution




Filtering often involves replacing the value of a pixel in the input image F with the weighted sum of its neighbors

convolution of 5×5 sized image matrix `x` with the kernel `h` of size 3×3, 

![what_is_edge](images/Fig1_2D_Conv.jpg)


kernel flipping and mirroring:

![what_is_edge](images/Fig2_2D_Conv.jpg)


sliding the kernel over the image:

![what_is_edge](images/2D_Convolution_Animation.gif)


# Non Linear Filtering

## Bilateral Filter
## Median Filter



1. The function does actually compute correlation, not the convolution
2. That is, the kernel is not mirrored around the anchor point. If you need a real convolution, flip the kernel using flip and set the new anchor to `(kernel.cols - anchor.x - 1, kernel.rows - anchor.y - 1)`.
3. The function uses the DFT-based algorithm in case of sufficiently large kernels (~11 x 11 or larger) and the direct algorithm for small kernels.



```cpp
void filter2D(Mat src,
              Mat dst,
              int ddepth,
              Mat kernel,
              Point anchor,
              double delta,
              int borderType);
```

</div>


Refs: [1](https://www.youtube.com/watch?v=lOEBsQodtEQ&t)
