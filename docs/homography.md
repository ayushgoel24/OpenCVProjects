# Homography
Any two images of the same planar surface in space are related by a homography

<img src="images/homography1.svg" />   



<br/>

<img src="https://latex.codecogs.com/svg.image?xp_1&space;=&space;\left&space;(&space;\frac{H_{11}&space;x_1&space;&plus;&space;H_{12}&space;y_1&space;&plus;&space;H_{13}}{H_{31}&space;x_1&space;&plus;&space;H_{32}&space;y_1&space;&plus;&space;M_{33}}&space;\right&space;)" title="https://latex.codecogs.com/svg.image?xp_1 = \left ( \frac{H_{11} x_1 + H_{12} y_1 + H_{13}}{H_{31} x_1 + H_{32} y_1 + M_{33}} \right )" />
<br/>


<img src="https://latex.codecogs.com/svg.image?yp_1&space;=&space;\left&space;(&space;\frac{H_{21}&space;x_1&space;&plus;&space;H_{22}&space;y_1&space;&plus;&space;H_{23}}{H_{31}&space;x_1&space;&plus;&space;H_{32}&space;y_1&space;&plus;&space;H_{33}}&space;\right&space;)" title="https://latex.codecogs.com/svg.image?yp_1 = \left ( \frac{H_{21} x_1 + H_{22} y_1 + H_{23}}{H_{31} x_1 + H_{32} y_1 + H_{33}} \right )" />
<br/>
<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?\begin{equation}&space;\left(&space;\begin{array}{ccccccccc}&space;-x_1&space;&&space;-y_1&space;&&space;-1&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;x_1*xp_1&space;&&space;y_1*xp_1&space;&&space;xp_1\\0&space;&&space;0&space;&&space;0&space;&&space;-x_1&space;&&space;-y_1&space;&&space;-1&space;&&space;x_1*yp_1&space;&&space;y_1*yp_1&space;&&space;yp_1\\-x_2&space;&&space;-y_2&space;&&space;-1&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;x_2*xp_2&space;&&space;y_2*xp_2&space;&&space;xp_2\\0&space;&&space;0&space;&&space;0&space;&&space;-x_2&space;&&space;-y_2&space;&&space;-1&space;&&space;x_2*yp_2&space;&&space;y_2*yp_2&space;&&space;yp_2\\-x_3&space;&&space;-y_3&space;&&space;-1&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;x_3*xp_3&space;&&space;y_3*xp_3&space;&&space;xp_3\\0&space;&&space;0&space;&&space;0&space;&&space;-x_3&space;&&space;-y_3&space;&&space;-1&space;&&space;x_3*yp_3&space;&&space;y_3*yp_3&space;&&space;yp_3\\-x_4&space;&&space;-y_4&space;&&space;-1&space;&&space;0&space;&&space;0&space;&&space;0&space;&&space;x_4*xp_4&space;&&space;y_4*xp_4&space;&&space;xp_4\\0&space;&&space;0&space;&&space;0&space;&&space;-x_4&space;&&space;-y_4&space;&&space;-1&space;&&space;x_4*yp_4&space;&&space;y_4*yp_4&space;&&space;yp_4\\\end{array}&space;\right)&space;*H=0&space;\end{equation}" title="https://latex.codecogs.com/svg.image?\begin{equation} \left( \begin{array}{ccccccccc} -x_1 & -y_1 & -1 & 0 & 0 & 0 & x_1*xp_1 & y_1*xp_1 & xp_1\\0 & 0 & 0 & -x_1 & -y_1 & -1 & x_1*yp_1 & y_1*yp_1 & yp_1\\-x_2 & -y_2 & -1 & 0 & 0 & 0 & x_2*xp_2 & y_2*xp_2 & xp_2\\0 & 0 & 0 & -x_2 & -y_2 & -1 & x_2*yp_2 & y_2*yp_2 & yp_2\\-x_3 & -y_3 & -1 & 0 & 0 & 0 & x_3*xp_3 & y_3*xp_3 & xp_3\\0 & 0 & 0 & -x_3 & -y_3 & -1 & x_3*yp_3 & y_3*yp_3 & yp_3\\-x_4 & -y_4 & -1 & 0 & 0 & 0 & x_4*xp_4 & y_4*xp_4 & xp_4\\0 & 0 & 0 & -x_4 & -y_4 & -1 & x_4*yp_4 & y_4*yp_4 & yp_4\\\end{array} \right) *H=0 \end{equation}" />

<br/>
<br/>
<br/>



<img src="https://latex.codecogs.com/svg.image?H^{*}&space;\underset{H}{\mathrm{argmin}}=&space;\|AH\|^{2}" title="https://latex.codecogs.com/svg.image?H^{*} \underset{H}{\mathrm{argmin}}= \|AH\|^{2}" />




Singular-value Decomposition (SVD) of any given matrix <img src="https://latex.codecogs.com/svg.image?A_{M{\times}N}" title="https://latex.codecogs.com/svg.image?A_{M{\times}N}" />

<br/>
<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\begin{equation}&space;\underbrace{\mathbf{A}}_{M&space;\times&space;N}&space;=&space;\underbrace{\mathbf{U}}_{M&space;\times&space;M}&space;\times&space;\underbrace{\mathbf{\Sigma}}_{M\times&space;N}&space;\times&space;\underbrace{\mathbf{V}^{\text{T}}}_{N&space;\times&space;N}&space;\end{equation}" title="https://latex.codecogs.com/svg.image?\begin{equation} \underbrace{\mathbf{A}}_{M \times N} = \underbrace{\mathbf{U}}_{M \times M} \times \underbrace{\mathbf{\Sigma}}_{M\times N} \times \underbrace{\mathbf{V}^{\text{T}}}_{N \times N} \end{equation}" />



<img src="https://latex.codecogs.com/svg.image?H^{*}" title="https://latex.codecogs.com/svg.image?H^{*}" /> is the last column of <img src="https://latex.codecogs.com/svg.image?V" title="https://latex.codecogs.com/svg.image?V" />



# OpenCV API

```cpp
cv::getPerspectiveTransform

cv::warpPerspective

cv::perspectiveTransform

cv::findHomography
```

[code](../src/homography.cpp)

