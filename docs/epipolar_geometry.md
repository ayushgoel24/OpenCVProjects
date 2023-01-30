# Epipolar Geometry


When a 3D scene projected into two cameras views , there are a number of geometric relations between the 3D points and their projections that lead to constraints between the image points.

<img src="images/Epipolar_geometry.svg">


1. <img src="https://latex.codecogs.com/svg.image?O_L" title="https://latex.codecogs.com/svg.image?O_L" /> and <img src="https://latex.codecogs.com/svg.image?O_R" title="https://latex.codecogs.com/svg.image?O_R" /> represent the optical centers of cameras.

2. <img src="https://latex.codecogs.com/svg.image?X" title="https://latex.codecogs.com/svg.image?X" /> represents a 3D point.

3. The line connecting <img src="https://latex.codecogs.com/svg.image?O_L" title="https://latex.codecogs.com/svg.image?O_L" /> and <img src="https://latex.codecogs.com/svg.image?O_R" title="https://latex.codecogs.com/svg.image?O_R" /> called **baseline**. 


4. Points <img src="https://latex.codecogs.com/svg.image?x_L" title="https://latex.codecogs.com/svg.image?x_L" /> and <img src="https://latex.codecogs.com/svg.image?x_R" title="https://latex.codecogs.com/svg.image?x_R" /> are the projections of point X onto the image planes.






5. The projection of the different points lying on the line <img src="https://latex.codecogs.com/svg.image?O_L&space;-&space;X" title="https://latex.codecogs.com/svg.image?O_L - X" /> are seen by the left camera as a point, However, the right camera sees these points as a line in its image plane. That line <img src="https://latex.codecogs.com/svg.image?l^{%27}" alt="https://latex.codecogs.com/svg.image?l^{'}" /> (<img src="https://latex.codecogs.com/svg.image?e_R&space;-&space;x_R" title="https://latex.codecogs.com/svg.image?e_R - x_R" />)  in the right camera is called an **epipolar line** or **epiline**. We call it epiline corresponding to the point <img src="https://latex.codecogs.com/svg.image?x_l" alt="https://latex.codecogs.com/svg.image?x_l" /> It means, to find the point <img src="https://latex.codecogs.com/svg.image?x" alt="https://latex.codecogs.com/svg.image?x" /> on the right image, search along this epiline. It should be somewhere on this line. An epipolar line is a function of the position of point <img src="https://latex.codecogs.com/svg.image?X" title="https://latex.codecogs.com/svg.image?X" /> in the 3D space.

5. The projection of optical centers of each the cameras lenses onto  the other camera's image plane denoted by <img src="https://latex.codecogs.com/svg.image?e_L" title="https://latex.codecogs.com/svg.image?e_L" /> and <img src="https://latex.codecogs.com/svg.image?e_R" title="https://latex.codecogs.com/svg.image?e_R" />, are called **epipoles** or **epipolar points**. The Baseline contact camera image plane on epipolar points. All the epilines pass through its epipole. So to find the location of epipole, we can find many epilines and find their intersection point.. In some cases you won't be able to locate the epipole in the image, they may be outside the image (which means, one camera doesn't see the other).

7.  <img src="https://latex.codecogs.com/svg.image?X,&space;O_L&space;\text{&space;and&space;}&space;O_R" title="https://latex.codecogs.com/svg.image?X, O_L \text{ and } O_R" /> form a plane that is called the **epipolar plane**. The epipolar plane intersects each camera's image plane where it forms the epipolar lines.


## Essential Matrix
contains the information about translation and rotation, which describe the location of the second camera relative to the first in global coordinates.


<img src="https://latex.codecogs.com/svg.image?\mathbf{E}=\left&space;\lfloor&space;T&space;\right&space;\rfloor_{3\times&space;3}&space;&space;&space;&space;&space;&space;R_{3\times&space;3}" title="https://latex.codecogs.com/svg.image?\mathbf{E}=\left \lfloor T \right \rfloor_{3\times 3} R_{3\times 3}" />
<br/>
<br/>

## Fundamental Matrix 
Fundamental Matrix contains the same information as Essential Matrix in addition to the information about the intrinsics of both cameras so that we can relate the two cameras in pixel coordinates. 
If we are using **rectified** images and **normalize the point** by dividing by the focal lengths, <img src="https://latex.codecogs.com/svg.image?F=E" alt="https://latex.codecogs.com/svg.image?F=E" />. In simple words, Fundamental Matrix F, maps a point in one image to a line (epiline) in the other image. This is calculated from matching points from both the images.

In <img src="https://latex.codecogs.com/svg.image?F" title="https://latex.codecogs.com/svg.image?F" /> if we set a point on the left camera <img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}&space;u&&space;v&space;&&space;1&space;\\\end{bmatrix}" title="https://latex.codecogs.com/svg.image?\begin{bmatrix} u& v & 1 \\\end{bmatrix}" /> it will give us a line on the other camera.
<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}u&space;&&space;v&space;&&space;1&space;\\\end{bmatrix}\begin{bmatrix}f_{11}&space;&&space;f_{12}&space;&&space;f_{13}&space;\\f_{21}&space;&&space;f_{22}&space;&&space;f_{23}&space;\\f_{31}&space;&&space;f_{32}&space;&&space;f_{33}&space;\\\end{bmatrix}\begin{bmatrix}u'&space;\\v'&space;\\1\end{bmatrix}=0" title="https://latex.codecogs.com/svg.image?\begin{bmatrix}u & v & 1 \\\end{bmatrix}\begin{bmatrix}f_{11} & f_{12} & f_{13} \\f_{21} & f_{22} & f_{23} \\f_{31} & f_{32} & f_{33} \\\end{bmatrix}\begin{bmatrix}u' \\v' \\1\end{bmatrix}=0" />



# Epipolar Constraint and Triangulation

If the points <img src="https://latex.codecogs.com/svg.image?x_L&space;\text{&space;and&space;}&space;x_R" title="https://latex.codecogs.com/svg.image?x_L \text{ and } x_R" /> are known, their projection lines can be found via camera matrix. These lines should intersect precisely at <img src="https://latex.codecogs.com/svg.image?X" title="https://latex.codecogs.com/svg.image?X" />. This process called **triangulation**.



<br/>
