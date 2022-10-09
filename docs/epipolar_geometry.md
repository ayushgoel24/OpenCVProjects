# Epipolar Geometry


When a 3D scene projected into two cameras views , there are a number of geometric relations between the 3D points and their projections that lead to constraints between the image points.

<img src="images/Epipolar_geometry.svg">


1. <img src="https://latex.codecogs.com/svg.image?O_L" title="https://latex.codecogs.com/svg.image?O_L" /> and <img src="https://latex.codecogs.com/svg.image?O_R" title="https://latex.codecogs.com/svg.image?O_R" /> represent the optical centers of cameras.

2. <img src="https://latex.codecogs.com/svg.image?X" title="https://latex.codecogs.com/svg.image?X" /> represents a 3D point.

3. The line connecting <img src="https://latex.codecogs.com/svg.image?O_L" title="https://latex.codecogs.com/svg.image?O_L" /> and <img src="https://latex.codecogs.com/svg.image?O_R" title="https://latex.codecogs.com/svg.image?O_R" /> called **baseline**. 


4. Points <img src="https://latex.codecogs.com/svg.image?x_L" title="https://latex.codecogs.com/svg.image?x_L" /> and <img src="https://latex.codecogs.com/svg.image?x_R" title="https://latex.codecogs.com/svg.image?x_R" /> are the projections of point X onto the image planes.

5. The projection of optical centers of each the cameras lenses onto  the other camera's image plane denoted by <img src="https://latex.codecogs.com/svg.image?e_L" title="https://latex.codecogs.com/svg.image?e_L" /> and <img src="https://latex.codecogs.com/svg.image?e_R" title="https://latex.codecogs.com/svg.image?e_R" />, are called **epipoles** or **epipolar points**. The Baseline contact camera image plane on epipolar points.

6. All the points lying on the line <img src="https://latex.codecogs.com/svg.image?O_L&space;-&space;X" title="https://latex.codecogs.com/svg.image?O_L - X" /> are seen by the left camera as a point, However, the right camera sees this line as a line in its image plane. That line <img src="https://latex.codecogs.com/svg.image?e_R&space;-&space;x_R" title="https://latex.codecogs.com/svg.image?e_R - x_R" /> in the right camera is called an **epipolar line**. An epipolar line is a function of the position of point <img src="https://latex.codecogs.com/svg.image?X" title="https://latex.codecogs.com/svg.image?X" /> in the 3D space.

7.  <img src="https://latex.codecogs.com/svg.image?X,&space;O_L&space;\text{&space;and&space;}&space;O_R" title="https://latex.codecogs.com/svg.image?X, O_L \text{ and } O_R" /> form a plane that is called the **epipolar plane**. The epipolar plane intersects each camera's image plane where it forms the epipolar lines.


# Epipolar Constraint and Triangulation

If the points <img src="https://latex.codecogs.com/svg.image?x_L&space;\text{&space;and&space;}&space;x_R" title="https://latex.codecogs.com/svg.image?x_L \text{ and } x_R" /> are known, their projection lines can be found via camera matrix. These lines should intersect precisely at <img src="https://latex.codecogs.com/svg.image?X" title="https://latex.codecogs.com/svg.image?X" />. This process called **triangulation**.

