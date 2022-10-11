# Image Resolution and Calibration Parameter

## Distortion Coefficients
The distortion coefficients <img src="https://latex.codecogs.com/svg.image?k_1,k_2,p_1,p_2,k_3,k_4,k_5,k_6" title="https://latex.codecogs.com/svg.image?k_1,k_2,p_1,p_2,k_3,k_4,k_5,k_6" /> 
do not depend on the scene viewed and they remain the **same** regardless of image resolution. If, for example, a camera has been calibrated on images of 320 x 240 resolution, absolutely the same distortion coefficients can be used for 640 x 480 images from the same camera

However, <img src="https://latex.codecogs.com/svg.image?f_x" title="https://latex.codecogs.com/svg.image?f_x" />, <img src="https://latex.codecogs.com/svg.image?f_y" title="https://latex.codecogs.com/svg.image?f_y" />, <img src="https://latex.codecogs.com/svg.image?c_x" title="https://latex.codecogs.com/svg.image?c_x" />, and <img src="https://latex.codecogs.com/svg.image?c_y" title="https://latex.codecogs.com/svg.image?c_y" /> need to be scaled appropriately.



## Focal Point




Refs: [1](https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#camera-calibration-and-3d-reconstruction), [2](https://dsp.stackexchange.com/questions/6055/how-does-resizing-an-image-affect-the-intrinsic-camera-matrix)
