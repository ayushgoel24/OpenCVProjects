#include <opencv4/opencv2/opencv.hpp>

//https://docs.opencv.org/3.4/d9/d61/tutorial_py_morphological_ops.html

void dialation()
{
//	//	Smoothing
//		char* ImagePath="/home/behnam/Documents/Courses stuff/Robotics_II_-_Visually_guided_robots-Documents/Slides/OpenCV_Exercise3/TestImage01L.png";
//		IplImage* src=cvLoadImage(ImagePath,CV_LOAD_IMAGE_COLOR) ;
//		IplImage* smoothed=cvCreateImage(cvGetSize(src),src->depth,src->nChannels);
//
//		cvSmooth(src,smoothed,CV_GAUSSIAN,7);
//		cvNamedWindow("src",CV_WINDOW_AUTOSIZE);
//		cvShowImage("src",src);
//
//		cvNamedWindow("smoothed",CV_WINDOW_AUTOSIZE);
//		cvShowImage("smoothed",smoothed);
//
//
//		IplImage* Dilated=cvCreateImage(cvGetSize(src),src->depth,src->nChannels);
//	//	In the NULL case, the kernel used is a 3-by-3 kernel with the anchor at its center
//		cvDilate(src,Dilated,NULL,1);
//
//		cvNamedWindow("Dilated",CV_WINDOW_AUTOSIZE);
//		cvShowImage("Dilated",Dilated);
//
//
//		IplImage* Eroded=cvCreateImage(cvGetSize(src),src->depth,src->nChannels);
//		cvErode(src,Eroded,NULL,1);
//
//		cvNamedWindow("Eroded",CV_WINDOW_AUTOSIZE);
//		cvShowImage("Eroded",Eroded);
//
//
//
//
//
//		cvWaitKey(0);
}

void Erosion()
{

}

//void Thresholding(int threshold_type,double Threshold, double MaxValue ,IplImage *src, IplImage * &dst )
//{
////	http://opencv.willowgarage.com/documentation/c/miscellaneous_image_transformations.html
///*
//	cvThreshold: Applies fixed-level threshold to 8 bit grayscale image.
//	The parameter CV_THRESH_BINARY will allow the user to specify a cut off value for pixels between 0 and 255.
//*/



///*
//	Parameters:

//	src: Source 8-bit single-channel image
//	dst: Destination image; will have the same size and the same type as src
//	threshold: Threshold value
//	thresholdType: Thresholding type  must be one of
//		1)CV_THRESH_BINARY
//		2)CV_THRESH_BINARY_INV
//		3)CV_THRESH_TRUNC
//		4)CV_THRESH_TOZERO
//		5)CV_THRESH_TOZERO_INV
//		6)CV_THRESH_MASK
//		7)CV_THRESH_OTSU ==> In this case the function determines the optimal threshold value using Otsu’s algorithm
//							 and iginore specified threshold.

//	MaxValue: Maximum value to use with CV_THRESH_BINARY, CV_THRESH_BINARY_INV, and CV_THRESH_TRUNC thresholding types.
//*/

///*
// 	thresholdType=CV_THRESH_BINARY:
//	dst(x,y) = maxValue, if src(x,y)>threshold
//	           0, otherwise

//	thresholdType=CV_THRESH_BINARY_INV:
//	dst(x,y) = 0, if src(x,y)>threshold
//	           maxValue, otherwise

//	thresholdType=CV_THRESH_TRUNC:
//	dst(x,y) = threshold, if src(x,y)>threshold
//	           src(x,y), otherwise

//	thresholdType=CV_THRESH_TOZERO:
//	dst(x,y) = src(x,y), if (x,y)>threshold
//	           0, otherwise

//	thresholdType=CV_THRESH_TOZERO_INV:
//	dst(x,y) = 0, if src(x,y)>threshold
//	           src(x,y), otherwise
//*/

//	cvThreshold(src,dst,Threshold, MaxValue,threshold_type);
//}
int main()
{

}