#include <opencv4/opencv2/opencv.hpp>

/*
Histogram Matching: 
    Creating new image which has new distribution function (pdf)
Histogram Equalization:
    Creating new image which has new uniform distribution

Contrast Stretching =HistogramNormalization= Histogram Stretching
*/
int main(int argc, char** argv)
{
/*    
    normalization of a set of data "S" is mapping the data to a new range i.e. [0,1]
    It's done in following steps:
    
    A=min(s)
    B=max(s)
    
    a=min value for new range
    b=max value for new range
    
    then for every x in S
    new_x=a+ (b-a)(x-A)/(B-A)
    
    to check the equation just put A and B in the equation and check if new value for A would be a and also new value for B would be b
    
    now this will strech our histogram  linearly, suppose we have an image which has very small range for colors for instance we have only 
    values between 100 and 150, if we use normalization, we set new max and mean into 255 and 0 so we sterch the contrast
    
    
    Gamma correction
    
    the formula for normalization sometime is written as:
    new_x=a+ (b-a)*[(x-A)/(B-A)]^Gamma
    
    becasue always 
    0< (x-A)/(B-A) < 1 
    so if Gamma is biger than one then the result became smaller and if Gamma if smaller than 1 it becames beigger
*/

/*
 
http://docs.opencv.org/modules/core/doc/operations_on_arrays.html?highlight=normalize#normalize
 
normalize(InputArray src, OutputArray dst, double alpha=1, double beta=0, int norm_type=NORM_L2, int dtype=-1, InputArray mask=noArray() )
    
    norm_type:
	NORM_INF=1, b=max=1, a=min=0 
	NORM_L1=2, we divide all values to W and W is sum of all pixels
	NORM_L2=4, we divide all values to W and W is sum of all (pixels power two)
	NORM_TYPE_MASK=7, 
	NORM_RELATIVE=8, 
	NORM_MINMAX=32
    
    
*/

    cv::Mat src_img= cv::imread(argv[1]);
    cv::Mat dst_img_NORM_L2;
    cv::Mat dst_img_NORM_L1;
    
    cv::Mat dst_img;
    src_img.copyTo(dst_img); 
    std::vector<cv::Mat> bgr_planes;
    split( src_img, bgr_planes );
    
    src_img.copyTo(dst_img_NORM_L2);
    src_img.copyTo(dst_img_NORM_L1);
    cv::normalize(src_img,dst_img_NORM_L2);
    cv::normalize(src_img,dst_img_NORM_L1);
   
    
    cv::normalize(bgr_planes[0],bgr_planes[0]);
    cv::normalize(bgr_planes[1],bgr_planes[1]);
    cv::normalize(bgr_planes[2],bgr_planes[2]);
    
    
    cv::merge(bgr_planes,dst_img);
    
//     imshow("NORM_L2",dst_img_NORM_L2);
//     imshow("NORM_L1",dst_img_NORM_L1);
    cv::imshow("Original",src_img);
    cv::imshow ("Normalized",dst_img);
    cv::waitKey(0);
}

//void HistogramEqualization_Test(int argc, char** argv)
//{

    
//    cv::Mat src_img= imread(argv[1],CV_LOAD_IMAGE_GRAYSCALE );
//    cv::Mat equalized_img;
//    src_img.copyTo(equalized_img);
//    cv::equalizeHist(src_img,equalized_img);
//    imshow("Equlized Histogram Image",equalized_img);
//    imshow("Original Image",src_img);

//    waitKey(0);
    
//}

//void CalculateHistogram(Mat src)
//{


//    if( !src.data )
//    { return ; }

//    /// Separate the image in 3 places ( B, G and R )
//    vector<Mat> bgr_planes;
//    split( src, bgr_planes );

//    /// Establish the number of bins
//    int histSize = 256;

//    /// Set the ranges ( for B,G,R) )
//    float range[] = { 0, 256 } ;
//    const float* histRange = { range };

//    bool uniform = true; bool accumulate = false;

//    Mat b_hist, g_hist, r_hist;

//    /// Compute the histograms:
//    calcHist( &bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
//    calcHist( &bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
//    calcHist( &bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );

//    // Draw the histograms for B, G and R
//    int hist_w = 512; int hist_h = 400;
//    int bin_w = cvRound( (double) hist_w/histSize );

//    Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

//    /// Normalize the result to [ 0, histImage.rows ]
//    normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
//    normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
//    normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

//    /// Draw for each channel
//    for( int i = 1; i < histSize; i++ )
//    {
//	line( histImage, Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,
//			Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
//			Scalar( 255, 0, 0), 2, 8, 0  );
//	line( histImage, Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ) ,
//			Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
//			Scalar( 0, 255, 0), 2, 8, 0  );
//	line( histImage, Point( bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ) ,
//			Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
//			Scalar( 0, 0, 255), 2, 8, 0  );
//    }

//    /// Display
//    namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE );
//    imshow("calcHist Demo", histImage );

//    waitKey(0);
    
//}

//void CalculateHistogram_Test(int argc, char** argv)
//{
//    Mat src;
//    src = imread( argv[1], 1 );
//    CalculateHistogram(src);
//}

