

/*Blob detection

Laplacian of Gaussian (LoG)
Difference of Gaussians (DoG)
Determinant of Hessian (DoH)
Maximally stable extremal regionsPCBR
*/
//https://en.wikipedia.org/wiki/Blob_detection


//void SmoothingBlurFilter(cv::Mat src,cv::Mat& dst)
//{
////	http://docs.opencv.org/2.4.2/doc/tutorials/imgproc/gausian_median_blur_bilateral_filter/gausian_median_blur_bilateral_filter.html
//	int DELAY_BLUR = 100;
//	int MAX_KERNEL_LENGTH = 31;
//	int DELAY_CAPTION = 1500;
////	Point(-1, -1): Indicates where the anchor point (the pixel evaluated) is located with respect to the neighborhood.
////	If there is a negative value, then the center of the kernel is considered the anchor point.

//	namedWindow( "Filter Demo", CV_WINDOW_AUTOSIZE );
//	CppStyeAddTextToImage(src, "Original Image" );


//	imshow( "Filter Demo", dst );
//	waitKey ( DELAY_CAPTION);


//	/// Applying Homogeneous blur
//	CppStyeAddTextToImage(src, "Homogeneous Blur (Average Fiter)");

//	for ( int i = 1; i < MAX_KERNEL_LENGTH; i = i + 2 )
//	{
//		blur( src, dst, Size( i, i ), Point(-1,-1) );
//		imshow( "Filter Demo", dst );
//		waitKey ( DELAY_BLUR);
//	}

//	/// Applying Gaussian blur
//	CppStyeAddTextToImage(src, "Gaussian Blur" ) ;

//	for ( int i = 1; i < MAX_KERNEL_LENGTH; i = i + 2 )
//	{
////		sigmax The standard deviation in x. 0 Writing  implies that  is calculated using kernel size.
////		sigmay The standard deviation in y. 0 Writing  implies that  is calculated using kernel size.

////		c style:
////		cvSmooth( img, out, CV_GAUSSIAN, 11, 11 ); kernel size is 11x11
//		GaussianBlur( src, dst, Size( i, i ), 0, 0 );
//		imshow( "Filter Demo", dst );
//		waitKey ( DELAY_BLUR);
//	}

//	/// Applying Median blur
//	CppStyeAddTextToImage(src, "Median Blur" );

//	for ( int i = 1; i < MAX_KERNEL_LENGTH; i = i + 2 )
//	{
//		medianBlur ( src, dst, i );
//		imshow( "Filter Demo", dst );
//		waitKey ( DELAY_BLUR);
//	}

//	/// Applying Bilateral Filter
//	CppStyeAddTextToImage(src, "Bilateral Blur" );

//	for ( int i = 1; i < MAX_KERNEL_LENGTH; i = i + 2 )
//	{
//		bilateralFilter ( src, dst, i, i*2, i/2 );
//		imshow( "Filter Demo", dst );
//		waitKey ( DELAY_BLUR);
//	}

//	/// Wait until user press a key
//	CppStyeAddTextToImage(src, "End: Press a key!" );

//	waitKey(0);
//	return;
//}

//void SmoothingBlurFilter_Test(char ** argv)
//{
//	cv::Mat src;
//	src=cv::imread(argv[1],1);
//	cv::Mat dst;
//	dst=src.clone();
//	SmoothingBlurFilter(src,dst);
//}

// Laplacian is based on second derivative 
//void LaplacianOfGaussiansEdgeDetector(char ** argv)
//{
//    Mat src, src_gray, dst;
//    int kernel_size = 3;
//    int scale = 1;
//    int delta = 0;
////  ddepth: Depth of the destination image. Since our input is CV_8U we define ddepth = CV_16S to avoid overflow
//    int ddepth = CV_16S;
//    char* window_name = "Laplace Demo";

//    int c;

//    /// Load an image
//    src = imread( argv[1] );

//    if( !src.data )
//    { return ; }

//    /// Remove noise by blurring with a Gaussian filter
//    GaussianBlur( src, src, Size(3,3), 0, 0, BORDER_DEFAULT );

//    /// Convert the image to grayscale
//    cvtColor( src, src_gray, CV_RGB2GRAY );

//    /// Create window
//    namedWindow( window_name, CV_WINDOW_AUTOSIZE );

//    /// Apply Laplace function
//    Mat abs_dst;

//    Laplacian( src_gray, dst, ddepth, kernel_size, scale, delta, BORDER_DEFAULT );
//    convertScaleAbs( dst, abs_dst );

//    /// Show what you got
//    imshow( window_name, abs_dst );

//    waitKey(0);
//}

//DoG is approximation of Log which is based on second derivative
//void DifferenceOfGaussians (IplImage* src, IplImage* dst, int kernel1, int kernel2, int invert)
//{

//  // Difference-Of-Gaussians (DOG) works by performing two different Gaussian blurs on the image,

//  // with a different blurring radius for each, and subtracting them to yield the result.

//  //   http://en.wikipedia.org/wiki/Difference_of_Gaussians

//  //   http://docs.gimp.org/en/plug-in-dog.html

//  IplImage *dog_1 = cvCreateImage(cvGetSize(src), src->depth, src->nChannels);

//  IplImage *dog_2 = cvCreateImage(cvGetSize(src), src->depth, src->nChannels);

//  cvSmooth(src, dog_2, CV_GAUSSIAN, kernel1, kernel1); // Gaussian blur

//  cvSmooth(src, dog_1, CV_GAUSSIAN, kernel2, kernel2);

//  cvSub(dog_2, dog_1, dst, 0);
//  return;

//}


int main()
{
    return 0;
}