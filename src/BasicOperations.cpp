#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d/calib3d.hpp>



#include <iostream>
#include <stdio.h>


using namespace std;
using namespace cv;

void CppStyleCreateMatrix()
{
    //CV_<bit_depth>(S|U|F)C<number_of_channels>
	//elements type (uchar,short,int,float,double)
	//CV_8UC1 means an 8-bit unsigned single channel
	//CV_32FC3 means a 32-bit float matrix with three
    cv::Mat D33 = Mat::zeros(3, 3, CV_32FC1) + 5.;
    Mat zeromatrix= Mat::zeros(21,3,CV_32FC1)+2.5 ;
    cout<<zeromatrix <<endl;
    cout<<"Total Number of Elements: " <<zeromatrix.total() <<endl;
    
    
    //	create matrix in several step;
    cv::Mat MatrixofRawData;
    int NumberofRows=2;
    int NumberofCols=3;
    MatrixofRawData.create(NumberofRows,NumberofCols,CV_32FC1);
    cout<<"MatrixofRawData: " <<MatrixofRawData<<endl;

}

void CppStyleMatrixAccessingPixel()
{
    int rows,columns;
    rows=10;
    columns=20;

    cv::Mat my_mat(rows, columns, CV_8UC4); //CV_8UC4 means unsinged char (8 bit 0-255) and 4 channel, so to access every pixel we use cv::Vec4b

    cv::Vec4b pixel = my_mat.at<cv::Vec4b>(cv::Point(3, 4));
    uchar xValue = my_mat.at<cv::Vec4b>(cv::Point(3, 4))[0];





}

void CppStyleMatrixVectorInterface()
{
    cv::Mat MatrixofRawData;
    int NumberofRows=2;
    int NumberofCols=3;
    MatrixofRawData.create(NumberofRows,NumberofCols,CV_32FC1);
       
    std::vector<float> RowVector;
    RowVector.push_back(0.2);
    RowVector.push_back(1.2);
    RowVector.push_back(4.2);
    MatrixofRawData.push_back(RowVector);
    //MatrixofRawData.convertTo();
    Mat MatFromVector(RowVector , true);
    cout<<"MatFromVector: "<< MatFromVector <<endl;    
}

void CppStyleMatrixOperations()
{
//http://www.aishack.in/2010/07/opencvs-c-interface/
    // matrix operation: multipication, add, subtraction, transpose
    Mat m1 = Mat::zeros(2, 3, CV_32FC1);
    Mat m2 = Mat::ones(3, 2, CV_32FC1);
    m1.at<float>(0, 0) = 2.0f;

    cout<<"m1" <<endl;
    cout<<m1 <<endl;

    cout<<"m2"<<endl;
    cout<<m2 <<endl;


    cout << "(m2*2)" << endl;
    cout << (m2*2) << endl;

    cout << "m2*m1" << endl;
    cout << m2*m1 << endl;

    cout << "m1+m2.t()" << endl;
    cout << m1+m2.t() << endl;

    cout << "m1-m2.t()" << endl;
    cout << m1-m2.t() << endl;



}

void drawingFunsction()
{
    cv::Mat img = Mat::zeros(400,600, CV_32FC1);
    cv::namedWindow("WorkingwitDrawingcommands",cv::WINDOW_AUTOSIZE);
    //Draw a box:
    // draw a box with red lines of width 1 between (100,100) and (200,200)
    cv::rectangle(img, cv::Point(300,100), cv::Point(200,200), cv::Scalar(255,0,0), 1);

    //	Draw a circle:

    // draw a circle at (300,300) with a radius of 20. Use green lines of width 1
    //cvCircle(img, cvPoint( 351.90,101.83), 20, cvScalar(0,255,0), 1);

    cv::circle(img, cv::Point( 350.43,100.23), 20, cv::Scalar(0,255,0), 1);

    //Draw a line segment:

    // draw a green line of width 1 between (100,100) and (200,200)
    cv::line(img, cv::Point(100,100), cv::Point(200,200), cv::Scalar(0,255,0), 1);

    //Draw a set of polylines:
/*
    image's points in opencv has the follow index:
	(0,0) (1,0) (2,0) 3,0)
	(0,1) (1,1) (2,1) 3,1)
	(0,2) (1,2) (2,2) 3,2)

            X           (cols,0)
            -------------►
            |
          y |
            |
    (0,rows)▼           (cols,rows)

*/
    cv::Point  curve1[]={cv::Point(10,10),  cv::Point(10,100),  cv::Point(100,100),  cv::Point(100,10)};
    cv::Point  curve2[]={cv::Point(30,30),  cv::Point(30,130),  cv::Point(130,130),  cv::Point(130,30),  cv::Point(150,10)};
    const cv::Point* curveArr[2]={curve1, curve2};
    int      nCurvePts[2]={4,5};
    int      nCurves=2;
    int      isCurveClosed=1;
    int      lineWidth=1;
    cv::polylines(img,curveArr,nCurvePts,nCurves,isCurveClosed,cv::Scalar(0,255,255),lineWidth);

    //Draw a set of filled polygons:

    cv::fillPoly(img,curveArr,nCurvePts,nCurves,cv::Scalar(0,255,255));

    //Add text:

    double fontScale=1.0;
    cv::putText  (img,"My comment",cv::Point(200,400), FONT_HERSHEY_SIMPLEX,fontScale ,cv::Scalar(255,255,0));
    //	Other possible fonts:
    //
    //	CV_FONT_HERSHEY_SIMPLEX, CV_FONT_HERSHEY_PLAIN,
    //	CV_FONT_HERSHEY_DUPLEX, CV_FONT_HERSHEY_COMPLEX,
    //	CV_FONT_HERSHEY_TRIPLEX, CV_FONT_HERSHEY_COMPLEX_SMALL,
    //	CV_FONT_HERSHEY_SCRIPT_SIMPLEX, CV_FONT_HERSHEY_SCRIPT_COMPLEX,


    cv::imshow("WorkingwitDrawingcommands",img);
    cv::waitKey(0);
}

void CppStyleResizeImage(cv::Mat src ,cv::Mat dst,float scale)
{
	
	cv::Size new_size;
	new_size.height=src.rows/scale;
	new_size.width=src.cols/scale;
	cv::resize(src,dst,new_size,INTER_LINEAR);
}

void CppStyleReadImage()
{
    std::string PathToImageFile="images/lena.jpg";
    cv::Mat img= imread( PathToImageFile);
}

void CppStyleDisplay(int argc, char **argv)
{

    
    cv::Mat img= imread( argv[1]);
    imshow(std::string(argv[1]), img);
    while(true)
    {
        if( (char)cv::waitKey(30)== (char)27)
            return;

    }

}

void CppStyleSaveImage()
{
    std::string PathToImageFile="images/lena.jpg";
    std::string filename="lena-backup.jpg";
    cv::Mat img= imread( PathToImageFile);
    imwrite( filename, img);
}

//void CppStyleDiffImage()
//{
//    Mat im = imread("cameraman.tif");
//    Mat im2 = imread("lena.tif");

//    Mat diff_im = im - im2;
//  // use first camera attached to computer
//  CvCapture *capture;
//  capture = cvCaptureFromCAM( 0 );
//  assert( capture );

//  // image data structures
//  IplImage *img1;
//  IplImage *img2;
//  IplImage *imggray1;
//  IplImage *imggray2;
//  IplImage *imggray3;

//  // get the camera image size
//  IplImage *imgsize;
//  imgsize = cvQueryFrame( capture );
//  if( !imgsize ) return ;

//  // grayscale buffers
//  imggray1 = cvCreateImage( cvGetSize( imgsize ), IPL_DEPTH_8U, 1);
//  imggray2 = cvCreateImage( cvGetSize( imgsize ), IPL_DEPTH_8U, 1);
//  imggray3 = cvCreateImage( cvGetSize( imgsize ), IPL_DEPTH_8U, 1);

//  int key = 0;
//  while ( key != 'q' ) {
//    // load image one
//    img1 = cvQueryFrame( capture );

//   // convert rgb to grayscale
//    cvCvtColor( img1, imggray1, CV_RGB2GRAY );

//    // quit if user press 'q' and wait a bit between images
//    key = cvWaitKey( 500 );

//    // load image two
//    img2 = cvQueryFrame( capture );

//    // convert rgb to grayscale
//    cvCvtColor( img2, imggray2, CV_RGB2GRAY );

//    // compute difference
//    cvAbsDiff( imggray1, imggray2, imggray3 );

//    // display difference
//    cvNamedWindow( "video", 1 );
//    cvShowImage( "video", imggray3 );
//  }

//  // release camera and clean up resources when "q" is pressed
//  cvReleaseCapture( &capture );
//  cvDestroyWindow( "video" );
//  return;
//}

//void Kalman_Filter()
//{
//    cvNamedWindow( "Kalman", 1 );
//    CvRandState rng;
//    double LowerBoundaryOfUniformDistribution=0;
//    double UpperBoundaryOfUniformDistribution=1;
//    int seed;

///*
//	cvRandInit( CvRandState* state, double param1, double param2, int seed, int distType=CV_RAND_UNI )

//	parameters:
//	state: Pointer to the initialized random number generator state structure.
//	param1:	The first distribution parameter. In case of uniform distribution it is the inclusive lower boundary of random numbers range. In case of normal distribution it is the standard deviation of random numbers.
//	param2: The second distribution parameter. In case of uniform distribution it is the exclusive upper boundary of random numbers range. In case of normal distribution it is the mean value of random numbers.
//	seed: 	Initial 32-bit value to start a random sequence.
//	distType :
//		1)CV_RAND_UNI - uniform distribution
//		2)CV_RAND_NORMAL - normal or Gaussian distribution
//*/

//    cvRandInit( &rng, LowerBoundaryOfUniformDistribution, UpperBoundaryOfUniformDistribution,seed, CV_RAND_UNI );

//    IplImage* img = cvCreateImage( cvSize(500,500), 8, 3 );
//    int NumberofDimensionOfStateSpace=2;//X is NumberofDimensionOfStateSpace x 1
//    int NumberofDimensionOfMeasurment=1 ;//Z is NumberofDimensionOfMeasurment x 1
//    int NumberofDimensionOfContrlParam=0 ;//U is NumberofDimensionOfContrlParam x 1

//    cv::KalmanFilter kalmanfilter( NumberofDimensionOfStateSpace , NumberofDimensionOfMeasurment, NumberofDimensionOfContrlParam, CV_32F);

///*

//    Mat statePre;           // predicted state (x'(k)):
//                            //    x(k)=A*x(k-1)+B*u(k)
//    Mat statePost;          // corrected state (x(k)):
//                            //    x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
//    Mat transitionMatrix;   // state transition matrix (A)
//    Mat controlMatrix;      // control matrix (B)
//                            //   (it is not used if there is no control)
//    Mat measurementMatrix;  // measurement matrix (H)
//    Mat processNoiseCov;    // process noise covariance matrix (Q)
//    Mat measurementNoiseCov;// measurement noise covariance matrix (R)
//    Mat errorCovPre;        // priori error estimate covariance matrix (P'(k)):
//                            //    P'(k)=A*P(k-1)*At + Q)
//    Mat gain;               // Kalman gain matrix (K(k)):
//                            //    K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R)
//    Mat errorCovPost;       // posteriori error estimate covariance matrix (P(k)):
//                            //    P(k)=(I-K(k)*H)*P'(k)
//*/
////    kalmanfilter.


///*
//    The covariance of process noise and of measurement noise are
//    set to reasonable but interesting values (you can play with these yourself), and we ini-
//    tialize the posterior error covariance to the identity as well (this is required to guarantee
//    the meaningfulness of the first iteration; it will subsequently be overwritten)
//*/

///*    RandSetRange
//	Changes the range of generated random numbers without touching RNG state

//	void cvRandSetRange( CvRandState* state, double param1, double param2, int index=-1 );
//	state:
//		State of random number generator (RNG).
//	param1:
//		New lower boundary/deviation of generated numbers.
//	param2:
//		New upper boundary/mean value of generated numbers.
//	index:
//		The 0-based index of dimension/channel for which the parameter are changed, -1 means changing the parameters for all dimensions.
//*/

//	cvRandSetRange( &rng, 0, 0.1, 0 );
//	rng.disttype = CV_RAND_NORMAL;

////	cvRand(&rng,kalmanfilter.statePost);
////	kalmanfilter.statePost

//	cout<<kalmanfilter.statePost.rows <<endl;
//	cout<<kalmanfilter.statePost.cols <<endl;


//	kalmanfilter.measurementMatrix= cv::Mat::zeros(kalmanfilter.measurementMatrix.rows,kalmanfilter.measurementMatrix.rows,CV_32F);

//	cv::Mat transitionMatrix(NumberofDimensionOfStateSpace,NumberofDimensionOfStateSpace,CV_32F);
//	float fps=30;
//	const float DEL_T = 1/fps;
//	transitionMatrix.at<float>(0,0)=1;
//	transitionMatrix.at<float>(0,1)=DEL_T;
//	transitionMatrix.at<float>(1,0)=0;
//	transitionMatrix.at<float>(1,1)=1;
////	cv::Mat M= cv::Mat::eye (2, 2, CV_32F)*(1e-5);
////	cout<<M <<endl;
//	kalmanfilter.measurementMatrix=cv::Mat::eye (kalmanfilter.measurementMatrix.rows, kalmanfilter.measurementMatrix.rows, CV_32F);
//	kalmanfilter.processNoiseCov=cv::Mat::eye (kalmanfilter.processNoiseCov.rows, kalmanfilter.processNoiseCov.rows, CV_32F)*(1e-5);
//	kalmanfilter.measurementNoiseCov=cv::Mat::eye (kalmanfilter.measurementNoiseCov.rows, kalmanfilter.measurementNoiseCov.rows, CV_32F)*(1e-1);
//	kalmanfilter.errorCovPost=cv::Mat::eye (kalmanfilter.errorCovPost.rows, kalmanfilter.errorCovPost.rows, CV_32F);

//	float gaussian_mean=0;
//	float standard_deviation=1;
//	cv::randn(kalmanfilter.statePost,gaussian_mean,standard_deviation);




//    while( 1 )
//    {
//    	kalmanfilter.predict();

////
////        // generate measurement (z_k)
////        //
////        cvRandSetRange(&rng,0, sqrt(kalman->measurement_noise_cov->data.fl[0]), 0);
////        cvRand( &rng, z_k );
////        cvMatMulAdd( kalman->measurement_matrix, x_k, z_k, z_k );
////        // plot points (eg convert to planar co-ordinates and draw)
////        //
////        cvZero( img );
////        cvCircle( img, phi2xy(z_k), 4, CVX_YELLOW );   // observed state
////        cvCircle( img, phi2xy(y_k), 4, CVX_WHITE, 2 ); // "predicted" state
////        cvCircle( img, phi2xy(x_k), 4, CVX_RED );      // real state
////        cvShowImage( "Kalman", img );
////        // adjust Kalman filter state
////        //
////        cvKalmanCorrect( kalman, z_k );
////
////        // Apply the transition matrix 'F' (eg, step time forward)
////        // and also apply the "process" noise w_k.
////        //
////        cvRandSetRange( &rng, 0,sqrt(kalman->process_noise_cov->data.fl[0]),0);
////        cvRand( &rng, w_k );
////        cvMatMulAdd( kalman->transition_matrix, x_k, w_k, x_k );
////
////        // exit if user hits 'Esc'
////        if( cvWaitKey( 100 ) == 27 ) break;
//    }





//    return;
//}

//void HOG(IplImage *src,vector<Rect> &found_filtered)
//{


//    Mat img(src);
//    vector<Rect> found;
//    HOGDescriptor hog;
//    hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
//    hog.detectMultiScale(img, found, 0, Size(8,8), Size(32,32), 1.05, 2);

//    size_t i, j;
//	for (i=0; i<found.size(); i++)
//	{
//	  Rect r = found[i];
//	  for (j=0; j<found.size(); j++)
//		  if (j!=i && (r & found[j]) == r)
//			  break;
//	  if (j== found.size())
//		  found_filtered.push_back(r);
//	}


//}

//void HOG_Test(char **argv)
//{

//	IplImage * src=cvLoadImage(argv[1],CV_LOAD_IMAGE_COLOR);
//	vector<Rect> found_filtered;
//	HOG(src,found_filtered);
//	cv::Mat img(src);
//	for (size_t i=0; i<found_filtered.size(); i++)
//	{
//		Rect r = found_filtered[i];
//		r.x += cvRound(r.width*0.1);
//		r.width = cvRound(r.width*0.8);
//		r.y += cvRound(r.height*0.07);
//		r.height = cvRound(r.height*0.8);
//		rectangle( img, r.tl(), r.br(), Scalar(0,255,0), 3);
//	}

//	cvNamedWindow("HOG",CV_WINDOW_AUTOSIZE);
//	cvShowImage("HOG",src);
//	waitKey(0);
//	cvReleaseImage(&src);
//	cvDestroyWindow("HOG");


//}

//void Moments(	CvMoments &moments,IplImage * img)
//{
///*	class Moments
//	{
//	public:
//	    Moments();
//	    Moments(double m00, double m10, double m01, double m20, double m11,
//	            double m02, double m30, double m21, double m12, double m03 );
//	    Moments( const CvMoments& moments );
//	    operator CvMoments() const;

//	    // spatial moments
//	    double  m00, m10, m01, m20, m11, m02, m30, m21, m12, m03;
//	    // central moments
//	    double  mu20, mu11, mu02, mu30, mu21, mu12, mu03;
//	    // central normalized moments
//	    double  nu20, nu11, nu02, nu30, nu21, nu12, nu03;

//		because:
//			m00=m00
//			mu01=0
//			mu10=0
//	}


//	Parameters:
//	array – Raster image (single-channel, 8-bit or floating-point 2D array) or an array (  or   ) of 2D points (Point or Point2f ).
//	binaryImage – If it is true, all non-zero image pixels are treated as 1’s. The parameter is used for images only.
//	moments – Output moments.

//	*/

///*
//	All of the moments—including spatial and central moments up to the 3rd order calculated and fills moment state structure.
//	void cvMoments(const CvArr* image,CvMoments* moments,	int	isBinary = 0 );

//	double  cvGetSpatialMoment( CvMoments* moments, int x_order, int y_order );//m00, m10, m01, m20, m11, m02, m30, m21, m12, m03

//	double cvGetCentralMoment( 	CvMoments* moments,	int x_order,int y_order );//mu20, mu11, mu02, mu30, mu21, mu12, mu03;

//	double cvGetNormalizedCentralMoment(CvMoments* moments,	int	x_order,int	y_order	);//u20, nu11, nu02, nu30, nu21, nu12, nu03;

//	void cvGetHuMoments(CvMoments* moments,	CvHuMoments* HuMoments);// h1, h2, h3,...
//*/
//}

//void Moments_Test(char ** argv)
//{
////	example of run:	images/gedeck1.jpg
//	IplImage* img=  cvLoadImage(argv[1],CV_LOAD_IMAGE_GRAYSCALE);
//	CvMoments moments;
//	cvMoments( img,&moments,1);
///*
//	All of the moments—including spatial and central moments up to the 3rd order calculated and fills moment state structure.
//	void cvMoments(const CvArr* image,CvMoments* moments,	int	isBinary = 0 );

//	double  cvGetSpatialMoment( CvMoments* moments, int x_order, int y_order );//m00, m10, m01, m20, m11, m02, m30, m21, m12, m03

//	double cvGetCentralMoment( 	CvMoments* moments,	int x_order,int y_order );//mu20, mu11, mu02, mu30, mu21, mu12, mu03;

//	double cvGetNormalizedCentralMoment(CvMoments* moments,	int	x_order,int	y_order	);//u20, nu11, nu02, nu30, nu21, nu12, nu03;

//	void cvGetHuMoments(CvMoments* moments,	CvHuMoments* HuMoments);// h1, h2, h3,...
//*/

//	double  m00, m10, m01, m20, m11, m02, m30, m21, m12, m03;
//	m00 = cvGetSpatialMoment(&moments,0,0);
//	m10 = cvGetSpatialMoment(&moments,1,0);
//	m01 = cvGetSpatialMoment(&moments,0,1);

////	http://opencv.willowgarage.com/wiki/cvBlobsLib#Download

////	cv::findContours()

//}

//void HoughLineTransform(Mat src )
//{
////	src  is in grayscale
//	Mat dst, coloerfull_dst;
//	Canny(src, dst, 50, 200, 3);
////	we turn our dst matrix to colorfull so we can draw red lines
//	cvtColor(dst, coloerfull_dst, CV_GRAY2BGR);

//	#if 0
//		// this vector contains rho and theta
//		vector<Vec2f> lines;

////		dst: Output of the edge detector. It should be a grayscale image (although in fact it is a binary one)
////		lines: A vector that will store the parameters  of the detected lines
////		rho : The resolution of the parameter  in pixels. We use 1 pixel.
////		theta: The resolution of the parameter  in radians. We use 1 degree (CV_PI/180)
////		threshold: The minimum number of intersections to “detect” a line
////		srn and stn: Default parameters to zero. Check OpenCV reference for more info.

//		double rho_resolution_parameter_in_pixels=1;
//		double theta_resolution_in_radian=CV_PI/180;
//		int threshold=100;
//		double srn=0;
//		double stn=0;
//		HoughLines(dst, lines, rho_resolution_parameter_in_pixels, theta_resolution_in_radian, 100, 0, 0 );
//		for( size_t i = 0; i < lines.size(); i++ )
//		{
//			 float rho = lines[i][0], theta = lines[i][1];
//			 Point pt1, pt2;
//			 double a = cos(theta), b = sin(theta);
//			 double x0 = a*rho, y0 = b*rho;
//			 pt1.x = cvRound(x0 + 1000*(-b));
//			 pt1.y = cvRound(y0 + 1000*(a));
//			 pt2.x = cvRound(x0 - 1000*(-b));
//			 pt2.y = cvRound(y0 - 1000*(a));
//			 line( cdst, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
//		}
//	#else
//		vector<Vec4i> lines;
//		/*
//		dst: Output of the edge detector. It should be a grayscale image (although in fact it is a binary one)
//		lines: A vector that will store the parameters  of the detected lines
//		rho : The resolution of the parameter  in pixels. We use 1 pixel.
//		theta: The resolution of the parameter  in radians. We use 1 degree (CV_PI/180)
//		threshold: The minimum number of intersections to “detect” a line
//		minLinLength: The minimum number of points that can form a line. Lines with less than this number of points are disregarded.
//		maxLineGap: The maximum gap between two points to be considered in the same line.
//		*/
//		double rho_resolution_parameter_in_pixels=1;
//		double theta_resolution_in_radian=CV_PI/180;
//		int minimum_number_of_intersections_to_detect_line=50;
//		double minLinLength=50;
//		double maximum_gap_between_two_points_to_be_considered_in_the_same_line=10;

//		HoughLinesP(dst, lines, rho_resolution_parameter_in_pixels, theta_resolution_in_radian,
//				minimum_number_of_intersections_to_detect_line, minLinLength, maximum_gap_between_two_points_to_be_considered_in_the_same_line);
//		for( size_t i = 0; i < lines.size(); i++ )
//		{
//			Vec4i l = lines[i];
//			line( coloerfull_dst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
//		}
//	#endif
//	imshow("source", src);
//	imshow("detected lines", coloerfull_dst);

//	waitKey();


//}

//void HoughLineTransform_Test(char ** argv)
//{
//	Mat src = imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE );
//	HoughLineTransform(src );
//}

//void HoughTransformCircle(IplImage *img_src)
//{
//	Mat src(img_src,false);
//	Mat src_gray;

////	Convert it to gray
//	cvtColor( src, src_gray, cv::COLOR_BGR2GRAY );

////	Convert it to gray
//	cvtColor( src, src_gray, cv::COLOR_BGR2GRAY );

////	Reduce the noise so we avoid false circle detection
//	GaussianBlur( src_gray, src_gray, Size(9, 9), 2, 2 );

//	vector<Vec3f> circles;
//	int UpperthresholdforCannyEdgeDetector=10;
////	Apply the Hough Transform to find the circles
////	src_gray: Input image (grayscale)
////	circles: A vector that stores sets of 3 values:  for each detected circle.
////	CV_HOUGH_GRADIENT: Define the detection method. Currently this is the only one available in OpenCV
////	dp = 1: The inverse ratio of resolution
////	min_dist = src_gray.rows/8: Minimum distance between detected centers
////	param_1 = 200: Upper threshold for the internal Canny edge detector
////	param_2 = 100*: Threshold for center detection.
////	min_radius = 0: Minimum radio to be detected. If unknown, put zero as default.
////	max_radius = 0: Maximum radius to be detected. If unknown, put zero as default
//	HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, UpperthresholdforCannyEdgeDetector, 100, 0, 0 );

//	/// Draw the circles detected
//	for( size_t i = 0; i < circles.size(); i++ )
//	{
//		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
//		int radius = cvRound(circles[i][2]);
//		// circle center
//		circle( src, center, 3, Scalar(0,255,0), -1, 8, 0 );
//		// circle outline
//		circle( src, center, radius, Scalar(0,0,255), 3, 8, 0 );
//	}

////	Show your results
//	namedWindow( "Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE );
//	imshow( "Hough Circle Transform Demo", src );
//	waitKey(0);
//	return ;
//}

//void HoughTransformCircle_Test(char ** argv)
//{
//	IplImage *img_src= cvLoadImage(argv[1]);
//	HoughTransformCircle( img_src);


//}

//void HomogeniousConversion( const CvMat* src, 	CvMat* 	dst  )
//{
////	if Dim(Mscr)==Dim(dst)
////		When the input dimension Mscr is equal to the output dimension Mdst, the data is simply copied (and, if necessary, transposed).

////	If Dim(Mscr) > Dim(Mdst),
////		then the elements in dst are computed by dividing all but the last elements of the corresponding vector from src by
////		the last element of that same vector (i.e., src is assumed to contain homogeneous coordinates).

////	If Dim(Mscr) < Dim(Mdst)
////		then the points are copied but with a 1 being inserted into the final coordinate of every vector in the dst array
////		(i.e., the vectors in src are extended to homogeneous coordinates).

//	cvConvertPointsHomogenious(  src, 	dst );


//}

//void FloodFill(IplImage * src,IplImage *out,CvPoint seed_point)
//{
////	http://opencv.willowgarage.com/documentation/miscellaneous_image_transformations.html

////	The cvFloodFill() function will color a neighboring pixel if it is within a specified range (loDiff to upDiff) of either the current pixel
////	or the original seedPoint value (depending on the settings of flags)

////	newVal
////	newVal is the value to which colorized pixels are set.
////________________________________________________________________________________________________________

////	loDiff, upDiff
////		loDidd:
////		Maximal lower brightness/color difference between the currently observed pixel and one of its neighbors belonging to the component,
////		or a seed pixel being added to the component. In the case of 8-bit color images it is a packed value

////		upDiff
////		Maximal upper brightness/color difference between the currently observed pixel and one of its neighbors belonging to the component,
////		or a seed pixel being added to the component. In the case of 8-bit color images it is a packed value

////	A pixel will be colorized if its intensity is not less than a colorized neighbor’s intensity minus loDiff
////	and not greater than the colorized neighbor’s intensity plus upDiff.
////	If the flags argument includes CV_FLOODFILL_FIXED_RANGE, then a pixel will be compared to the original seed point
////	rather than to its neighbors.
////________________________________________________________________________________________________________

////	comp
////	If non-NULL, comp is a CvConnectedComp structure that will hold statistics about the areas filled.
////________________________________________________________________________________________________________

////	mask
////	The argument mask indicates a mask that can function both as:
////	1)input to cvFloodFill() (in this case it constrains the regions that can be filled)
////	2)output from cvFloodFill() (in this case it will indicate the regions that actually were filled).
////	If set to a non-NULL value, then mask must be a one-channel, 8-bit image whose size is exactly two pixels
////	larger in width and height than the source image (this is to make processing easier and
////	faster for the internal algorithm). Pixel (x + 1, y + 1) in the mask image corresponds
////	to image pixel (x, y) in the source image.
////________________________________________________________________________________________________________

////	Note that cvFloodFill() will not flood across  nonzero pixels in the mask,
////	so you should be careful to zero it before use if you don’t want masking to block the flooding operation.
////	Flood fill can be set to colorize either the source image img or the mask image mask.


////	flags:
////	flags argument has three parts:

////	1)The low 8 bits (0–7) can be set to 4 or 8. This controls the connectivity considered by the filling algorithm.
////		If set to 4, only horizontal and vertical neighbors to the current pixel are considered in the filling process;
////		if set to 8, flood fill will additionally include diagonal neighbors.
////	2)The high 8 bits (16–23) can be set with the flags
////		CV_FLOODFILL_FIXED_RANGE
////		(fill relative to the seed point pixel value; otherwise, fill relative to the neighbor’s value),

////	 	CV_FLOODFILL_MASK_ONLY (fill the mask location instead of the source image location). Obviously, you must supply an appropriate mask
////		if CV_FLOODFILL_MASK_ONLY is set.

////	3)The middle bits (8–15) of flags can be set to the value with which you want the mask to be filled.
////	If the middle bits of flags are 0s, the mask will be filled with 1s.

////	All these flags may be linked together via OR. For example, if you want an 8-way connectivity fill
////	filling only a fixed range, filling the mask not the image, and filling using a value of 47, then the parameter to pass in would be:
////	flags =	8| CV_FLOODFILL_MASK_ONLY | CV_FLOODFILL_FIXED_RANGE |	(47<<8);
////	(47<<8) means we want to shift



////	CvPoint seed_point = cvPoint(100,100);
//	CvScalar color = CV_RGB(250,0,0);

//	cvFloodFill( out, seed_point, color, cvScalarAll(5.0), cvScalarAll(5.0), NULL,8, NULL );
//	return;
//}

//void FloodFill_Test(char ** argv)
//{
//    char * name="FloodFill Demo";
//	IplImage * src;
//	CvPoint seed_point = cvPoint(100,100);
//	src=cvLoadImage(argv[1],1);
//	IplImage * out=cvCloneImage(src);
//	cvNamedWindow( name, 1 );
//	FloodFill(src,out, seed_point);
//	cvShowImage(name, out);
//	cvWaitKey( 0 );
//	cvReleaseImage( &src );
//	cvReleaseImage( &out );
//	cvDestroyWindow( name );
//	return;

//}

void CppStyeAddTextToImage(cv::Mat src, std::string caption )
{
	int DELAY_CAPTION = 1500;
	cv::Mat dst;
	dst = Mat::zeros( src.size(), src.type() );
    putText( dst, caption.c_str(), Point( src.cols/4, src.rows/2),cv::FONT_HERSHEY_COMPLEX, 1, Scalar(255, 255, 255) );
	imshow( "Filter Demo", dst );
	waitKey( DELAY_CAPTION );
	return;
}

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


//Canny is based on first derivative

/// Global variables

Mat canny_edge_src, canny_edge_src_gray;
Mat canny_edge_dst, canny_edge_detected_edges;

int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
char* window_name = "Edge Map";


//void CannyEdgeDetector(int, void*)
//{
//    /*
//    Step 1:
//        Apply a Gaussian blur
//    Step 2:
//        Find edge gradient strength and angle
//    Step 3:
//        classify direction of gradient into 0,1,2,3
//        round the gradient direction theta to nearest 45 degree
//        i.e if gradient angle is 23 degree it will fall into 0 interval

//    Step 4:
//        Suppress non-maximum edges
//        for a 3x3 neighbor of each pixel
//        select two neigbors which are in the direction of gradient, i.e if the direction of gradient at point x is 1, we select the neighbor at top right and
//        bottom left:

//        3 2 1
//        0 x 0
//        1 2 3

//        and if the magnitude of gradient is biger than both keep it otherwise set it to zero

//    Step 5:
//        Hysteresis thresholding: use of two thresholds, a high   HT and a low LT:
//        pixel value in non-maxima suppressed image M (x,y) greater than HT is immediately accepted as an edge pixel
//        pixel value in non-maxima suppressed image M (x,y) below the LT is immediately rejected.
//        pixels in M (x,y) whose values lie between the two thresholds are accepted if they are connected to the already detected edge pixels
//    */

//    /// Reduce noise with a kernel 3x3
//    blur( canny_edge_src_gray, canny_edge_detected_edges, Size(3,3) );

//    /// Canny detector
//    cv::Canny( canny_edge_detected_edges, canny_edge_detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

//    /// Using Canny's output as a mask, we display our result
//    canny_edge_dst = Scalar::all(0);

//    canny_edge_src.copyTo( canny_edge_dst, canny_edge_detected_edges);
//    imshow( window_name, canny_edge_dst );
//}

//int CannyEdgeDetector_Test( char** argv )
//{
//    /// Load an image
//    canny_edge_src = imread( argv[1] );

//    if( !canny_edge_src.data )
//    { return -1; }

//    /// Create a matrix of the same type and size as src (for dst)
//    canny_edge_dst.create( canny_edge_src.size(), canny_edge_src.type() );

//    /// Convert the image to grayscale
//    cvtColor( canny_edge_src, canny_edge_src_gray, cv::COLOR_BGR2GRAY );

//    /// Create a window
//    namedWindow( window_name, CV_WINDOW_AUTOSIZE );

//    /// Create a Trackbar for user to enter threshold
//    createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyEdgeDetector );

//    /// Show the image
//    CannyEdgeDetector(0, 0);

//    /// Wait until user exit program by pressing a key
//    waitKey(0);

//    return 0;
//}


//Sobel is based on first derivative so we have to set a threshold value after applying the algorithm to threshold the edges
//void SobelEdgeDetector(char ** argv)
//{
//    Mat src, src_gray;
//    Mat grad;
//    char* window_name = "Sobel Demo - Simple Edge Detector";
//    int scale = 1;
//    int delta = 0;
////    If src is 8-bit then the dst must be of depth IPL_DEPTH_16S to avoid overflow.
//    int ddepth = CV_16S;

//    int c;

//    /// Load an image
//    src = imread( argv[1] );

//    if( !src.data )
//    { return ; }

//    GaussianBlur( src, src, Size(3,3), 0, 0, BORDER_DEFAULT );

//    /// Convert it to gray
//    cvtColor( src, src_gray, CV_RGB2GRAY );

//    /// Create window
//    namedWindow( window_name, CV_WINDOW_AUTOSIZE );

//    /// Generate grad_x and grad_y
//    Mat grad_x, grad_y;
//    Mat abs_grad_x, abs_grad_y;
//    int dx_order;
//    int dy_order;
///*
//    kernel_size=aperture_size
//    The aperture_size parameter should be odd and is the width (and the height) of the square fi lter. Currently, aperture_sizes of 1, 3, 5, and 7 are supported.
//    The larger kernels give a better approximation to the derivative because the smaller kernels are very sensitive to noise.
//*/
//    int kernel_size=3;
   
    

//    /// Gradient X
//    //Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
//    dx_order=1;
//    dy_order=0;
//    Sobel( src_gray, grad_x, ddepth, dx_order, dy_order, kernel_size, scale, delta, BORDER_DEFAULT );
//    //because we can't display negative values so we have to convert the scale
//    convertScaleAbs( grad_x, abs_grad_x );

//    /// Gradient Y
//    dx_order=0;
//    dy_order=1;
//    //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
//    Sobel( src_gray, grad_y, ddepth, dx_order, dy_order, kernel_size, scale, delta, BORDER_DEFAULT );
//    //because we can't display negative values so we have to convert the scale
//    convertScaleAbs( grad_y, abs_grad_y );

//    /// Total Gradient (approximate)
//    //TotalGradient=sqrt( grad_y^2 + grad_t^2 ) ==>approximate  =0.5*abs_grad_x+ 0.5*abs_grad_y
//    addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );

//    imshow( window_name, grad );

//    waitKey(0);
//}

//void CppStyleDisplayingCamera()
//{
//    VideoCapture webCam(1); // open the default camera
//    webCam.set(CV_CAP_PROP_FRAME_WIDTH,640);
//    webCam.set(CV_CAP_PROP_FRAME_HEIGHT,480);
//    if(!webCam.isOpened())  // check if we succeeded
//	return;

//    namedWindow("camera",1);
//    for(;;)
//    {
//        Mat frame;
//        webCam >> frame; // get a new frame from camera
//        imshow("camera", frame);
//        if(waitKey(200) >= 0) break;
//    }
//    // the camera will be deinitialized automatically in VideoCapture destructor
//    return ;
//}

void CppStyleDisplayingVideo(char ** argv)
{
    VideoCapture vid;
    vid.open("VIDEO0005.mp4");
    namedWindow("playing a vidoe file",1);

    for(;;)
    {
        Mat frame;
        vid >> frame; // get a new frame from camera
        imshow("playing a vidoe file", frame);
        if(waitKey(20) >= 0) break;
    }
    
}


//void DetectFaceUsingHaar(IplImage* img, double scale,char* cascade_name)
//{
//	CvHaarClassifierCascade* cascade = (CvHaarClassifierCascade*)cvLoad( cascade_name,  NULL, NULL, NULL);
//	CvMemStorage* storage = cvCreateMemStorage(0);

//	static CvScalar colors[] =
//	{
//			{{0,0,255}}, {{0,128,255}},{{0,255,255}},{{0,255,0}},
//			{{255,128,0}},{{255,255,0}},{{255,0,0}}, {{255,0,255}}
//	};//Just some pretty colors to draw with


//	IplImage* gray = cvCreateImage( cvSize(img->width,img->height), 8, 1 );
//	IplImage* small_img = cvCreateImage(cvSize( cvRound(img->width/scale), cvRound(img->height/scale)), 8, 1 );
//	cvCvtColor( img, gray, cv::COLOR_BGR2GRAY );
//	cvResize( gray, small_img, CV_INTER_LINEAR );
//	cvEqualizeHist( small_img, small_img );
//	// DETECT OBJECTS IF ANY
//	//
//	cvClearMemStorage( storage );

////	specifies how quickly OpenCV should increase the scale for face detections with each pass it makes over an image.
////	Setting this higher makes the detector run faster (by running fewer passes),
////	but if it's too high, you may jump too quickly between scales and miss faces.
////	The default in OpenCV is 1.1, in other words, scale increases by a factor of 1.1 (10%) each pass.
//	double scaleFactor=1.1;

////	Minimum number (minus 1) of neighbor rectangles that makes up an object. All the groups of a smaller number of rectangles
////	than min_neighbors-1 are rejected. If min_neighbors is 0, the function does not any grouping at all and returns all the
////	detected candidate rectangles, which may be useful if the user wants to apply a customized grouping procedure
////	http://www.cognotics.com/opencv/servo_2007_series/part_2/page_2.html
//	int minNeighbors=10;

////	The sixth parameter to cvHaarDetectObjects() is a flag variable. There are currently only two options:
////	0 or CV_HAAR_DO_CANNY_PRUNING.
////	If the Canny Pruning option is selected, the detector skips image regions that are unlikely to contain a face,
////	reducing computational overhead and possibly eliminating some false detections.
////	The regions to skip are identified by running an edge detector (the Canny edge detector) over the image before running
////	the face detector.
//	int flags=CV_HAAR_DO_CANNY_PRUNING;

////	Minimum window size. By default, it is set to the size of samples the classifier has been trained
////	on (~20x20 for face detection), have look in the xml file, <size> 24 24 </size>.
//	CvSize minSize=cvSize(20,20) ;


//	CvSeq* objects = cvHaarDetectObjects(small_img, cascade, storage,scaleFactor,minNeighbors,flags,minSize);
//	std::cout<<objects->total<<std::endl;

//	// LOOP THROUGH FOUND OBJECTS AND DRAW BOXES AROUND THEM
//	//
//	cvNamedWindow("face",CV_WINDOW_AUTOSIZE);


//	  std::stringstream ImageWithRectangulareFarme;


//	for(int i = 0; i < (objects ? objects->total : 0); i++ )
//	{
//		CvRect* r = (CvRect*)cvGetSeqElem( objects, i );
//		cvRectangle(img, cvPoint(r->x ,r->y), 	cvPoint(r->x+r->width,r->y+r->height),	colors[i%8]);
////		cout<<ImageWithRectangulareFarme.str().c_str()<<endl;
////		cvSaveImage(ImageWithRectangulareFarme.str().c_str(),img);


////		CvRect* r = (CvRect*)cvGetSeqElem( faces, i );
////		CvPoint center;
////		int radius;
////		center.x = cvRound((r->x + r->width*0.5)*scale);
////		center.y = cvRound((r->y + r->height*0.5)*scale);
////		radius = cvRound((r->width + r->height)*0.25*scale);
////		cvCircle( img, center, radius, colors[i%8], 3, 8, 0 );
//	}

////	cvShowImage("face",small_img);
//	cvShowImage("face",img);
//	cvWaitKey(2000);

//	cvReleaseImage( &gray );
//	cvReleaseImage( &small_img );
//}

//void DetectFaceUsingHaar_Test( char** argv )
//{
// //	exmaple of call images/ "haar/haarcascade_frontalface_alt2.xml"
//	IplImage* src= cvLoadImage(argv[1],1);
//	DetectFaceUsingHaar(src,1,argv[2] );
//}

void Dialation()
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

//void Thresholding_Test(char **argv)
//{
//	int threshold_type;
//	double Threshold;
//	double MaxValue ;
//	IplImage *src;


//	CStyleLoadImage(argv[1],CV_LOAD_IMAGE_GRAYSCALE,src);
//	IplImage * dst=cvCloneImage(src);
//	Thresholding(CV_THRESH_BINARY ,10,255,src,dst);


//	cvNamedWindow( "Original", CV_WINDOW_AUTOSIZE );
//	cvShowImage( "Original", src );


//	cvNamedWindow( "CV_THRESH_BINARY", CV_WINDOW_AUTOSIZE );
//	cvShowImage( "CV_THRESH_BINARY", dst );


//	waitKey(0);

//	cvReleaseImage(&src);
//	cvReleaseImage(&dst);
//	cvDestroyWindow("Original");
//	cvDestroyWindow("CV_THRESH_BINARY");
//}


/*
Histogram Matching: 
    Creating new image which has new distribution function (pdf)
Histogram Equalization:
    Creating new image which has new uniform distribution

Contrast Stretching =HistogramNormalization= Histogram Stretching
*/
void HistogramNormalization(int argc, char** argv)
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

    cv::Mat src_img= imread(argv[1]);
    cv::Mat dst_img_NORM_L2;
    cv::Mat dst_img_NORM_L1;
    
    cv::Mat dst_img;
    src_img.copyTo(dst_img); 
    vector<Mat> bgr_planes;
    split( src_img, bgr_planes );
    
    src_img.copyTo(dst_img_NORM_L2);
    src_img.copyTo(dst_img_NORM_L1);
    cv::normalize(src_img,dst_img_NORM_L2);
    cv::normalize(src_img,dst_img_NORM_L1);
   
    
    cv::normalize(bgr_planes[0],bgr_planes[0]);
    cv::normalize(bgr_planes[1],bgr_planes[1]);
    cv::normalize(bgr_planes[2],bgr_planes[2]);
    
    
    merge(bgr_planes,dst_img);
    
//     imshow("NORM_L2",dst_img_NORM_L2);
//     imshow("NORM_L1",dst_img_NORM_L1);
    imshow("Original",src_img);
    imshow ("Normalized",dst_img);
    waitKey(0);
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




void MatrixTypeNumber()
{
/*

    A Mapping of Type to Numbers in OpenCV

            C1	C2	C3	C4
    CV_8U	0	8	16	24
    CV_8S	1	9	17	25
    CV_16U	2	10	18	26
    CV_16S	3	11	19	27
    CV_32S	4	12	20	28
    CV_32F	5	13	21	29
    CV_64F	6	14	22	30
*/
    int number_of_rows, number_of_columns;
    number_of_rows=3;
    number_of_columns=4;
    cv::Mat mat_CV_8U(number_of_rows, number_of_columns,CV_8UC1);
    std::cout<<"mat_CV_8U.type(): " <<mat_CV_8U.type() <<std::endl;


    cv::Mat mat_CV_64FC1(number_of_rows, number_of_columns,CV_64FC1);
    std::cout<<"mat_CV_64FC1.type(): " <<mat_CV_64FC1.type() <<std::endl;

/*

    Unsigned 8bits uchar 0~255
    IplImage: IPL_DEPTH_8U
    Mat: CV_8UC1, CV_8UC2, CV_8UC3, CV_8UC4

    Signed 8bits char -128~127
    IplImage: IPL_DEPTH_8S
    Mat: CV_8SC1，CV_8SC2，CV_8SC3，CV_8SC4

    Unsigned 16bits ushort 0~65535
    IplImage: IPL_DEPTH_16U
    Mat: CV_16UC1，CV_16UC2，CV_16UC3，CV_16UC4

    Signed 16bits short -32768~32767
    IplImage: IPL_DEPTH_16S
    Mat: CV_16SC1，CV_16SC2，CV_16SC3，CV_16SC4

    Signed 32bits int -2147483648~2147483647
    IplImage: IPL_DEPTH_32S
    Mat: CV_32SC1，CV_32SC2，CV_32SC3，CV_32SC4

    Float 32bits float -1.18*10-38~3.40*10-38
    IplImage: IPL_DEPTH_32F
    Mat: CV_32FC1，CV_32FC2，CV_32FC3，CV_32FC4

    Double 64bits double
    Mat: CV_64FC1，CV_64FC2，CV_64FC3，CV_64FC4

    Unsigned 1bit bool
    IplImage: IPL_DEPTH_1U


*/



}

void MeanShift()
{

}

void MeanShift_Test()
{

}

void CamShift_Test()
{

}

/*PCA example
./BasicOperations ../images/pca_rectangle.png
./BasicOperations ../images/gedeck1.jpg
*/

void drawAxis(Mat& img, Point p, Point q, Scalar colour, const float scale = 0.2)
{
    double angle;
    double hypotenuse;
    angle = atan2( (double) p.y - q.y, (double) p.x - q.x ); // angle in radians
    hypotenuse = sqrt( (double) (p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));
//    double degrees = angle * 180 / CV_PI; // convert radians to degrees (0-180 range)
//    cout << "Degrees: " << abs(degrees - 180) << endl; // angle in 0-360 degrees range
    // Here we lengthen the arrow by a factor of scale
    q.x = (int) (p.x - scale * hypotenuse * cos(angle));
    q.y = (int) (p.y - scale * hypotenuse * sin(angle));
    cv:line(img, p, q, colour, 1, LINE_AA);
    // create the arrow hooks
    p.x = (int) (q.x + 9 * cos(angle + CV_PI / 4));
    p.y = (int) (q.y + 9 * sin(angle + CV_PI / 4));
    cv::line(img, p, q, colour, 1, LINE_AA);
    p.x = (int) (q.x + 9 * cos(angle - CV_PI / 4));
    p.y = (int) (q.y + 9 * sin(angle - CV_PI / 4));
    line(img, p, q, colour, 1, LINE_AA);
}
double getOrientation(const vector<Point> &pts, Mat &img)
{
    //Construct a buffer used by the pca analysis
    int sz = static_cast<int>(pts.size());
    Mat data_pts = Mat(sz, 2, CV_64FC1);
    for (int i = 0; i < data_pts.rows; ++i)
    {
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }
    //Perform PCA analysis
    PCA pca_analysis(data_pts, Mat(), PCA::DATA_AS_ROW);
    //Store the center of the object
    Point cntr = Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
                      static_cast<int>(pca_analysis.mean.at<double>(0, 1)));
    //Store the eigenvalues and eigenvectors
    vector<Point2d> eigen_vecs(2);
    vector<double> eigen_val(2);
    for (int i = 0; i < 2; ++i)
    {
        eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                                pca_analysis.eigenvectors.at<double>(i, 1));
        eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
    }
    // Draw the principal components
    circle(img, cntr, 3, Scalar(255, 0, 255), 2);
    Point p1 = cntr + 0.02 * Point(static_cast<int>(eigen_vecs[0].x * eigen_val[0]), static_cast<int>(eigen_vecs[0].y * eigen_val[0]));
    Point p2 = cntr - 0.02 * Point(static_cast<int>(eigen_vecs[1].x * eigen_val[1]), static_cast<int>(eigen_vecs[1].y * eigen_val[1]));
    drawAxis(img, cntr, p1, Scalar(0, 255, 0), 1);
    drawAxis(img, cntr, p2, Scalar(255, 255, 0), 5);
    double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x); // orientation in radians
    return angle;
}
int draw_PCA(int argc , char** argv)
{
    // Load image
    Mat src = imread(argv[1]);
    // Check if image is loaded successfully
    if(!src.data || src.empty())
    {
        cout << "Problem loading image!!!" << endl;
        return EXIT_FAILURE;
    }
    imshow("src", src);
    // Convert image to grayscale
    Mat gray;
    cvtColor(src, gray, COLOR_BGR2GRAY);
    // Convert image to binary
    Mat bw;
    threshold(gray, bw, 50, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    // Find all the contours in the thresholded image
    vector<Vec4i> hierarchy;
    vector<vector<Point> > contours;
    findContours(bw, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
    for (size_t i = 0; i < contours.size(); ++i)
    {
        // Calculate the area of each contour
        double area = contourArea(contours[i]);
        // Ignore contours that are too small or too large
        if (area < 1e2 || 1e5 < area) continue;
        // Draw each contour only for visualisation purposes
        drawContours(src, contours, static_cast<int>(i), Scalar(0, 0, 255), 2, 8, hierarchy, 0);
        // Find the orientation of each shape
        getOrientation(contours[i], src);
    }
    imshow("output", src);
    waitKey(0);
    return 0;
}


void remap_example(int argc, char** argv )
{


    cv::Mat input=cv::imread(argv[1]);
    //just to create a matrix with exact same dims and type
    cv::Mat output,map_y,map_x;
    output.create(input.size(),input.type());
        map_x.create( input.size(), CV_32FC1 );
        map_y.create( input.size(), CV_32FC1 );


    /*Here we want to mirror the input image which is lena and has the dims of 512x512
     that means we map every point of input

    map_x is responsible for rows
    map_x.at(0,0)=512
    map_x.at(0,1)=511
    .
    .
    .
    map_y is responsible for cols
    map_y.at(0,0)=0
    map_y.at(0,1)=0
    map_y.at(0,2)=0

    */
    for(int i=0;i<input.rows;i++)
    {
        for(int j=0;j<input.cols;j++)
        {
            map_x.at<float>(i,j) = (float)(input.cols - j) ;
            map_y.at<float>(i,j) = (float)i ;
//            std::cout<<"map_x.at("<<i<<","<<j<<")=" << map_x.at<float>(i,j)<<std::endl;
//            std::cout<<"map_y.at("<<i<<","<<j<<")=" <<map_y.at<float>(i,j)  <<std::endl;
        }
    }
    for(;;)
    {
        int c = waitKey( 1000 );
        if( (char)c == 27 )
        { break; }
        cv::remap( input, output, map_x, map_y, INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0) );
        // Display results
        cv::imshow( "input_image", input );
        cv::imshow( "output_image", output );
        cv::imshow( "map_x", map_x );
        cv::imshow( "map_y", map_y );
    }
}




/*
 Apply Shi-Tomasi corner detector
 http://aishack.in/tutorials/shitomasi-corner-detector/
*/

/// Global variables
cv::Mat src_SubPix, src_gray_SubPix;

int maxCorners = 50;
int maxTrackbar = 100;

RNG rng(12345);
char* source_window = "Image";


void goodFeaturesToTrack_Demo( int, void* )
{
  if( maxCorners < 1 ) { maxCorners = 1; }

  /// Parameters for Shi-Tomasi algorithm
  vector<Point2f> corners;
  double qualityLevel = 0.01;
  double minDistance = 10;
  int blockSize = 3;
  bool useHarrisDetector = false;
  double k = 0.04;

  /// Copy the source image
  Mat copy;
  copy = src_SubPix.clone();

  /// Apply corner detection
  goodFeaturesToTrack( src_gray_SubPix,
                       corners,
                       maxCorners,
                       qualityLevel,
                       minDistance,
                       Mat(),
                       blockSize,
                       useHarrisDetector,
                       k );


  /// Draw corners detected
  cout<<"** Number of corners detected: "<<corners.size()<<endl;
  int r = 4;
  for( int i = 0; i < corners.size(); i++ )
     { circle( copy, corners[i], r, Scalar(rng.uniform(0,255), rng.uniform(0,255),
                                                 rng.uniform(0,255)), -1, 8, 0 ); }

  /// Show what you got
  cv::namedWindow( source_window, cv::WINDOW_AUTOSIZE );
  imshow( source_window, copy );

  /// Set the neeed parameters to find the refined corners
  Size winSize = Size( 5, 5 );
  Size zeroZone = Size( -1, -1 );
//  TermCriteria criteria = TermCriteria( cv::TERMCRIT_EPS + cv::TERMCRIT_ITER, 40, 0.001 );
   TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::MAX_ITER|cv::TermCriteria::EPS, 40, 0.001);
  /// Calculate the refined corner locations
  cornerSubPix( src_gray_SubPix, corners, winSize, zeroZone, criteria );

  /// Write them down
  for( int i = 0; i < corners.size(); i++ )
     { cout<<" -- Refined Corner ["<<i<<"]  ("<<corners[i].x<<","<<corners[i].y<<")"<<endl; }
}

//void cornerSubPix_Example(int argc, char ** argv)
//{
//    /// Load source image and convert it to gray
//    src_SubPix = cv::imread( argv[1], 1 );
//    cvtColor( src_SubPix, src_gray_SubPix, cv::COLOR_BGR2GRAY );

//    /// Create Window
//    cv::namedWindow( source_window, CV_WINDOW_AUTOSIZE );

//    /// Create Trackbar to set the number of corners
//    cv::createTrackbar( "Max  corners:", source_window, &maxCorners, maxTrackbar, goodFeaturesToTrack_Demo);

//    cv::imshow( source_window, src_SubPix );

//    goodFeaturesToTrack_Demo( 0, 0 );

//    waitKey(0);
//    return;
//}

//http://docs.opencv.org/3.0-beta/doc/py_tutorials/py_calib3d/py_pose/py_pose.html
void draw_xyz_frame_over_image(cv::Mat image,cv::Point2f reference_point,std::vector<cv::Point2f >end_points ,int thickness=5)
{
    cv::line(image,reference_point, end_points.at(0),cv::Scalar(255,0,0),thickness);
    cv::line(image,reference_point,end_points.at(1) ,cv::Scalar(0,255,0),thickness);
    cv::line(image,reference_point,end_points.at(2) ,cv::Scalar(0,0,255),thickness);
}

void chessboard_pose_estimation(int argc, char **argv)
{
    cv::VideoCapture camera(0);
    cv::Mat image,image_gray;
    std::vector<cv::Point2f> chess_board_corners;
    int width, height;
    width=9;
    height=6;
    double square_size=0.025;
    std::vector<cv::Point3f> chess_board_point;
    cv::Size board_size=cv::Size (width, height);
    bool found;

    cv::Mat rvec(1,3,cv::DataType<double>::type);
    cv::Mat tvec(1,3,cv::DataType<double>::type);
    cv::Mat axis=cv::Mat::eye(3,3,CV_32F)*3;
    std::vector<cv::Point2f> axis_end_point;


    for(std::size_t i=0;i<width;i++)
    {
        for(std::size_t j=0;j<height;j++)
        {
            cv::Point3f chess_board_3d_point;
            chess_board_3d_point.x=i*square_size;
            chess_board_3d_point.y=j*square_size;
            chess_board_3d_point.z=0;
            chess_board_point.push_back(chess_board_3d_point);
//            std::cout<<"x: "<<i*square_size <<std::endl;
//            std::cout<<"y: "<<j*square_size <<std::endl;
        }
    }



    std::string camera_calibration_path="front_webcam.yml";
    cv::FileStorage fs(camera_calibration_path,cv::FileStorage::READ);
    cv::Mat camera_matrix, distortion_coefficient;
    fs["camera_matrix"]>>camera_matrix;
    fs["distortion_coefficients"]>>distortion_coefficient;

    while(true)
    {
        camera>>image;
        cv::cvtColor(image,image_gray,cv::COLOR_BGR2GRAY);
        int key=cv::waitKey(30);
        if( (char)key==(char)27 )
        {
            break;
        }
        found=cv::findChessboardCorners(image,board_size,chess_board_corners);
        if(found)
        {
//            std::cout<<"found" <<std::endl;
            TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::MAX_ITER|cv::TermCriteria::EPS, 30, 0.1);
            cv::cornerSubPix(image_gray,chess_board_corners, Size(11, 11), Size(-1, -1), criteria);

            //cv::drawChessboardCorners(image,board_size,chess_board_corners,found);
            cv::solvePnPRansac(chess_board_point,chess_board_corners,camera_matrix,distortion_coefficient,rvec, tvec);
            std::cout<<tvec <<std::endl;
            std::cout<<"-----------------------------------------" <<std::endl;
            cv::projectPoints(axis,rvec,tvec,camera_matrix,distortion_coefficient,axis_end_point);

            draw_xyz_frame_over_image(image,chess_board_corners.at(0), axis_end_point );
        }

        cv::imshow("pose",image);
    }

    return;
}


void readingCameraMatrix()
{

    std::string camera_calibration_path="../data/front_webcam.yml";
    cv::FileStorage fs(camera_calibration_path,cv::FileStorage::READ);
    cv::Mat camera_matrix, distortion_coefficient;
    fs["camera_matrix"]>>camera_matrix;
    fs["distortion_coefficients"]>>distortion_coefficient;

//    std::cout<<"Camera Matrix:" <<std::endl;
//    std::cout<<camera_matrix <<std::endl;
    std::cout<<"Fx: " <<camera_matrix.at<double>(0,0) <<std::endl;
    std::cout<<"Fy: " <<camera_matrix.at<double>(1,1) <<std::endl;

    std::cout<<"Cx: " <<camera_matrix.at<double>(0,2) <<std::endl;
    std::cout<<"Cy: " <<camera_matrix.at<double>(1,2) <<std::endl;
//    std::cout<< "Distortion Coefficient:"<<std::endl;
//    std::cout<<distortion_coefficient <<std::endl;

    std::cout<<"K1: "<<distortion_coefficient.at<double>(0,0) <<std::endl;
    std::cout<<"K2: "<<distortion_coefficient.at<double>(0,1) <<std::endl;
    std::cout<<"P1: "<<distortion_coefficient.at<double>(0,2) <<std::endl;
    std::cout<<"P2: "<<distortion_coefficient.at<double>(0,3) <<std::endl;
    std::cout<<"K3: "<<distortion_coefficient.at<double>(0,4) <<std::endl;

}


//http://paulbourke.net/miscellaneous/correlate/
//http://www.ee.ic.ac.uk/hp/staff/dmb/courses/E1Fourier/00800_Correlation.pdf
//https://docs.opencv.org/3.2.0/de/da9/tutorial_template_matching.html

/*
CV_TM_SQDIFF -> sum of square difference (ssd)
CV_TM_SQDIFF_NORMED ->  squared difference normalized
CV_TM_CCORR -> cross correlation
CV_TM_CCORR_NORMED cross correlation normalized
CV_TM_CCOEFF -> cross correlation coefficient
CV_TM_CCOEFF_NORMED -> cross correlation coefficient normalized

Pearson correlation coefficient ??
*/

/// Global Variables
Mat img; Mat templ; Mat result;
char* image_window = "Source Image";
char* result_window = "Result window";

int match_method;
int max_Trackbar = 5;


//void MatchingMethod( int, void* )
//{
//    /// Source image to display
//    Mat img_display;
//    img.copyTo( img_display );

//    /// Create the result matrix
//    int result_cols =  img.cols - templ.cols + 1;
//    int result_rows = img.rows - templ.rows + 1;

//    result.create( result_cols, result_rows, CV_32FC1 );

//    /// Do the Matching and Normalize
//    matchTemplate( img, templ, result, match_method );
//    normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );

//    /// Localizing the best match with minMaxLoc
//    double minVal; double maxVal; Point minLoc; Point maxLoc;
//    Point matchLoc;

//    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

//    /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
//    if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
//    { matchLoc = minLoc; }
//    else
//    { matchLoc = maxLoc; }

//    /// Show me what you got
//    rectangle( img_display, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
//    rectangle( result, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );

//    imshow( image_window, img_display );
//    imshow( result_window, result );

//    return;
//}

/** @function main */
//int template_matching_example( int argc, char** argv )
//{
//  /// Load image and template
//    img = imread( argv[1], 1 );
//    templ = imread( argv[2], 1 );

//    /// Create windows
//    namedWindow( image_window, CV_WINDOW_AUTOSIZE );
//    namedWindow( result_window, CV_WINDOW_AUTOSIZE );

//    /// Create Trackbar
//    char* trackbar_label = "Method: \n 0: SQDIFF \n 1: SQDIFF NORMED \n 2: TM CCORR \n 3: TM CCORR NORMED \n 4: TM COEFF \n 5: TM COEFF NORMED";
//    createTrackbar( trackbar_label, image_window, &match_method, max_Trackbar, MatchingMethod );

//    MatchingMethod( 0, 0 );

//    waitKey(0);
//    return 0;
//}

 //cv::FAS

int main(int argc, char** argv)
{
    drawingFunsction();
}




